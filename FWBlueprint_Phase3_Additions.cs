// FILE: FWBlueprint_Phase3A_EdgeIntersections.cs
// Phase 3A: 2D panel extraction + edge notch detection (edge-line intersections) + interior hole detection + auto-dimensioning

using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Rhino.UI;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Drawing;
using System.Linq;

namespace FWBlueprintPlugin
{
    public partial class FWBlueprintPluginCommand : Command
    {
        // ===============================================
        // HELPER CLASS: Edge Segment Data
        // ===============================================
        private class EdgeSegmentData
        {
            public List<(Curve seg, int index)> BottomSegments { get; set; } = new List<(Curve seg, int index)>();
            public List<(Curve seg, int index)> RightSegments { get; set; } = new List<(Curve seg, int index)>();
            public List<(Curve seg, int index)> TopSegments { get; set; } = new List<(Curve seg, int index)>();
            public List<(Curve seg, int index)> LeftSegments { get; set; } = new List<(Curve seg, int index)>();
            public List<(Curve seg, int index)> InteriorSegments { get; set; } = new List<(Curve seg, int index)>();
        }

        // ===============================================
        // HELPER CLASS: Edge Feature Detection Data
        // ===============================================
        private class EdgeFeature
        {
            public string EdgeName { get; set; }      // "BOTTOM", "RIGHT", "TOP", "LEFT"
            public double StartPos { get; set; }      // Position along edge where feature starts
            public double EndPos { get; set; }        // Position along edge where feature ends
            public double Width { get; set; }         // Width of feature (EndPos - StartPos)
            public double Depth { get; set; }         // Depth into panel (from ray scanning)
            public string Type { get; set; }          // "EdgeHole" or "EdgeSlot"
            public double CenterPos { get; set; }     // For holes: center position (StartPos + Width/2)

            public EdgeFeature(string edgeName, double startPos, double endPos, double width, double depth, string type)
            {
                EdgeName = edgeName;
                StartPos = startPos;
                EndPos = endPos;
                Width = width;
                Depth = depth;
                Type = type;
                CenterPos = startPos + (width / 2.0);
            }
        }

        // ===============================================
        // PHASE 3A ENTRYPOINT
        // ===============================================
        private void ExtractAndCreate2DPanels(RhinoDoc doc, Layer parentLayer)
        {

            // Find or create Blueprint layer subtree
            Layer blueprintLayer = FindOrCreateBlueprintLayer(doc, parentLayer);
            if (blueprintLayer == null)
            {
                return;
            }

            // Rename Panels → 3D Panels (if needed); create 2D layers
            string panelsPath = $"{blueprintLayer.FullPath}::Panels";
            int idx3D = GetLayerIndexByFullPath(doc, panelsPath);

            if (idx3D >= 0)
            {
                var panelsLayer = doc.Layers[idx3D];
                panelsLayer.Name = "3D Panels";
                panelsLayer.Color = Color.Black;
                doc.Layers.Modify(panelsLayer, idx3D, true);
            }
            else
            {
                idx3D = FindOrCreateChildLayer(doc, blueprintLayer, "3D Panels", Color.Black);
            }

            int idx2D = FindOrCreateChildLayer(doc, blueprintLayer, "2D Panels", Color.Black);
            int idxCutouts = FindOrCreateChildLayer(doc, blueprintLayer, "Cutouts", Color.Black);
            int idxPocket = FindOrCreateChildLayer(doc, blueprintLayer, "Pocket Curves", Color.Black);
            int idxDims = FindOrCreateChildLayer(doc, blueprintLayer, "Dimensions", Color.Red);
            doc.Layers[idxPocket].IsVisible = false;

            var tol = doc.ModelAbsoluteTolerance > 0 ? doc.ModelAbsoluteTolerance : 0.01;

            // Dashed linetype for bboxes
            int dashLineTypeIdx = EnsureDashedLinetype(doc);

            // Collect 3D panels
            RhinoObject[] srcObjs = doc.Objects.FindByLayer(doc.Layers[idx3D]);
            if (srcObjs == null || srcObjs.Length == 0)
            {
                return;
            }


            var panelCutouts = new List<(RhinoObject Panel, List<Brep> ChordCutouts)>();
            int bboxCount = 0;
            int cutoutCount = 0;

            foreach (var obj in srcObjs)
            {
                var brep = obj.Geometry as Brep;
                if (brep == null) continue;

                string category = obj.Attributes.GetUserString("PanelCategory") ?? "Unknown";


                // Get bounding box rectangle in Top view (XY)
                var bbox = GetTopViewBoundingBox(brep);
                if (!bbox.IsValid)
                {
                    continue;
                }

                // Draw bbox in 2D layer (dashed)
                var bboxCurve = bbox.ToNurbsCurve();
                var bboxAttr = new ObjectAttributes
                {
                    LayerIndex = idx2D,
                    LinetypeSource = ObjectLinetypeSource.LinetypeFromObject,
                    LinetypeIndex = dashLineTypeIdx,
                    ColorSource = ObjectColorSource.ColorFromLayer
                };
                bboxAttr.SetUserString("PanelCategory", category);
                bboxAttr.SetUserString("GeometryType", "BoundingBox");
                doc.Objects.AddCurve(bboxCurve, bboxAttr);
                bboxCount++;


                // Extract true boundary (trimmed outer loop → edge curves → join → project)
                var panelBoundary = GetPanelBoundaryCurve(brep, tol);
                if (panelBoundary == null)
                {
                    continue;
                }


                // NEW: Analyze and label boundary segments by edge
                var segmentData = AnalyzeAndLabelBoundarySegments_WithData(panelBoundary, bbox, category, tol);

                // Get the panel Brep (should already exist from surface extraction)
                Brep panelBrep = brep;  // Use the Brep from line 79
                var edgeCutouts = DetectEdgeFeaturesFromSegments_RayProtocol(doc, bbox, panelBoundary, segmentData, panelBrep, tol, idxDims);


                // Interior features (unchanged)
                var interiorCutouts = DetectInteriorFeatures(brep, bbox, tol);


                // Classify, colorize (magenta = has curves), and collect chord passthroughs
                var chordCutouts = new List<Brep>();
                foreach (var cutout in edgeCutouts.Concat(interiorCutouts))
                {
                    if (ClassifyCutoutSurface(cutout, tol) == Color.Magenta)
                    {
                        // chord passthrough (rounded/curved)
                        chordCutouts.Add(cutout);
                    }
                    else
                    {
                        // rectangular notch: add as green
                        var notchAttr = new ObjectAttributes
                        {
                            LayerIndex = idxCutouts,
                            ObjectColor = Color.Green,
                            ColorSource = ObjectColorSource.ColorFromObject
                        };
                        notchAttr.SetUserString("PanelCategory", category);
                        doc.Objects.AddBrep(cutout, notchAttr);
                    }
                    cutoutCount++;
                }

                if (chordCutouts.Count > 0)
                {
                    panelCutouts.Add((obj, chordCutouts));
                }

                string featureSummary;
                if (edgeCutouts.Count == 0 && interiorCutouts.Count == 0)
                {
                    featureSummary = "Clean ✓";
                }
                else
                {
                    featureSummary = $"{edgeCutouts.Count} edge, {interiorCutouts.Count} interior → Dimensioned ✓";
                }
                RhinoApp.WriteLine($"{category} ({bbox.Width:F2}\" × {bbox.Height:F2}\"): {featureSummary}");
            }

            // Keep layers visible and redraw
            doc.Layers[idx3D].IsVisible = true;
            doc.Layers[idx2D].IsVisible = true;
            doc.Layers[idxCutouts].IsVisible = true;
            doc.Views.Redraw();


            // Align any existing dims to Z=0 on this sheet
            DropBlueprintDimsToZ0(doc, blueprintLayer);

            // Auto-process chord pass-throughs (your existing cumulative chain + interior dims)
            if (panelCutouts.Count > 0)
            {
                ProcessChordPassThroughs(doc, FindOrCreateChildLayer(doc, blueprintLayer, "Cutouts", Color.Black),
                                         FindOrCreateChildLayer(doc, blueprintLayer, "Dimensions", Color.Red),
                                         panelCutouts, tol);
            }
        }

        // ===============================================
        // COMPLETE REWRITE: DetectEdgeFeatures_PerpendicularScanning
        // Replace entire DetectEdgeFeatures_Intersections method with this
        // ===============================================

        private List<Brep> DetectEdgeFeatures_Intersections(Rectangle3d bbox, Curve panelBoundary, double tol)
        {
            var cutouts = new List<Brep>();


            // Build the four panel edges
            var edges = new List<(Point3d A, Point3d B, string Name, Vector3d Inward)>
    {
        (bbox.Corner(0), bbox.Corner(1), "Bottom",  Vector3d.YAxis),
        (bbox.Corner(1), bbox.Corner(2), "Right",  -Vector3d.XAxis),
        (bbox.Corner(2), bbox.Corner(3), "Top",    -Vector3d.YAxis),
        (bbox.Corner(3), bbox.Corner(0), "Left",    Vector3d.XAxis)
    };

            const double scanInterval = 0.25;  // Scan every 0.25 inches along edge
            const double rayOffset = 1.0;      // Start rays 1" outside bbox
            const double baselineDepth = rayOffset;  // Expected depth for straight edges
            const double minFeatureWidth = 0.5;  // Minimum feature size to detect
            const double clusterGap = 1.0;  // Max gap between scan points in same feature
            const double depthTolerance = 0.1;  // Tolerance for "straight edge" detection

            // Process each edge
            foreach (var (ptA, ptB, edgeName, inwardDir) in edges)
            {


                double edgeLength = ptA.DistanceTo(ptB);
                Vector3d edgeDir = (ptB - ptA);
                edgeDir.Unitize();

                // Outward direction (opposite of inward)
                Vector3d outwardDir = -inwardDir;


                // Scan along the edge with offset rays
                var featurePoints = new List<(Point3d EdgePoint, double Distance, double Depth)>();

                int numScans = (int)Math.Ceiling(edgeLength / scanInterval);

                for (int i = 0; i <= numScans; i++)
                {
                    double distAlongEdge = Math.Min(i * scanInterval, edgeLength);
                    Point3d edgePoint = ptA + edgeDir * distAlongEdge;

                    // Start ray OUTSIDE the bbox by rayOffset distance
                    Point3d rayStart = edgePoint + outwardDir * rayOffset;
                    Point3d rayEnd = edgePoint + inwardDir * 50.0;  // Ray points inward across panel

                    Line ray = new Line(rayStart, rayEnd);
                    Curve rayCurve = ray.ToNurbsCurve();

                    // Intersect with boundary
                    var intersections = Intersection.CurveCurve(rayCurve, panelBoundary, tol, tol);

                    if (intersections != null && intersections.Count > 0)
                    {
                        // Find first intersection (closest to ray start)
                        double closestDist = double.MaxValue;
                        Point3d closestPoint = Point3d.Unset;

                        foreach (var inter in intersections)
                        {
                            double dist = rayStart.DistanceTo(inter.PointA);
                            if (dist < closestDist && dist > 0.01)  // Ignore tiny distances
                            {
                                closestDist = dist;
                                closestPoint = inter.PointA;
                            }
                        }

                        // Check if this indicates an indentation
                        if (closestDist < double.MaxValue)
                        {
                            // Depth of feature = how much farther than baseline
                            double featureDepth = closestDist - baselineDepth;

                            // Only record if depth exceeds tolerance (not a straight edge)
                            if (featureDepth > depthTolerance)
                            {
                                featurePoints.Add((edgePoint, distAlongEdge, closestDist));
                            }
                        }
                    }
                }


                if (featurePoints.Count == 0)
                {
                    continue;
                }

                // Cluster consecutive feature points
                var clusters = new List<List<(Point3d EdgePoint, double Distance, double Depth)>>();

                foreach (var point in featurePoints)
                {
                    bool addedToCluster = false;

                    foreach (var cluster in clusters)
                    {
                        // Check if close to any point in cluster
                        var lastInCluster = cluster.Last();
                        double gap = point.Distance - lastInCluster.Distance;

                        if (gap <= clusterGap)
                        {
                            cluster.Add(point);
                            addedToCluster = true;
                            break;
                        }
                    }

                    if (!addedToCluster)
                    {
                        clusters.Add(new List<(Point3d EdgePoint, double Distance, double Depth)> { point });
                    }
                }


                // Create Brep for each cluster
                int featureCount = 0;
                for (int c = 0; c < clusters.Count; c++)
                {
                    var cluster = clusters[c];

                    if (cluster.Count < 2)
                    {
                        continue;
                    }

                    // Calculate feature properties
                    double startDist = cluster.First().Distance;
                    double endDist = cluster.Last().Distance;
                    double mouthWidth = endDist - startDist;
                    double maxRayDist = cluster.Max(p => p.Depth);
                    double featureDepth = maxRayDist - baselineDepth;  // Actual depth of indentation


                    // Filter out features that are too small
                    if (mouthWidth < minFeatureWidth)
                    {
                        continue;
                    }

                    // Create Brep for this feature
                    try
                    {
                        Point3d startPoint = cluster.First().EdgePoint;
                        Point3d endPoint = cluster.Last().EdgePoint;

                        // Create corners of cutout rectangle
                        Point3d c1 = startPoint;
                        Point3d c2 = endPoint;
                        Point3d c3 = endPoint + inwardDir * featureDepth;
                        Point3d c4 = startPoint + inwardDir * featureDepth;

                        var cutoutCorners = new Point3d[] { c1, c2, c3, c4, c1 };
                        var cutoutCurve = new PolylineCurve(cutoutCorners);

                        if (cutoutCurve.IsClosed)
                        {
                            var planarBrep = Brep.CreatePlanarBreps(cutoutCurve, tol);
                            if (planarBrep != null && planarBrep.Length > 0)
                            {
                                cutouts.Add(planarBrep[0]);
                                featureCount++;
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                    }
                }


            }



            return cutouts;
        }

        // Helper to intersect a boundary curve with a finite segment and return robust points on the segment
        private List<Point3d> GetIntersectionsOnSegment(Curve boundary, LineCurve segment, double tol)
        {
            var list = new List<Point3d>();
            var ccx = Intersection.CurveCurve(boundary, segment, tol, tol);
            if (ccx == null || ccx.Count == 0) return list;

            foreach (var ev in ccx)
            {
                var pt = ev.PointA;
                // Keep only points that truly lie on the segment domain (with tolerance)
                double tSeg;
                if (segment.ClosestPoint(pt, out tSeg))
                {
                    var onSeg = tSeg >= segment.Domain.Min - tol && tSeg <= segment.Domain.Max + tol;
                    if (onSeg) list.Add(pt);
                }
            }
            return list;
        }


        // ===============================================
        // INTERIOR FEATURE DETECTION (unchanged)
        // ===============================================
        private List<Brep> DetectInteriorFeatures(Brep brep, Rectangle3d bbox, double tol)
        {
            var interiorCutouts = new List<Brep>();
            var bboxCurve = bbox.ToNurbsCurve();

            // Most horizontal face
            BrepFace horizontalFace = FindHorizontalFace(brep);
            if (horizontalFace == null) return interiorCutouts;

            foreach (var loop in horizontalFace.Loops)
            {
                if (loop.LoopType != BrepLoopType.Inner) continue;

                var loopCurve = loop.To3dCurve();
                if (loopCurve == null) continue;

                var projected = Curve.ProjectToPlane(loopCurve, Plane.WorldXY);
                if (projected == null || !projected.IsClosed) continue;

                // Must be fully interior (not touching bbox)
                var interEvents = Intersection.CurveCurve(projected, bboxCurve, tol, tol);
                if (interEvents.Count > 0) continue;

                var cutoutBreps = Brep.CreatePlanarBreps(projected, tol);
                if (cutoutBreps != null && cutoutBreps.Length > 0)
                    interiorCutouts.Add(cutoutBreps[0]);
            }

            return interiorCutouts;
        }

        // ===============================================
        // HORIZONTAL FACE (unchanged robust finder)
        // ===============================================
        private BrepFace FindHorizontalFace(Brep brep)
        {
            BrepFace horizontalFace = null;
            double bestAlignment = 0;

            foreach (var face in brep.Faces)
            {
                double u = face.Domain(0).Mid;
                double v = face.Domain(1).Mid;
                Vector3d normal = face.NormalAt(u, v);
                double alignment = Math.Abs(normal * Vector3d.ZAxis);
                if (alignment > bestAlignment)
                {
                    bestAlignment = alignment;
                    horizontalFace = face;
                }
            }
            return horizontalFace;
        }

        // ===============================================
        // CORRECTED VERSION: GetPanelBoundaryCurve
        // This fixes the API error - BrepFace doesn't have .Edges property
        // ===============================================

        private Curve GetPanelBoundaryCurve(Brep brep, double tol)
        {

            // Step 1: Find the top face (largest face by area)
            BrepFace topFace = null;
            double maxArea = 0;

            foreach (var face in brep.Faces)
            {
                var area = AreaMassProperties.Compute(face);
                if (area != null && area.Area > maxArea)
                {
                    maxArea = area.Area;
                    topFace = face;
                }
            }

            if (topFace == null)
            {
                return null;
            }

            // Step 2: Get edges from the Brep and filter by which face they belong to
            var edgeCurves = new List<Curve>();
            var processedEdges = new HashSet<int>();

            // Iterate through ALL edges in the Brep
            foreach (var edge in brep.Edges)
            {
                // Check if this edge is adjacent to our top face
                bool belongsToTopFace = false;

                // Each edge can be adjacent to 1 or 2 faces
                for (int i = 0; i < edge.AdjacentFaces().Length; i++)
                {
                    if (edge.AdjacentFaces()[i] == topFace.FaceIndex)
                    {
                        belongsToTopFace = true;
                        break;
                    }
                }

                if (!belongsToTopFace)
                    continue;

                // Skip duplicates
                if (processedEdges.Contains(edge.EdgeIndex))
                    continue;

                processedEdges.Add(edge.EdgeIndex);

                // Get the 3D curve for this edge
                var edgeCurve = edge.DuplicateCurve();
                if (edgeCurve != null)
                {
                    edgeCurves.Add(edgeCurve);
                }
            }

            if (edgeCurves.Count == 0)
            {
                return null;
            }

            // Step 3: Project all edges to Z=0 plane (flatten to 2D)
            var flattenedCurves = new List<Curve>();
            var xyPlane = Plane.WorldXY;

            foreach (var curve in edgeCurves)
            {
                // Project to XY plane
                var projected = Curve.ProjectToPlane(curve, xyPlane);
                if (projected != null)
                {
                    // Move to Z=0
                    var moveToZ0 = Transform.Translation(0, 0, -projected.PointAtStart.Z);
                    projected.Transform(moveToZ0);
                    flattenedCurves.Add(projected);
                }
            }

            // Step 4: Join curves into a single boundary
            var joined = Curve.JoinCurves(flattenedCurves, tol);

            if (joined == null || joined.Length == 0)
            {
                return null;
            }

            // Return the longest joined curve (should be the outer boundary)
            Curve longestCurve = joined[0];
            double longestLength = longestCurve.GetLength();

            for (int i = 1; i < joined.Length; i++)
            {
                double len = joined[i].GetLength();
                if (len > longestLength)
                {
                    longestLength = len;
                    longestCurve = joined[i];
                }
            }

            bool isClosed = longestCurve.IsClosed;

            // DEBUG: Count segments
            var segments = longestCurve.DuplicateSegments();
            if (segments != null)
            {
            }

            return longestCurve;
        }

        // ===============================================
        // USAGE:
        // ===============================================
        // Replace the ENTIRE GetPanelBoundaryCurve method (around line 430-540)
        // with this corrected version.
        //
        // KEY CHANGE:
        // OLD: foreach (var edge in topFace.Edges)  // ❌ This property doesn't exist
        // NEW: foreach (var edge in brep.Edges)      // ✅ Iterate Brep edges, filter by face
        // ===============================================

        // ===============================================
        // HELPER METHOD: Get Top Face
        // (Alternative approach if the above doesn't work)
        // ===============================================

        private BrepFace GetTopFace(Brep brep)
        {
            // Find face with normal closest to +Z (pointing up)
            BrepFace topFace = null;
            double maxZComponent = -1;

            foreach (var face in brep.Faces)
            {
                // Get face normal at center
                double u = face.Domain(0).Mid;
                double v = face.Domain(1).Mid;

                if (face.FrameAt(u, v, out Plane plane))
                {
                    double zComponent = plane.Normal.Z;
                    if (Math.Abs(zComponent) > maxZComponent)
                    {
                        maxZComponent = Math.Abs(zComponent);
                        topFace = face;
                    }
                }
            }

            return topFace;
        }

        // ===============================================
        // USAGE INSTRUCTIONS:
        // ===============================================
        // 1. Find GetPanelBoundaryCurve() in Addition.cs (around line 400-500)
        // 2. Replace the ENTIRE method with the version above
        // 3. If GetTopFace() helper doesn't exist, add it too
        // 4. Build and test
        //
        // WHAT TO EXPECT:
        // - Boundary extraction logs will show:
        //   * Number of edges extracted from face
        //   * Which edges are linear vs curved
        //   * Number of segments in final boundary
        // 
        // - Simple rectangle: 4 edges, 4 segments
        // - Rectangle with 2 cord holes: 8-12 edges, 8-12 segments
        // - Rectangle with 2 cord slots: 12-16 edges, 12-16 segments
        //
        // NEXT STEP AFTER THIS WORKS:
        // - Apply the debug version of DetectEdgeFeatures_Intersections()
        // - Should now see intersection points being found!
        // ===============================================

        // ===============================================
        // CLASSIFY SURFACE (magenta if curved edges → chord passthrough)
        // ===============================================
        private Color ClassifyCutoutSurface(Brep cutout, double tol)
        {
            bool hasCurves = false;
            foreach (var edge in cutout.Edges)
            {
                var curve = edge.EdgeCurve;
                if (curve != null && !curve.IsLinear(tol))
                {
                    hasCurves = true;
                    break;
                }
            }
            return hasCurves ? Color.Magenta : Color.Green;
        }

        // ===============================================
        // CHORD PASS-THROUGHS → DIMENSIONS (unchanged behavior)
        // ===============================================
        private void ProcessChordPassThroughs(
            RhinoDoc doc, int cutoutsLayerIdx, int dimsLayerIdx,
            List<(RhinoObject Panel, List<Brep> ChordCutouts)> panelCutouts, double tol)
        {


            var confirmed = new List<(RhinoObject Panel, Brep Cutout)>();
            foreach (var (panel, cutouts) in panelCutouts)
                foreach (var c in cutouts) confirmed.Add((panel, c));

            if (confirmed.Count == 0)
            {
                return;
            }

            AddChordDimensions(doc, dimsLayerIdx, confirmed, tol);
        }

        private void AddChordDimensions(
    RhinoDoc doc, int dimsLayerIdx,
    List<(RhinoObject Panel, Brep Cutout)> confirmedCutouts, double tol)
        {

            var panelGroups = confirmedCutouts.GroupBy(c => c.Panel);

            foreach (var group in panelGroups)
            {
                var panel = group.Key;
                var panelBBox = panel.Geometry.GetBoundingBox(true);
                var dimStyle = doc.DimStyles.Current;
                var attr = new ObjectAttributes { LayerIndex = dimsLayerIdx };

                var edgeFeatures = new List<(Brep Cutout, string Type, string EdgeSide)>();
                var interior = new List<(Brep Cutout, string Type, string EdgeSide)>();

                // --- Read tags if present; otherwise fallback classify ---
                foreach (var (_, cutout) in group)
                {
                    var side = cutout.GetUserString("EdgeSide");
                    var type = cutout.GetUserString("EdgeType");

                    if (!string.IsNullOrEmpty(side))
                    {
                        edgeFeatures.Add((cutout, string.IsNullOrEmpty(type) ? "Unknown" : type, side));
                    }
                    else
                    {
                        // Fallback classify with true panel boundary
                        Curve panelBoundary = null;
                        if (panel.Geometry is Brep pb) panelBoundary = GetPanelBoundaryCurve(pb, tol);
                        if (panelBoundary == null)
                        {
                            var rect = new Rectangle3d(Plane.WorldXY,
                                new Point3d(panelBBox.Min.X, panelBBox.Min.Y, 0),
                                new Point3d(panelBBox.Max.X, panelBBox.Max.Y, 0));
                            panelBoundary = rect.ToNurbsCurve();
                        }

                        var (fbType, fbSide) = ClassifyChordTypeWithEdge(cutout, panelBBox, panelBoundary, tol);
                        if (fbSide != null) edgeFeatures.Add((cutout, fbType, fbSide));
                        else interior.Add((cutout, fbType, null));
                    }
                }

                var holes = edgeFeatures.Count(e => e.Type == "EdgeCordHole");
                var slots = edgeFeatures.Count(e => e.Type == "EdgeCordSlot");
                var unknown = edgeFeatures.Count(e => e.Type == "Unknown");


                if (edgeFeatures.Count > 0)
                    AddPerFeatureEdgeDimensions(doc, dimsLayerIdx, edgeFeatures, panelBBox, tol, dimStyle);

                foreach (var (cutout, type, _) in interior)
                    AddInteriorDimensions(doc, dimsLayerIdx, cutout, type, panelBBox, tol, dimStyle, attr, Plane.WorldXY);
            }
        }




        // ===============================================
        // Per-feature Edge Dimensions (replaces running chain)
        // - Edge hole: corner → hole center (one dim)
        // - Edge slot: corner → slot start, then start → end (width)
        // ===============================================
        // REPLACE ENTIRE METHOD
        private void AddPerFeatureEdgeDimensions(
            RhinoDoc doc, int dimsLayerIdx,
            List<(Brep Cutout, string Type, string EdgeSide)> edgeFeatures,
            BoundingBox panelBBox, double tol, DimensionStyle dimStyle)
        {
            var attr = new ObjectAttributes { LayerIndex = dimsLayerIdx };

            foreach (var (cutout, type, edgeSide) in edgeFeatures)
            {

                var (edgeCurve, cornerA, cornerB, isHorizontal) = GetEdgeGeom(panelBBox, edgeSide);
                if (edgeCurve == null) { continue; }

                // Utility to project a point back to the actual edge line
                Point3d ProjectToEdge(Point3d p) { double t; edgeCurve.ClosestPoint(p, out t); return edgeCurve.PointAt(t); }

                // Offset direction for dim line
                Vector3d offsetDir = Vector3d.YAxis;
                switch (edgeSide)
                {
                    case "Bottom": offsetDir = -Vector3d.YAxis; break;
                    case "Top": offsetDir = Vector3d.YAxis; break;
                    case "Left": offsetDir = -Vector3d.XAxis; break;
                    case "Right": offsetDir = Vector3d.XAxis; break;
                }
                double offset = 3.0;
                var plane = Plane.WorldXY;
                var dimDir = isHorizontal ? Vector3d.XAxis : Vector3d.YAxis;

                if (type == "EdgeCordHole")
                {
                    var mass = AreaMassProperties.Compute(cutout);
                    if (mass == null) continue;
                    var center = ProjectToEdge(mass.Centroid);
                    var corner = (center.DistanceTo(cornerA) <= center.DistanceTo(cornerB)) ? cornerA : cornerB;
                    var cornerOnEdge = ProjectToEdge(corner);

                    var mid = (cornerOnEdge + center) / 2.0;
                    var dimLinePoint = mid + offsetDir * offset;
                    var dim = LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, dimDir, cornerOnEdge, center, dimLinePoint, 0);
                    if (dim != null) doc.Objects.AddLinearDimension(dim, attr);
                    continue;
                }


                // For EdgeCordSlot and Notch we need the mouth endpoints
                var mouth = GetEdgeMouthEndpoints(cutout, edgeCurve, edgeSide, tol);
                if (mouth == null) { continue; }
                var (mStartRaw, mEndRaw) = mouth.Value;

                // Project to edge and order along the edge
                var mStart = ProjectToEdge(mStartRaw);
                var mEnd = ProjectToEdge(mEndRaw);
                double ts, te; edgeCurve.ClosestPoint(mStart, out ts); edgeCurve.ClosestPoint(mEnd, out te);
                if (ts > te) { var tmp = mStart; mStart = mEnd; mEnd = tmp; }

                // Common dim: corner → slot/notch start + mouth width
                var nearCorner = (mStart.DistanceTo(cornerA) <= mStart.DistanceTo(cornerB)) ? cornerA : cornerB;
                var pCorner = ProjectToEdge(nearCorner);

                // 1) Corner → start
                {
                    var mid = (pCorner + mStart) / 2.0;
                    var dimLinePoint = mid + offsetDir * offset;
                    var d = LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, dimDir, pCorner, mStart, dimLinePoint, 0);
                    if (d != null) doc.Objects.AddLinearDimension(d, attr);
                }
                // 2) Width (start → end)
                {
                    var mid = (mStart + mEnd) / 2.0;
                    var dimLinePoint = mid + offsetDir * offset;
                    var d = LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, dimDir, mStart, mEnd, dimLinePoint, 0);
                    if (d != null) doc.Objects.AddLinearDimension(d, attr);
                }

                // 3) If it’s a NOTCH, also add DEPTH (perpendicular) and “inset from corner” when applicable
                if (type == "Notch")
                {
                    // Depth: farthest point of the notch from the edge, measured ⟂ to edge
                    double depth = GetMaxDepthFromEdge(cutout, edgeCurve);

                    // Build a small helper to place a perp dimension marker for depth
                    // We’ll use the mid of the mouth segment as the “edge” reference
                    var mouthMid = new Point3d((mStart.X + mEnd.X) * 0.5, (mStart.Y + mEnd.Y) * 0.5, 0);
                    // Find the deepest point
                    var deepest = GetDeepestPointFromEdge(cutout, edgeCurve);
                    var depthMid = new Point3d((mouthMid.X + deepest.X) * 0.5, (mouthMid.Y + deepest.Y) * 0.5, 0);
                    // Place depth dim perpendicular to the edge
                    var perpDir = isHorizontal ? Vector3d.YAxis : Vector3d.XAxis;
                    var depthDimPoint = depthMid + (perpDir * (offset * 0.9));
                    var depthDim = LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, perpDir, mouthMid, deepest, depthDimPoint, 0);
                    if (depthDim != null) doc.Objects.AddLinearDimension(depthDim, attr);

                    // Inset-from-corner: only if not flush at a corner (within tol)
                    bool flushToCorner = (pCorner.DistanceTo(mStart) < tol) || (pCorner.DistanceTo(mEnd) < tol);
                    if (!flushToCorner)
                    {
                        var insetMid = new Point3d((pCorner.X + mStart.X) * 0.5, (pCorner.Y + mStart.Y) * 0.5, 0);
                        var insetPoint = insetMid + offsetDir * (offset * 1.2);
                        var insetDim = LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, dimDir, pCorner, mStart, insetPoint, 0);
                        if (insetDim != null) doc.Objects.AddLinearDimension(insetDim, attr);
                    }

                }
            }
        }




        // Return the finite edge line + the two corners + orientation
        private (LineCurve EdgeCurve, Point3d CornerA, Point3d CornerB, bool IsHorizontal)
            GetEdgeGeom(BoundingBox panelBBox, string edgeSide)
        {
            switch (edgeSide)
            {
                case "Bottom":
                    return (new LineCurve(
                                new Point3d(panelBBox.Min.X, panelBBox.Min.Y, 0),
                                new Point3d(panelBBox.Max.X, panelBBox.Min.Y, 0)),
                            new Point3d(panelBBox.Min.X, panelBBox.Min.Y, 0),
                            new Point3d(panelBBox.Max.X, panelBBox.Min.Y, 0),
                            true);
                case "Top":
                    return (new LineCurve(
                                new Point3d(panelBBox.Min.X, panelBBox.Max.Y, 0),
                                new Point3d(panelBBox.Max.X, panelBBox.Max.Y, 0)),
                            new Point3d(panelBBox.Min.X, panelBBox.Max.Y, 0),
                            new Point3d(panelBBox.Max.X, panelBBox.Max.Y, 0),
                            true);
                case "Left":
                    return (new LineCurve(
                                new Point3d(panelBBox.Min.X, panelBBox.Min.Y, 0),
                                new Point3d(panelBBox.Min.X, panelBBox.Max.Y, 0)),
                            new Point3d(panelBBox.Min.X, panelBBox.Min.Y, 0),
                            new Point3d(panelBBox.Min.X, panelBBox.Max.Y, 0),
                            false);
                case "Right":
                    return (new LineCurve(
                                new Point3d(panelBBox.Max.X, panelBBox.Min.Y, 0),
                                new Point3d(panelBBox.Max.X, panelBBox.Max.Y, 0)),
                            new Point3d(panelBBox.Max.X, panelBBox.Min.Y, 0),
                            new Point3d(panelBBox.Max.X, panelBBox.Max.Y, 0),
                            false);
                default:
                    return (null, Point3d.Unset, Point3d.Unset, true);
            }
        }

        // Find the two points where a cutout meets the given edge (start/end along the edge).
        // If exact edge gives <2 hits (tangency), we also probe a tiny inset parallel.
        private (Point3d Start, Point3d End)? GetEdgeMouthEndpoints(Brep cutout, LineCurve edgeCurve, string edgeSide, double tol)
        {
            var outer = cutout.Loops.FirstOrDefault(l => l.LoopType == BrepLoopType.Outer);
            if (outer == null) return null;
            var boundary = outer.To3dCurve();
            if (boundary == null) return null;

            // 1) Try real edge
            var hits = GetIntersectionsOnSegment(boundary, edgeCurve, tol);

            // 2) If <2, probe inset (same epsilon as detector)
            if (hits.Count < 2)
            {
                Vector3d inward =
                    (edgeSide == "Bottom") ? Vector3d.YAxis :
                    (edgeSide == "Top") ? -Vector3d.YAxis :
                    (edgeSide == "Left") ? Vector3d.XAxis :
                                              -Vector3d.XAxis;

                inward.Unitize();
                inward *= 0.02; // inset eps

                var a = edgeCurve.PointAt(edgeCurve.Domain.Min);
                var b = edgeCurve.PointAt(edgeCurve.Domain.Max);
                var insetA = new Point3d(a.X + inward.X, a.Y + inward.Y, 0);
                var insetB = new Point3d(b.X + inward.X, b.Y + inward.Y, 0);
                var inset = new LineCurve(insetA, insetB);

                var insetHits = GetIntersectionsOnSegment(boundary, inset, tol);
                if (insetHits.Count >= 2)
                {
                    // Remap to original edge
                    hits = insetHits.Select(p =>
                    {
                        double t; edgeCurve.ClosestPoint(p, out t);
                        return edgeCurve.PointAt(t);
                    }).ToList();
                }
            }

            if (hits.Count < 2) return null;

            // Order along the edge and return the pair
            var withT = hits.Select(p =>
            {
                double t; edgeCurve.ClosestPoint(p, out t);
                return (T: t, P: p);
            }).OrderBy(x => x.T).ToList();

            var p0 = withT[0].P;
            var p1 = withT[1].P;
            return (p0, p1);
        }



        // (Same classifier you already use: ≤ 2.0" → EdgeCordHole, > 2.0" → EdgeCordSlot)
        private (string Type, string EdgeSide) ClassifyChordTypeWithEdge(Brep cutout, BoundingBox panelBBox,
            Curve panelBoundary, double tol)
        {
            var boundary = cutout.Loops.FirstOrDefault(l => l.LoopType == BrepLoopType.Outer)?.To3dCurve();
            if (boundary == null) return ("Unknown", null);

            var interEvents = Intersection.CurveCurve(boundary, panelBoundary, tol, tol);
            bool isEdge = interEvents.Count > 0;

            var cutBBox = cutout.GetBoundingBox(true);
            double width = cutBBox.Max.X - cutBBox.Min.X;
            double height = cutBBox.Max.Y - cutBBox.Min.Y;
            double minDim = Math.Min(width, height);
            double maxDim = Math.Max(width, height);
            double aspect = maxDim / minDim;

            string type;
            string edgeSide = null;

            if (isEdge)
            {
                var interPoints = interEvents.Select(e => e.PointA).Distinct(new Point3dComparer(tol)).ToList();
                if (interPoints.Count >= 2)
                {
                    double edgeWidth = interPoints[0].DistanceTo(interPoints[1]);
                    type = edgeWidth <= 2.0 ? "EdgeCordHole" : "EdgeCordSlot";

                    double avgX = interPoints.Average(p => p.X);
                    double avgY = interPoints.Average(p => p.Y);

                    if (Math.Abs(avgY - panelBBox.Min.Y) < tol) edgeSide = "Bottom";
                    else if (Math.Abs(avgY - panelBBox.Max.Y) < tol) edgeSide = "Top";
                    else if (Math.Abs(avgX - panelBBox.Min.X) < tol) edgeSide = "Left";
                    else if (Math.Abs(avgX - panelBBox.Max.X) < tol) edgeSide = "Right";
                }
                else type = "Unknown";
            }
            else
            {
                if (aspect < 1.1 && (Math.Abs(minDim - 1.5) < tol || Math.Abs(minDim - 2.0) < tol))
                    type = "InteriorCordHole";
                else if (aspect > 2.0 && minDim <= 9.0)
                    type = "InteriorCordSlot";
                else
                    type = "Unknown";
            }

            return (type, edgeSide);
        }

        private void AddInteriorDimensions(RhinoDoc doc, int dimsLayerIdx, Brep cutout, string type,
            BoundingBox panelBBox, double tol, DimensionStyle dimStyle, ObjectAttributes attr, Plane dimPlane)
        {
            var massProp = AreaMassProperties.Compute(cutout);
            if (massProp == null) return;
            Point3d center = massProp.Centroid;

            double xDistLeft = center.X - panelBBox.Min.X;
            double xDistRight = panelBBox.Max.X - center.X;
            double yDistBottom = center.Y - panelBBox.Min.Y;
            double yDistTop = center.Y - panelBBox.Max.Y;

            bool fromLeft = xDistLeft < xDistRight;
            Point3d xStart = fromLeft ? new Point3d(panelBBox.Min.X, center.Y, 0) : new Point3d(panelBBox.Max.X, center.Y, 0);
            Point3d xTextPos = new Point3d((xStart.X + center.X) / 2.0, (xStart.Y + center.Y) / 2.0, 0);

            var xDim = LinearDimension.Create(AnnotationType.Aligned, dimStyle, dimPlane, Vector3d.XAxis, xStart, center, xTextPos, 0);
            xDim.PlainText = FormatDimension(fromLeft ? xDistLeft : xDistRight, 4);
            doc.Objects.AddLinearDimension(xDim, attr);

            bool fromBottom = yDistBottom < yDistTop;
            Point3d yStart = fromBottom ? new Point3d(center.X, panelBBox.Min.Y, 0) : new Point3d(center.X, panelBBox.Max.Y, 0);
            Point3d yTextPos = new Point3d((yStart.X + center.X) / 2.0, (yStart.Y + center.Y) / 2.0, 0);

            var yDim = LinearDimension.Create(AnnotationType.Aligned, dimStyle, dimPlane, Vector3d.YAxis, yStart, center, yTextPos, 0);
            yDim.PlainText = FormatDimension(fromBottom ? yDistBottom : yDistTop, 4);
            doc.Objects.AddLinearDimension(yDim, attr);

            // Diameter for circular holes (interior)
            if (type == "InteriorCordHole")
            {
                Circle circle;
                var boundary = cutout.Loops.FirstOrDefault()?.To3dCurve();
                if (boundary != null && boundary.TryGetCircle(out circle, tol))
                {
                    Point3d left = new Point3d(circle.Center.X - circle.Radius, circle.Center.Y, 0);
                    Point3d right = new Point3d(circle.Center.X + circle.Radius, circle.Center.Y, 0);
                    Point3d diaTextPos = new Point3d((left.X + right.X) / 2.0, (left.Y + right.Y) / 2.0 + 1.0, 0);

                    var diaDim = LinearDimension.Create(AnnotationType.Aligned, dimStyle, dimPlane, Vector3d.XAxis, left, right, diaTextPos, 0);
                    diaDim.PlainText = "Ø " + FormatDimension(circle.Diameter, 4);
                    doc.Objects.AddLinearDimension(diaDim, attr);
                }
            }
        }

        // Running cumulative chain (unchanged)
        private void AddRunningCumulativeDimensions(RhinoDoc doc, int dimsLayerIdx,
            List<(Brep Cutout, string Type, string EdgeSide)> edgeFeatures,
            BoundingBox panelBBox, string edgeSide, double tol)
        {
            var sorted = edgeFeatures.OrderBy(f =>
            {
                var bbox = f.Cutout.GetBoundingBox(true);
                return (edgeSide == "Bottom" || edgeSide == "Top") ? bbox.Min.X : bbox.Min.Y;
            }).ToList();

            var chainPoints = new List<Point3d>();
            double offset = 3.0;
            bool isHorizontal = (edgeSide == "Bottom" || edgeSide == "Top");
            Point3d edgeStart, edgeEnd;
            Vector3d offsetDir;

            switch (edgeSide)
            {
                case "Bottom":
                    edgeStart = new Point3d(panelBBox.Min.X, panelBBox.Min.Y, 0);
                    edgeEnd = new Point3d(panelBBox.Max.X, panelBBox.Min.Y, 0);
                    offsetDir = -Vector3d.YAxis; // draw below
                    break;
                case "Top":
                    edgeStart = new Point3d(panelBBox.Min.X, panelBBox.Max.Y, 0);
                    edgeEnd = new Point3d(panelBBox.Max.X, panelBBox.Max.Y, 0);
                    offsetDir = Vector3d.YAxis; // draw above
                    break;
                case "Left":
                    edgeStart = new Point3d(panelBBox.Min.X, panelBBox.Min.Y, 0);
                    edgeEnd = new Point3d(panelBBox.Min.X, panelBBox.Max.Y, 0);
                    offsetDir = -Vector3d.XAxis; // draw left
                    break;
                case "Right":
                    edgeStart = new Point3d(panelBBox.Max.X, panelBBox.Min.Y, 0);
                    edgeEnd = new Point3d(panelBBox.Max.X, panelBBox.Max.Y, 0);
                    offsetDir = Vector3d.XAxis; // draw right
                    break;
                default:
                    return;
            }

            chainPoints.Add(edgeStart);

            foreach (var (cutout, _, _) in sorted)
            {
                var b = cutout.GetBoundingBox(true);
                if (isHorizontal)
                {
                    chainPoints.Add(new Point3d(b.Min.X, edgeStart.Y, 0));
                    chainPoints.Add(new Point3d(b.Max.X, edgeStart.Y, 0));
                }
                else
                {
                    chainPoints.Add(new Point3d(edgeStart.X, b.Min.Y, 0));
                    chainPoints.Add(new Point3d(edgeStart.X, b.Max.Y, 0));
                }
            }

            chainPoints.Add(edgeEnd);

            var redColor = System.Drawing.Color.FromArgb(255, 0, 0);
            var attr = new ObjectAttributes
            {
                LayerIndex = dimsLayerIdx,
                ColorSource = ObjectColorSource.ColorFromObject,
                ObjectColor = redColor
            };

            DimensionStyle dimStyle = doc.DimStyles.Current;
            for (int i = 0; i < chainPoints.Count - 1; i++)
            {
                Point3d p1 = chainPoints[i];
                Point3d p2 = chainPoints[i + 1];
                double segLen = p1.DistanceTo(p2);
                if (segLen < tol) continue;

                Point3d mid = (p1 + p2) / 2.0;
                Point3d dimLinePoint = mid + (offsetDir * offset);

                var dimPlane = Plane.WorldXY;
                Vector3d dimDir = isHorizontal ? Vector3d.XAxis : Vector3d.YAxis;

                var dim = LinearDimension.Create(
                    AnnotationType.Aligned,
                    dimStyle,
                    dimPlane,
                    dimDir,
                    p1,
                    p2,
                    dimLinePoint,
                    0);

                if (dim != null)
                {
                    doc.Objects.AddLinearDimension(dim, attr);
                }
            }
        }

        // ===============================================
        // NEW RAY PROTOCOL IMPLEMENTATION
        // Segment-Boundary Ray Validation for Edge Features
        // ===============================================

        // INTEGRATION: Replace DetectEdgeFeaturesFromSegments with this implementation

        private List<Brep> DetectEdgeFeaturesFromSegments_RayProtocol(
        RhinoDoc doc,                    // ← ADD THIS
        Rectangle3d bbox,
        Curve panelBoundary,
        EdgeSegmentData segmentData,
        Brep panelBrep,
        double tol,
        int dimensionsLayerIndex)        // ← ADD THIS

        {
            var detectedFeatures = new List<EdgeFeature>();

            // Collect all edge features
            var allEdgeFeatures = new List<EdgeFeature>();

            var bottomFeatures = ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.BottomSegments, "BOTTOM", panelBrep, tol);
            var rightFeatures = ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.RightSegments, "RIGHT", panelBrep, tol);
            var topFeatures = ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.TopSegments, "TOP", panelBrep, tol);
            var leftFeatures = ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.LeftSegments, "LEFT", panelBrep, tol);

            allEdgeFeatures.AddRange(bottomFeatures);
            allEdgeFeatures.AddRange(rightFeatures);
            allEdgeFeatures.AddRange(topFeatures);
            allEdgeFeatures.AddRange(leftFeatures);


            // Add dimensions for detected edge features
            if (allEdgeFeatures.Count > 0)
            {
                AddEdgeFeatureDimensions(doc, bbox, allEdgeFeatures, dimensionsLayerIndex);
            }

            // Return empty list for now (we're not using Breps anymore)
            return new List<Brep>();
        }

        private List<EdgeFeature> ProcessEdge_RayProtocol(
            Rectangle3d bbox,
            Curve panelBoundary,
            List<(Curve seg, int index)> segments,
            string edgeName,
            Brep panelBrep,
            double tol)

        {
            var detectedFeatures = new List<EdgeFeature>();
            if (segments.Count <= 1)
            {
                return new List<EdgeFeature>();  // ✅ Return empty list
            }


            // Sort segments by position along edge
            bool isHorizontal = edgeName == "BOTTOM" || edgeName == "TOP";
            segments.Sort((a, b) =>
            {
                Point3d midA = a.seg.PointAt(a.seg.Domain.Mid);
                Point3d midB = b.seg.PointAt(b.seg.Domain.Mid);
                return isHorizontal ? midA.X.CompareTo(midB.X) : midA.Y.CompareTo(midB.Y);
            });

            // Get edge line parameters
            Point3d edgeStart, edgeEnd;
            Vector3d edgeDirection, perpendicular;
            GetEdgeParameters(bbox, edgeName, out edgeStart, out edgeEnd, out edgeDirection, out perpendicular);

            double edgeLength = edgeStart.DistanceTo(edgeEnd);
            const double baselineDepth = 0.001;  // Expected depth for straight edge

            // DEBUG: Log edge parameters
            RhinoApp.WriteLine($"  [DEBUG] [{edgeName}] Edge length: {edgeLength:F2}\", Segments: {segments.Count}");
            if (edgeLength > 1000)
            {
                RhinoApp.WriteLine($"  [DEBUG] ⚠️ WARNING: Edge length exceeds 1000\" - possible geometry issue!");
            }

            // Track position along edge
            double currentPos = 0.0;
            int segmentIndex = 0;

            while (segmentIndex < segments.Count - 1)  // Skip last segment (always straight edge to corner)
            {
                var (seg, idx) = segments[segmentIndex];
                double segLength = GetSegmentLengthAlongEdge(seg, edgeStart, edgeDirection, isHorizontal);

                // DEBUG: Log segment processing
                RhinoApp.WriteLine($"  [DEBUG] Seg {segmentIndex}: Length={segLength:F2}\", Pos={currentPos:F2}\" → {currentPos + segLength:F2}\"");

                // Check boundary at end of this segment
                double boundaryPos = currentPos + segLength;


                // Shoot ±0.01" rays
                Point3d rayPoint1 = edgeStart + edgeDirection * (boundaryPos - 0.01);
                // Calculate next segment length for adaptive ray depth
                double nextSegLength = 0.25;  // Default fallback
                if (segmentIndex + 1 < segments.Count)
                {
                    var (nextSeg, _) = segments[segmentIndex + 1];
                    nextSegLength = GetSegmentLengthAlongEdge(nextSeg, edgeStart, edgeDirection, isHorizontal);
                }

                Point3d rayPoint2 = edgeStart + edgeDirection * (boundaryPos + Math.Max(0.5, nextSegLength / 2));

                double depth1 = ShootRayDepth(rayPoint1, perpendicular, panelBoundary);
                double depth2 = ShootRayDepth(rayPoint2, perpendicular, panelBoundary);

                // DEBUG: Show actual depth measurements
                RhinoApp.WriteLine($"    [RAY] At boundary {boundaryPos:F3}\": depth1={depth1:F3}\", depth2={depth2:F3}\", baseline={baselineDepth:F3}\"");
                                  

                    // Check if cutout detected (depth change)
                    if (Math.Abs(depth1 - baselineDepth) < 0.1 && depth2 > baselineDepth + 0.1)
                    {


                    // Scan at 0.05" intervals to find cutout end
                    double scanPos = boundaryPos;
                    double maxDepth = depth2;
                    const double scanInterval = 0.03;  // 1/20"


                    double prevScanPos = scanPos;  // Track previous position
                    while (scanPos < edgeLength)
                    {
                        scanPos += scanInterval;
                        Point3d scanRayPoint = edgeStart + edgeDirection * scanPos;
                        double scanDepth = ShootRayDepth(scanRayPoint, perpendicular, panelBoundary);

                        maxDepth = Math.Max(maxDepth, scanDepth);

                        // Check if we're back to straight edge
                        if (Math.Abs(scanDepth - baselineDepth) < 0.1)
                        {
                            // Use the PREVIOUS position (last one inside cutout)
                            scanPos = prevScanPos;
                            break;
                        }

                        prevScanPos = scanPos;  // Update before next iteration
                    }

                    // Round scanPos to nearest 1/16" for accuracy
                    scanPos = Math.Round(scanPos * 16.0) / 16.0;


                    // Determine which segments form this cutout FIRST
                    var cutoutSegments = new List<int>();
                    double cutoutSegmentLength = 0.0;  // Track total length of cutout segments

                    for (int i = segmentIndex + 1; i < segments.Count; i++)
                    {
                        var (cutSeg, cutIdx) = segments[i];
                        double cutSegLength = GetSegmentLengthAlongEdge(cutSeg, edgeStart, edgeDirection, isHorizontal);
                        double cutSegEnd = currentPos + segLength + cutoutSegmentLength + cutSegLength;

                        // Check if this segment overlaps with cutout range
                        if (cutSegEnd <= scanPos + 0.01)  // Small tolerance
                        {
                            cutoutSegments.Add(i + 1);  // 1-based for display
                            cutoutSegmentLength += cutSegLength;
                            segLength += cutSegLength;  // Also update main segLength for position tracking
                        }
                        else
                        {
                            break;
                        }
                    }

                    // ========== DEBUG: Log cutout detection details ==========
                    RhinoApp.WriteLine($"    [CUTOUT DEBUG] Detected at currentPos={currentPos:F3}\", boundaryPos={boundaryPos:F3}\"");
                    RhinoApp.WriteLine($"    [CUTOUT DEBUG] scanPos found cutout end at: {scanPos:F3}\"");
                    RhinoApp.WriteLine($"    [CUTOUT DEBUG] Segment lengths in cutout:");
                    for (int i = segmentIndex + 1; i <= segmentIndex + cutoutSegments.Count && i < segments.Count; i++)
                    {
                        var (debugSeg, debugIdx) = segments[i];
                        double len = GetSegmentLengthAlongEdge(debugSeg, edgeStart, edgeDirection, isHorizontal);
                        RhinoApp.WriteLine($"      Seg {i}: Length={len:F3}\"");
                    }
                    RhinoApp.WriteLine($"    [CUTOUT DEBUG] Total cutoutSegmentLength={cutoutSegmentLength:F3}\"");
                    RhinoApp.WriteLine($"    [CUTOUT DEBUG] cutoutSegments.Count={cutoutSegments.Count}");
                    // ========== END DEBUG ==========

                    // Calculate cutout dimensions using SEGMENT LENGTHS (more accurate than ray positions)
                    double cutoutStart = boundaryPos;  // Cutout starts at end of current segment

                    // Use segment length if available AND it matches ray scan
                    // (If ray scan finds significantly larger cutout, trust the ray scan)
                    double rayWidth = scanPos - boundaryPos;
                    double cutoutWidth;

                    if (cutoutSegmentLength > 0.01)
                    {
                        // If ray scan is 2x larger than segment length, trust ray scan
                        if (rayWidth > cutoutSegmentLength * 2.0)
                        {
                            cutoutWidth = rayWidth;  // Use ray scan (large discrepancy)
                        }
                        else
                        {
                            cutoutWidth = cutoutSegmentLength;  // Use segment length
                        }
                    }
                    else
                    {
                        cutoutWidth = rayWidth;  // No segment length, use ray scan
                    }

                    double cutoutEnd = cutoutStart + cutoutWidth;  // End position

                    // Round to nearest 1/16"
                    cutoutStart = Math.Round(cutoutStart * 16.0) / 16.0;
                    cutoutEnd = Math.Round(cutoutEnd * 16.0) / 16.0;
                    cutoutWidth = cutoutEnd - cutoutStart;  // Recalculate after rounding

                    // DEBUG: Log final values
                    RhinoApp.WriteLine($"    [CUTOUT DEBUG] FINAL: start={cutoutStart:F3}\", end={cutoutEnd:F3}\", width={cutoutWidth:F3}\"");

                    // Round depth to nearest 1/16"
                    double depthRounded = Math.Round(maxDepth * 16.0) / 16.0;


                    // Classify feature type
                    string featureType = cutoutWidth <= 2.0 ? "Edge Hole" : "Edge Slot";


                    // Store the detected feature
                    var feature = new EdgeFeature(
                        edgeName,
                        cutoutStart,
                        cutoutEnd,
                        cutoutWidth,
                        depthRounded,
                        featureType
                    );
                    detectedFeatures.Add(feature);

                    // Advance to next segment after cutout
                    int segmentsToSkip = cutoutSegments.Count;
                    if (segmentsToSkip == 0)
                    {
                        RhinoApp.WriteLine($"    ⚠️ Zero segments detected - forcing +1 advance to prevent infinite loop");
                        segmentIndex++;  // Force at least 1 segment advance
                    }
                    else
                    {
                        segmentIndex += segmentsToSkip;
                    }
                    currentPos = cutoutEnd;
                    continue;  // ← CRITICAL: Skip rest of loop and start next iteration
                }
                // NEW: Detect inset holes (both rays inside cutout)
                else if (depth1 > baselineDepth + 0.1 && depth2 > baselineDepth + 0.1 && segmentIndex > 0)
                {


                    // Scan BACKWARD to find actual cutout start
                    double cutoutStart = boundaryPos;
                    double backScanPos = boundaryPos - 0.01;
                    const double backScanInterval = 1.0 / 16.0;

                    while (backScanPos > currentPos - segLength)
                    {
                        Point3d backRayPoint = edgeStart + edgeDirection * backScanPos;
                        double backDepth = ShootRayDepth(backRayPoint, perpendicular, panelBoundary);

                        if (Math.Abs(backDepth - baselineDepth) < 0.1)
                        {
                            cutoutStart = backScanPos;
                            break;
                        }

                        backScanPos -= backScanInterval;
                    }

                    // Scan FORWARD to find cutout end
                    double cutoutEnd = boundaryPos;
                    double forwardScanPos = boundaryPos + 0.01;
                    double maxDepth = Math.Max(depth1, depth2);
                    const double forwardScanInterval = 1.0 / 16.0;

                    while (forwardScanPos < edgeLength)
                    {
                        Point3d forwardRayPoint = edgeStart + edgeDirection * forwardScanPos;
                        double forwardDepth = ShootRayDepth(forwardRayPoint, perpendicular, panelBoundary);

                        maxDepth = Math.Max(maxDepth, forwardDepth);

                        if (Math.Abs(forwardDepth - baselineDepth) < 0.1)
                        {
                            cutoutEnd = forwardScanPos;
                            break;
                        }

                        forwardScanPos += forwardScanInterval;
                    }

                    // Round to nearest 1/16" before calculating width
                    cutoutStart = Math.Round(cutoutStart * 16.0) / 16.0;
                    cutoutEnd = Math.Round(cutoutEnd * 16.0) / 16.0;

                    // Calculate cutout dimensions
                    double cutoutWidth = cutoutEnd - cutoutStart;

                    // SAFETY CHECK: Skip invalid cutouts
                    if (cutoutWidth < 0.01)
                    {
                        RhinoApp.WriteLine($"    ⚠️ Invalid inset hole: width={cutoutWidth:F3}\" - forcing segment advance");
                        currentPos = boundaryPos;
                        segmentIndex++;
                        continue;  // Skip to next iteration
                    }

                    // Calculate cutout dimensions
                    double depthRounded = Math.Round(maxDepth * 16.0) / 16.0;


                    // Determine which segments form this cutout
                    var cutoutSegments = new List<int>();

                    for (int i = Math.Max(0, segmentIndex - 1); i < segments.Count; i++)
                    {
                        var (cutSeg, cutIdx) = segments[i];
                        double segPos = 0;
                        for (int j = 0; j < i; j++)
                        {
                            segPos += GetSegmentLengthAlongEdge(segments[j].Item1, edgeStart, edgeDirection, isHorizontal);
                        }
                        double segEndPos = segPos + GetSegmentLengthAlongEdge(cutSeg, edgeStart, edgeDirection, isHorizontal);

                        if (segEndPos >= cutoutStart - 0.01 && segPos <= cutoutEnd + 0.01)
                        {
                            cutoutSegments.Add(i + 1);
                        }
                    }


                    // Classify feature type
                    string featureType = cutoutWidth <= 2.0 ? "Edge Hole" : "Edge Slot";


                    // Store the detected feature
                    var feature = new EdgeFeature(
                        edgeName,
                        cutoutStart,
                        cutoutEnd,
                        cutoutWidth,
                        depthRounded,
                        featureType
                    );
                    detectedFeatures.Add(feature);

                    // Skip ahead past the cutout segments
                    int segmentsToSkip = cutoutSegments.Count;
                    if (segmentsToSkip == 0)
                    {
                        RhinoApp.WriteLine($"    ⚠️ Zero segments detected - forcing +1 advance to prevent infinite loop");
                        segmentIndex++;  // Force at least 1 segment advance
                    }
                    else
                    {
                        segmentIndex += segmentsToSkip;
                    }
                    currentPos = cutoutEnd;
                    continue;
                }
                else
                {
                    // No cutout, advance to next segment
                    currentPos = boundaryPos;
                    segmentIndex++;
                }
            }
            return detectedFeatures;
        }

        private void GetEdgeParameters(
            Rectangle3d bbox,
            string edgeName,
            out Point3d edgeStart,
            out Point3d edgeEnd,
            out Vector3d edgeDirection,
            out Vector3d perpendicular)
        {
            Point3d bottomLeft = bbox.Corner(0);
            Point3d bottomRight = bbox.Corner(1);
            Point3d topRight = bbox.Corner(2);
            Point3d topLeft = bbox.Corner(3);

            switch (edgeName)
            {
                case "BOTTOM":
                    edgeStart = bottomLeft;
                    edgeEnd = bottomRight;
                    edgeDirection = Vector3d.XAxis;
                    perpendicular = Vector3d.YAxis;  // Shoot UP into panel (for detection)
                    break;
                case "RIGHT":
                    edgeStart = bottomRight;
                    edgeEnd = topRight;
                    edgeDirection = Vector3d.YAxis;
                    perpendicular = -Vector3d.XAxis;  // Shoot LEFT into panel (for detection)
                    break;
                case "TOP":
                    edgeStart = topRight;
                    edgeEnd = topLeft;
                    edgeDirection = -Vector3d.XAxis;
                    perpendicular = -Vector3d.YAxis;  // Shoot DOWN into panel (for detection)
                    break;
                case "LEFT":
                    edgeStart = topLeft;
                    edgeEnd = bottomLeft;
                    edgeDirection = -Vector3d.YAxis;
                    perpendicular = Vector3d.XAxis;  // Shoot RIGHT into panel (for detection)
                    break;
                default:
                    throw new ArgumentException($"Unknown edge name: {edgeName}");
            }

            edgeDirection.Unitize();
            perpendicular.Unitize();
        }

        private double GetSegmentLengthAlongEdge(Curve segment, Point3d edgeStart, Vector3d edgeDirection, bool isHorizontal)
        {
            // For straight edge segments, length along edge = segment length
            // For perpendicular segments (like 0.5" drops), length along edge = 0
            // For curved segments, project endpoints to get horizontal/vertical span

            Point3d segStart = segment.PointAtStart;
            Point3d segEnd = segment.PointAtEnd;

            if (isHorizontal)
            {
                // Return horizontal span
                return Math.Abs(segEnd.X - segStart.X);
            }
            else
            {
                // Return vertical span
                return Math.Abs(segEnd.Y - segStart.Y);
            }
        }

        private double ShootRayDepth(Point3d origin, Vector3d direction, Curve panelBoundary)
        {
            // FIRST-HIT SOLUTION: Start ray 0.001" outside edge, take closest intersection
            // This avoids ghost geometry and far-edge hits
            Point3d rayStart = origin + direction * -0.001;  // Start 0.001" outside panel (almost touching)

            // Create a long line representing the ray
            // (20" is overkill but ensures we hit everything - we only use first hit anyway)
            Point3d rayEnd = rayStart + direction * 20.0;
            Line rayLine = new Line(rayStart, rayEnd);
            LineCurve rayCurve = new LineCurve(rayLine);

            // Intersect ray with panel boundary curve
            var intersections = Rhino.Geometry.Intersect.Intersection.CurveCurve(
                panelBoundary,
                rayCurve,
                0.001,  // Tolerance
                0.001   // Overlap tolerance
            );

            if (intersections != null && intersections.Count > 0)
            {
                // NEW BASELINE: Ray starts 0.001" outside edge
                const double baselineDepth = 0.001;

                // Collect all intersection depths
                List<double> depths = new List<double>();
                for (int i = 0; i < intersections.Count; i++)
                {
                    Point3d hitPoint = intersections[i].PointA;
                    double depth = rayStart.DistanceTo(hitPoint);
                    depths.Add(depth);
                }

                // Sort depths (nearest to furthest)
                depths.Sort();

                // FIRST-HIT LOGIC: Return the FIRST (closest) intersection
                // No filtering needed - first hit is always the real surface
                if (depths.Count > 0)
                {
                    return depths[0];  // Closest surface = real geometry
                }

                // Fallback (should never happen if intersections.Count > 0)
                return baselineDepth;
            }

            // No intersection - ray missed panel (should never happen)
            return double.MaxValue;
        }


        // ===============================================
        // HELPERS (layers, linetype, bbox, etc.)
        // ===============================================
        private Layer FindOrCreateBlueprintLayer(RhinoDoc doc, Layer parentLayer)
        {
            string blueprintPath = $"{parentLayer.FullPath}::Blueprint";
            int blueprintIdx = GetLayerIndexByFullPath(doc, blueprintPath);

            if (blueprintIdx >= 0)
            {
                return doc.Layers[blueprintIdx];
            }
            else
            {
                var newBlueprint = new Layer
                {
                    Name = "Blueprint",
                    ParentLayerId = parentLayer.Id,
                    Color = Color.White
                };
                blueprintIdx = doc.Layers.Add(newBlueprint);
                return doc.Layers[blueprintIdx];
            }
        }

        private int GetLayerIndexByFullPath(RhinoDoc doc, string fullPath)
        {
            for (int i = 0; i < doc.Layers.Count; i++)
                if (string.Equals(doc.Layers[i].FullPath, fullPath, StringComparison.OrdinalIgnoreCase))
                    return i;
            return -1;
        }

        private int FindOrCreateChildLayer(RhinoDoc doc, Layer parentLayer, string childName, Color layerColor)
        {
            string fullPath = $"{parentLayer.FullPath}::{childName}";
            int idx = GetLayerIndexByFullPath(doc, fullPath);

            if (idx >= 0)
            {
                var existingLayer = doc.Layers[idx];
                existingLayer.Color = layerColor;
                doc.Layers.Modify(existingLayer, idx, true);
                return idx;
            }

            var newLayer = new Layer { Name = childName, ParentLayerId = parentLayer.Id, Color = layerColor };
            return doc.Layers.Add(newLayer);
        }

        private int EnsureDashedLinetype(RhinoDoc doc)
        {
            for (int i = 0; i < doc.Linetypes.Count; i++)
            {
                var lt = doc.Linetypes[i];
                if (lt.Name.IndexOf("Dash", StringComparison.OrdinalIgnoreCase) >= 0)
                    return i;
            }

            var newLinetype = new Linetype { Name = "Blueprint Dash" };
            newLinetype.AppendSegment(0.25, true);
            newLinetype.AppendSegment(0.125, false);
            int idx = doc.Linetypes.Add(newLinetype);
            return idx;
        }

        private Rectangle3d GetTopViewBoundingBox(Brep brep)
        {
            var bbox = brep.GetBoundingBox(true);
            if (!bbox.IsValid) return Rectangle3d.Unset;

            var corner = new Point3d(bbox.Min.X, bbox.Min.Y, 0);
            var plane = new Plane(corner, Vector3d.XAxis, Vector3d.YAxis);
            double width = bbox.Max.X - bbox.Min.X;
            double height = bbox.Max.Y - bbox.Min.Y;

            return new Rectangle3d(plane, width, height);
        }

        private void DropBlueprintDimsToZ0(RhinoDoc doc, Layer blueprintLayer)
        {
            var dimsLayer = GetLayerIndexByFullPath(doc, $"{blueprintLayer.FullPath}::Dimensions");
            if (dimsLayer < 0) return;

            var objs = doc.Objects.FindByLayer(doc.Layers[dimsLayer]);
            if (objs == null) return;

            foreach (var obj in objs)
            {
                if (obj.Geometry is AnnotationBase anno)
                {
                    var plane = anno.Plane;
                    plane.Origin = new Point3d(plane.Origin.X, plane.Origin.Y, 0);
                    anno.Plane = plane;
                    obj.CommitChanges();
                }
            }
            doc.Views.Redraw();
        }

        // comparer used in classification
        private class Point3dComparer : IEqualityComparer<Point3d>
        {
            private readonly double _tol;
            public Point3dComparer(double tol) { _tol = tol; }
            public bool Equals(Point3d a, Point3d b) => a.DistanceTo(b) < _tol;
            public int GetHashCode(Point3d p) => p.GetHashCode();
        }

        // Curvature check on a closed curve: true if any segment has radius >= minRadius
        private bool HasCurvatureAboveRadius(Curve closed, double minRadius, double tol)
        {
            var segs = closed.DuplicateSegments();
            if (segs == null || segs.Length == 0) return false;
            foreach (var s in segs)
            {
                if (s.IsLinear(tol)) continue;

                // Arc?
                if (s.TryGetArc(out Arc arc))
                {
                    if (arc.Radius + tol >= minRadius) return true;
                    continue;
                }

                // NURBS: estimate curvature radius at the mid parameter
                double t = s.Domain.Mid;
                if (s.TryGetPlane(out Plane _))
                {
                    var k = s.CurvatureAt(t);
                    double kLen = k.Length;
                    if (kLen > 1e-8)
                    {
                        double rad = 1.0 / kLen;
                        if (rad + tol >= minRadius) return true;
                    }
                }

                // If we can’t evaluate reliably, assume curved (safer)
                return true;
            }
            return false; // all segments linear (i.e., a notch)
        }

        // Circle-ness + diameter
        private bool IsNearlyCircular(Brep brep, double circularityMax, double diaMax, out double dia)
        {
            dia = 0.0;
            var a = AreaMassProperties.Compute(brep);

            // RhinoCommon expects the int overload here
            var wires = brep.GetWireframe(0);
            double p = 0.0;
            if (wires != null)
            {
                foreach (var crv in wires) p += crv.GetLength();
            }
            if (a == null || p <= 0.0) return false;

            double C = (p * p) / (4.0 * Math.PI * a.Area);

            double r = Math.Sqrt(a.Area / Math.PI);
            dia = 2.0 * r;

            return (C <= circularityMax + 1e-6) && (dia <= diaMax + 1e-6);
        }

        // Max perpendicular depth of a cutout from a given edge
        private double GetMaxDepthFromEdge(Brep cutout, Curve edgeCurve, int samples = 48)
        {
            double maxD = 0.0;
            var loops = cutout.Loops;
            if (loops == null || loops.Count == 0) return 0.0;
            var c = loops[0].To3dCurve();

            double len = c.GetLength();
            for (int i = 0; i <= samples; i++)
            {
                double t; c.LengthParameter(len * i / samples, out t);
                var pt = c.PointAt(t);
                double te; edgeCurve.ClosestPoint(pt, out te);
                var onEdge = edgeCurve.PointAt(te);
                maxD = Math.Max(maxD, pt.DistanceTo(onEdge));
            }
            return maxD;
        }

        // Deepest point (for drawing the depth dimension)
        private Point3d GetDeepestPointFromEdge(Brep cutout, Curve edgeCurve, int samples = 64)
        {
            double maxD = -1.0;
            Point3d deepest = Point3d.Unset;
            var loops = cutout.Loops;
            if (loops == null || loops.Count == 0) return deepest;
            var c = loops[0].To3dCurve();
            double len = c.GetLength();

            for (int i = 0; i <= samples; i++)
            {
                double t; c.LengthParameter(len * i / samples, out t);
                var pt = c.PointAt(t);
                double te; edgeCurve.ClosestPoint(pt, out te);
                var onEdge = edgeCurve.PointAt(te);
                var d = pt.DistanceTo(onEdge);
                if (d > maxD)
                {
                    maxD = d;
                    deepest = pt;
                }
            }
            return deepest;
        }

        // ===============================================
        // NEW METHOD: Label Boundary Segments by Edge
        // ===============================================

        // NEW: Version that returns EdgeSegmentData instead of just printing
        private EdgeSegmentData AnalyzeAndLabelBoundarySegments_WithData(
            Curve boundaryCurve, Rectangle3d bbox, string panelCategory, double tol)
        {
            var data = new EdgeSegmentData();

            if (boundaryCurve == null || !boundaryCurve.IsClosed)
            {
                return data;
            }

            // Get all segments
            var segments = boundaryCurve.DuplicateSegments();
            if (segments == null || segments.Length == 0)
            {
                return data;
            }

            // Define bbox edges with tolerance for matching
            const double edgeTolerance = 0.5;

            // Get bbox corners
            Point3d bottomLeft = bbox.Corner(0);
            Point3d bottomRight = bbox.Corner(1);
            Point3d topRight = bbox.Corner(2);
            Point3d topLeft = bbox.Corner(3);

            // Define edge boundaries
            double leftX = bottomLeft.X;
            double rightX = bottomRight.X;
            double bottomY = bottomLeft.Y;
            double topY = topLeft.Y;

            // Initial assignment
            for (int i = 0; i < segments.Length; i++)
            {
                var seg = segments[i];
                bool isCurved = !seg.IsLinear(tol);
                bool assigned = false;

                if (isCurved)
                {
                    // Check endpoints for curved segments
                    Point3d startPt = seg.PointAtStart;
                    Point3d endPt = seg.PointAtEnd;

                    bool startOnBottom = Math.Abs(startPt.Y - bottomY) < edgeTolerance;
                    bool endOnBottom = Math.Abs(endPt.Y - bottomY) < edgeTolerance;
                    bool startOnTop = Math.Abs(startPt.Y - topY) < edgeTolerance;
                    bool endOnTop = Math.Abs(endPt.Y - topY) < edgeTolerance;
                    bool startOnLeft = Math.Abs(startPt.X - leftX) < edgeTolerance;
                    bool endOnLeft = Math.Abs(endPt.X - leftX) < edgeTolerance;
                    bool startOnRight = Math.Abs(startPt.X - rightX) < edgeTolerance;
                    bool endOnRight = Math.Abs(endPt.X - rightX) < edgeTolerance;

                    if (startOnBottom && endOnBottom)
                    {
                        data.BottomSegments.Add((seg, i));
                        assigned = true;
                    }
                    else if (startOnTop && endOnTop)
                    {
                        data.TopSegments.Add((seg, i));
                        assigned = true;
                    }
                    else if (startOnLeft && endOnLeft)
                    {
                        data.LeftSegments.Add((seg, i));
                        assigned = true;
                    }
                    else if (startOnRight && endOnRight)
                    {
                        data.RightSegments.Add((seg, i));
                        assigned = true;
                    }
                }

                // Use midpoint for linear segments or unassigned curved segments
                if (!assigned)
                {
                    double tParam = seg.Domain.Mid;
                    Point3d midpoint = seg.PointAt(tParam);

                    bool onBottom = Math.Abs(midpoint.Y - bottomY) < edgeTolerance;
                    bool onTop = Math.Abs(midpoint.Y - topY) < edgeTolerance;
                    bool onLeft = Math.Abs(midpoint.X - leftX) < edgeTolerance;
                    bool onRight = Math.Abs(midpoint.X - rightX) < edgeTolerance;

                    if (onBottom)
                        data.BottomSegments.Add((seg, i));
                    else if (onTop)
                        data.TopSegments.Add((seg, i));
                    else if (onLeft)
                        data.LeftSegments.Add((seg, i));
                    else if (onRight)
                        data.RightSegments.Add((seg, i));
                    else
                        data.InteriorSegments.Add((seg, i));
                }
            }

            // Propagation pass
            const double connectTolerance = 0.1;
            bool changesMade = true;
            int propagationIterations = 0;
            const int maxIterations = 10;

            while (changesMade && propagationIterations < maxIterations)
            {
                changesMade = false;
                propagationIterations++;

                var interiorToReassign = new List<(Curve seg, int index, string targetEdge)>();

                foreach (var (seg, idx) in data.InteriorSegments)
                {
                    Point3d startPt = seg.PointAtStart;
                    Point3d endPt = seg.PointAtEnd;

                    string startEdge = FindConnectedEdge(startPt, data.BottomSegments, data.RightSegments,
                                                          data.TopSegments, data.LeftSegments, connectTolerance);
                    string endEdge = FindConnectedEdge(endPt, data.BottomSegments, data.RightSegments,
                                                        data.TopSegments, data.LeftSegments, connectTolerance);

                    if (startEdge != null && startEdge == endEdge)
                    {
                        interiorToReassign.Add((seg, idx, startEdge));
                        changesMade = true;
                    }
                }

                // Apply reassignments
                foreach (var (seg, idx, targetEdge) in interiorToReassign)
                {
                    data.InteriorSegments.RemoveAll(x => x.index == idx);

                    if (targetEdge == "BOTTOM")
                        data.BottomSegments.Add((seg, idx));
                    else if (targetEdge == "RIGHT")
                        data.RightSegments.Add((seg, idx));
                    else if (targetEdge == "TOP")
                        data.TopSegments.Add((seg, idx));
                    else if (targetEdge == "LEFT")
                        data.LeftSegments.Add((seg, idx));
                }

            }


            if (data.InteriorSegments.Count > 0)
            {
                foreach (var (seg, idx) in data.InteriorSegments)
                {
                    string segType = seg.IsLinear(tol) ? "Linear" : "Curved";
                    double length = seg.GetLength();
                }
            }

            return data;
        }

        private void AnalyzeAndLabelBoundarySegments(Curve boundaryCurve, Rectangle3d bbox, string panelCategory, double tol)
        {

            if (boundaryCurve == null || !boundaryCurve.IsClosed)
            {
                return;
            }

            // Get all segments
            var segments = boundaryCurve.DuplicateSegments();
            if (segments == null || segments.Length == 0)
            {
                return;
            }

            // Define bbox edges with tolerance for matching
            const double edgeTolerance = 0.5;  // 0.5" tolerance for edge assignment

            // Get bbox corners (Rectangle3d uses: Corner(0)=bottom-left, (1)=bottom-right, (2)=top-right, (3)=top-left)
            Point3d bottomLeft = bbox.Corner(0);
            Point3d bottomRight = bbox.Corner(1);
            Point3d topRight = bbox.Corner(2);
            Point3d topLeft = bbox.Corner(3);

            // Define edge boundaries
            double leftX = bottomLeft.X;
            double rightX = bottomRight.X;
            double bottomY = bottomLeft.Y;
            double topY = topLeft.Y;

            // Group segments by edge
            var bottomSegments = new List<(Curve seg, int index)>();
            var rightSegments = new List<(Curve seg, int index)>();
            var topSegments = new List<(Curve seg, int index)>();
            var leftSegments = new List<(Curve seg, int index)>();
            var interiorSegments = new List<(Curve seg, int index)>();

            for (int i = 0; i < segments.Length; i++)
            {
                var seg = segments[i];

                // For curved segments, check ENDPOINTS to see if they're on an edge
                // For linear segments, check midpoint (existing logic)
                bool isCurved = !seg.IsLinear(tol);
                bool assigned = false;

                if (isCurved)
                {
                    // Get segment endpoints
                    Point3d startPt = seg.PointAtStart;
                    Point3d endPt = seg.PointAtEnd;

                    // Check if both endpoints are on the same edge
                    bool startOnBottom = Math.Abs(startPt.Y - bottomY) < edgeTolerance;
                    bool endOnBottom = Math.Abs(endPt.Y - bottomY) < edgeTolerance;
                    bool startOnTop = Math.Abs(startPt.Y - topY) < edgeTolerance;
                    bool endOnTop = Math.Abs(endPt.Y - topY) < edgeTolerance;
                    bool startOnLeft = Math.Abs(startPt.X - leftX) < edgeTolerance;
                    bool endOnLeft = Math.Abs(endPt.X - leftX) < edgeTolerance;
                    bool startOnRight = Math.Abs(startPt.X - rightX) < edgeTolerance;
                    bool endOnRight = Math.Abs(endPt.X - rightX) < edgeTolerance;

                    // Assign to edge if both endpoints are on same edge
                    if (startOnBottom && endOnBottom)
                    {
                        bottomSegments.Add((seg, i));
                        assigned = true;
                    }
                    else if (startOnTop && endOnTop)
                    {
                        topSegments.Add((seg, i));
                        assigned = true;
                    }
                    else if (startOnLeft && endOnLeft)
                    {
                        leftSegments.Add((seg, i));
                        assigned = true;
                    }
                    else if (startOnRight && endOnRight)
                    {
                        rightSegments.Add((seg, i));
                        assigned = true;
                    }
                }

                // If not assigned yet (linear segment OR curved segment not on edge), use midpoint
                if (!assigned)
                {
                    double tParam = seg.Domain.Mid;
                    Point3d midpoint = seg.PointAt(tParam);

                    // Determine which edge this segment is closest to
                    bool onBottom = Math.Abs(midpoint.Y - bottomY) < edgeTolerance;
                    bool onTop = Math.Abs(midpoint.Y - topY) < edgeTolerance;
                    bool onLeft = Math.Abs(midpoint.X - leftX) < edgeTolerance;
                    bool onRight = Math.Abs(midpoint.X - rightX) < edgeTolerance;

                    if (onBottom)
                        bottomSegments.Add((seg, i));
                    else if (onTop)
                        topSegments.Add((seg, i));
                    else if (onLeft)
                        leftSegments.Add((seg, i));
                    else if (onRight)
                        rightSegments.Add((seg, i));
                    else
                        interiorSegments.Add((seg, i));
                }
            }

            // === PROPAGATION PASS: Reassign interior segments that connect to edge segments ===
            const double connectTolerance = 0.1;  // 0.1" tolerance for connection detection
            bool changesMade = true;
            int propagationIterations = 0;
            const int maxIterations = 10;

            while (changesMade && propagationIterations < maxIterations)
            {
                changesMade = false;
                propagationIterations++;

                var interiorToReassign = new List<(Curve seg, int index, string targetEdge)>();

                foreach (var (seg, idx) in interiorSegments)
                {
                    Point3d startPt = seg.PointAtStart;
                    Point3d endPt = seg.PointAtEnd;

                    // Find what this segment connects to at start and end
                    string startEdge = FindConnectedEdge(startPt, bottomSegments, rightSegments, topSegments, leftSegments, connectTolerance);
                    string endEdge = FindConnectedEdge(endPt, bottomSegments, rightSegments, topSegments, leftSegments, connectTolerance);

                    // If both ends connect to segments on the SAME edge, reassign this segment to that edge
                    if (startEdge != null && startEdge == endEdge)
                    {
                        interiorToReassign.Add((seg, idx, startEdge));
                        changesMade = true;
                    }
                }

                // Apply reassignments
                foreach (var (seg, idx, targetEdge) in interiorToReassign)
                {
                    interiorSegments.RemoveAll(x => x.index == idx);

                    if (targetEdge == "BOTTOM")
                        bottomSegments.Add((seg, idx));
                    else if (targetEdge == "RIGHT")
                        rightSegments.Add((seg, idx));
                    else if (targetEdge == "TOP")
                        topSegments.Add((seg, idx));
                    else if (targetEdge == "LEFT")
                        leftSegments.Add((seg, idx));
                }

            }


            if (interiorSegments.Count > 0)
            {
                foreach (var (seg, idx) in interiorSegments)
                {
                    string segType = seg.IsLinear(tol) ? "Linear" : "Curved";
                    double length = seg.GetLength();
                }
            }

        }

        private void PrintEdgeSegments(string edgeName, List<(Curve seg, int index)> segments, double tol)
        {
            if (segments.Count == 0)
            {
                return;
            }


            bool isHorizontal = edgeName.Contains("BOTTOM") || edgeName.Contains("TOP");

            if (isHorizontal)
            {
                segments.Sort((a, b) =>
                {
                    Point3d midA = a.seg.PointAt(a.seg.Domain.Mid);
                    Point3d midB = b.seg.PointAt(b.seg.Domain.Mid);
                    return midA.X.CompareTo(midB.X);
                });
            }
            else
            {
                segments.Sort((a, b) =>
                {
                    Point3d midA = a.seg.PointAt(a.seg.Domain.Mid);
                    Point3d midB = b.seg.PointAt(b.seg.Domain.Mid);
                    return midA.Y.CompareTo(midB.Y);
                });
            }

            // Print each segment
            for (int i = 0; i < segments.Count; i++)
            {
                var (seg, originalIdx) = segments[i];
                string segType = seg.IsLinear(tol) ? "Linear" : "Curved";
                double length = seg.GetLength();

                // Additional info for curved segments
                string extraInfo = "";
                if (!seg.IsLinear(tol))
                {
                    // Check if it's an arc
                    Arc arc;
                    if (seg.TryGetArc(out arc, tol))
                    {
                        double radius = arc.Radius;
                        double diameter = radius * 2.0;
                        extraInfo = $" (Arc, Radius={FormatDimension(radius, 5)}, Diameter={FormatDimension(diameter, 5)})";
                    }
                    else
                    {
                        extraInfo = " (Curved, not an arc)";
                    }
                }

            }
        }

        // Helper method to find which edge a point connects to
        private string FindConnectedEdge(
            Point3d pt,
            List<(Curve seg, int index)> bottomSegments,
            List<(Curve seg, int index)> rightSegments,
            List<(Curve seg, int index)> topSegments,
            List<(Curve seg, int index)> leftSegments,
            double tolerance)
        {
            // Check each edge's segments to see if point connects to any
            foreach (var (seg, idx) in bottomSegments)
            {
                if (pt.DistanceTo(seg.PointAtStart) < tolerance || pt.DistanceTo(seg.PointAtEnd) < tolerance)
                    return "BOTTOM";
            }

            foreach (var (seg, idx) in rightSegments)
            {
                if (pt.DistanceTo(seg.PointAtStart) < tolerance || pt.DistanceTo(seg.PointAtEnd) < tolerance)
                    return "RIGHT";
            }

            foreach (var (seg, idx) in topSegments)
            {
                if (pt.DistanceTo(seg.PointAtStart) < tolerance || pt.DistanceTo(seg.PointAtEnd) < tolerance)
                    return "TOP";
            }

            foreach (var (seg, idx) in leftSegments)
            {
                if (pt.DistanceTo(seg.PointAtStart) < tolerance || pt.DistanceTo(seg.PointAtEnd) < tolerance)
                    return "LEFT";
            }

            return null;  // Not connected to any edge
        }
    }
}