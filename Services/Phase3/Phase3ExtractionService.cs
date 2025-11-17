using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.Linq;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using FWBlueprintPlugin;
using FWBlueprintPlugin.Models.Dimensioning;
using FWBlueprintPlugin.Models.Extraction;
using static FWBlueprintPlugin.Services.Dimensioning.DimensionFormatting;

namespace FWBlueprintPlugin.Services.Phase3
{
    /// <summary>
    /// Coordinates Phase 3 panel extraction, edge feature detection, and downstream dimensioning.
    /// </summary>
    internal class Phase3ExtractionService
    {
        private readonly RhinoDoc _doc;
        private readonly LayerSetupService _layerSetupService;
        private readonly EdgeFeatureDetectionService _edgeFeatureDetectionService;

        public Phase3ExtractionService(
            RhinoDoc doc,
            LayerSetupService layerSetupService,
            EdgeFeatureDetectionService edgeFeatureDetectionService)
        {
            _doc = doc ?? throw new ArgumentNullException(nameof(doc));
            _layerSetupService = layerSetupService ?? throw new ArgumentNullException(nameof(layerSetupService));
            _edgeFeatureDetectionService = edgeFeatureDetectionService ?? throw new ArgumentNullException(nameof(edgeFeatureDetectionService));
        }

        public Panel2DExtractionResult ExtractPanels(Layer parentLayer)
        {
            if (parentLayer == null)
            {
                throw new ArgumentNullException(nameof(parentLayer));
            }

            var result = new Panel2DExtractionResult();
            var layerContext = _layerSetupService.PrepareLayers(parentLayer);
            double tol = _doc.ModelAbsoluteTolerance > 0 ? _doc.ModelAbsoluteTolerance : 0.01;

            var sourceObjects = _doc.Objects.FindByLayer(_doc.Layers[layerContext.Panels3DLayerIndex]);
            if (sourceObjects == null || sourceObjects.Length == 0)
            {
                RhinoApp.WriteLine("No 3D panels found under Blueprint::3D Panels.");
                return result;
            }

            foreach (var obj in sourceObjects)
            {
                var brep = obj.Geometry as Brep;
                if (brep == null)
                {
                    continue;
                }

                string category = obj.Attributes.GetUserString("PanelCategory") ?? "Unknown";
                string panelLabel = obj.Attributes.GetUserString("PanelLabel") ?? string.Empty;
                string panelName = obj.Attributes.GetUserString("PanelName");
                string attributeName = obj.Attributes.Name;

                string panelDisplayName = category;
                if (!string.IsNullOrEmpty(panelLabel))
                {
                    panelDisplayName = $"{category} {panelLabel}";
                }
                else if (!string.IsNullOrEmpty(panelName))
                {
                    panelDisplayName = panelName.Replace(": ", " ");
                }
                else if (!string.IsNullOrEmpty(attributeName))
                {
                    panelDisplayName = attributeName.Replace(": ", " ");
                }

                var bbox = GetTopViewBoundingBox(brep);
                if (!bbox.IsValid)
                {
                    continue;
                }

                AddBoundingBoxCurve(bbox, category, panelLabel, panelDisplayName, layerContext);
                result.BoundingBoxCount++;

                var panelBoundary = GetPanelBoundaryCurve(brep, tol);
                if (panelBoundary == null)
                {
                    continue;
                }

                var segmentData = AnalyzeAndLabelBoundarySegments_WithData(panelBoundary, bbox, category, tol);

                double panelThickness = GetPanelThickness(obj, brep);

                bool skipEdgeFeatures = category.Equals("Door", StringComparison.OrdinalIgnoreCase);
                List<EdgeFeature> edgeFeatures = new List<EdgeFeature>();

                if (!skipEdgeFeatures)
                {
                    edgeFeatures = _edgeFeatureDetectionService.DimensionEdgeFeatures(
                        bbox,
                        panelBoundary,
                        segmentData,
                        brep,
                        panelThickness,
                        tol,
                        layerContext.DimensionsLayerIndex,
                        panelDisplayName);

                    if (edgeFeatures.Count > 0)
                    {
                        result.EdgeFeatureCount += edgeFeatures.Count;
                        result.EdgeFeatures.AddRange(edgeFeatures);
                    }
                }

                var interiorCutouts = DetectInteriorFeatures(brep, bbox, tol);

                var chordCutouts = new List<Brep>();
                foreach (var cutout in interiorCutouts)
                {
                    if (ClassifyCutoutSurface(cutout, tol) == Color.Magenta)
                    {
                        chordCutouts.Add(cutout);
                    }
                    else
                    {
                        var notchAttr = new ObjectAttributes
                        {
                            LayerIndex = layerContext.CutoutsLayerIndex,
                            ObjectColor = Color.Green,
                            ColorSource = ObjectColorSource.ColorFromObject
                        };
                        notchAttr.SetUserString("PanelCategory", category);
                        if (!string.IsNullOrEmpty(panelLabel))
                        {
                            notchAttr.SetUserString("PanelLabel", panelLabel);
                        }
                        if (!string.IsNullOrEmpty(panelDisplayName))
                        {
                            notchAttr.SetUserString("PanelDisplayName", panelDisplayName);
                        }
                        _doc.Objects.AddBrep(cutout, notchAttr);
                    }

                    result.CutoutCount++;
                }

                if (chordCutouts.Count > 0)
                {
                    result.ChordCutouts.Add((obj, chordCutouts));
                }

            }

            SetLayerVisibilities(layerContext);
            _layerSetupService.DropDimensionsToZ0(layerContext);

            if (result.ChordCutouts.Count > 0)
            {
                ProcessChordPassThroughs(_doc, layerContext.DimensionsLayerIndex, result.ChordCutouts, tol);
            }

            _doc.Views.Redraw();
            return result;
        }

        private void AddBoundingBoxCurve(Rectangle3d bbox, string category, string panelLabel, string panelDisplayName, BlueprintLayerContext context)
        {
            var bboxCurve = bbox.ToNurbsCurve();
            var bboxAttr = new ObjectAttributes
            {
                LayerIndex = context.Panels2DLayerIndex,
                LinetypeSource = ObjectLinetypeSource.LinetypeFromObject,
                LinetypeIndex = context.DashLinetypeIndex,
                ColorSource = ObjectColorSource.ColorFromLayer
            };
            bboxAttr.SetUserString("PanelCategory", category);
            if (!string.IsNullOrEmpty(panelLabel))
            {
                bboxAttr.SetUserString("PanelLabel", panelLabel);
            }
            if (!string.IsNullOrEmpty(panelDisplayName))
            {
                bboxAttr.SetUserString("PanelDisplayName", panelDisplayName);
            }
            bboxAttr.SetUserString("GeometryType", "BoundingBox");
            _doc.Objects.AddCurve(bboxCurve, bboxAttr);
        }

        private void SetLayerVisibilities(BlueprintLayerContext context)
        {
            _doc.Layers[context.Panels3DLayerIndex].IsVisible = true;
            _doc.Layers[context.Panels2DLayerIndex].IsVisible = true;
            _doc.Layers[context.CutoutsLayerIndex].IsVisible = true;
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

        private List<Brep> DetectInteriorFeatures(Brep brep, Rectangle3d bbox, double tol)
        {
            var interiorCutouts = new List<Brep>();
            var bboxCurve = bbox.ToNurbsCurve();

            BrepFace horizontalFace = FindHorizontalFace(brep);
            if (horizontalFace == null) return interiorCutouts;

            foreach (var loop in horizontalFace.Loops)
            {
                if (loop.LoopType != BrepLoopType.Inner) continue;

                var loopCurve = loop.To3dCurve();
                if (loopCurve == null) continue;

                var projected = Curve.ProjectToPlane(loopCurve, Plane.WorldXY);
                if (projected == null || !projected.IsClosed) continue;

                var interEvents = Intersection.CurveCurve(projected, bboxCurve, tol, tol);
                if (interEvents.Count > 0) continue;

                var cutoutBreps = Brep.CreatePlanarBreps(projected, tol);
                if (cutoutBreps != null && cutoutBreps.Length > 0)
                    interiorCutouts.Add(cutoutBreps[0]);
            }

            return interiorCutouts;
        }

        private double GetPanelThickness(RhinoObject panelObj, Brep brep)
        {
            if (panelObj != null)
            {
                string thicknessValue = panelObj.Attributes.GetUserString("PanelThickness");
                if (!string.IsNullOrWhiteSpace(thicknessValue) &&
                    double.TryParse(thicknessValue, NumberStyles.Float, CultureInfo.InvariantCulture, out double storedThickness) &&
                    storedThickness > 0)
                {
                    return storedThickness;
                }
            }

            if (brep != null)
            {
                var bbox = brep.GetBoundingBox(true);
                if (bbox.IsValid)
                {
                    double xSize = bbox.Max.X - bbox.Min.X;
                    double ySize = bbox.Max.Y - bbox.Min.Y;
                    double zSize = bbox.Max.Z - bbox.Min.Z;
                    return Math.Min(Math.Min(xSize, ySize), zSize);
                }
            }

            return 0.0;
        }

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

        private Curve GetPanelBoundaryCurve(Brep brep, double tol)
        {
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

            var edgeCurves = new List<Curve>();
            var processedEdges = new HashSet<int>();

            foreach (var edge in brep.Edges)
            {
                bool belongsToTopFace = false;

                foreach (var adj in edge.AdjacentFaces())
                {
                    if (adj == topFace.FaceIndex)
                    {
                        belongsToTopFace = true;
                        break;
                    }
                }

                if (!belongsToTopFace)
                    continue;

                if (processedEdges.Contains(edge.EdgeIndex))
                    continue;

                processedEdges.Add(edge.EdgeIndex);

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

            var flattenedCurves = new List<Curve>();
            var xyPlane = Plane.WorldXY;

            foreach (var curve in edgeCurves)
            {
                var projected = Curve.ProjectToPlane(curve, xyPlane);
                if (projected != null)
                {
                    var moveToZ0 = Transform.Translation(0, 0, -projected.PointAtStart.Z);
                    projected.Transform(moveToZ0);
                    flattenedCurves.Add(projected);
                }
            }

            var joined = Curve.JoinCurves(flattenedCurves, tol);

            if (joined == null || joined.Length == 0)
            {
                return null;
            }

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

            return longestCurve;
        }

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

        private void ProcessChordPassThroughs(
            RhinoDoc doc,
            int dimsLayerIdx,
            List<(RhinoObject Panel, List<Brep> ChordCutouts)> panelCutouts,
            double tol)
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
            RhinoDoc doc,
            int dimsLayerIdx,
            List<(RhinoObject Panel, Brep Cutout)> confirmedCutouts,
            double tol)
        {
            var panelGroups = confirmedCutouts.GroupBy(c => c.Panel);

            foreach (var group in panelGroups)
            {
                var panel = group.Key;
                var panelBBox = panel.Geometry.GetBoundingBox(true);
                var dimStyle = ResolveBlueprintStyle(doc, "AddChordDimensions");
                if (dimStyle == null)
                {
                    if (LoggingOptions.EnableVerboseLogging)
                    {
                        RhinoApp.WriteLine("[Blueprint Styles][Phase3ExtractionService.AddChordDimensions] Unable to resolve blueprint dimension style; skipping chord dimensions.");
                    }
                    continue;
                }

                var edgeFeatures = new List<(Brep Cutout, string Type, string EdgeSide)>();
                var interior = new List<InteriorFeatureInfo>();

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
                        if (fbSide != null)
                        {
                            edgeFeatures.Add((cutout, fbType, fbSide));
                        }
                        else
                        {
                            var featureInfo = AnalyzeInteriorFeature(cutout, tol);
                            if (featureInfo.Kind != InteriorFeatureKind.Unknown)
                            {
                                interior.Add(featureInfo);
                            }
                        }
                    }
                }

                if (edgeFeatures.Count > 0)
                {
                    AddPerFeatureEdgeDimensions(doc, dimsLayerIdx, edgeFeatures, panelBBox, tol, dimStyle);
                }

                foreach (var info in interior)
                {
                    DimensionInteriorFeature(doc, dimsLayerIdx, info, panelBBox, dimStyle);
                }
            }
        }

        private void AddPerFeatureEdgeDimensions(
            RhinoDoc doc,
            int dimsLayerIdx,
            List<(Brep Cutout, string Type, string EdgeSide)> edgeFeatures,
            BoundingBox panelBBox,
            double tol,
            DimensionStyle dimStyle)
        {
            var attr = new ObjectAttributes { LayerIndex = dimsLayerIdx };

            foreach (var (cutout, type, edgeSide) in edgeFeatures)
            {
                var (edgeCurve, cornerA, cornerB, isHorizontal) = GetEdgeGeom(panelBBox, edgeSide);
                if (edgeCurve == null)
                {
                    continue;
                }

                Point3d ProjectToEdge(Point3d p)
                {
                    double t;
                    edgeCurve.ClosestPoint(p, out t);
                    return edgeCurve.PointAt(t);
                }

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
                    var dim = ApplyDimStyle(LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, dimDir, cornerOnEdge, center, dimLinePoint, 0), dimStyle);
                    BlueprintAnnotationDebug.LogDimensionRequest(
                        "Phase3ExtractionService.AddChordDimensions",
                        "CornerToCenter",
                        dimStyle,
                        cornerOnEdge,
                        center,
                        dimLinePoint,
                        attr?.LayerIndex ?? -1);
                    TryAddDimension(doc, attr, dim, "Phase3.AddChordDimensions.CornerToCenter");
                    continue;
                }

                var mouth = GetEdgeMouthEndpoints(cutout, edgeCurve, edgeSide, tol);
                if (mouth == null) { continue; }
                var (mStartRaw, mEndRaw) = mouth.Value;

                var mStart = ProjectToEdge(mStartRaw);
                var mEnd = ProjectToEdge(mEndRaw);
                double ts, te; edgeCurve.ClosestPoint(mStart, out ts); edgeCurve.ClosestPoint(mEnd, out te);
                if (ts > te) { var tmp = mStart; mStart = mEnd; mEnd = tmp; }

                var nearCorner = (mStart.DistanceTo(cornerA) <= mStart.DistanceTo(cornerB)) ? cornerA : cornerB;
                var pCorner = ProjectToEdge(nearCorner);

                {
                    var mid = (pCorner + mStart) / 2.0;
                    var dimLinePoint = mid + offsetDir * offset;
                    var d = ApplyDimStyle(LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, dimDir, pCorner, mStart, dimLinePoint, 0), dimStyle);
                    BlueprintAnnotationDebug.LogDimensionRequest(
                        "Phase3ExtractionService.AddChordDimensions",
                        "CornerToMouth",
                        dimStyle,
                        pCorner,
                        mStart,
                        dimLinePoint,
                        attr?.LayerIndex ?? -1);
                    TryAddDimension(doc, attr, d, "Phase3.AddChordDimensions.CornerToMouth");
                }
                {
                    var mid = (mStart + mEnd) / 2.0;
                    var dimLinePoint = mid + offsetDir * offset;
                    var d = ApplyDimStyle(LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, dimDir, mStart, mEnd, dimLinePoint, 0), dimStyle);
                    BlueprintAnnotationDebug.LogDimensionRequest(
                        "Phase3ExtractionService.AddChordDimensions",
                        "MouthWidth",
                        dimStyle,
                        mStart,
                        mEnd,
                        dimLinePoint,
                        attr?.LayerIndex ?? -1);
                    TryAddDimension(doc, attr, d, "Phase3.AddChordDimensions.MouthWidth");
                }

                if (type == "Notch")
                {
                    double depth = GetMaxDepthFromEdge(cutout, edgeCurve);

                    var mouthMid = new Point3d((mStart.X + mEnd.X) * 0.5, (mStart.Y + mEnd.Y) * 0.5, 0);
                    var deepest = GetDeepestPointFromEdge(cutout, edgeCurve);
                    var depthMid = new Point3d((mouthMid.X + deepest.X) * 0.5, (mouthMid.Y + deepest.Y) * 0.5, 0);
                    var perpDir = isHorizontal ? Vector3d.YAxis : Vector3d.XAxis;
                    var depthDimPoint = depthMid + (perpDir * (offset * 0.9));
                    var depthDim = ApplyDimStyle(LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, perpDir, mouthMid, deepest, depthDimPoint, 0), dimStyle);
                    BlueprintAnnotationDebug.LogDimensionRequest(
                        "Phase3ExtractionService.AddChordDimensions",
                        "NotchDepth",
                        dimStyle,
                        mouthMid,
                        deepest,
                        depthDimPoint,
                        attr?.LayerIndex ?? -1);
                    TryAddDimension(doc, attr, depthDim, "Phase3.AddChordDimensions.NotchDepth");

                    bool flushToCorner = (pCorner.DistanceTo(mStart) < tol) || (pCorner.DistanceTo(mEnd) < tol);
                    if (!flushToCorner)
                    {
                        var insetMid = new Point3d((pCorner.X + mStart.X) * 0.5, (pCorner.Y + mStart.Y) * 0.5, 0);
                        var insetPoint = insetMid + offsetDir * (offset * 1.2);
                        var insetDim = ApplyDimStyle(LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, dimDir, pCorner, mStart, insetPoint, 0), dimStyle);
                        BlueprintAnnotationDebug.LogDimensionRequest(
                            "Phase3ExtractionService.AddChordDimensions",
                            "Inset",
                            dimStyle,
                            pCorner,
                            mStart,
                            insetPoint,
                            attr?.LayerIndex ?? -1);
                        TryAddDimension(doc, attr, insetDim, "Phase3.AddChordDimensions.Inset");
                    }
                }
            }
        }

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

        private (Point3d Start, Point3d End)? GetEdgeMouthEndpoints(Brep cutout, LineCurve edgeCurve, string edgeSide, double tol)
        {
            var outer = cutout.Loops.FirstOrDefault(l => l.LoopType == BrepLoopType.Outer);
            if (outer == null) return null;
            var boundary = outer.To3dCurve();
            if (boundary == null) return null;

            var hits = GetIntersectionsOnSegment(boundary, edgeCurve, tol);

            if (hits.Count < 2)
            {
                Vector3d inward =
                    (edgeSide == "Bottom") ? Vector3d.YAxis :
                    (edgeSide == "Top") ? -Vector3d.YAxis :
                    (edgeSide == "Left") ? Vector3d.XAxis :
                                              -Vector3d.XAxis;

                inward.Unitize();
                inward *= 0.02;

                var a = edgeCurve.PointAt(edgeCurve.Domain.Min);
                var b = edgeCurve.PointAt(edgeCurve.Domain.Max);
                var insetA = new Point3d(a.X + inward.X, a.Y + inward.Y, 0);
                var insetB = new Point3d(b.X + inward.X, b.Y + inward.Y, 0);
                var inset = new LineCurve(insetA, insetB);

                var insetHits = GetIntersectionsOnSegment(boundary, inset, tol);
                if (insetHits.Count >= 2)
                {
                    hits = insetHits.Select(p =>
                    {
                        double t; edgeCurve.ClosestPoint(p, out t);
                        return edgeCurve.PointAt(t);
                    }).ToList();
                }
            }

            if (hits.Count < 2) return null;

            var withT = hits.Select(p =>
            {
                double t; edgeCurve.ClosestPoint(p, out t);
                return (T: t, P: p);
            }).OrderBy(x => x.T).ToList();

            var p0 = withT[0].P;
            var p1 = withT[1].P;
            return (p0, p1);
        }

        private List<Point3d> GetIntersectionsOnSegment(Curve boundary, LineCurve segment, double tol)
        {
            var list = new List<Point3d>();
            var ccx = Intersection.CurveCurve(boundary, segment, tol, tol);
            if (ccx == null || ccx.Count == 0) return list;

            foreach (var ev in ccx)
            {
                var pt = ev.PointA;
                double tSeg;
                if (segment.ClosestPoint(pt, out tSeg))
                {
                    var onSeg = tSeg >= segment.Domain.Min - tol && tSeg <= segment.Domain.Max + tol;
                    if (onSeg) list.Add(pt);
                }
            }
            return list;
        }

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
                    type = edgeWidth <= 2.0625 ? "EdgeCordHole" : "EdgeCordSlot";

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

        [Obsolete("Legacy interior pass-through dimensioning. Use DimensionInteriorFeature instead.")]
        private void AddInteriorDimensionsV1(RhinoDoc doc, int dimsLayerIdx, Brep cutout, string type,
            BoundingBox panelBBox, double tol, DimensionStyle dimStyle, ObjectAttributes attr, Plane dimPlane)
        {
            var massProp = AreaMassProperties.Compute(cutout);
            if (massProp == null) return;
            Point3d center = massProp.Centroid;

            double xDistLeft = center.X - panelBBox.Min.X;
            double xDistRight = panelBBox.Max.X - center.X;
            double yDistBottom = center.Y - panelBBox.Min.Y;
            double yDistTop = center.Y - panelBBox.Max.Y;

            if (type.Equals("InteriorCordHole", StringComparison.OrdinalIgnoreCase))
            {
                const double cornerThreshold = 2.0625;
                int nearEdges = 0;

                if (Math.Abs(center.X - panelBBox.Min.X) <= cornerThreshold) nearEdges++;
                if (Math.Abs(panelBBox.Max.X - center.X) <= cornerThreshold) nearEdges++;
                if (Math.Abs(center.Y - panelBBox.Min.Y) <= cornerThreshold) nearEdges++;
                if (Math.Abs(panelBBox.Max.Y - center.Y) <= cornerThreshold) nearEdges++;

                if (nearEdges >= 2)
                {
                    if (LoggingOptions.EnableVerboseLogging)
                    {
                        RhinoApp.WriteLine("[Interior Cord Hole] Skipping dimensions because the feature is within 2.0625\" of two edges.");
                    }
                    return;
                }
            }

            bool fromLeft = xDistLeft < xDistRight;
            Point3d xStart = fromLeft ? new Point3d(panelBBox.Min.X, center.Y, 0) : new Point3d(panelBBox.Max.X, center.Y, 0);
            Point3d xTextPos = new Point3d((xStart.X + center.X) / 2.0, (xStart.Y + center.Y) / 2.0, 0);

            var xDim = ApplyDimStyle(LinearDimension.Create(AnnotationType.Aligned, dimStyle, dimPlane, Vector3d.XAxis, xStart, center, xTextPos, 0), dimStyle);
            if (xDim != null)
            {
                xDim.PlainText = FormatDimension(fromLeft ? xDistLeft : xDistRight, 4);
                BlueprintAnnotationDebug.LogDimensionRequest(
                    "Phase3ExtractionService.AddInteriorHoleDimensions",
                    "HoleX",
                    dimStyle,
                    xStart,
                    center,
                    xTextPos,
                    attr?.LayerIndex ?? -1);
                TryAddDimension(doc, attr, xDim, "Phase3.AddInteriorHoleDimensions.HoleX");
            }

            bool fromBottom = yDistBottom < yDistTop;
            Point3d yStart = fromBottom ? new Point3d(center.X, panelBBox.Min.Y, 0) : new Point3d(center.X, panelBBox.Max.Y, 0);
            Point3d yTextPos = new Point3d((yStart.X + center.X) / 2.0, (yStart.Y + center.Y) / 2.0, 0);

            var yDim = ApplyDimStyle(LinearDimension.Create(AnnotationType.Aligned, dimStyle, dimPlane, Vector3d.YAxis, yStart, center, yTextPos, 0), dimStyle);
            if (yDim != null)
            {
                yDim.PlainText = FormatDimension(fromBottom ? yDistBottom : yDistTop, 4);
                BlueprintAnnotationDebug.LogDimensionRequest(
                    "Phase3ExtractionService.AddInteriorHoleDimensions",
                    "HoleY",
                    dimStyle,
                    yStart,
                    center,
                    yTextPos,
                    attr?.LayerIndex ?? -1);
                TryAddDimension(doc, attr, yDim, "Phase3.AddInteriorHoleDimensions.HoleY");
            }
        }

        private void DimensionInteriorFeature(
            RhinoDoc doc,
            int dimsLayerIdx,
            InteriorFeatureInfo feature,
            BoundingBox panelBBox,
            DimensionStyle dimStyle)
        {
            switch (feature.Kind)
            {
                case InteriorFeatureKind.Hole:
                    DimensionInteriorHole(doc, dimsLayerIdx, feature, panelBBox, dimStyle);
                    break;
                case InteriorFeatureKind.Slot:
                    DimensionInteriorSlot(doc, dimsLayerIdx, feature, panelBBox, dimStyle);
                    break;
            }
        }

        private void DimensionInteriorHole(
            RhinoDoc doc,
            int dimsLayerIdx,
            InteriorFeatureInfo feature,
            BoundingBox panelBBox,
            DimensionStyle dimStyle)
        {
            const double CornerThreshold = 2.0625;

            var attr = new ObjectAttributes { LayerIndex = dimsLayerIdx };
            var plane = Plane.WorldXY;
            var center = new Point3d(feature.Center.X, feature.Center.Y, 0);

            double left = center.X - panelBBox.Min.X;
            double right = panelBBox.Max.X - center.X;
            double bottom = center.Y - panelBBox.Min.Y;
            double top = panelBBox.Max.Y - center.Y;

            if (Math.Min(left, right) <= CornerThreshold && Math.Min(bottom, top) <= CornerThreshold)
            {
                return;
            }

            bool fromLeft = left <= right;
            var xStart = new Point3d(fromLeft ? panelBBox.Min.X : panelBBox.Max.X, center.Y, 0);
            var xDim = ApplyDimStyle(LinearDimension.Create(
                AnnotationType.Rotated,
                dimStyle,
                plane,
                Vector3d.YAxis,
                xStart,
                center,
                center,
                0), dimStyle);
            BlueprintAnnotationDebug.LogDimensionRequest(
                "Phase3ExtractionService.AddInteriorSlotDimensions",
                "SlotXCenter",
                dimStyle,
                xStart,
                center,
                center,
                attr?.LayerIndex ?? -1);
            TryAddDimension(doc, attr, xDim, "Phase3.AddInteriorSlotDimensions.SlotXCenter");

            bool fromBottom = bottom <= top;
            var yStart = new Point3d(center.X, fromBottom ? panelBBox.Min.Y : panelBBox.Max.Y, 0);
            var yDim = ApplyDimStyle(LinearDimension.Create(
                AnnotationType.Rotated,
                dimStyle,
                plane,
                Vector3d.XAxis,
                yStart,
                center,
                center,
                Math.PI / 2.0), dimStyle);
            BlueprintAnnotationDebug.LogDimensionRequest(
                "Phase3ExtractionService.AddInteriorSlotDimensions",
                "SlotYCenter",
                dimStyle,
                yStart,
                center,
                center,
                attr?.LayerIndex ?? -1);
            TryAddDimension(doc, attr, yDim, "Phase3.AddInteriorSlotDimensions.SlotYCenter");
        }

        private void DimensionInteriorSlot(
            RhinoDoc doc,
            int dimsLayerIdx,
            InteriorFeatureInfo feature,
            BoundingBox panelBBox,
            DimensionStyle dimStyle)
        {
            var attr = new ObjectAttributes { LayerIndex = dimsLayerIdx };
            var plane = Plane.WorldXY;

            double slotMinX = feature.Bounds.Min.X;
            double slotMaxX = feature.Bounds.Max.X;
            double slotMinY = feature.Bounds.Min.Y;
            double slotMaxY = feature.Bounds.Max.Y;
            double slotCenterX = (slotMinX + slotMaxX) / 2.0;
            double slotCenterY = (slotMinY + slotMaxY) / 2.0;

            double leftGap = slotMinX - panelBBox.Min.X;
            double rightGap = panelBBox.Max.X - slotMaxX;
            double bottomGap = slotMinY - panelBBox.Min.Y;
            double topGap = panelBBox.Max.Y - slotMaxY;
            if (LoggingOptions.EnableVerboseLogging)
            {
                RhinoApp.WriteLine($"[SlotHeight] leftGap={leftGap:F3}, rightGap={rightGap:F3}, topGap={topGap:F3}, bottomGap={bottomGap:F3}");
            }

            bool fromLeft = leftGap <= rightGap;
            var xStart = new Point3d(fromLeft ? panelBBox.Min.X : panelBBox.Max.X, slotCenterY, 0);
            var xEnd = new Point3d(fromLeft ? slotMinX : slotMaxX, slotCenterY, 0);
            var xLinePoint = new Point3d((xStart.X + xEnd.X) / 2.0, slotCenterY, 0);
            var xDim = ApplyDimStyle(LinearDimension.Create(AnnotationType.Aligned, dimStyle, plane, Vector3d.XAxis, xStart, xEnd, xLinePoint, 0), dimStyle);
            BlueprintAnnotationDebug.LogDimensionRequest(
                "Phase3ExtractionService.AddSlotOffsetDimensions",
                "SlotOffsetX",
                dimStyle,
                xStart,
                xEnd,
                xLinePoint,
                attr?.LayerIndex ?? -1);
            TryAddDimension(doc, attr, xDim, "Phase3.AddSlotOffsetDimensions.X");

            bool fromBottom = bottomGap <= topGap;
            var yStart = new Point3d(slotCenterX, fromBottom ? panelBBox.Min.Y : panelBBox.Max.Y, 0);
            var yEnd = new Point3d(slotCenterX, fromBottom ? slotMinY : slotMaxY, 0);
            var yLinePoint = new Point3d(slotCenterX, (yStart.Y + yEnd.Y) / 2.0, 0);
            var yDim = ApplyDimStyle(LinearDimension.Create(
                AnnotationType.Rotated,
                dimStyle,
                plane,
                Vector3d.XAxis,
                yStart,
                yEnd,
                yLinePoint,
                Math.PI / 2.0), dimStyle);
            BlueprintAnnotationDebug.LogDimensionRequest(
                "Phase3ExtractionService.AddSlotOffsetDimensions",
                "SlotOffsetY",
                dimStyle,
                yStart,
                yEnd,
                yLinePoint,
                attr?.LayerIndex ?? -1);
            TryAddDimension(doc, attr, yDim, "Phase3.AddSlotOffsetDimensions.Y");

            double widthOffset = topGap <= bottomGap ? -3.0 : 3.0;
            bool widthOffsetBelow = widthOffset < 0;
            var widthStart = new Point3d(slotMinX, slotCenterY, 0);
            var widthEnd = new Point3d(slotMaxX, slotCenterY, 0);
            var widthLinePoint = new Point3d((slotMinX + slotMaxX) / 2.0, slotCenterY + widthOffset, 0);
            var widthDim = ApplyDimStyle(LinearDimension.Create(
                AnnotationType.Rotated,
                dimStyle,
                plane,
                Vector3d.YAxis,
                widthStart,
                widthEnd,
                widthLinePoint,
                0), dimStyle);
            BlueprintAnnotationDebug.LogDimensionRequest(
                "Phase3ExtractionService.AddSlotSizeDimensions",
                "SlotWidth",
                dimStyle,
                widthStart,
                widthEnd,
                widthLinePoint,
                attr?.LayerIndex ?? -1);
            TryAddDimension(doc, attr, widthDim, "Phase3.AddSlotSizeDimensions.Width");

            bool nearestLeftEdge = leftGap <= rightGap;
            double slotAnchorX = nearestLeftEdge ? slotMaxX : slotMinX;
            double dimensionX = slotAnchorX + (nearestLeftEdge ? 3.0 : -3.0);
            double heightLabelOffset = widthOffsetBelow ? 3.0 : -3.0;
            if (LoggingOptions.EnableVerboseLogging)
            {
                RhinoApp.WriteLine($"[SlotHeight] nearestLeftEdge={nearestLeftEdge}, widthOffset={widthOffset:F3}, widthOffsetBelow={widthOffsetBelow}, slotAnchorX={slotAnchorX:F3}, dimensionX={dimensionX:F3}, labelOffset={heightLabelOffset:F3}");
            }

            var heightStart = new Point3d(slotAnchorX, slotMinY, 0);
            var heightEnd = new Point3d(slotAnchorX, slotMaxY, 0);
            var heightLinePoint = new Point3d(dimensionX, slotCenterY + heightLabelOffset, 0);
            var heightDim = ApplyDimStyle(LinearDimension.Create(
                AnnotationType.Rotated,
                dimStyle,
                plane,
                Vector3d.XAxis,
                heightStart,
                heightEnd,
                heightLinePoint,
                Math.PI / 2.0), dimStyle);
            BlueprintAnnotationDebug.LogDimensionRequest(
                "Phase3ExtractionService.AddSlotSizeDimensions",
                "SlotHeight",
                dimStyle,
                heightStart,
                heightEnd,
                heightLinePoint,
                attr?.LayerIndex ?? -1);
            TryAddDimension(doc, attr, heightDim, "Phase3.AddSlotSizeDimensions.Height");
        }

        private InteriorFeatureInfo AnalyzeInteriorFeature(Brep cutout, double tol)
        {
            const double HoleMinDiameter = 1.4375;
            const double HoleMaxDiameter = 2.0625;
            const double SlotMinLength = 4.0;
            const double CircularTolerance = 0.02;

            if (cutout == null)
            {
                return new InteriorFeatureInfo();
            }

            var bbox = cutout.GetBoundingBox(true);
            if (!bbox.IsValid)
            {
                return new InteriorFeatureInfo();
            }

            double width = bbox.Max.X - bbox.Min.X;
            double height = bbox.Max.Y - bbox.Min.Y;
            double minDim = Math.Min(width, height);
            double maxDim = Math.Max(width, height);
            var center = new Point3d((bbox.Min.X + bbox.Max.X) / 2.0, (bbox.Min.Y + bbox.Max.Y) / 2.0, 0);

            if (Math.Abs(width - height) <= CircularTolerance &&
                minDim >= HoleMinDiameter - tol &&
                maxDim <= HoleMaxDiameter + tol)
            {
                return new InteriorFeatureInfo
                {
                    Kind = InteriorFeatureKind.Hole,
                    Center = center,
                    Bounds = bbox,
                    Diameter = (width + height) / 2.0,
                    Width = width,
                    Height = height
                };
            }

            if (minDim >= HoleMinDiameter - tol &&
                minDim <= HoleMaxDiameter + tol &&
                maxDim >= SlotMinLength - tol &&
                maxDim >= minDim * 1.5)
            {
                return new InteriorFeatureInfo
                {
                    Kind = InteriorFeatureKind.Slot,
                    Center = center,
                    Bounds = bbox,
                    Width = maxDim,
                    Height = minDim
                };
            }

            return new InteriorFeatureInfo();
        }

        private static void TryAddDimension(RhinoDoc doc, ObjectAttributes attr, LinearDimension dimension, string context)
        {
            if (dimension == null)
            {
                if (LoggingOptions.EnableVerboseLogging)
                {
                    RhinoApp.WriteLine($"[Blueprint Styles][{context}] Dimension instance is null; skipping add.");
                }
                return;
            }

            if (LoggingOptions.EnableVerboseLogging)
            {
                var style = doc?.DimStyles.FindId(dimension.DimensionStyleId);
                var textPoint2d = dimension.TextPosition;
                var textPoint = new Point3d(textPoint2d.X, textPoint2d.Y, 0);
                RhinoApp.WriteLine(
                    $"[Blueprint Styles][{context}] Adding dimension style='{style?.Name ?? "(unknown)"}' (Id={dimension.DimensionStyleId}) layer={attr?.LayerIndex ?? -1} textPt={FormatPoint(textPoint)} value='{dimension.PlainText}'");
            }

            doc?.Objects.AddLinearDimension(dimension, attr);
        }

        private static DimensionStyle ResolveBlueprintStyle(RhinoDoc doc, string caller)
        {
            return BlueprintAnnotationDebug.ResolveDefaultStyle(doc, $"Phase3ExtractionService.{caller}");
        }

        private static string FormatPoint(Point3d pt) => $"({pt.X:F3}, {pt.Y:F3}, {pt.Z:F3})";

        private static LinearDimension ApplyDimStyle(LinearDimension dimension, DimensionStyle style)
        {
            if (dimension != null && style != null)
            {
                dimension.DimensionStyleId = style.Id;
            }

            return dimension;
        }

        private enum InteriorFeatureKind
        {
            Unknown,
            Hole,
            Slot
        }

        private sealed class InteriorFeatureInfo
        {
            public InteriorFeatureKind Kind { get; set; } = InteriorFeatureKind.Unknown;
            public Point3d Center { get; set; }
            public BoundingBox Bounds { get; set; }
            public double Diameter { get; set; }
            public double Width { get; set; }
            public double Height { get; set; }
        }

        private double GetMaxDepthFromEdge(Brep cutout, Curve edgeCurve, int samples = 48)
        {
            double maxD = 0.0;
            var loops = cutout.Loops;
            if (loops == null || loops.Count == 0) return 0.0;
            var c = loops[0].To3dCurve();

            double len = c.GetLength();
            for (int i = 0; i <= samples; i++)
            {
                double t;
                c.LengthParameter(len * i / samples, out t);
                var pt = c.PointAt(t);
                double te;
                edgeCurve.ClosestPoint(pt, out te);
                var onEdge = edgeCurve.PointAt(te);
                maxD = Math.Max(maxD, pt.DistanceTo(onEdge));
            }
            return maxD;
        }

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
                double t;
                c.LengthParameter(len * i / samples, out t);
                var pt = c.PointAt(t);
                double te;
                edgeCurve.ClosestPoint(pt, out te);
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

        private EdgeSegmentData AnalyzeAndLabelBoundarySegments_WithData(
            Curve boundaryCurve, Rectangle3d bbox, string panelCategory, double tol)
        {
            var data = new EdgeSegmentData();

            if (boundaryCurve == null || !boundaryCurve.IsClosed)
            {
                return data;
            }

            var segments = boundaryCurve.DuplicateSegments();
            if (segments == null || segments.Length == 0)
            {
                return data;
            }

            const double edgeTolerance = 0.5;

            Point3d bottomLeft = bbox.Corner(0);
            Point3d bottomRight = bbox.Corner(1);
            Point3d topRight = bbox.Corner(2);
            Point3d topLeft = bbox.Corner(3);

            double leftX = bottomLeft.X;
            double rightX = bottomRight.X;
            double bottomY = bottomLeft.Y;
            double topY = topLeft.Y;

            for (int i = 0; i < segments.Length; i++)
            {
                var seg = segments[i];
                bool isCurved = !seg.IsLinear(tol);
                bool assigned = false;

                if (isCurved)
                {
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

                    string targetEdge = null;

                    if (!string.IsNullOrEmpty(startEdge) && startEdge == endEdge)
                    {
                        targetEdge = startEdge;
                    }
                    else if (!string.IsNullOrEmpty(startEdge) && string.IsNullOrEmpty(endEdge))
                    {
                        targetEdge = startEdge;
                    }
                    else if (!string.IsNullOrEmpty(endEdge) && string.IsNullOrEmpty(startEdge))
                    {
                        targetEdge = endEdge;
                    }

                    if (targetEdge != null)
                    {
                        interiorToReassign.Add((seg, idx, targetEdge));
                        changesMade = true;
                    }
                }

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

            return data;
        }

        private string FindConnectedEdge(
            Point3d pt,
            List<(Curve seg, int index)> bottomSegments,
            List<(Curve seg, int index)> rightSegments,
            List<(Curve seg, int index)> topSegments,
            List<(Curve seg, int index)> leftSegments,
            double tolerance)
        {
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

            return null;
        }

        private sealed class Point3dComparer : IEqualityComparer<Point3d>
        {
            private readonly double _tol;
            public Point3dComparer(double tol) { _tol = tol; }

            public bool Equals(Point3d x, Point3d y) => x.DistanceTo(y) < _tol;

            public int GetHashCode(Point3d obj) => obj.GetHashCode();
        }
    }
}
