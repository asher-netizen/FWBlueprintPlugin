using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using FWBlueprintPlugin;
using FWBlueprintPlugin.Services;

namespace FWBlueprintPlugin.Services.Phase3
{
    /// <summary>
    /// Detects edge features along panel boundaries and routes the data to edge dimensioning.
    /// </summary>
    internal class EdgeFeatureDetectionService
    {
        private readonly EdgeDimensioningService _edgeDimensioningService;

        public EdgeFeatureDetectionService(EdgeDimensioningService edgeDimensioningService)
        {
            _edgeDimensioningService = edgeDimensioningService ?? throw new ArgumentNullException(nameof(edgeDimensioningService));
        }

        public List<EdgeFeature> DimensionEdgeFeatures(
            Rectangle3d bbox,
            Curve panelBoundary,
            EdgeSegmentData segmentData,
            Brep panelBrep,
            double tol,
            int dimensionsLayerIndex)
        {
            if (segmentData == null)
            {
                return new List<EdgeFeature>();
            }

            _ = panelBrep; // Unused in ray protocol implementation but kept for compatibility
            _ = tol;

            var allEdgeFeatures = new List<EdgeFeature>();

            allEdgeFeatures.AddRange(ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.BottomSegments, "BOTTOM"));
            allEdgeFeatures.AddRange(ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.RightSegments, "RIGHT"));
            allEdgeFeatures.AddRange(ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.TopSegments, "TOP"));
            allEdgeFeatures.AddRange(ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.LeftSegments, "LEFT"));

            if (allEdgeFeatures.Count > 0)
            {
                _edgeDimensioningService.AddEdgeFeatureDimensions(bbox, allEdgeFeatures, dimensionsLayerIndex);
            }

            return allEdgeFeatures;
        }

        private List<EdgeFeature> ProcessEdge_RayProtocol(
            Rectangle3d bbox,
            Curve panelBoundary,
            List<(Curve seg, int index)> segments,
            string edgeName)
        {
            var detectedFeatures = new List<EdgeFeature>();
            if (segments == null || segments.Count <= 1)
            {
                return detectedFeatures;
            }

            bool isHorizontal = edgeName == "BOTTOM" || edgeName == "TOP";
            segments.Sort((a, b) =>
            {
                Point3d midA = a.seg.PointAt(a.seg.Domain.Mid);
                Point3d midB = b.seg.PointAt(b.seg.Domain.Mid);
                return isHorizontal ? midA.X.CompareTo(midB.X) : midA.Y.CompareTo(midB.Y);
            });

            GetEdgeParameters(bbox, edgeName, out Point3d edgeStart, out Point3d edgeEnd, out Vector3d edgeDirection, out Vector3d perpendicular);

            double edgeLength = edgeStart.DistanceTo(edgeEnd);
            const double baselineDepth = 0.001;

            RhinoApp.WriteLine($"  [DEBUG] [{edgeName}] Edge length: {edgeLength:F2}\", Segments: {segments.Count}");
            if (edgeLength > 1000)
            {
                RhinoApp.WriteLine($"  [DEBUG] ⚠️ WARNING: Edge length exceeds 1000\" - possible geometry issue!");
            }

            double currentPos = 0.0;
            int segmentIndex = 0;

            while (segmentIndex < segments.Count - 1)
            {
                var (seg, _) = segments[segmentIndex];
                double segLength = GetSegmentLengthAlongEdge(seg, edgeStart, edgeDirection, isHorizontal);

                RhinoApp.WriteLine($"  [DEBUG] Seg {segmentIndex}: Length={segLength:F2}\", Pos={currentPos:F2}\" → {currentPos + segLength:F2}\"");

                double boundaryPos = currentPos + segLength;

                Point3d rayPoint1 = edgeStart + edgeDirection * (boundaryPos - 0.01);
                double nextSegLength = 0.25;
                if (segmentIndex + 1 < segments.Count)
                {
                    var (nextSeg, _) = segments[segmentIndex + 1];
                    nextSegLength = GetSegmentLengthAlongEdge(nextSeg, edgeStart, edgeDirection, isHorizontal);
                }

                Point3d rayPoint2 = edgeStart + edgeDirection * (boundaryPos + Math.Max(0.5, nextSegLength / 2));

                double depth1 = ShootRayDepth(rayPoint1, perpendicular, panelBoundary);
                double depth2 = ShootRayDepth(rayPoint2, perpendicular, panelBoundary);

                RhinoApp.WriteLine($"    [RAY] At boundary {boundaryPos:F3}\": depth1={depth1:F3}\", depth2={depth2:F3}\", baseline={baselineDepth:F3}\"");

                if (System.Math.Abs(depth1 - baselineDepth) < 0.1 && depth2 > baselineDepth + 0.1)
                {
                    var result = ResolveForwardCutout(
                        segments,
                        edgeStart,
                        edgeDirection,
                        perpendicular,
                        edgeLength,
                        boundaryPos,
                        segLength,
                        depth2,
                        segmentIndex,
                        isHorizontal,
                        panelBoundary,
                        edgeName,
                        currentPos);

                    if (result != null)
                    {
                        detectedFeatures.Add(result.Value.Feature);
                        segmentIndex = result.Value.NextSegmentIndex;
                        currentPos = result.Value.NextPosition;
                        continue;
                    }
                }
                else if (depth1 > baselineDepth + 0.1 && depth2 > baselineDepth + 0.1 && segmentIndex > 0)
                {
                    var result = ResolveInsetCutout(
                        segments,
                        edgeStart,
                        edgeDirection,
                        perpendicular,
                        edgeLength,
                        boundaryPos,
                        segLength,
                        depth1,
                        depth2,
                        segmentIndex,
                        isHorizontal,
                        panelBoundary,
                        edgeName);

                    if (result != null)
                    {
                        detectedFeatures.Add(result.Value.Feature);
                        segmentIndex = result.Value.NextSegmentIndex;
                        currentPos = result.Value.NextPosition;
                        continue;
                    }
                }

                currentPos = boundaryPos;
                segmentIndex++;
            }

            return detectedFeatures;
        }

        private (EdgeFeature Feature, int NextSegmentIndex, double NextPosition)? ResolveForwardCutout(
            List<(Curve seg, int index)> segments,
            Point3d edgeStart,
            Vector3d edgeDirection,
            Vector3d perpendicular,
            double edgeLength,
            double boundaryPos,
            double segLength,
            double initialDepth,
            int segmentIndex,
            bool isHorizontal,
            Curve panelBoundary,
            string edgeName,
            double currentPos)
        {
            const double baselineDepth = 0.001;
            const double scanInterval = 0.03;
            double scanPos = boundaryPos;
            double maxDepth = initialDepth;
            double prevScanPos = scanPos;

            while (scanPos < edgeLength)
            {
                scanPos += scanInterval;
                Point3d scanRayPoint = edgeStart + edgeDirection * scanPos;
                double scanDepth = ShootRayDepth(scanRayPoint, perpendicular, panelBoundary);

                maxDepth = System.Math.Max(maxDepth, scanDepth);

                if (System.Math.Abs(scanDepth - baselineDepth) < 0.1)
                {
                    scanPos = prevScanPos;
                    break;
                }

                prevScanPos = scanPos;
            }

            scanPos = System.Math.Round(scanPos * 16.0) / 16.0;

            var cutoutSegments = new List<int>();
            double cutoutSegmentLength = 0.0;

            for (int i = segmentIndex + 1; i < segments.Count; i++)
            {
                var (cutSeg, _) = segments[i];
                double cutSegLength = GetSegmentLengthAlongEdge(cutSeg, edgeStart, edgeDirection, isHorizontal);
                double cutSegEnd = currentPos + segLength + cutoutSegmentLength + cutSegLength;

                if (cutSegEnd <= scanPos + 0.01)
                {
                    cutoutSegments.Add(i + 1);
                    cutoutSegmentLength += cutSegLength;
                    segLength += cutSegLength;
                }
                else
                {
                    break;
                }
            }

            RhinoApp.WriteLine($"    [CUTOUT DEBUG] Detected at currentPos={currentPos:F3}\", boundaryPos={boundaryPos:F3}\"");
            RhinoApp.WriteLine($"    [CUTOUT DEBUG] scanPos found cutout end at: {scanPos:F3}\"");
            RhinoApp.WriteLine($"    [CUTOUT DEBUG] Segment lengths in cutout:");
            for (int i = segmentIndex + 1; i <= segmentIndex + cutoutSegments.Count && i < segments.Count; i++)
            {
                var (debugSeg, _) = segments[i];
                double len = GetSegmentLengthAlongEdge(debugSeg, edgeStart, edgeDirection, isHorizontal);
                RhinoApp.WriteLine($"      Seg {i}: Length={len:F3}\"");
            }
            RhinoApp.WriteLine($"    [CUTOUT DEBUG] Total cutoutSegmentLength={cutoutSegmentLength:F3}\"");
            RhinoApp.WriteLine($"    [CUTOUT DEBUG] cutoutSegments.Count={cutoutSegments.Count}");

            double cutoutStart = boundaryPos;
            double rayWidth = scanPos - boundaryPos;
            double cutoutWidth;

            if (cutoutSegmentLength > 0.01)
            {
                cutoutWidth = rayWidth > cutoutSegmentLength * 2.0 ? rayWidth : cutoutSegmentLength;
            }
            else
            {
                cutoutWidth = rayWidth;
            }

            double cutoutEnd = cutoutStart + cutoutWidth;

            cutoutStart = System.Math.Round(cutoutStart * 16.0) / 16.0;
            cutoutEnd = System.Math.Round(cutoutEnd * 16.0) / 16.0;
            cutoutWidth = cutoutEnd - cutoutStart;

            RhinoApp.WriteLine($"    [CUTOUT DEBUG] FINAL: start={cutoutStart:F3}\", end={cutoutEnd:F3}\", width={cutoutWidth:F3}\"");

            double depthRounded = System.Math.Round(maxDepth * 16.0) / 16.0;
            string featureType = cutoutWidth <= 2.0 ? "Edge Hole" : "Edge Slot";

            int nextSegmentIndex = segmentIndex + cutoutSegments.Count;
            if (cutoutSegments.Count == 0)
            {
                RhinoApp.WriteLine("    ⚠️ Zero segments detected - forcing +1 advance to prevent infinite loop");
                nextSegmentIndex = segmentIndex + 1;
            }

            return (
                new EdgeFeature(edgeName, cutoutStart, cutoutEnd, cutoutWidth, depthRounded, featureType),
                nextSegmentIndex,
                cutoutEnd);
        }

        private (EdgeFeature Feature, int NextSegmentIndex, double NextPosition)? ResolveInsetCutout(
            List<(Curve seg, int index)> segments,
            Point3d edgeStart,
            Vector3d edgeDirection,
            Vector3d perpendicular,
            double edgeLength,
            double boundaryPos,
            double segLength,
            double depth1,
            double depth2,
            int segmentIndex,
            bool isHorizontal,
            Curve panelBoundary,
            string edgeName)
        {
            const double baselineDepth = 0.001;
            const double scanInterval = 1.0 / 16.0;

            double cutoutStart = boundaryPos;
            double backScanPos = boundaryPos - 0.01;

            while (backScanPos > boundaryPos - segLength)
            {
                Point3d backRayPoint = edgeStart + edgeDirection * backScanPos;
                double backDepth = ShootRayDepth(backRayPoint, perpendicular, panelBoundary);

                if (System.Math.Abs(backDepth - baselineDepth) < 0.1)
                {
                    cutoutStart = backScanPos;
                    break;
                }

                backScanPos -= scanInterval;
            }

            double cutoutEnd = boundaryPos;
            double forwardScanPos = boundaryPos + 0.01;
            double maxDepth = System.Math.Max(depth1, depth2);

            while (forwardScanPos < edgeLength)
            {
                Point3d forwardRayPoint = edgeStart + edgeDirection * forwardScanPos;
                double forwardDepth = ShootRayDepth(forwardRayPoint, perpendicular, panelBoundary);

                maxDepth = System.Math.Max(maxDepth, forwardDepth);

                if (System.Math.Abs(forwardDepth - baselineDepth) < 0.1)
                {
                    cutoutEnd = forwardScanPos;
                    break;
                }

                forwardScanPos += scanInterval;
            }

            cutoutStart = System.Math.Round(cutoutStart * 16.0) / 16.0;
            cutoutEnd = System.Math.Round(cutoutEnd * 16.0) / 16.0;

            double cutoutWidth = cutoutEnd - cutoutStart;
            if (cutoutWidth < 0.01)
            {
                RhinoApp.WriteLine($"    ⚠️ Invalid inset hole: width={cutoutWidth:F3}\" - forcing segment advance");
                return null;
            }

            double depthRounded = System.Math.Round(maxDepth * 16.0) / 16.0;

            var cutoutSegments = new List<int>();

            for (int i = System.Math.Max(0, segmentIndex - 1); i < segments.Count; i++)
            {
                var (cutSeg, _) = segments[i];
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

            string featureType = cutoutWidth <= 2.0 ? "Edge Hole" : "Edge Slot";

            int nextSegmentIndex = segmentIndex + cutoutSegments.Count;
            if (cutoutSegments.Count == 0)
            {
                RhinoApp.WriteLine("    ⚠️ Zero segments detected - forcing +1 advance to prevent infinite loop");
                nextSegmentIndex = segmentIndex + 1;
            }

            return (
                new EdgeFeature(edgeName, cutoutStart, cutoutEnd, cutoutWidth, depthRounded, featureType),
                nextSegmentIndex,
                cutoutEnd);
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
                    perpendicular = Vector3d.YAxis;
                    break;
                case "RIGHT":
                    edgeStart = bottomRight;
                    edgeEnd = topRight;
                    edgeDirection = Vector3d.YAxis;
                    perpendicular = -Vector3d.XAxis;
                    break;
                case "TOP":
                    edgeStart = topRight;
                    edgeEnd = topLeft;
                    edgeDirection = -Vector3d.XAxis;
                    perpendicular = -Vector3d.YAxis;
                    break;
                case "LEFT":
                    edgeStart = topLeft;
                    edgeEnd = bottomLeft;
                    edgeDirection = -Vector3d.YAxis;
                    perpendicular = Vector3d.XAxis;
                    break;
                default:
                    throw new ArgumentException($"Unknown edge name: {edgeName}");
            }

            edgeDirection.Unitize();
            perpendicular.Unitize();
        }

        private double GetSegmentLengthAlongEdge(Curve segment, Point3d edgeStart, Vector3d edgeDirection, bool isHorizontal)
        {
            Point3d segStart = segment.PointAtStart;
            Point3d segEnd = segment.PointAtEnd;

            return isHorizontal
                ? Math.Abs(segEnd.X - segStart.X)
                : Math.Abs(segEnd.Y - segStart.Y);
        }

        private double ShootRayDepth(Point3d origin, Vector3d direction, Curve panelBoundary)
        {
            Point3d rayStart = origin + direction * -0.001;
            Point3d rayEnd = rayStart + direction * 20.0;
            var rayCurve = new LineCurve(rayStart, rayEnd);

            var intersections = Intersection.CurveCurve(panelBoundary, rayCurve, 0.001, 0.001);

            if (intersections != null && intersections.Count > 0)
            {
                const double baselineDepth = 0.001;
                var depths = new List<double>();
                foreach (var intersection in intersections)
                {
                    depths.Add(rayStart.DistanceTo(intersection.PointA));
                }

                depths.Sort();
                return depths.Count > 0 ? depths[0] : baselineDepth;
            }

            return double.MaxValue;
        }
    }
}
