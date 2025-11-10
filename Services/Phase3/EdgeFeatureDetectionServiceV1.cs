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
    /// Edge detection system version 1 (ray protocol) that detects cutouts and forwards them to edge dimensioning.
    /// </summary>
    internal class EdgeFeatureDetectionServiceV1
    {
        private readonly EdgeDimensioningService _edgeDimensioningService;
        private const double HoleWidthThreshold = 2.0625;

        public EdgeFeatureDetectionServiceV1(EdgeDimensioningService edgeDimensioningService)
        {
            _edgeDimensioningService = edgeDimensioningService ?? throw new ArgumentNullException(nameof(edgeDimensioningService));
        }

        public List<EdgeFeature> DimensionEdgeFeatures(
            Rectangle3d bbox,
            Curve panelBoundary,
            EdgeSegmentData segmentData,
            Brep panelBrep,
            double panelThickness,
            double tol,
            int dimensionsLayerIndex,
            string panelDisplayName)
        {
            if (segmentData == null)
            {
                return new List<EdgeFeature>();
            }

            _ = panelBrep; // Unused in ray protocol implementation but kept for compatibility
            _ = panelThickness;
            _ = tol;

            var allEdgeFeatures = new List<EdgeFeature>();

            allEdgeFeatures.AddRange(ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.BottomSegments, "BOTTOM", tol, panelDisplayName));
            allEdgeFeatures.AddRange(ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.RightSegments, "RIGHT", tol, panelDisplayName));
            allEdgeFeatures.AddRange(ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.TopSegments, "TOP", tol, panelDisplayName));
            allEdgeFeatures.AddRange(ProcessEdge_RayProtocol(bbox, panelBoundary, segmentData.LeftSegments, "LEFT", tol, panelDisplayName));

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
            string edgeName,
            double tol,
            string panelDisplayName)
        {
            var detectedFeatures = new List<EdgeFeature>();
            if (segments == null || segments.Count <= 1)
            {
                return detectedFeatures;
            }

            string panelPrefix = string.IsNullOrEmpty(panelDisplayName) ? string.Empty : $"[{panelDisplayName}] ";

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

            RhinoApp.WriteLine($"  [DEBUG] {panelPrefix}[{edgeName}] Edge length: {edgeLength:F2}\", Segments: {segments.Count}");
            if (edgeLength > 1000)
            {
                RhinoApp.WriteLine($"  [DEBUG] {panelPrefix}⚠️ WARNING: Edge length exceeds 1000\" - possible geometry issue!");
            }

            double currentPos = 0.0;
            int segmentIndex = 0;

            while (segmentIndex < segments.Count - 1)
            {
                var (seg, _) = segments[segmentIndex];
                double segLength = GetSegmentLengthAlongEdge(seg, edgeStart, edgeDirection, isHorizontal);

                RhinoApp.WriteLine($"  [DEBUG] {panelPrefix}Seg {segmentIndex + 1}: Length={segLength:F2}\", Pos={currentPos:F2}\" → {currentPos + segLength:F2}\"");

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

                RhinoApp.WriteLine($"    [RAY] {panelPrefix}At boundary {boundaryPos:F3}\": depth1={depth1:F3}\", depth2={depth2:F3}\", baseline={baselineDepth:F3}\"");

                bool isZeroLengthSegment = segLength < 0.001;

                if (System.Math.Abs(depth1 - baselineDepth) < 0.1 && depth2 > baselineDepth + 0.1 && !isZeroLengthSegment)
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
                        currentPos,
                        tol,
                        panelPrefix);

                    if (result != null)
                    {
                        detectedFeatures.Add(result.Value.Feature);
                        segmentIndex = result.Value.NextSegmentIndex;
                        currentPos = result.Value.NextPosition;
                        continue;
                    }
                }
                else if ((depth1 > baselineDepth + 0.1 && depth2 > baselineDepth + 0.1 && segmentIndex > 0) ||
                         (isZeroLengthSegment && depth2 > baselineDepth + 0.1))
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
                        edgeName,
                        tol,
                        panelPrefix);

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
            double currentPos,
            double tol,
            string panelPrefix)
        {
            const double baselineDepth = 0.001;
            const double scanInterval = 0.03;
            double scanPos = boundaryPos;
            double maxDepth = initialDepth;
            double prevScanPos = scanPos;
            bool seenInteriorDepth = (initialDepth > baselineDepth + 0.1);
            double minAdvanceBeforeClosure = EstimateForwardCutoutMinAdvance(
                segments,
                segmentIndex,
                edgeStart,
                edgeDirection,
                isHorizontal);

            RhinoApp.WriteLine($"    [SCAN START] {panelPrefix}initialDepth={initialDepth:F3}\", seenInteriorDepth={seenInteriorDepth}");

            while (scanPos < edgeLength)
            {
                scanPos += scanInterval;
                Point3d scanRayPoint = edgeStart + edgeDirection * scanPos;
                double scanDepth = ShootRayDepth(scanRayPoint, perpendicular, panelBoundary);

                maxDepth = System.Math.Max(maxDepth, scanDepth);

                if (scanDepth > baselineDepth + 0.1)
                {
                    seenInteriorDepth = true;
                }

                if (seenInteriorDepth && System.Math.Abs(scanDepth - baselineDepth) < 0.1)
                {
                    if (scanPos - boundaryPos < minAdvanceBeforeClosure)
                    {
                        prevScanPos = scanPos;
                        continue;
                    }

                    RhinoApp.WriteLine($"    [SCAN TERMINATE] {panelPrefix}at scanPos={scanPos:F3}\", scanDepth={scanDepth:F3}\"");
                    scanPos = prevScanPos;
                    break;
                }

                prevScanPos = scanPos;
            }

            RhinoApp.WriteLine($"    [SCAN COMPLETE] {panelPrefix}finalScanPos={scanPos:F3}\", maxDepth={maxDepth:F3}\"");

            scanPos = System.Math.Round(scanPos * 16.0) / 16.0;

            var cutoutSegments = new List<int>();
            double cutoutSegmentLength = 0.0;

            for (int i = segmentIndex + 1; i < segments.Count; i++)
            {
                var (cutSeg, _) = segments[i];
                double cutSegLength = GetSegmentLengthAlongEdge(cutSeg, edgeStart, edgeDirection, isHorizontal);
                double cutSegEnd = currentPos + segLength + cutoutSegmentLength + cutSegLength;

                if (cutSegEnd <= scanPos + 0.05)
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

            RhinoApp.WriteLine($"    [CUTOUT DEBUG] {panelPrefix}Detected at currentPos={currentPos:F3}\", boundaryPos={boundaryPos:F3}\"");
            RhinoApp.WriteLine($"    [CUTOUT DEBUG] {panelPrefix}scanPos found cutout end at: {scanPos:F3}\"");
            RhinoApp.WriteLine($"    [CUTOUT DEBUG] {panelPrefix}Segment lengths in cutout:");
            for (int i = segmentIndex + 1; i <= segmentIndex + cutoutSegments.Count && i < segments.Count; i++)
            {
                var (debugSeg, _) = segments[i];
                double len = GetSegmentLengthAlongEdge(debugSeg, edgeStart, edgeDirection, isHorizontal);
                RhinoApp.WriteLine($"      {panelPrefix}Seg {i + 1}: Length={len:F3}\"");
            }
            RhinoApp.WriteLine($"    [CUTOUT DEBUG] {panelPrefix}Total cutoutSegmentLength={cutoutSegmentLength:F3}\"");
            RhinoApp.WriteLine($"    [CUTOUT DEBUG] {panelPrefix}cutoutSegments.Count={cutoutSegments.Count}");

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

            RhinoApp.WriteLine($"    [CUTOUT DEBUG] {panelPrefix}FINAL: start={cutoutStart:F3}\", end={cutoutEnd:F3}\", width={cutoutWidth:F3}\"");

            double depthRounded = System.Math.Round(maxDepth * 16.0) / 16.0;
            bool hasCurvature = HasCurvatureInSpan(segments, edgeStart, edgeDirection, isHorizontal, cutoutStart, cutoutEnd, tol);

            // Curvature-aware classification: prefer Edge Hole when a near-semicircular arc ~2" diameter is detected.
            string featureType;
            double? inferredHoleDiameter = hasCurvature
                ? EstimateHoleDiameterFromCurvature(segments, edgeStart, edgeDirection, isHorizontal, cutoutStart, cutoutEnd, tol)
                : (double?)null;

            if (inferredHoleDiameter.HasValue)
            {
                featureType = "Edge Hole";
                // normalize width to inferred diameter to avoid small overshoot artifacts
                cutoutEnd = cutoutStart + inferredHoleDiameter.Value;
                cutoutEnd = System.Math.Round(cutoutEnd * 16.0) / 16.0;
                cutoutWidth = cutoutEnd - cutoutStart;
            }
            else
            {
                featureType = !hasCurvature
                    ? "Edge Notch"
                    : (cutoutWidth <= HoleWidthThreshold ? "Edge Hole" : "Edge Slot");
            }

            int nextSegmentIndex = segmentIndex + cutoutSegments.Count;
            if (cutoutSegments.Count == 0)
            {
                RhinoApp.WriteLine($"    ⚠️ {panelPrefix}Zero segments detected - forcing +1 advance to prevent infinite loop");
                nextSegmentIndex = segmentIndex + 1;
            }

            return (
                new EdgeFeature(edgeName, cutoutStart, cutoutEnd, cutoutWidth, depthRounded, featureType, hasCurvature),
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
            string edgeName,
            double tol,
            string panelPrefix)
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
                RhinoApp.WriteLine($"    ⚠️ {panelPrefix}Invalid inset hole: width={cutoutWidth:F3}\" - forcing segment advance");
                return null;
            }

            double depthRounded = System.Math.Round(maxDepth * 16.0) / 16.0;
            bool hasCurvature = HasCurvatureInSpan(segments, edgeStart, edgeDirection, isHorizontal, cutoutStart, cutoutEnd, tol);

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

            string featureType;
            double? inferredHoleDiameter = hasCurvature
                ? EstimateHoleDiameterFromCurvature(segments, edgeStart, edgeDirection, isHorizontal, cutoutStart, cutoutEnd, tol)
                : (double?)null;

            if (inferredHoleDiameter.HasValue)
            {
                featureType = "Edge Hole";
                cutoutEnd = cutoutStart + inferredHoleDiameter.Value;
                cutoutEnd = System.Math.Round(cutoutEnd * 16.0) / 16.0;
                cutoutWidth = cutoutEnd - cutoutStart;
            }
            else
            {
                featureType = !hasCurvature
                    ? "Edge Notch"
                    : (cutoutWidth <= HoleWidthThreshold ? "Edge Hole" : "Edge Slot");
            }

            int nextSegmentIndex = segmentIndex + cutoutSegments.Count;
            if (cutoutSegments.Count == 0)
            {
                RhinoApp.WriteLine($"    ⚠️ {panelPrefix}Zero segments detected - forcing +1 advance to prevent infinite loop");
                nextSegmentIndex = segmentIndex + 1;
            }

            return (
                new EdgeFeature(edgeName, cutoutStart, cutoutEnd, cutoutWidth, depthRounded, featureType, hasCurvature),
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
            return segment.GetLength();
        }

        private double EstimateForwardCutoutMinAdvance(
            List<(Curve seg, int index)> segments,
            int segmentIndex,
            Point3d edgeStart,
            Vector3d edgeDirection,
            bool isHorizontal)
        {
            double firstMeaningfulLength = 0.0;

            for (int i = segmentIndex + 1; i < segments.Count; i++)
            {
                var (candidateSeg, _) = segments[i];
                double len = GetSegmentLengthAlongEdge(candidateSeg, edgeStart, edgeDirection, isHorizontal);

                if (len < 0.005)
                {
                    continue;
                }

                firstMeaningfulLength = len;
                break;
            }

            if (firstMeaningfulLength <= 0.0)
            {
                return 0.1;
            }

            double minAdvance = System.Math.Max(firstMeaningfulLength * 0.5, 0.1);
            return System.Math.Min(minAdvance, 1.0);
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
                const double parallelTolerance = 0.98; // cosine threshold for near-parallel

                var candidateDepths = new List<double>();
                double? fallbackDepth = null;

                foreach (var intersection in intersections)
                {
                    double depth = rayStart.DistanceTo(intersection.PointA);
                    Vector3d tangent = Vector3d.Unset;

                    try
                    {
                        tangent = panelBoundary.TangentAt(intersection.ParameterA);
                    }
                    catch
                    {
                        tangent = Vector3d.Unset;
                    }

                    if (tangent.IsValid && tangent.SquareLength > 0)
                    {
                        tangent.Unitize();
                        double alignment = System.Math.Abs(Vector3d.Multiply(tangent, direction));
                        if (alignment >= parallelTolerance)
                        {
                            // Ray is nearly parallel to the boundary segment (e.g. slot wall); skip but remember depth
                            fallbackDepth = fallbackDepth.HasValue
                                ? System.Math.Min(fallbackDepth.Value, depth)
                                : depth;
                            continue;
                        }
                    }

                    candidateDepths.Add(depth);
                }

                if (candidateDepths.Count > 0)
                {
                    candidateDepths.Sort();
                    return candidateDepths[0];
                }

                if (fallbackDepth.HasValue)
                {
                    return fallbackDepth.Value;
                }

                return baselineDepth;
            }

            return double.MaxValue;
        }

        private bool HasCurvatureInSpan(
            List<(Curve seg, int index)> segments,
            Point3d edgeStart,
            Vector3d edgeDirection,
            bool isHorizontal,
            double spanStart,
            double spanEnd,
            double tol)
        {
            if (segments == null || segments.Count == 0)
            {
                return false;
            }

            double cumulative = 0.0;
            foreach (var (segment, _) in segments)
            {
                double length = GetSegmentLengthAlongEdge(segment, edgeStart, edgeDirection, isHorizontal);
                double segStart = cumulative;
                double segEnd = cumulative + length;

                bool overlaps = spanEnd > segStart - 0.001 && spanStart < segEnd + 0.001;
                if (overlaps && !segment.IsLinear(tol))
                {
                    return true;
                }

                cumulative = segEnd;
            }

            return false;
        }

        private double? EstimateHoleDiameterFromCurvature(
            List<(Curve seg, int index)> segments,
            Point3d edgeStart,
            Vector3d edgeDirection,
            bool isHorizontal,
            double spanStart,
            double spanEnd,
            double tol)
        {
            // Typical cord hole: semicircular arc of ~pi inches arc length (≈3.14")
            const double minAngle = 2.6;  // ~149°
            const double maxAngle = 3.6;  // ~206°
            const double minDiam = 1.7;   // generous lower bound
            const double maxDiam = 2.3;   // generous upper bound

            double cumulative = 0.0;
            foreach (var (segment, _) in segments)
            {
                double length = GetSegmentLengthAlongEdge(segment, edgeStart, edgeDirection, isHorizontal);
                double segStart = cumulative;
                double segEnd = cumulative + length;

                bool overlaps = spanEnd > segStart - 0.001 && spanStart < segEnd + 0.001;
                if (!overlaps)
                {
                    cumulative = segEnd;
                    continue;
                }

                if (segment.IsLinear(tol))
                {
                    cumulative = segEnd;
                    continue;
                }

                // Try to extract an arc description
                if (TryGetArcFromCurve(segment, out Arc arc))
                {
                    // Arc.Angle in RhinoCommon provides the sweep (radians)
                    double angle = System.Math.Abs(arc.Angle);
                    double diameter = 2.0 * arc.Radius;
                    if (angle >= minAngle && angle <= maxAngle && diameter >= minDiam && diameter <= maxDiam)
                    {
                        return System.Math.Round(diameter * 16.0) / 16.0;
                    }
                }
                else
                {
                    // Fall back: approximate diameter from curved length assuming near semicircle
                    double curvedLen = segment.GetLength();
                    double diameter = (curvedLen / System.Math.PI) * 2.0; // if arc were ~pi radians, len ≈ r*angle; for pi, r = len/pi; d = 2r
                    if (diameter >= minDiam && diameter <= maxDiam)
                    {
                        return System.Math.Round(diameter * 16.0) / 16.0;
                    }
                }

                cumulative = segEnd;
            }

            return null;
        }

        private bool TryGetArcFromCurve(Curve c, out Arc arc)
        {
            arc = Arc.Unset;
            try
            {
                if (c is ArcCurve ac && ac.IsArc())
                {
                    arc = ac.Arc;
                    return true;
                }
            }
            catch { }

            try
            {
                return c.TryGetArc(out arc);
            }
            catch
            {
                arc = Arc.Unset;
                return false;
            }
        }
    }
}
