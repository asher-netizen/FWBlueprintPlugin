using System;
using System.Collections.Generic;
using System.Linq;
using Rhino.Geometry;
using FWBlueprintPlugin;
using FWBlueprintPlugin.Services;

namespace FWBlueprintPlugin.Services.Phase3
{
    /// <summary>
    /// Edge detection entry point for the Version 2 scanner. Legacy version 1 lives in <see cref="EdgeFeatureDetectionServiceV1"/>.
    /// </summary>
    internal class EdgeFeatureDetectionService
    {
        private readonly EdgeDimensioningService _edgeDimensioningService;
        private const double HoleWidthThreshold = 2.0625;
        private const double RoundingIncrement = 1.0 / 16.0;
        private const double MinimumFeatureWidth = 0.1;
        private const double MinimumFeatureDepth = 0.05;

        public EdgeFeatureDetectionService(EdgeDimensioningService edgeDimensioningService)
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
            _ = dimensionsLayerIndex;
            _ = panelBoundary;

            if (!bbox.IsValid || panelBoundary == null || segmentData == null || panelBrep == null)
            {
                return new List<EdgeFeature>();
            }

            var panelBrepBox = panelBrep.GetBoundingBox(true);
            double brepThickness = panelBrepBox.IsValid ? panelBrepBox.Max.Z - panelBrepBox.Min.Z : 0.0;
            double effectiveThickness = panelThickness > 0 ? panelThickness : brepThickness;
            if (effectiveThickness <= 0 && brepThickness > 0)
            {
                effectiveThickness = brepThickness;
            }

            var analyzer = new EdgeDetectionV2Analyzer(
                bbox,
                panelBrep,
                panelBrepBox,
                effectiveThickness,
                tol);

            var summaries = analyzer.Analyze(segmentData);

            var detectedFeatures = summaries.SelectMany(s => s.Cutouts).ToList();

            if (detectedFeatures.Count > 0)
            {
                _edgeDimensioningService.AddEdgeFeatureDimensions(bbox, detectedFeatures, dimensionsLayerIndex);
            }

            return detectedFeatures;
        }

        #region Analyzer helpers

        private sealed class EdgeDetectionV2Analyzer
        {
            private readonly Rectangle3d _bbox;
            private readonly Brep _panelBrep;
            private readonly BoundingBox _panelBrepBox;
            private readonly double _panelThickness;
            private readonly double _tol;
            private const double BaselineTolerance = 0.01;

            public EdgeDetectionV2Analyzer(
                Rectangle3d bbox,
                Brep panelBrep,
                BoundingBox panelBrepBox,
                double panelThickness,
                double tol)
            {
                _bbox = bbox;
                _panelBrep = panelBrep;
                _panelBrepBox = panelBrepBox;
                _panelThickness = panelThickness;
                _tol = tol;
            }

            public List<EdgeCutoutSummary> Analyze(EdgeSegmentData data)
            {
                var summaries = new List<EdgeCutoutSummary>
                {
                    AnalyzeEdge("BOTTOM", data.BottomSegments),
                    AnalyzeEdge("RIGHT", data.RightSegments),
                    AnalyzeEdge("TOP", data.TopSegments),
                    AnalyzeEdge("LEFT", data.LeftSegments)
                };

                return summaries;
            }

            private EdgeCutoutSummary AnalyzeEdge(string edgeName, List<(Curve seg, int index)> segments)
            {
                var frame = EdgeFrame.Create(_bbox, edgeName);
                var summary = new EdgeCutoutSummary(edgeName, frame.Length);

                if (segments == null || segments.Count == 0)
                {
                    summary.Gaps.Add(Math.Round(frame.Length, 4));
                    return summary;
                }

                var ordered = segments.OrderBy(s => s.index).ToList();
                var candidates = DetectCandidates(ordered, frame);
                var features = BuildFeatures(frame, candidates);

                summary.Cutouts.AddRange(features);
                summary.CalculateGaps();

                return summary;
            }

            private List<EdgeCutoutCandidate> DetectCandidates(List<(Curve seg, int index)> segments, EdgeFrame frame)
            {
                var candidates = new List<EdgeCutoutCandidate>();
                bool insideCutout = false;
                double candidateStart = 0;
                double maxDepth = 0;
                bool hasCurvature = false;

                foreach (var (seg, _) in segments)
                {
                    var sample = AnalyzeSegment(seg, frame);

                    if (!insideCutout && IsStandaloneBulge(sample))
                    {
                        var bulgeCandidate = CreateCandidate(sample.StartAxis, sample.EndAxis, sample.MaxPerp, true);
                        if (bulgeCandidate != null)
                        {
                            candidates.Add(bulgeCandidate);
                        }
                        continue;
                    }

                    if (!insideCutout && !sample.StartOnBaseline)
                    {
                        insideCutout = true;
                        candidateStart = sample.StartAxis;
                        maxDepth = Math.Abs(sample.StartPerp);
                        hasCurvature = sample.IsCurved;
                    }

                    if (!insideCutout && sample.StartOnBaseline && !sample.EndOnBaseline)
                    {
                        insideCutout = true;
                        candidateStart = sample.StartAxis;
                        maxDepth = 0;
                        hasCurvature = sample.IsCurved;
                    }

                    if (insideCutout)
                    {
                        maxDepth = Math.Max(maxDepth, sample.MaxPerp);
                        hasCurvature |= sample.IsCurved;

                        if (sample.EndOnBaseline)
                        {
                            var candidate = CreateCandidate(candidateStart, sample.EndAxis, maxDepth, hasCurvature);
                            if (candidate != null)
                            {
                                candidates.Add(candidate);
                            }

                            insideCutout = false;
                            maxDepth = 0;
                            hasCurvature = false;
                        }
                    }
                }

                if (insideCutout)
                {
                    var candidate = CreateCandidate(candidateStart, frame.Length, maxDepth, hasCurvature);
                    if (candidate != null)
                    {
                        candidates.Add(candidate);
                    }
                }

                return candidates;
            }

            private bool IsStandaloneBulge(EdgeSegmentSample sample)
            {
                return sample.IsCurved &&
                       sample.StartOnBaseline &&
                       sample.EndOnBaseline &&
                       sample.MaxPerp >= MinimumFeatureDepth;
            }

            private EdgeCutoutCandidate CreateCandidate(double start, double end, double depth, bool hasCurvature)
            {
                double width = Math.Abs(end - start);
                if (width < MinimumFeatureWidth || depth < MinimumFeatureDepth)
                {
                    return null;
                }

                double actualStart = Math.Min(start, end);
                double actualEnd = Math.Max(start, end);

                return new EdgeCutoutCandidate
                {
                    Start = actualStart,
                    End = actualEnd,
                    Depth = depth,
                    HasCurvature = hasCurvature
                };
            }

            private EdgeSegmentSample AnalyzeSegment(Curve segment, EdgeFrame frame)
            {
                var startPoint = segment.PointAtStart;
                var endPoint = segment.PointAtEnd;
                bool isCurved = !segment.IsLinear(_tol);

                double startAxis = frame.ClampAxis(frame.ProjectToAxis(startPoint));
                double endAxis = frame.ClampAxis(frame.ProjectToAxis(endPoint));
                double startPerp = frame.ProjectToPerp(startPoint);
                double endPerp = frame.ProjectToPerp(endPoint);
                double maxPerp = Math.Max(Math.Abs(startPerp), Math.Abs(endPerp));

                int samples = isCurved ? 8 : 4;
                for (int i = 1; i < samples - 1; i++)
                {
                    double normalized = (double)i / (samples - 1);
                    double t = segment.Domain.ParameterAt(normalized);
                    var pt = segment.PointAt(t);
                    double perp = Math.Abs(frame.ProjectToPerp(pt));
                    if (perp > maxPerp)
                    {
                        maxPerp = perp;
                    }
                }

                return new EdgeSegmentSample
                {
                    StartAxis = startAxis,
                    EndAxis = endAxis,
                    StartPerp = startPerp,
                    EndPerp = endPerp,
                    MaxPerp = maxPerp,
                    IsCurved = isCurved,
                    BaselineTolerance = BaselineTolerance
                };
            }

            private List<EdgeFeature> BuildFeatures(EdgeFrame frame, List<EdgeCutoutCandidate> candidates)
            {
                var features = new List<EdgeFeature>();

                foreach (var candidate in candidates)
                {
                    if (!IsThroughCutout(candidate, frame))
                    {
                        continue;
                    }

                    double start = Round(candidate.Start);
                    double end = Round(candidate.End);
                    double width = Round(end - start);
                    double depth = Round(candidate.Depth);
                    string type = DetermineType(candidate);
                    bool hasCurvature = candidate.HasCurvature;

                    var feature = new EdgeFeature(frame.EdgeName, start, end, width, depth, type, hasCurvature);
                    feature.EdgeLength = frame.Length;
                    features.Add(feature);
                }

                features = features.OrderBy(f => f.StartPos).ToList();
                AssignGapData(features, frame.Length);

                return features;
            }

            private void AssignGapData(List<EdgeFeature> features, double edgeLength)
            {
                if (features.Count == 0)
                {
                    return;
                }

                for (int i = 0; i < features.Count; i++)
                {
                    double gapBefore = i == 0
                        ? Math.Max(0, features[i].StartPos)
                        : Math.Max(0, features[i].StartPos - features[i - 1].EndPos);

                    double gapAfter = i == features.Count - 1
                        ? Math.Max(0, edgeLength - features[i].EndPos)
                        : Math.Max(0, features[i + 1].StartPos - features[i].EndPos);

                    features[i].GapBefore = Round(gapBefore);
                    features[i].GapAfter = Round(gapAfter);
                }
            }

            private bool IsThroughCutout(EdgeCutoutCandidate candidate, EdgeFrame frame)
            {
                if (_panelBrep == null || !_panelBrepBox.IsValid)
                {
                    return true;
                }

                double axisMid = (candidate.Start + candidate.End) / 2.0;
                double depthProbe = Math.Max(candidate.Depth * 0.5, 0.01);
                depthProbe = Math.Min(depthProbe, candidate.Depth - 0.005);
                if (depthProbe <= 0)
                {
                    depthProbe = candidate.Depth * 0.5;
                }

                var sample2D = frame.PointAt(axisMid, depthProbe);
                double zMargin = Math.Min(Math.Max(_panelThickness * 0.1, 0.01), 0.05);

                var topPoint = new Point3d(sample2D.X, sample2D.Y, _panelBrepBox.Max.Z - zMargin);
                var bottomPoint = new Point3d(sample2D.X, sample2D.Y, _panelBrepBox.Min.Z + zMargin);

                bool topInside;
                bool bottomInside;

                try
                {
                    topInside = _panelBrep.IsPointInside(topPoint, _tol, true);
                    bottomInside = _panelBrep.IsPointInside(bottomPoint, _tol, true);
                }
                catch
                {
                    return true;
                }

                // Through cutouts will be outside the Brep at both probes.
                return !topInside && !bottomInside;
            }

            private static double Round(double value)
            {
                return Math.Round(value / RoundingIncrement) * RoundingIncrement;
            }

            private string DetermineType(EdgeCutoutCandidate candidate)
            {
                if (!candidate.HasCurvature)
                {
                    return "EdgeNotch";
                }

                return (candidate.End - candidate.Start) <= HoleWidthThreshold
                    ? "EdgeCordHole"
                    : "EdgeCordSlot";
            }
        }

        private sealed class EdgeCutoutCandidate
        {
            public double Start { get; set; }
            public double End { get; set; }
            public double Depth { get; set; }
            public bool HasCurvature { get; set; }
        }

        private sealed class EdgeSegmentSample
        {
            public double StartAxis { get; set; }
            public double EndAxis { get; set; }
            public double StartPerp { get; set; }
            public double EndPerp { get; set; }
            public double MaxPerp { get; set; }
            public bool IsCurved { get; set; }
            public double BaselineTolerance { get; set; }

            public bool StartOnBaseline => Math.Abs(StartPerp) <= BaselineTolerance;
            public bool EndOnBaseline => Math.Abs(EndPerp) <= BaselineTolerance;
        }

        private sealed class EdgeFrame
        {
            public string EdgeName { get; }
            public Point3d Start { get; }
            public Point3d End { get; }
            public Vector3d Direction { get; }
            public Vector3d Perpendicular { get; }
            public double Length { get; }

            private EdgeFrame(string edgeName, Point3d start, Point3d end, Vector3d direction, Vector3d perpendicular)
            {
                EdgeName = edgeName;
                Start = start;
                End = end;
                Direction = direction;
                Perpendicular = perpendicular;
                Length = start.DistanceTo(end);
            }

            public static EdgeFrame Create(Rectangle3d bbox, string edgeName)
            {
                Point3d bottomLeft = bbox.Corner(0);
                Point3d bottomRight = bbox.Corner(1);
                Point3d topRight = bbox.Corner(2);
                Point3d topLeft = bbox.Corner(3);

                switch (edgeName)
                {
                    case "BOTTOM":
                        return new EdgeFrame(edgeName, bottomLeft, bottomRight, Vector3d.XAxis, Vector3d.YAxis);
                    case "RIGHT":
                        return new EdgeFrame(edgeName, bottomRight, topRight, Vector3d.YAxis, -Vector3d.XAxis);
                    case "TOP":
                        return new EdgeFrame(edgeName, topLeft, topRight, Vector3d.XAxis, -Vector3d.YAxis);
                    case "LEFT":
                        return new EdgeFrame(edgeName, bottomLeft, topLeft, Vector3d.YAxis, Vector3d.XAxis);
                    default:
                        throw new ArgumentException($"Unknown edge name: {edgeName}");
                }
            }

            public double ProjectToAxis(Point3d point)
            {
                var vector = point - Start;
                return vector * Direction;
            }

            public double ProjectToPerp(Point3d point)
            {
                var vector = point - Start;
                return vector * Perpendicular;
            }

            public Point3d PointAt(double axis, double perp)
            {
                return Start + Direction * axis + Perpendicular * perp;
            }

            public double ClampAxis(double axis)
            {
                if (axis < 0)
                {
                    return 0;
                }

                if (axis > Length)
                {
                    return Length;
                }

                return axis;
            }
        }

        private sealed class EdgeCutoutSummary
        {
            public EdgeCutoutSummary(string edgeName, double totalLength)
            {
                EdgeName = edgeName;
                TotalLength = totalLength;
            }

            public string EdgeName { get; }
            public double TotalLength { get; }
            public List<EdgeFeature> Cutouts { get; } = new List<EdgeFeature>();
            public List<double> Gaps { get; } = new List<double>();

            public void CalculateGaps()
            {
                Gaps.Clear();

                if (Cutouts.Count == 0)
                {
                    Gaps.Add(Math.Round(TotalLength, 4));
                    return;
                }

                double firstGap = Math.Max(0, Cutouts[0].StartPos);
                Gaps.Add(Math.Round(firstGap, 4));

                for (int i = 1; i < Cutouts.Count; i++)
                {
                    double gap = Math.Max(0, Cutouts[i].StartPos - Cutouts[i - 1].EndPos);
                    Gaps.Add(Math.Round(gap, 4));
                }

                double finalGap = Math.Max(0, TotalLength - Cutouts[Cutouts.Count - 1].EndPos);
                Gaps.Add(Math.Round(finalGap, 4));
            }
        }

        #endregion
    }
}
