using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using static FWBlueprintPlugin.Services.DimensionFormatting;

namespace FWBlueprintPlugin.Services
{
    /// <summary>
    /// Handles creation of edge feature dimension chains for detected cutouts.
    /// </summary>
    internal class EdgeDimensioningService
    {
        private readonly RhinoDoc _doc;

        public EdgeDimensioningService(RhinoDoc doc)
        {
            _doc = doc ?? throw new ArgumentNullException(nameof(doc));
        }

        private static readonly bool VerboseLogging = false;

        public void AddEdgeFeatureDimensions(Rectangle3d panelBBox, IList<EdgeFeature> features, int dimensionsLayerIndex)
        {
            if (features == null || features.Count == 0)
            {
                if (VerboseLogging)
                {
                    RhinoApp.WriteLine("  No edge features to dimension");
                }
                return;
            }

            if (VerboseLogging)
            {
                RhinoApp.WriteLine(string.Empty);
                RhinoApp.WriteLine("=== Adding Edge Feature Dimensions ===");
            }

            var groupedByEdge = features
                .GroupBy(f => f.EdgeName)
                .OrderBy(g => g.Key, StringComparer.OrdinalIgnoreCase);

            foreach (var group in groupedByEdge)
            {
                string edgeName = group.Key;
                var orderedFeatures = group.OrderBy(f => f.StartPos).ToList();

                if (VerboseLogging)
                {
                    RhinoApp.WriteLine($"[{edgeName}] {orderedFeatures.Count} feature(s)");
                    foreach (var feature in orderedFeatures)
                    {
                        RhinoApp.WriteLine(
                            $"    • {feature.Type} | start={feature.StartPos:F3}\" end={feature.EndPos:F3}\" width={feature.Width:F3}\" depth={feature.Depth:F3}\" curved={feature.HasCurvature}");
                    }

                    RhinoApp.WriteLine($"\n[{edgeName}] Dimensioning {orderedFeatures.Count} feature(s)...");
                }

                var (edgeStart, edgeEnd, edgeDirection, perpendicular) = GetEdgeParameters(panelBBox, edgeName);
                double edgeLength = edgeStart.DistanceTo(edgeEnd);

                var notchFeatures = orderedFeatures.Where(IsNotchFeature).ToList();
                var nonNotchFeatures = orderedFeatures.Where(f => !IsNotchFeature(f)).ToList();
                var holeFeatures = nonNotchFeatures.Where(IsHoleFeature).ToList();
                var slotFeatures = nonNotchFeatures.Where(IsSlotFeature).ToList();

                if (nonNotchFeatures.Count > 0)
                {
                    bool hasHoles = holeFeatures.Count > 0;
                    bool hasSlots = slotFeatures.Count > 0;

                    if (hasHoles && !hasSlots)
                    {
                        AddEdgeHoleDimensions(panelBBox, holeFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                    }
                    else if (hasSlots && !hasHoles)
                    {
                        AddEdgeSlotDimensions(panelBBox, slotFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                    }
                    else if (hasSlots && hasHoles)
                    {
                        AddMixedEdgeDimensions(panelBBox, nonNotchFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                    }
                    else
                    {
                        AddEdgeHoleDimensions(panelBBox, nonNotchFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                    }
                }

                if (notchFeatures.Count > 0)
                {
                    AddEdgeNotchDimensions(panelBBox, notchFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                }
            }

            RhinoApp.WriteLine($"[Edge Dimensions] Completed dimensioning {features.Count} feature(s) across {groupedByEdge.Count()} edge(s).");
        }

        private void AddEdgeHoleDimensions(Rectangle3d bbox, IList<EdgeFeature> holes, string edgeName, double edgeLength, int dimensionsLayerIndex)
        {
            LogVerbose("  Strategy: Edge Holes (dimension to centers, inset onto panel)");

            var (edgeStart, _, edgeDirection, perpendicular) = GetEdgeParameters(bbox, edgeName);

            var segments = new List<double>();
            var segmentLabels = new List<string>();

            double firstCenter = holes[0].CenterPos;
            segments.Add(firstCenter);
            segmentLabels.Add($"Left→C1: {FormatDimension(firstCenter, 5)}");

            for (int i = 0; i < holes.Count - 1; i++)
            {
                double gap = holes[i + 1].CenterPos - holes[i].CenterPos;
                segments.Add(gap);
                segmentLabels.Add($"C{i + 1}→C{i + 2}: {FormatDimension(gap, 5)}");
            }

            double lastCenter = holes[holes.Count - 1].CenterPos;
            double rightMargin = edgeLength - lastCenter;
            segments.Add(rightMargin);
            segmentLabels.Add($"C{holes.Count}→Right: {FormatDimension(rightMargin, 5)}");

            ValidateSegmentSum(edgeLength, segments.Sum());
            CreateRunningDimensionLine(edgeStart, edgeDirection, perpendicular, segments, edgeName, dimensionsLayerIndex);

            if (VerboseLogging)
            {
                LogSegments(segments, edgeLength, segmentLabels);
            }
        }

        private void AddEdgeSlotDimensions(Rectangle3d bbox, IList<EdgeFeature> slots, string edgeName, double edgeLength, int dimensionsLayerIndex)
        {
            LogVerbose("  Strategy: Edge Slots/Notches (running cumulative, offset away from panel)");

            var (edgeStart, _, edgeDirection, perpendicular) = GetEdgeParameters(bbox, edgeName);
            perpendicular = -perpendicular;

            var segments = new List<double>();
            var segmentLabels = new List<string>();

            double leftMargin = slots[0].StartPos;
            segments.Add(leftMargin);
            segmentLabels.Add($"Left margin: {FormatDimension(leftMargin, 5)}");

            for (int i = 0; i < slots.Count; i++)
            {
                segments.Add(slots[i].Width);
                segmentLabels.Add($"Slot {i + 1} width: {FormatDimension(slots[i].Width, 5)}");

                if (i < slots.Count - 1)
                {
                    double gap = slots[i + 1].StartPos - slots[i].EndPos;
                    segments.Add(gap);
                    segmentLabels.Add($"Gap: {FormatDimension(gap, 5)}");
                }
            }

            double rightMargin = edgeLength - slots[slots.Count - 1].EndPos;
            segments.Add(rightMargin);
            segmentLabels.Add($"Right margin: {FormatDimension(rightMargin, 5)}");

            ValidateSegmentSum(edgeLength, segments.Sum());
            CreateRunningDimensionLine(edgeStart, edgeDirection, perpendicular, segments, edgeName, dimensionsLayerIndex);

            if (VerboseLogging)
            {
                LogSegments(segments, edgeLength, segmentLabels);
            }
        }

        private void AddMixedEdgeDimensions(Rectangle3d bbox, IList<EdgeFeature> features, string edgeName, double edgeLength, int dimensionsLayerIndex)
        {
            LogVerbose("  Strategy: Mixed (holes + slots, running cumulative, offset away from panel)");

            bool hasSlots = features.Any(IsSlotFeature);
            if (!hasSlots)
            {
                LogVerbose("  Correcting: Only holes detected, using hole-only strategy");
                AddEdgeHoleDimensions(bbox, features, edgeName, edgeLength, dimensionsLayerIndex);
                return;
            }

            var (edgeStart, _, edgeDirection, perpendicular) = GetEdgeParameters(bbox, edgeName);
            perpendicular = -perpendicular;

            var segments = new List<double>();
            var segmentLabels = new List<string>();
            double currentPos = 0.0;

            foreach (var feature in features)
            {
                if (IsHoleFeature(feature))
                {
                    double distanceToCenter = feature.CenterPos - currentPos;
                    segments.Add(distanceToCenter);
                    segmentLabels.Add($"→Hole center: {FormatDimension(distanceToCenter, 5)}");
                    currentPos = feature.CenterPos;
                }
                else if (IsSlotFeature(feature))
                {
                    if (feature.StartPos > currentPos)
                    {
                        double gap = feature.StartPos - currentPos;
                        segments.Add(gap);
                        segmentLabels.Add($"Gap: {FormatDimension(gap, 5)}");
                    }

                    segments.Add(feature.Width);
                    segmentLabels.Add($"Slot width: {FormatDimension(feature.Width, 5)}");
                    currentPos = feature.EndPos;
                }
            }

            double finalMargin = edgeLength - currentPos;
            segments.Add(finalMargin);
            segmentLabels.Add($"→Right: {FormatDimension(finalMargin, 5)}");

            ValidateSegmentSum(edgeLength, segments.Sum());
            CreateRunningDimensionLine(edgeStart, edgeDirection, perpendicular, segments, edgeName, dimensionsLayerIndex);

            LogSegments(segments, edgeLength, segmentLabels);
        }

        private void CreateRunningDimensionLine(Point3d edgeStart, Vector3d edgeDirection, Vector3d perpendicular, IList<double> segments, string edgeName, int dimensionsLayerIndex)
        {
            const double dimOffset = 3.0;
            Point3d dimLineStart = edgeStart + perpendicular * dimOffset;

            DimensionStyle dimStyle = ResolveBlueprintStyle("CreateRunningDimensionLine");
            if (dimStyle == null)
            {
                RhinoApp.WriteLine("[Blueprint Styles][EdgeDimensioningService.CreateRunningDimensionLine] Unable to resolve dimension style; skipping dimension chain.");
                return;
            }

            var attr = new ObjectAttributes
            {
                LayerIndex = dimensionsLayerIndex,
                ColorSource = ObjectColorSource.ColorFromObject,
                ObjectColor = Color.FromArgb(255, 0, 0)
            };

            double cumulativeDistance = 0.0;

            for (int i = 0; i < segments.Count; i++)
            {
                double segmentLength = segments[i];
                Point3d pt1 = edgeStart + edgeDirection * cumulativeDistance;
                Point3d pt2 = edgeStart + edgeDirection * (cumulativeDistance + segmentLength);
                Point3d dimLinePt = dimLineStart + edgeDirection * (cumulativeDistance + segmentLength / 2.0);

                bool isVertical = edgeName.Equals("LEFT", StringComparison.OrdinalIgnoreCase) ||
                                  edgeName.Equals("RIGHT", StringComparison.OrdinalIgnoreCase);

                double dimZ = edgeStart.Z + 0.01;
                var dimPlane = new Plane(new Point3d(0, 0, dimZ), Vector3d.ZAxis);

                BlueprintAnnotationDebug.LogDimensionRequest(
                    "EdgeDimensioningService.CreateRunningDimensionLine",
                    $"Chain Segment {i}",
                    dimStyle,
                    pt1,
                    pt2,
                    dimLinePt,
                    attr.LayerIndex);

                LinearDimension dim = isVertical
                    ? LinearDimension.Create(
                        AnnotationType.Rotated,
                        dimStyle,
                        dimPlane,
                        Vector3d.XAxis,
                        pt1,
                        pt2,
                        dimLinePt,
                        Math.PI / 2.0)
                    : LinearDimension.Create(
                        AnnotationType.Aligned,
                        dimStyle,
                        dimPlane,
                        edgeDirection,
                        pt1,
                        pt2,
                        dimLinePt,
                        0);
                ApplyDimensionStyle(dim, dimStyle);

                if (dim != null)
                {
                    _doc.Objects.AddLinearDimension(dim, attr);
                }

                cumulativeDistance += segmentLength;
            }
        }

        private void AddEdgeNotchDimensions(Rectangle3d bbox, IList<EdgeFeature> notches, string edgeName, double edgeLength, int dimensionsLayerIndex)
        {
            LogVerbose("  Strategy: Edge Notches (individual width/depth + nearest corner)");

            var (edgeStart, edgeEnd, edgeDirection, perpendicular) = GetEdgeParameters(bbox, edgeName);
            var dimStyle = ResolveBlueprintStyle("AddEdgeNotchDimensions");
            if (dimStyle == null)
            {
                RhinoApp.WriteLine("[Blueprint Styles][EdgeDimensioningService.AddEdgeNotchDimensions] Unable to resolve dimension style; skipping notch dimensions.");
                return;
            }
            var attr = new ObjectAttributes
            {
                LayerIndex = dimensionsLayerIndex,
                ColorSource = ObjectColorSource.ColorFromObject,
                ObjectColor = Color.FromArgb(255, 0, 0)
            };

            var orderedNotches = notches.OrderBy(n => n.StartPos).ToList();
            double cornerTolerance = 0.01;
            double dimOffset = 3.0;

            foreach (var notch in orderedNotches)
            {
                Point3d startPt = edgeStart + edgeDirection * notch.StartPos;
                Point3d endPt = edgeStart + edgeDirection * notch.EndPos;
                Point3d midPt = edgeStart + edgeDirection * (notch.StartPos + notch.Width / 2.0);

                Point3d dimLinePointWidth = (startPt + endPt) / 2.0 + (-perpendicular) * dimOffset;
                AddLinearDimension(edgeName, edgeDirection, startPt, endPt, dimLinePointWidth, dimStyle, attr);

                Vector3d depthDir = perpendicular;
                Point3d depthEnd = midPt + depthDir * notch.Depth;
                Point3d depthDimPoint = (midPt + depthEnd) / 2.0 + (-perpendicular) * dimOffset;
                AddPerpendicularDimension(depthDir, midPt, depthEnd, depthDimPoint, dimStyle, attr);

                double startDistance = notch.StartPos;
                double endDistance = edgeLength - notch.EndPos;
                bool touchesLeftCorner = startDistance < cornerTolerance;
                bool touchesRightCorner = endDistance < cornerTolerance;

                if (!touchesLeftCorner && !touchesRightCorner)
                {
                    bool nearerLeft = startDistance <= endDistance;
                    Point3d cornerPt = nearerLeft ? edgeStart : edgeEnd;
                    Point3d notchEdgePt = nearerLeft ? startPt : endPt;
                    Point3d cornerDimPoint = (cornerPt + notchEdgePt) / 2.0 + (-perpendicular) * dimOffset;
                    AddLinearDimension(edgeName, edgeDirection, cornerPt, notchEdgePt, cornerDimPoint, dimStyle, attr);
                }
            }
        }

        private void AddLinearDimension(string edgeName, Vector3d edgeDirection, Point3d pt1, Point3d pt2, Point3d dimLinePoint, DimensionStyle dimStyle, ObjectAttributes attr)
        {
            double dimZ = pt1.Z + 0.01;
            var dimPlane = new Plane(new Point3d(0, 0, dimZ), Vector3d.ZAxis);
            bool isVertical = edgeName.Equals("LEFT", StringComparison.OrdinalIgnoreCase) || edgeName.Equals("RIGHT", StringComparison.OrdinalIgnoreCase);

            BlueprintAnnotationDebug.LogDimensionRequest(
                "EdgeDimensioningService.AddLinearDimension",
                edgeName,
                dimStyle,
                pt1,
                pt2,
                dimLinePoint,
                attr?.LayerIndex ?? -1);

            LinearDimension dim = isVertical
                ? LinearDimension.Create(AnnotationType.Rotated, dimStyle, dimPlane, Vector3d.XAxis, pt1, pt2, dimLinePoint, Math.PI / 2.0)
                : LinearDimension.Create(AnnotationType.Aligned, dimStyle, dimPlane, edgeDirection, pt1, pt2, dimLinePoint, 0);
            ApplyDimensionStyle(dim, dimStyle);

            if (dim != null)
            {
                _doc.Objects.AddLinearDimension(dim, attr);
            }
        }

        private void AddPerpendicularDimension(Vector3d measurementDirection, Point3d pt1, Point3d pt2, Point3d dimLinePoint, DimensionStyle dimStyle, ObjectAttributes attr)
        {
            double dimZ = pt1.Z + 0.01;
            var dimPlane = new Plane(new Point3d(0, 0, dimZ), Vector3d.ZAxis);
            Vector3d axis = Math.Abs(measurementDirection.Y) > Math.Abs(measurementDirection.X) ? Vector3d.YAxis : Vector3d.XAxis;

            BlueprintAnnotationDebug.LogDimensionRequest(
                "EdgeDimensioningService.AddPerpendicularDimension",
                "Perpendicular",
                dimStyle,
                pt1,
                pt2,
                dimLinePoint,
                attr?.LayerIndex ?? -1);

            LinearDimension dim = LinearDimension.Create(AnnotationType.Aligned, dimStyle, dimPlane, axis, pt1, pt2, dimLinePoint, 0);
            ApplyDimensionStyle(dim, dimStyle);

            if (dim != null)
            {
                _doc.Objects.AddLinearDimension(dim, attr);
            }
        }

        private static void ApplyDimensionStyle(AnnotationBase dimension, DimensionStyle style)
        {
            if (dimension == null || style == null)
            {
                return;
            }

            dimension.DimensionStyleId = style.Id;
        }

        private static bool IsHoleFeature(EdgeFeature feature)
        {
            if (feature?.Type == null)
            {
                return false;
            }

            return feature.Type.Equals("Edge Hole", StringComparison.OrdinalIgnoreCase) ||
                   feature.Type.Equals("EdgeCordHole", StringComparison.OrdinalIgnoreCase);
        }

        private static bool IsSlotFeature(EdgeFeature feature)
        {
            if (feature?.Type == null)
            {
                return false;
            }

            return feature.Type.Equals("Edge Slot", StringComparison.OrdinalIgnoreCase) ||
                   feature.Type.Equals("EdgeCordSlot", StringComparison.OrdinalIgnoreCase);
        }

        private static bool IsNotchFeature(EdgeFeature feature)
        {
            if (feature?.Type == null)
            {
                return false;
            }

            return feature.Type.Equals("Edge Notch", StringComparison.OrdinalIgnoreCase) ||
                   feature.Type.Equals("EdgeNotch", StringComparison.OrdinalIgnoreCase) ||
                   feature.Type.Equals("CornerNotch", StringComparison.OrdinalIgnoreCase);
        }

        private static (Point3d Start, Point3d End, Vector3d Direction, Vector3d Perpendicular) GetEdgeParameters(Rectangle3d bbox, string edgeName)
        {
            // Rectangle3d exposes corners via BoundingBox to provide consistent min/max in world space.
            var boundingBox = bbox.BoundingBox;
            var min = boundingBox.Min;
            var max = boundingBox.Max;

            switch (edgeName.ToUpperInvariant())
            {
                case "BOTTOM":
                    return (
                        new Point3d(min.X, min.Y, 0),
                        new Point3d(max.X, min.Y, 0),
                        Vector3d.XAxis,
                        Vector3d.YAxis);
                case "TOP":
                    return (
                        new Point3d(min.X, max.Y, 0),
                        new Point3d(max.X, max.Y, 0),
                        Vector3d.XAxis,
                        -Vector3d.YAxis);
                case "LEFT":
                    return (
                        new Point3d(min.X, min.Y, 0),
                        new Point3d(min.X, max.Y, 0),
                        Vector3d.YAxis,
                        Vector3d.XAxis);
                case "RIGHT":
                    return (
                        new Point3d(max.X, min.Y, 0),
                        new Point3d(max.X, max.Y, 0),
                        Vector3d.YAxis,
                        -Vector3d.XAxis);
                default:
                    throw new ArgumentException($"Unknown edge name '{edgeName}'", nameof(edgeName));
            }
        }

        private DimensionStyle ResolveBlueprintStyle(string caller)
        {
            return BlueprintAnnotationDebug.ResolveDefaultStyle(_doc, $"EdgeDimensioningService.{caller}");
        }

        private static void ValidateSegmentSum(double edgeLength, double sum)
        {
            if (Math.Abs(sum - edgeLength) > 0.01)
            {
                LogVerbose($"  WARNING: Dimension sum {sum:F3}\" vs edge length {edgeLength:F3}\"");
            }
        }

        private static void LogSegments(IEnumerable<double> segments, double edgeLength, IEnumerable<string> labels)
        {
            var segmentList = segments.ToList();
            LogVerbose($"  Segments: {string.Join(" + ", labels)}");
            double sum = segmentList.Sum();
            LogVerbose($"  Total: {FormatDimension(sum, 5)} (edge: {FormatDimension(edgeLength, 5)})");
        }

        private static void LogVerbose(string message)
        {
            if (VerboseLogging)
            {
                RhinoApp.WriteLine(message);
            }
        }
    }
}
