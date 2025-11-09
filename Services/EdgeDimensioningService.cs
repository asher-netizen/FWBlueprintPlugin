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

        public void AddEdgeFeatureDimensions(Rectangle3d panelBBox, IList<EdgeFeature> features, int dimensionsLayerIndex)
        {
            if (features == null || features.Count == 0)
            {
                RhinoApp.WriteLine("  No edge features to dimension");
                return;
            }

            RhinoApp.WriteLine(string.Empty);
            RhinoApp.WriteLine("=== Adding Edge Feature Dimensions ===");

            var groupedByEdge = features
                .GroupBy(f => f.EdgeName)
                .OrderBy(g => g.Key, StringComparer.OrdinalIgnoreCase);

            foreach (var group in groupedByEdge)
            {
                var edgeFeatures = group.OrderBy(f => f.StartPos).ToList();
                string edgeName = group.Key;

                RhinoApp.WriteLine($"\n[{edgeName}] Dimensioning {edgeFeatures.Count} feature(s)...");

                var (edgeStart, edgeEnd, edgeDirection, perpendicular) = GetEdgeParameters(panelBBox, edgeName);
                double edgeLength = edgeStart.DistanceTo(edgeEnd);

                bool hasHoles = edgeFeatures.Any(f => f.Type == "Edge Hole");
                bool hasSlots = edgeFeatures.Any(f => f.Type == "Edge Slot");

                if (hasHoles && !hasSlots)
                {
                    AddEdgeHoleDimensions(panelBBox, edgeFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                }
                else if (hasSlots && !hasHoles)
                {
                    AddEdgeSlotDimensions(panelBBox, edgeFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                }
                else
                {
                    AddMixedEdgeDimensions(panelBBox, edgeFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                }
            }

            RhinoApp.WriteLine("\n✓ Edge feature dimensioning complete");
        }

        private void AddEdgeHoleDimensions(Rectangle3d bbox, IList<EdgeFeature> holes, string edgeName, double edgeLength, int dimensionsLayerIndex)
        {
            RhinoApp.WriteLine("  Strategy: Edge Holes (dimension to centers, inset onto panel)");

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

            LogSegments(segments, edgeLength, segmentLabels);
        }

        private void AddEdgeSlotDimensions(Rectangle3d bbox, IList<EdgeFeature> slots, string edgeName, double edgeLength, int dimensionsLayerIndex)
        {
            RhinoApp.WriteLine("  Strategy: Edge Slots (running cumulative, offset away from panel)");

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

            LogSegments(segments, edgeLength, segmentLabels);
        }

        private void AddMixedEdgeDimensions(Rectangle3d bbox, IList<EdgeFeature> features, string edgeName, double edgeLength, int dimensionsLayerIndex)
        {
            RhinoApp.WriteLine("  Strategy: Mixed (holes + slots, running cumulative, offset away from panel)");

            bool hasSlots = features.Any(f => f.Type == "Edge Slot");
            if (!hasSlots)
            {
                RhinoApp.WriteLine("  ⚠ Correcting: Only holes detected, using hole-only strategy");
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
                if (feature.Type == "Edge Hole")
                {
                    double distanceToCenter = feature.CenterPos - currentPos;
                    segments.Add(distanceToCenter);
                    segmentLabels.Add($"→Hole center: {FormatDimension(distanceToCenter, 5)}");
                    currentPos = feature.CenterPos;
                }
                else
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

            DimensionStyle dimStyle = _doc.DimStyles.FindName("Furniture Dim - CG") ?? _doc.DimStyles.Current;

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

                if (dim != null)
                {
                    _doc.Objects.AddLinearDimension(dim, attr);
                }

                cumulativeDistance += segmentLength;
            }
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

        private static void ValidateSegmentSum(double edgeLength, double sum)
        {
            if (Math.Abs(sum - edgeLength) > 0.01)
            {
                RhinoApp.WriteLine($"  ⚠ WARNING: Dimension sum {sum:F3}\" ≠ edge length {edgeLength:F3}\"");
            }
        }

        private static void LogSegments(IEnumerable<double> segments, double edgeLength, IEnumerable<string> labels)
        {
            var segmentList = segments.ToList();
            RhinoApp.WriteLine($"  Segments: {string.Join(" + ", labels)}");
            double sum = segmentList.Sum();
            RhinoApp.WriteLine($"  Total: {FormatDimension(sum, 5)} (edge: {FormatDimension(edgeLength, 5)})");
        }
    }
}
