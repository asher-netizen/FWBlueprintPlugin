// FILE: EdgeFeatureDimensioning.cs
// Phase 3B: Edge Feature Dimensioning
// Handles dimensioning for Edge Holes (≤2.00") and Edge Slots (≥2.01")
// Implements running cumulative format per CUTOUT_DIMENSIONING_COMPLETE_SPEC.md

using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

namespace FWBlueprintPlugin
{
    public partial class FWBlueprintPluginCommand : Command
    {
        // ===============================================
        // MAIN ENTRY POINT FOR EDGE FEATURE DIMENSIONING
        // ===============================================
        private void AddEdgeFeatureDimensions(
            RhinoDoc doc,
            Rectangle3d bbox,
            List<EdgeFeature> features,
            int dimensionsLayerIndex)
        {
            if (features == null || features.Count == 0)
            {
                RhinoApp.WriteLine("  No edge features to dimension");
                return;
            }

            RhinoApp.WriteLine("");
            RhinoApp.WriteLine("=== Adding Edge Feature Dimensions ===");

            // Group features by edge
            var byEdge = features.GroupBy(f => f.EdgeName);

            foreach (var edgeGroup in byEdge)
            {
                string edgeName = edgeGroup.Key;
                var edgeFeatures = edgeGroup.OrderBy(f => f.StartPos).ToList();

                RhinoApp.WriteLine($"\n[{edgeName}] Dimensioning {edgeFeatures.Count} feature(s)...");

                // Get edge parameters
                Point3d edgeStart, edgeEnd;
                Vector3d edgeDirection, perpendicular;
                GetEdgeParameters(bbox, edgeName, out edgeStart, out edgeEnd, out edgeDirection, out perpendicular);

                double edgeLength = edgeStart.DistanceTo(edgeEnd);

                // Classify features on this edge (FIX: Use "Edge Hole" and "Edge Slot" with spaces)
                bool hasHoles = edgeFeatures.Any(f => f.Type == "Edge Hole");
                bool hasSlots = edgeFeatures.Any(f => f.Type == "Edge Slot");

                // Choose dimensioning strategy
                if (hasHoles && !hasSlots)
                {
                    // All holes - dimension to centers
                    AddEdgeHoleDimensions(doc, bbox, edgeFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                }
                else if (hasSlots && !hasHoles)
                {
                    // All slots - running cumulative
                    AddEdgeSlotDimensions(doc, bbox, edgeFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                }
                else
                {
                    // Mixed - running cumulative with centers for holes
                    AddMixedEdgeDimensions(doc, bbox, edgeFeatures, edgeName, edgeLength, dimensionsLayerIndex);
                }
            }

            RhinoApp.WriteLine("\n✓ Edge feature dimensioning complete");
        }

        // ===============================================
        // STRATEGY 1: EDGE HOLES ONLY (≤2.00")
        // Dimension to CENTER from both edges
        // SPECIAL: Dimensions are INSET onto panel (perpendicular points into panel)
        // ===============================================
        private void AddEdgeHoleDimensions(
            RhinoDoc doc,
            Rectangle3d bbox,
            List<EdgeFeature> holes,
            string edgeName,
            double edgeLength,
            int dimensionsLayerIndex)
        {
            RhinoApp.WriteLine($"  Strategy: Edge Holes (dimension to centers, inset onto panel)");

            // Get edge parameters
            Point3d edgeStart, edgeEnd;
            Vector3d edgeDirection, perpendicular;
            GetEdgeParameters(bbox, edgeName, out edgeStart, out edgeEnd, out edgeDirection, out perpendicular);

            // Perpendicular already points INTO panel (from detection logic)
            // This creates the 3" INSET effect we want for holes

            // Build dimension segments: [left→center1] [center1→center2] ... [lastCenter→right]
            var segments = new List<double>();
            var segmentLabels = new List<string>();

            // Left margin to first hole center
            double firstCenter = holes[0].CenterPos;
            segments.Add(firstCenter);
            segmentLabels.Add($"Left→C1: {FormatDimension(firstCenter, 5)}");

            // Gaps between hole centers
            for (int i = 0; i < holes.Count - 1; i++)
            {
                double gap = holes[i + 1].CenterPos - holes[i].CenterPos;
                segments.Add(gap);
                segmentLabels.Add($"C{i + 1}→C{i + 2}: {FormatDimension(gap, 5)}");
            }

            // Last hole center to right margin
            double lastCenter = holes[holes.Count - 1].CenterPos;
            double rightMargin = edgeLength - lastCenter;
            segments.Add(rightMargin);
            segmentLabels.Add($"C{holes.Count}→Right: {FormatDimension(rightMargin, 5)}");

            // Validate sum
            double sum = segments.Sum();
            if (Math.Abs(sum - edgeLength) > 0.01)
            {
                RhinoApp.WriteLine($"  ⚠ WARNING: Dimension sum {sum:F3}\" ≠ edge length {edgeLength:F3}\"");
            }

            // Create dimension line
            CreateRunningDimensionLine(
                doc,
                edgeStart,
                edgeDirection,
                perpendicular,
                segments,
                edgeName,
                dimensionsLayerIndex);

            // Log segments
            RhinoApp.WriteLine($"  Segments: {string.Join(" + ", segments.Select(s => FormatDimension(s, 5)))}");
            RhinoApp.WriteLine($"  Total: {FormatDimension(sum, 5)} (edge: {FormatDimension(edgeLength, 5)})");
        }

        // ===============================================
        // STRATEGY 2: EDGE SLOTS ONLY (≥2.01")
        // Running cumulative: [margin] [width] [gap] [width] [margin]
        // SPECIAL: Dimensions are OFFSET away from panel (perpendicular negated)
        // ===============================================
        private void AddEdgeSlotDimensions(
            RhinoDoc doc,
            Rectangle3d bbox,
            List<EdgeFeature> slots,
            string edgeName,
            double edgeLength,
            int dimensionsLayerIndex)
        {
            RhinoApp.WriteLine($"  Strategy: Edge Slots (running cumulative, offset away from panel)");

            // Get edge parameters
            Point3d edgeStart, edgeEnd;
            Vector3d edgeDirection, perpendicular;
            GetEdgeParameters(bbox, edgeName, out edgeStart, out edgeEnd, out edgeDirection, out perpendicular);

            // CRITICAL: For slots, NEGATE perpendicular to go AWAY from panel
            perpendicular = -perpendicular;

            // Build dimension segments: [leftMargin] [slot1Width] [gap] [slot2Width] [gap] ... [rightMargin]
            var segments = new List<double>();
            var segmentLabels = new List<string>();

            // Left margin to first slot
            double leftMargin = slots[0].StartPos;
            segments.Add(leftMargin);
            segmentLabels.Add($"Left margin: {FormatDimension(leftMargin, 5)}");

            // Add each slot width and gap
            for (int i = 0; i < slots.Count; i++)
            {
                // Slot width
                segments.Add(slots[i].Width);
                segmentLabels.Add($"Slot {i + 1} width: {FormatDimension(slots[i].Width, 5)}");

                // Gap to next slot (if not last)
                if (i < slots.Count - 1)
                {
                    double gap = slots[i + 1].StartPos - slots[i].EndPos;
                    segments.Add(gap);
                    segmentLabels.Add($"Gap: {FormatDimension(gap, 5)}");
                }
            }

            // Right margin from last slot
            double rightMargin = edgeLength - slots[slots.Count - 1].EndPos;
            segments.Add(rightMargin);
            segmentLabels.Add($"Right margin: {FormatDimension(rightMargin, 5)}");

            // Validate sum
            double sum = segments.Sum();
            if (Math.Abs(sum - edgeLength) > 0.01)
            {
                RhinoApp.WriteLine($"  ⚠ WARNING: Dimension sum {sum:F3}\" ≠ edge length {edgeLength:F3}\"");
            }

            // Create dimension line
            CreateRunningDimensionLine(
                doc,
                edgeStart,
                edgeDirection,
                perpendicular,
                segments,
                edgeName,
                dimensionsLayerIndex);

            // Log segments
            RhinoApp.WriteLine($"  Segments: {string.Join(" + ", segments.Select(s => FormatDimension(s, 5)))}");
            RhinoApp.WriteLine($"  Total: {FormatDimension(sum, 5)} (edge: {FormatDimension(edgeLength, 5)})");
        }

        // ===============================================
        // STRATEGY 3: MIXED (HOLES + SLOTS)
        // Running cumulative with centers for holes, widths for slots
        // SPECIAL: Dimensions are OFFSET away from panel (for consistency)
        // ===============================================
        private void AddMixedEdgeDimensions(
            RhinoDoc doc,
            Rectangle3d bbox,
            List<EdgeFeature> features,
            string edgeName,
            double edgeLength,
            int dimensionsLayerIndex)
        {
            RhinoApp.WriteLine($"  Strategy: Mixed (holes + slots, running cumulative, offset away from panel)");

            // CRITICAL FIX: If only holes (no actual slots), use hole strategy instead
            bool hasSlots = features.Any(f => f.Type == "Edge Slot");  // FIX: Add space
            if (!hasSlots)
            {
                RhinoApp.WriteLine($"  ⚠ Correcting: Only holes detected, using hole-only strategy");
                AddEdgeHoleDimensions(doc, bbox, features, edgeName, edgeLength, dimensionsLayerIndex);
                return;
            }

            // Get edge parameters
            Point3d edgeStart, edgeEnd;
            Vector3d edgeDirection, perpendicular;
            GetEdgeParameters(bbox, edgeName, out edgeStart, out edgeEnd, out edgeDirection, out perpendicular);

            // CRITICAL: For mixed edges, dimension AWAY from panel (like slots)
            // This keeps the continuous dimension line readable
            perpendicular = -perpendicular;

            // Build segments by walking along the edge
            var segments = new List<double>();
            var segmentLabels = new List<string>();

            double currentPos = 0.0;

            foreach (var feature in features)
            {
                if (feature.Type == "Edge Hole")  // FIX: Add space
                {
                    // Dimension from current position to hole center
                    double distanceToCenter = feature.CenterPos - currentPos;
                    segments.Add(distanceToCenter);
                    segmentLabels.Add($"→Hole center: {FormatDimension(distanceToCenter, 5)}");
                    currentPos = feature.CenterPos;
                }
                else // Edge Slot
                {
                    // Gap to slot start
                    if (feature.StartPos > currentPos)
                    {
                        double gap = feature.StartPos - currentPos;
                        segments.Add(gap);
                        segmentLabels.Add($"Gap: {FormatDimension(gap, 5)}");
                    }

                    // Slot width
                    segments.Add(feature.Width);
                    segmentLabels.Add($"Slot width: {FormatDimension(feature.Width, 5)}");
                    currentPos = feature.EndPos;
                }
            }

            // Final margin to edge end
            double finalMargin = edgeLength - currentPos;
            segments.Add(finalMargin);
            segmentLabels.Add($"→Right: {FormatDimension(finalMargin, 5)}");

            // Validate sum
            double sum = segments.Sum();
            if (Math.Abs(sum - edgeLength) > 0.01)
            {
                RhinoApp.WriteLine($"  ⚠ WARNING: Dimension sum {sum:F3}\" ≠ edge length {edgeLength:F3}\"");
            }

            // Create dimension line
            CreateRunningDimensionLine(
                doc,
                edgeStart,
                edgeDirection,
                perpendicular,
                segments,
                edgeName,
                dimensionsLayerIndex);

            // Log segments
            RhinoApp.WriteLine($"  Segments: {string.Join(" + ", segments.Select(s => FormatDimension(s, 5)))}");
            RhinoApp.WriteLine($"  Total: {FormatDimension(sum, 5)} (edge: {FormatDimension(edgeLength, 5)})");
        }

        // ===============================================
        // HELPER: CREATE RUNNING DIMENSION LINE
        // Creates a single continuous dimension chain with multiple segments
        // ===============================================
        private void CreateRunningDimensionLine(
            RhinoDoc doc,
            Point3d edgeStart,
            Vector3d edgeDirection,
            Vector3d perpendicular,
            List<double> segments,
            string edgeName,
            int dimensionsLayerIndex)
        {
            // Dimension offset: 3" perpendicular from edge
            const double dimOffset = 3.0;
            Point3d dimLineStart = edgeStart + perpendicular * dimOffset;

            // Get dimension style
            DimensionStyle dimStyle = doc.DimStyles.FindName("Furniture Dim - CG");
            if (dimStyle == null)
            {
                dimStyle = doc.DimStyles.Current;
            }

            // Create attributes
            var attr = new ObjectAttributes
            {
                LayerIndex = dimensionsLayerIndex,
                ColorSource = ObjectColorSource.ColorFromObject,
                ObjectColor = Color.FromArgb(255, 0, 0)
            };

            // Create each dimension segment in the chain
            double cumulativeDistance = 0.0;

            for (int i = 0; i < segments.Count; i++)
            {
                double segmentLength = segments[i];

                // Define dimension points
                Point3d pt1 = edgeStart + edgeDirection * cumulativeDistance;
                Point3d pt2 = edgeStart + edgeDirection * (cumulativeDistance + segmentLength);
                Point3d dimLinePt = dimLineStart + edgeDirection * (cumulativeDistance + segmentLength / 2.0);

                // Determine if dimension should be rotated (for vertical edges)
                bool isVertical = edgeName == "LEFT" || edgeName == "RIGHT";

                // Create dimension plane at Z = bbox.Max.Z + 0.01
                double dimZ = edgeStart.Z + 0.01;
                var dimPlane = new Plane(new Point3d(0, 0, dimZ), Vector3d.ZAxis);

                LinearDimension dim;

                if (isVertical)
                {
                    // Rotated dimension for vertical edges
                    dim = LinearDimension.Create(
                        AnnotationType.Rotated,
                        dimStyle,
                        dimPlane,
                        Vector3d.XAxis,
                        pt1,
                        pt2,
                        dimLinePt,
                        Math.PI / 2.0);
                }
                else
                {
                    // Aligned dimension for horizontal edges
                    dim = LinearDimension.Create(
                        AnnotationType.Aligned,
                        dimStyle,
                        dimPlane,
                        edgeDirection,
                        pt1,
                        pt2,
                        dimLinePt,
                        0);
                }

                if (dim != null)
                {
                    doc.Objects.AddLinearDimension(dim, attr);
                }

                cumulativeDistance += segmentLength;
            }
        }
    }
}