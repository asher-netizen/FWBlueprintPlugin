using Rhino.DocObjects;
using Rhino.Geometry;
using System.Collections.Generic;

namespace FWBlueprintPlugin
{
    /// <summary>
    /// Describes deferred panel dimension data collected during layout.
    /// </summary>
    internal class PanelDimensionInfo
    {
        public BoundingBox BBox { get; set; }
        public string PanelType { get; set; }
        public int Quantity { get; set; }
        public bool IsChild { get; set; }
        public BoundingBox? GroupBBox { get; set; }
        public bool IsRightHeightDim { get; set; }
        public int LayerIndex { get; set; }
        public bool DrawWidth { get; set; } = true;
        public bool DrawHeight { get; set; } = true;
        public double? CustomHeightOffset { get; set; } = null;
    }

    /// <summary>
    /// Captures label leader data after panel arrangement.
    /// </summary>
    internal class PanelLeaderInfo
    {
        public BoundingBox BBox { get; set; }
        public string PanelType { get; set; }
        public int Quantity { get; set; }
        public int LayerIndex { get; set; }
    }

    /// <summary>
    /// Represents an edge cutout feature detected on a panel perimeter.
    /// </summary>
    internal class EdgeFeature
    {
        public string EdgeName { get; set; }
        public double StartPos { get; set; }
        public double EndPos { get; set; }
        public double Width { get; set; }
        public double Depth { get; set; }
        public string Type { get; set; }
        public double CenterPos { get; set; }
        public bool HasCurvature { get; set; }
        public double GapBefore { get; set; }
        public double GapAfter { get; set; }
        public double EdgeLength { get; set; }

        public EdgeFeature(
            string edgeName,
            double startPos,
            double endPos,
            double width,
            double depth,
            string type,
            bool hasCurvature,
            double gapBefore = 0,
            double gapAfter = 0,
            double edgeLength = 0)
        {
            EdgeName = edgeName;
            StartPos = startPos;
            EndPos = endPos;
            Width = width;
            Depth = depth;
            Type = type;
            CenterPos = startPos + (width / 2.0);
            HasCurvature = hasCurvature;
            GapBefore = gapBefore;
            GapAfter = gapAfter;
            EdgeLength = edgeLength;
        }
    }

    /// <summary>
    /// Result from panel arrangement pass.
    /// </summary>
    internal class PanelArrangementResult
    {
        public bool Success { get; set; }
        public Dictionary<string, double> PanelThicknesses { get; set; } = new Dictionary<string, double>();
        public List<BoundingBox> PanelBounds { get; set; } = new List<BoundingBox>();
        public List<PanelDimensionInfo> DeferredDimensions { get; set; } = new List<PanelDimensionInfo>();
        public List<PanelLeaderInfo> DeferredLeaders { get; set; } = new List<PanelLeaderInfo>();
    }

    /// <summary>
    /// Represents top-component groupings for lift-lid configurations.
    /// </summary>
    internal class LiftLidTopComponents
    {
        public List<RhinoObject> BackerPlates { get; set; } = new List<RhinoObject>();
        public List<RhinoObject> LiftLids { get; set; } = new List<RhinoObject>();
        public List<RhinoObject> TopPlates { get; set; } = new List<RhinoObject>();
        public int TotalComponentCount { get; set; }
    }

    /// <summary>
    /// Captures bounding data produced during lift-lid flattening.
    /// </summary>
    internal class LiftLidFlattenedInfo
    {
        public BoundingBox OverallBBox { get; set; }
        public List<BoundingBox> WidthGroupBBoxes { get; set; } = new List<BoundingBox>();
        public List<BoundingBox> BackerDepthBBoxes { get; set; } = new List<BoundingBox>();
        public List<BoundingBox> LidDepthBBoxes { get; set; } = new List<BoundingBox>();
    }

    /// <summary>
    /// Stores a flattened geometry object and its bounding box.
    /// </summary>
    internal struct FlattenedComponent
    {
        public GeometryBase Geom { get; set; }
        public BoundingBox BBox { get; set; }
    }

    /// <summary>
    /// Collects boundary segment groupings for Phase 3 edge analysis.
    /// </summary>
    internal class EdgeSegmentData
    {
        public List<(Curve seg, int index)> BottomSegments { get; set; } = new List<(Curve seg, int index)>();
        public List<(Curve seg, int index)> RightSegments { get; set; } = new List<(Curve seg, int index)>();
        public List<(Curve seg, int index)> TopSegments { get; set; } = new List<(Curve seg, int index)>();
        public List<(Curve seg, int index)> LeftSegments { get; set; } = new List<(Curve seg, int index)>();
        public List<(Curve seg, int index)> InteriorSegments { get; set; } = new List<(Curve seg, int index)>();
    }

    /// <summary>
    /// Carries layer indices used during Phase 3 2D extraction.
    /// </summary>
    internal class BlueprintLayerContext
    {
        public Layer BlueprintLayer { get; set; }
        public int Panels3DLayerIndex { get; set; }
        public int Panels2DLayerIndex { get; set; }
        public int CutoutsLayerIndex { get; set; }
        public int PocketLayerIndex { get; set; }
        public int DimensionsLayerIndex { get; set; }
        public int DashLinetypeIndex { get; set; }
    }

    /// <summary>
    /// Bundles extracted panel data for downstream Phase 3 processing.
    /// </summary>
    internal class Panel2DExtractionResult
    {
        public List<(RhinoObject Panel, List<Brep> ChordCutouts)> ChordCutouts { get; set; } = new List<(RhinoObject Panel, List<Brep> ChordCutouts)>();
        public int BoundingBoxCount { get; set; }
        public int CutoutCount { get; set; }
        public int EdgeFeatureCount { get; set; }
        public List<EdgeFeature> EdgeFeatures { get; set; } = new List<EdgeFeature>();
    }
}
