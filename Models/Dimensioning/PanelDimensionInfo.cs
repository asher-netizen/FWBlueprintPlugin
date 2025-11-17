using Rhino.Geometry;

namespace FWBlueprintPlugin.Models.Dimensioning
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
        public double? CustomHeightOffset { get; set; }
    }
}
