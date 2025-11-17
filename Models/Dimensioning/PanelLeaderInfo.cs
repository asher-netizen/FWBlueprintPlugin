using Rhino.Geometry;

namespace FWBlueprintPlugin.Models.Dimensioning
{
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
}
