using System.Collections.Generic;
using Rhino.Geometry;
using FWBlueprintPlugin.Models.Dimensioning;

namespace FWBlueprintPlugin.Models.Layout
{
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
}
