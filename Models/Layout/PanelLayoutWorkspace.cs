using System.Collections.Generic;
using Rhino.Geometry;
using FWBlueprintPlugin.Models.Dimensioning;

namespace FWBlueprintPlugin.Models.Layout
{
    /// <summary>
    /// Holds mutable state for panel layout operations.
    /// </summary>
    internal sealed class PanelLayoutWorkspace
    {
        public Dictionary<string, double> PanelThicknesses { get; } = new Dictionary<string, double>();
        public List<BoundingBox> PanelBounds { get; } = new List<BoundingBox>();
        public List<PanelDimensionInfo> DeferredDimensions { get; } = new List<PanelDimensionInfo>();
        public List<PanelLeaderInfo> DeferredLeaders { get; } = new List<PanelLeaderInfo>();

        public void Reset()
        {
            PanelThicknesses.Clear();
            PanelBounds.Clear();
            DeferredDimensions.Clear();
            DeferredLeaders.Clear();
        }

        public void AddDimension(PanelDimensionInfo info)
        {
            if (info != null)
            {
                DeferredDimensions.Add(info);
            }
        }

        public void AddLeader(PanelLeaderInfo info)
        {
            if (info != null)
            {
                DeferredLeaders.Add(info);
            }
        }
    }
}
