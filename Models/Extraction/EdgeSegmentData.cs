using System.Collections.Generic;
using Rhino.Geometry;

namespace FWBlueprintPlugin.Models.Extraction
{
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
}
