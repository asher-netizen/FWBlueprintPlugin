using System.Collections.Generic;
using Rhino.Geometry;

namespace FWBlueprintPlugin.Models.Layout
{
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
}
