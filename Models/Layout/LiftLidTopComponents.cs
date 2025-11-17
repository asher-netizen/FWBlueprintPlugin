using System.Collections.Generic;
using Rhino.DocObjects;

namespace FWBlueprintPlugin.Models.Layout
{
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
}
