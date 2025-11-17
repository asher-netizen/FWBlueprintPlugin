using System.Collections.Generic;
using FWBlueprintPlugin.Models.Dimensioning;
using Rhino.DocObjects;

namespace FWBlueprintPlugin.Models.Layout.Context
{
    internal sealed class PanelLayoutContext
    {
        public PanelLayoutWorkspace Workspace { get; } = new PanelLayoutWorkspace();
        public Dictionary<string, double> PanelThicknesses => Workspace.PanelThicknesses;
        public Layer ParentLayer { get; set; }
        public LiftLidTopComponents TopComponents { get; set; }
        public RhinoObject BackPanel { get; set; }
        public RhinoObject BottomPanel { get; set; }
        public List<RhinoObject> ExteriorWalls { get; set; } = new List<RhinoObject>();
        public List<RhinoObject> InteriorWalls { get; set; } = new List<RhinoObject>();
        public List<RhinoObject> RemovableShelves { get; set; } = new List<RhinoObject>();
        public List<RhinoObject> PermanentShelves { get; set; } = new List<RhinoObject>();
        public List<RhinoObject> Doors { get; set; } = new List<RhinoObject>();
    }
}
