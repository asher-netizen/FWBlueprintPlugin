using Rhino.DocObjects;

namespace FWBlueprintPlugin.Models.Extraction
{
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
}
