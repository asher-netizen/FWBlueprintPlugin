using System.Collections.Generic;
using Rhino.DocObjects;
using Rhino.Geometry;
using FWBlueprintPlugin.Models.Dimensioning;

namespace FWBlueprintPlugin.Models.Extraction
{
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
