using System;
using System.Drawing;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using FWBlueprintPlugin;
using FWBlueprintPlugin.Models.Extraction;

namespace FWBlueprintPlugin.Services.Phase3
{
    /// <summary>
    /// Handles creation and maintenance of the Blueprint layer subtree for Phase 3 extraction.
    /// </summary>
    internal class LayerSetupService
    {
        private readonly RhinoDoc _doc;

        public LayerSetupService(RhinoDoc doc)
        {
            _doc = doc ?? throw new ArgumentNullException(nameof(doc));
        }

        public BlueprintLayerContext PrepareLayers(Layer parentLayer)
        {
            if (parentLayer == null)
            {
                throw new ArgumentNullException(nameof(parentLayer));
            }

            var blueprintLayer = FindOrCreateBlueprintLayer(parentLayer);
            if (blueprintLayer == null)
            {
                throw new InvalidOperationException("Unable to create or locate the Blueprint layer.");
            }

            var context = new BlueprintLayerContext
            {
                BlueprintLayer = blueprintLayer,
                Panels3DLayerIndex = Prepare3DPanelsLayer(blueprintLayer),
                Panels2DLayerIndex = FindOrCreateChildLayer(blueprintLayer, "2D Panels", Color.Black),
                CutoutsLayerIndex = FindOrCreateChildLayer(blueprintLayer, "Cutouts", Color.Black),
                PocketLayerIndex = FindOrCreateChildLayer(blueprintLayer, "Pocket Curves", Color.Black),
                DimensionsLayerIndex = FindOrCreateChildLayer(blueprintLayer, "Dimensions", Color.Red),
                DashLinetypeIndex = EnsureDashedLinetype()
            };

            // Hide pocket layer by default to mirror legacy behavior.
            _doc.Layers[context.PocketLayerIndex].IsVisible = false;

            return context;
        }

        public void DropDimensionsToZ0(BlueprintLayerContext context)
        {
            if (context == null) return;

            int dimsLayer = GetLayerIndexByFullPath($"{context.BlueprintLayer.FullPath}::Dimensions");
            if (dimsLayer < 0) return;

            var dimsObjects = _doc.Objects.FindByLayer(_doc.Layers[dimsLayer]);
            if (dimsObjects == null) return;

            foreach (var obj in dimsObjects)
            {
                if (obj.Geometry is AnnotationBase anno)
                {
                    var transform = Transform.PlanarProjection(Plane.WorldXY);
                    anno.Transform(transform);
                    obj.CommitChanges();
                }
            }

            _doc.Views.Redraw();
        }

        private Layer FindOrCreateBlueprintLayer(Layer parentLayer)
        {
            string blueprintPath = $"{parentLayer.FullPath}::Blueprint";
            int index = GetLayerIndexByFullPath(blueprintPath);
            if (index >= 0)
            {
                return _doc.Layers[index];
            }

            var newLayer = new Layer
            {
                Name = "Blueprint",
                ParentLayerId = parentLayer.Id,
                Color = Color.White
            };
            index = _doc.Layers.Add(newLayer);
            return index >= 0 ? _doc.Layers[index] : null;
        }

        private int Prepare3DPanelsLayer(Layer blueprintLayer)
        {
            string panelsPath = $"{blueprintLayer.FullPath}::Panels";
            int index = GetLayerIndexByFullPath(panelsPath);
            if (index >= 0)
            {
                var panelsLayer = _doc.Layers[index];
                panelsLayer.Name = "3D Panels";
                panelsLayer.Color = Color.Black;
                _doc.Layers.Modify(panelsLayer, index, true);
                return index;
            }

            return FindOrCreateChildLayer(blueprintLayer, "3D Panels", Color.Black);
        }

        private int FindOrCreateChildLayer(Layer parentLayer, string childName, Color color)
        {
            string fullPath = $"{parentLayer.FullPath}::{childName}";
            int index = GetLayerIndexByFullPath(fullPath);
            if (index >= 0)
            {
                var existing = _doc.Layers[index];
                existing.Color = color;
                _doc.Layers.Modify(existing, index, true);
                return index;
            }

            var newLayer = new Layer
            {
                Name = childName,
                ParentLayerId = parentLayer.Id,
                Color = color
            };

            return _doc.Layers.Add(newLayer);
        }

        private int GetLayerIndexByFullPath(string fullPath)
        {
            for (int i = 0; i < _doc.Layers.Count; i++)
            {
                if (string.Equals(_doc.Layers[i].FullPath, fullPath, StringComparison.OrdinalIgnoreCase))
                {
                    return i;
                }
            }

            return -1;
        }

        private int EnsureDashedLinetype()
        {
            for (int i = 0; i < _doc.Linetypes.Count; i++)
            {
                var lt = _doc.Linetypes[i];
                if (lt.Name.IndexOf("Dash", StringComparison.OrdinalIgnoreCase) >= 0)
                {
                    return i;
                }
            }

            var newLinetype = new Linetype { Name = "Blueprint Dash" };
            newLinetype.AppendSegment(0.25, true);
            newLinetype.AppendSegment(0.125, false);
            return _doc.Linetypes.Add(newLinetype);
        }
    }
}
