using System;
using System.Drawing;
using FWBlueprintPlugin.Infrastructure.Logging;
using Rhino;
using Rhino.DocObjects;

namespace FWBlueprintPlugin.Services.Layout
{
    /// <summary>
    /// Encapsulates creation of layout layers and related metadata.
    /// </summary>
    internal sealed class PanelLayerConfigurator
    {
        private readonly RhinoDoc _doc;

        public PanelLayerConfigurator(RhinoDoc doc)
        {
            _doc = doc ?? throw new ArgumentNullException(nameof(doc));
        }

        public void SetupLayersAndStyles(Layer parentLayer)
        {
            CreateLayerStructure(parentLayer);
            EnsureDimensionStyle();
        }

        private void CreateLayerStructure(Layer parentLayer)
        {
            var blueprintLayer = FindOrCreateBlueprintLayer(parentLayer);
            if (blueprintLayer == null)
            {
                throw new InvalidOperationException("Failed to create Blueprint layer.");
            }

            Prepare3DPanelsLayer(blueprintLayer);
            FindOrCreateChildLayer(blueprintLayer, "2D Panels", Color.Black);
            FindOrCreateChildLayer(blueprintLayer, "Cutouts", Color.Black);
            int pocketLayerIndex = FindOrCreateChildLayer(blueprintLayer, "Pocket Curves", Color.Black);
            var pocketLayer = _doc.Layers[pocketLayerIndex];
            pocketLayer.IsVisible = false;
            _doc.Layers.Modify(pocketLayer, pocketLayerIndex, true);
            FindOrCreateChildLayer(blueprintLayer, "Dimensions", Color.Red);
        }

        private void EnsureDimensionStyle()
        {
            const string styleName = "Blueprint Dim - Default";
            if (_doc.DimStyles.FindName(styleName) != null)
            {
                return;
            }

            if (LoggingService.IsEnabled(LogLevel.Debug))
            {
                LoggingService.Debug($"[Blueprint Styles] Dimension style '{styleName}' is missing; ensure the resource model is available.");
            }
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

        private void Prepare3DPanelsLayer(Layer blueprintLayer)
        {
            string panelsPath = $"{blueprintLayer.FullPath}::Panels";
            int index = GetLayerIndexByFullPath(panelsPath);
            if (index >= 0)
            {
                var panelsLayer = _doc.Layers[index];
                panelsLayer.Name = "3D Panels";
                panelsLayer.Color = Color.Black;
                _doc.Layers.Modify(panelsLayer, index, true);
                return;
            }

            FindOrCreateChildLayer(blueprintLayer, "3D Panels", Color.Black);
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
    }
}
