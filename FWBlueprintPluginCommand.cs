using System;
using System.Collections.Generic;
using FWBlueprintPlugin.Services;
using FWBlueprintPlugin.Services.Dimensioning;
using FWBlueprintPlugin.Services.Extraction;
using FWBlueprintPlugin.Services.Layout;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;

namespace FWBlueprintPlugin
{
    public class FWBlueprintPluginCommand : Command
    {
        public FWBlueprintPluginCommand()
        {
            Instance = this;
        }

        public static FWBlueprintPluginCommand Instance { get; private set; }

        public override string EnglishName => "FWBlueprint";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if (doc == null)
            {
                return Result.Failure;
            }

            try
            {
                var annotationStyleProvider = new BlueprintAnnotationStyleProvider(doc);
                annotationStyleProvider.EnsureStyles();

                var selectionService = new PanelSelectionService(doc);

                var topComponents = selectionService.SelectTopComponents();
                if (topComponents == null || topComponents.Count == 0)
                {
                    return Result.Cancel;
                }

                var topConfig = selectionService.DetectLiftLidComponents(topComponents);
                if (topConfig == null)
                {
                    return Result.Cancel;
                }

                HideObjects(doc, topComponents);

                var backPanel = selectionService.SelectSinglePanel("Select the BACK PANEL");
                if (backPanel == null)
                {
                    return Result.Cancel;
                }
                HideObject(doc, backPanel);

                var bottomPanel = selectionService.SelectSinglePanel("Select the BOTTOM PANEL");
                if (bottomPanel == null)
                {
                    return Result.Cancel;
                }
                HideObject(doc, bottomPanel);

                var exteriorWalls = selectionService.SelectMultiple("Select EXTERIOR WALLS (Press Enter when done)") ?? new List<RhinoObject>();
                HideObjects(doc, exteriorWalls);

                var interiorWalls = selectionService.SelectMultiple("Select INTERIOR WALLS (Press Enter when done)") ?? new List<RhinoObject>();
                HideObjects(doc, interiorWalls);

                var removableShelves = selectionService.SelectMultiple("Select REMOVABLE SHELVES (Press Enter when done)") ?? new List<RhinoObject>();
                HideObjects(doc, removableShelves);

                var permanentShelves = selectionService.SelectMultiple("Select PERMANENT SHELVES (Press Enter when done)") ?? new List<RhinoObject>();
                HideObjects(doc, permanentShelves);

                var doors = selectionService.SelectMultiple("Select DOORS (Press Enter when done)") ?? new List<RhinoObject>();
                HideObjects(doc, doors);

                var parentLayer = doc.Layers[topComponents[0].Attributes.LayerIndex];
                if (parentLayer == null)
                {
                    return Result.Failure;
                }

                var arrangementService = new PanelArrangementService(doc);
                arrangementService.SetupLayersAndStyles(parentLayer);

                var arrangementResult = arrangementService.ArrangePanels(
                    topConfig,
                    backPanel,
                    bottomPanel,
                    exteriorWalls,
                    interiorWalls,
                    removableShelves,
                    permanentShelves,
                    doors,
                    parentLayer);

                if (!arrangementResult.Success)
                {
                    return Result.Failure;
                }

                var dimensionService = new PanelDimensioningService(doc, arrangementResult.PanelThicknesses);

                foreach (var dimInfo in arrangementResult.DeferredDimensions)
                {
                    dimensionService.AddPanelDimensions(dimInfo);
                }

                foreach (var leaderInfo in arrangementResult.DeferredLeaders)
                {
                    dimensionService.AddPanelLeader(leaderInfo.BBox, leaderInfo.PanelType, leaderInfo.Quantity, leaderInfo.LayerIndex);
                }

                var layerSetupService = new LayerSetupService(doc);
                var edgeDimensioningService = new EdgeDimensioningService(doc);
                var edgeFeatureDetectionService = new EdgeFeatureDetectionService(edgeDimensioningService);
                var phase3ExtractionService = new Phase3ExtractionService(doc, layerSetupService, edgeFeatureDetectionService);

                var extractionResult = phase3ExtractionService.ExtractPanels(parentLayer);

                RhinoApp.WriteLine($"Phase 3 extraction summary: {extractionResult.BoundingBoxCount} panels, {extractionResult.EdgeFeatureCount} edge features, {extractionResult.CutoutCount} interior cutouts, {extractionResult.ChordCutouts.Count} chord groups.");

                doc.Views.Redraw();
                return Result.Success;
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"FWBlueprint command failed: {ex.Message}");
                return Result.Failure;
            }
        }

        private static void HideObjects(RhinoDoc doc, IEnumerable<RhinoObject> objects)
        {
            if (doc == null || objects == null)
            {
                return;
            }

            var hidden = false;
            foreach (var obj in objects)
            {
                if (obj == null)
                {
                    continue;
                }

                if (doc.Objects.Hide(obj.Id, true))
                {
                    hidden = true;
                }
            }

            if (hidden)
            {
                doc.Views.Redraw();
            }
        }

        private static void HideObject(RhinoDoc doc, RhinoObject obj)
        {
            if (obj == null)
            {
                return;
            }

            HideObjects(doc, new[] { obj });
        }
    }
}
