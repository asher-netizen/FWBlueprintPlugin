// FILE: FWBlueprint_Phase3A_EdgeIntersections.cs
// Phase 3A: 2D panel extraction + edge notch detection (edge-line intersections) + interior hole detection + auto-dimensioning

using Rhino;
using Rhino.Commands;
using Rhino.Display;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Rhino.Input;
using Rhino.Input.Custom;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;

namespace FWBlueprintPlugin
{
    public partial class FWBlueprintPluginCommand : Command
    {
        public FWBlueprintPluginCommand()
        {
            Instance = this;
        }

        public static FWBlueprintPluginCommand Instance { get; private set; }

        public override string EnglishName => "FWBlueprint";

        // Dictionary to store material thickness before flattening
        private Dictionary<string, double> panelThicknesses = new Dictionary<string, double>();
        private List<BoundingBox> allPanelBounds = new List<BoundingBox>();
        private List<BoundingBox> allDimensionBounds = new List<BoundingBox>();

        // Helper class to store panel dimension info for deferred dimensioning
        private class PanelDimensionInfo
        {
            public BoundingBox BBox { get; set; }
            public string PanelType { get; set; }
            public int Quantity { get; set; }
            public bool IsChild { get; set; }
            public BoundingBox? GroupBBox { get; set; }
            public bool IsRightHeightDim { get; set; }
            public int LayerIndex { get; set; }
            public bool DrawWidth { get; set; } = true;
            public bool DrawHeight { get; set; } = true;
            public double? CustomHeightOffset { get; set; } = null;  // NEW: Optional custom offset
        }

        // Helper class for lift lid top components
        private class LiftLidTopComponents
        {
            public List<RhinoObject> BackerPlates { get; set; } = new List<RhinoObject>();
            public List<RhinoObject> LiftLids { get; set; } = new List<RhinoObject>();
            public List<RhinoObject> TopPlates { get; set; } = new List<RhinoObject>();
            public int TotalComponentCount { get; set; }
        }

        // Helper class for flattened lift lid info
        private class LiftLidFlattenedInfo
        {
            public BoundingBox OverallBBox { get; set; }
            public List<BoundingBox> WidthGroupBBoxes { get; set; } = new List<BoundingBox>();
            public List<BoundingBox> BackerDepthBBoxes { get; set; } = new List<BoundingBox>();
            public List<BoundingBox> LidDepthBBoxes { get; set; } = new List<BoundingBox>();
        }

        // Struct for flattened components (replaces named tuple for compatibility)
        private struct FlattenedComponent
        {
            public GeometryBase Geom { get; set; }
            public BoundingBox BBox { get; set; }
        }

        private List<PanelDimensionInfo> deferredDimensions = new List<PanelDimensionInfo>();
        private List<Tuple<BoundingBox, string, int, int>> deferredLeaders = new List<Tuple<BoundingBox, string, int, int>>();

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            // Clear previous data
            panelThicknesses.Clear();
            allPanelBounds.Clear();
            allDimensionBounds.Clear();
            deferredDimensions.Clear();
            deferredLeaders.Clear();


            // Step 1: Select Top Panel Components (Multi-select for lift lid support)
            var topComponents = SelectMultipleTopComponents(doc);
            if (topComponents == null || topComponents.Count == 0)
            {
                return Result.Cancel;
            }

            // Detect and confirm lift lid configuration
            var topConfig = DetectLiftLidComponents(doc, topComponents);
            if (topConfig == null)
            {
                return Result.Cancel;
            }

            // Hide top components after confirmation
            foreach (var obj in topComponents)
            {
                doc.Objects.Hide(obj.Id, true);
            }
            doc.Views.Redraw();

            // Step 2: Select Back Panel
            var backPanel = SelectObject(doc, "Select the BACK PANEL");
            if (backPanel == null)
            {
                return Result.Cancel;
            }
            doc.Objects.Hide(backPanel.Id, true);
            doc.Views.Redraw();

            // Step 3: Select Bottom Panel
            var bottomPanel = SelectObject(doc, "Select the BOTTOM PANEL");
            if (bottomPanel == null)
            {
                return Result.Cancel;
            }
            doc.Objects.Hide(bottomPanel.Id, true);
            doc.Views.Redraw();

            // Step 4: Select Exterior Walls
            var exteriorWalls = SelectMultipleObjects(doc, "Select EXTERIOR WALLS (Press Enter when done)");
            if (exteriorWalls == null || exteriorWalls.Count == 0)
            {
                exteriorWalls = new List<RhinoObject>();
            }
            else
            {
                foreach (var wall in exteriorWalls)
                {
                    doc.Objects.Hide(wall.Id, true);
                }
                doc.Views.Redraw();
            }

            // Step 5: Select Interior Walls
            var interiorWalls = SelectMultipleObjects(doc, "Select INTERIOR WALLS (Press Enter when done)");
            if (interiorWalls == null || interiorWalls.Count == 0)
            {
                interiorWalls = new List<RhinoObject>();
            }
            else
            {
                foreach (var wall in interiorWalls)
                {
                    doc.Objects.Hide(wall.Id, true);
                }
                doc.Views.Redraw();
            }

            // Step 6: Select Removable Shelves
            var removableShelves = SelectMultipleObjects(doc, "Select REMOVABLE SHELVES (Press Enter when done)");
            if (removableShelves == null || removableShelves.Count == 0)
            {
                removableShelves = new List<RhinoObject>();
            }
            else
            {
                foreach (var shelf in removableShelves)
                {
                    doc.Objects.Hide(shelf.Id, true);
                }
                doc.Views.Redraw();
            }

            // Step 7: Select Permanent Shelves
            var permanentShelves = SelectMultipleObjects(doc, "Select PERMANENT SHELVES (Press Enter when done)");
            if (permanentShelves == null || permanentShelves.Count == 0)
            {
                permanentShelves = new List<RhinoObject>();
            }
            else
            {
                foreach (var shelf in permanentShelves)
                {
                    doc.Objects.Hide(shelf.Id, true);
                }
                doc.Views.Redraw();
            }

            // Step 8: Select Doors
            var doors = SelectMultipleObjects(doc, "Select DOORS (Press Enter when done)");
            if (doors == null || doors.Count == 0)
            {
                doors = new List<RhinoObject>();
            }
            else
            {
                foreach (var door in doors)
                {
                    doc.Objects.Hide(door.Id, true);
                }
                doc.Views.Redraw();
            }


            // Setup layers and styles
            var parentLayer = doc.Layers[topComponents[0].Attributes.LayerIndex];
            SetupLayersAndStyles(doc, parentLayer);

            // Flatten and arrange panels
            bool success = FlattenAndArrangePanels(doc, topConfig, backPanel, bottomPanel,
                                                   exteriorWalls, interiorWalls,
                                                   removableShelves, permanentShelves, doors,
                                                   parentLayer);

            if (success)
            {
                doc.Views.Redraw();
                return Result.Success;
            }
            else
            {
                return Result.Failure;
            }
        }

        private List<RhinoObject> SelectMultipleTopComponents(RhinoDoc doc)
        {

            var go = new GetObject();
            go.SetCommandPrompt("Select TOP PANEL components (Press Enter when done)");
            go.GeometryFilter = ObjectType.Surface | ObjectType.PolysrfFilter | ObjectType.Brep;
            go.SubObjectSelect = false;
            go.DisablePreSelect();
            go.DeselectAllBeforePostSelect = false;
            go.GroupSelect = true;
            go.EnableClearObjectsOnEntry(false);

            go.GetMultiple(1, 0);

            if (go.CommandResult() != Result.Success)
                return null;

            var objects = new List<RhinoObject>();
            for (int i = 0; i < go.ObjectCount; i++)
            {
                var objRef = go.Object(i);
                objects.Add(objRef.Object());
            }

            return objects;
        }

        private LiftLidTopComponents DetectLiftLidComponents(RhinoDoc doc, List<RhinoObject> topComponents)
        {

            var config = new LiftLidTopComponents();
            config.TotalComponentCount = topComponents.Count;

            // CRITICAL: If only 1 component, it's a simple single-piece top
            if (topComponents.Count == 1)
            {
                config.TopPlates = topComponents;
                return config;
            }

            // Calculate heights (depths) for all components
            var componentData = new List<(RhinoObject obj, BoundingBox bbox, double height, double width)>();
            foreach (var obj in topComponents)
            {
                var bbox = obj.Geometry.GetBoundingBox(true);
                double height = bbox.Max.Y - bbox.Min.Y; // Front-to-back depth
                double width = bbox.Max.X - bbox.Min.X;
                componentData.Add((obj, bbox, height, width));

            }

            // Find minimum height (backer plate candidates)
            double minHeight = componentData.Min(c => c.height);
            const double heightTolerance = 0.1;


            var backerCandidates = componentData.Where(c => Math.Abs(c.height - minHeight) < heightTolerance).ToList();


            // Identify backer plates and their corresponding lift lids
            var remainingComponents = new List<RhinoObject>(topComponents);

            foreach (var backerData in backerCandidates)
            {
                config.BackerPlates.Add(backerData.obj);
                remainingComponents.Remove(backerData.obj);

                double backerWidth = backerData.width;
                double backerCenterX = (backerData.bbox.Min.X + backerData.bbox.Max.X) / 2;
                double backerMinY = backerData.bbox.Min.Y;
                double backerMaxY = backerData.bbox.Max.Y;


                // Find lift lid: same width, horizontally aligned, adjacent
                RhinoObject matchingLiftLid = null;
                double bestScore = double.MaxValue;

                foreach (var potentialLid in remainingComponents.ToList())
                {
                    var lidBBox = potentialLid.Geometry.GetBoundingBox(true);
                    double lidWidth = lidBBox.Max.X - lidBBox.Min.X;
                    double lidCenterX = (lidBBox.Min.X + lidBBox.Max.X) / 2;
                    double lidMinY = lidBBox.Min.Y;
                    double lidMaxY = lidBBox.Max.Y;
                    double lidHeight = lidMaxY - lidMinY;

                    // Width match (within 1.0")
                    double widthDiff = Math.Abs(lidWidth - backerWidth);
                    bool widthMatches = widthDiff < 1.0;

                    // X alignment (within 2.0")
                    double xDiff = Math.Abs(lidCenterX - backerCenterX);
                    bool xAligned = xDiff < 2.0;

                    // Y adjacency
                    double frontGap = Math.Abs(lidMinY - backerMaxY);
                    double backGap = Math.Abs(lidMaxY - backerMinY);
                    double minGap = Math.Min(frontGap, backGap);
                    bool adjacent = minGap < 1.0;

                    // Lid should be larger than backer
                    bool largerHeight = lidHeight > minHeight + 0.5;


                    if (widthMatches && xAligned && adjacent && largerHeight)
                    {
                        double score = widthDiff + xDiff + minGap;


                        if (score < bestScore)
                        {
                            bestScore = score;
                            matchingLiftLid = potentialLid;
                        }
                    }
                }

                if (matchingLiftLid != null)
                {
                    config.LiftLids.Add(matchingLiftLid);
                    remainingComponents.Remove(matchingLiftLid);
                }
                else
                {
                }
            }

            // Remaining components are top plates
            config.TopPlates = remainingComponents;


            // Visual confirmation
            return ConfirmTopConfiguration(doc, config);
        }

        private LiftLidTopComponents ConfirmTopConfiguration(RhinoDoc doc, LiftLidTopComponents config)
        {
            // Color-code the components
            var originalColors = new Dictionary<Guid, System.Drawing.Color>();

            // Blue for backers
            foreach (var obj in config.BackerPlates)
            {
                originalColors[obj.Id] = obj.Attributes.ObjectColor;
                obj.Attributes.ObjectColor = System.Drawing.Color.Blue;
                obj.Attributes.ColorSource = ObjectColorSource.ColorFromObject;
                obj.CommitChanges();
            }

            // Green for lift lids
            foreach (var obj in config.LiftLids)
            {
                originalColors[obj.Id] = obj.Attributes.ObjectColor;
                obj.Attributes.ObjectColor = System.Drawing.Color.Green;
                obj.Attributes.ColorSource = ObjectColorSource.ColorFromObject;
                obj.CommitChanges();
            }

            // Red for top plates
            foreach (var obj in config.TopPlates)
            {
                originalColors[obj.Id] = obj.Attributes.ObjectColor;
                obj.Attributes.ObjectColor = System.Drawing.Color.Red;
                obj.Attributes.ColorSource = ObjectColorSource.ColorFromObject;
                obj.CommitChanges();
            }

            doc.Views.Redraw();


            var gk = new GetString();
            gk.SetCommandPrompt("[Enter]=Correct and continue  [B]=Reselect backers  [L]=Reselect lift lids  [Esc]=Cancel");
            gk.AcceptNothing(true);
            gk.AddOption("Backers");
            gk.AddOption("LiftLids");

            var result = gk.Get();

            // Restore original colors
            foreach (var kvp in originalColors)
            {
                var obj = doc.Objects.FindId(kvp.Key);
                if (obj != null)
                {
                    obj.Attributes.ObjectColor = kvp.Value;
                    obj.Attributes.ColorSource = ObjectColorSource.ColorFromLayer;
                    obj.CommitChanges();
                }
            }
            doc.Views.Redraw();

            if (result == GetResult.Cancel)
            {
                return null;
            }
            else if (result == GetResult.Option)
            {
                string option = gk.Option().EnglishName;
                if (option == "Backers")
                {
                    return config;
                }
                else if (option == "LiftLids")
                {
                    return config;
                }
            }

            return config;
        }

        private RhinoObject SelectObject(RhinoDoc doc, string prompt)
        {
            var go = new GetObject();
            go.SetCommandPrompt(prompt);
            go.GeometryFilter = ObjectType.Surface | ObjectType.PolysrfFilter | ObjectType.Brep;
            go.SubObjectSelect = false;
            go.DisablePreSelect();
            go.DeselectAllBeforePostSelect = false;
            go.Get();

            if (go.CommandResult() != Result.Success)
                return null;

            if (go.ObjectCount == 1)
            {
                var objRef = go.Object(0);
                return objRef.Object();
            }

            return null;
        }

        private List<RhinoObject> SelectMultipleObjects(RhinoDoc doc, string prompt)
        {
            var go = new GetObject();
            go.SetCommandPrompt(prompt);
            go.GeometryFilter = ObjectType.Surface | ObjectType.PolysrfFilter | ObjectType.Brep;
            go.SubObjectSelect = false;
            go.DisablePreSelect();
            go.DeselectAllBeforePostSelect = false;
            go.GroupSelect = true;
            go.EnableClearObjectsOnEntry(false);

            go.GetMultiple(1, 0);

            if (go.CommandResult() != Result.Success)
                return null;

            var objects = new List<RhinoObject>();
            for (int i = 0; i < go.ObjectCount; i++)
            {
                var objRef = go.Object(i);
                objects.Add(objRef.Object());
            }

            return objects;
        }

        private void SetupLayersAndStyles(RhinoDoc doc, Layer parentLayer)
        {
            CreateLayerStructure(doc, parentLayer);
            CreateDimensionStyle(doc);
        }

        private void CreateLayerStructure(RhinoDoc doc, Layer parentLayer)
        {
            var blueprintLayer = new Layer();
            blueprintLayer.Name = "Blueprint";
            blueprintLayer.ParentLayerId = parentLayer.Id;
            blueprintLayer.Color = System.Drawing.Color.Black;
            int blueprintIndex = doc.Layers.Add(blueprintLayer);

            if (blueprintIndex < 0)
            {
                for (int i = 0; i < doc.Layers.Count; i++)
                {
                    if (doc.Layers[i].Name == "Blueprint" && doc.Layers[i].ParentLayerId == parentLayer.Id)
                    {
                        blueprintIndex = i;
                        break;
                    }
                }
            }

            var blueprintLayerObj = doc.Layers[blueprintIndex];

            var panelsLayer = new Layer();
            panelsLayer.Name = "Panels";
            panelsLayer.ParentLayerId = blueprintLayerObj.Id;
            panelsLayer.Color = System.Drawing.Color.Black;
            doc.Layers.Add(panelsLayer);

            var dimensionsLayer = new Layer();
            dimensionsLayer.Name = "Dimensions";
            dimensionsLayer.ParentLayerId = blueprintLayerObj.Id;
            dimensionsLayer.Color = System.Drawing.Color.Red;
            doc.Layers.Add(dimensionsLayer);

        }

        private void CreateDimensionStyle(RhinoDoc doc)
        {
            const string styleName = "Furniture Dim - CG";

            bool styleExists = false;
            for (int i = 0; i < doc.DimStyles.Count; i++)
            {
                if (doc.DimStyles[i].Name == styleName)
                {
                    styleExists = true;
                    break;
                }
            }

            if (styleExists)
            {
                return;
            }

        }

        private double CalculateThickness(GeometryBase geometry)
        {
            var bbox = geometry.GetBoundingBox(true);
            double xSize = bbox.Max.X - bbox.Min.X;
            double ySize = bbox.Max.Y - bbox.Min.Y;
            double zSize = bbox.Max.Z - bbox.Min.Z;

            return Math.Min(Math.Min(xSize, ySize), zSize);
        }

        private bool FlattenAndArrangePanels(
    RhinoDoc doc,
    LiftLidTopComponents topConfig,
    RhinoObject backPanel,
    RhinoObject bottomPanel,
    List<RhinoObject> exteriorWalls,
    List<RhinoObject> interiorWalls,
    List<RhinoObject> removableShelves,
    List<RhinoObject> permanentShelves,
    List<RhinoObject> doors,
    Layer parentLayer)
        {
            try
            {
                int panelsLayerIndex = FindLayerIndex(doc, parentLayer, "Blueprint", "Panels");
                int dimensionsLayerIndex = FindLayerIndex(doc, parentLayer, "Blueprint", "Dimensions");

                double verticalSpacing = 12.0;
                double behindConsoleOffset = 300.0;
                double sheetSpacing = 100.0;

                // Calculate console bounding box from top components
                BoundingBox consoleBBox = BoundingBox.Empty;
                var allTopComponents = topConfig.BackerPlates
                    .Concat(topConfig.LiftLids)
                    .Concat(topConfig.TopPlates)
                    .ToList();

                foreach (var obj in allTopComponents)
                    consoleBBox.Union(obj.Geometry.GetBoundingBox(true));

                double leftAlignX = consoleBBox.Min.X;
                double startY = consoleBBox.Max.Y + behindConsoleOffset;

                // Store thicknesses for top components
                int topIdx = 0;
                foreach (var obj in allTopComponents)
                {
                    panelThicknesses[$"Top_{topIdx}"] = CalculateThickness(obj.Geometry);
                    topIdx++;
                }

                panelThicknesses["Back"] = CalculateThickness(backPanel.Geometry);
                panelThicknesses["Bottom"] = CalculateThickness(bottomPanel.Geometry);

                // === PHASE 1: POSITION ALL PANELS (NO DIMENSIONS YET) ===


                var topInfo = FlattenLiftLidTop(doc, topConfig, leftAlignX, startY, panelsLayerIndex);
                allPanelBounds.Add(topInfo.OverallBBox);

                // Defer dimensioning for top
                DeferLiftLidTopDimensions(topConfig, topInfo, dimensionsLayerIndex);

                // Position Back Panel
                double currentY = topInfo.OverallBBox.Min.Y - verticalSpacing;
                double maxX;
                currentY = ProcessPanelsHorizontalButt(
                    doc,
                    new List<RhinoObject> { backPanel },
                    "Back",
                    leftAlignX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out maxX);

                // Position Bottom Panel
                currentY = ProcessPanelsHorizontalButt(
                    doc,
                    new List<RhinoObject> { bottomPanel },
                    "Bottom",
                    leftAlignX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out maxX);


                double sheet1MaxX = Math.Max(topInfo.OverallBBox.Max.X, maxX);
                double sheet2StartX = sheet1MaxX + sheetSpacing;

                // === SHEET 2: Walls & Shelves ===
                currentY = startY;
                double sheet2MaxX = sheet2StartX;

                currentY = ProcessPanelsHorizontalButt(
                    doc,
                    exteriorWalls,
                    "Exterior Wall",
                    sheet2StartX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out sheet2MaxX);

                currentY = ProcessPanelsHorizontalButt(
                    doc,
                    interiorWalls,
                    "Interior Wall",
                    sheet2StartX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out sheet2MaxX);

                currentY = ProcessPanelsHorizontalButt(
                    doc,
                    removableShelves,
                    "Removable Shelf",
                    sheet2StartX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out sheet2MaxX);

                double sheet3StartX = sheet2MaxX + sheetSpacing;

                // === SHEET 3: Permanent Shelves & Doors ===
                currentY = startY;

                double permMaxX;
                currentY = ProcessPanelsHorizontalButt(
                    doc,
                    permanentShelves,
                    "Permanent Shelf",
                    sheet3StartX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out permMaxX);

                double doorMaxX;
                currentY = ProcessPanelsHorizontalButt(
                    doc,
                    doors,
                    "Door",
                    sheet3StartX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out doorMaxX);


                // === Switch to Top View ===
                SwitchToTopViewportAndCenter(doc);

                // === PHASE 2: ADD DIMENSIONS & LEADERS ===


                foreach (var dimInfo in deferredDimensions)
                    AddPanelDimensions(doc, dimInfo);

                foreach (var leaderInfo in deferredLeaders)
                    AddPanelLeader(doc, leaderInfo.Item1, leaderInfo.Item2, leaderInfo.Item3, leaderInfo.Item4);


                // === ✅ PHASE 3: CALL NEW 2D EXTRACTION ===
                ExtractAndCreate2DPanels(doc, parentLayer);


                return true;

            }
            catch (Exception ex)
            {
                return false;
            }
        }


        private LiftLidFlattenedInfo FlattenLiftLidTop(RhinoDoc doc, LiftLidTopComponents config, double leftAlignX, double topY, int layerIndex)
        {
            var flattenedBackers = new List<FlattenedComponent>();
            var flattenedLids = new List<FlattenedComponent>();
            var flattenedTopPlates = new List<FlattenedComponent>();

            foreach (var obj in config.BackerPlates)
            {
                var flattened = FlattenPanel(obj.Geometry, "Top");
                flattenedBackers.Add(new FlattenedComponent { Geom = flattened, BBox = flattened.GetBoundingBox(true) });
            }

            foreach (var obj in config.LiftLids)
            {
                var flattened = FlattenPanel(obj.Geometry, "Top");
                flattenedLids.Add(new FlattenedComponent { Geom = flattened, BBox = flattened.GetBoundingBox(true) });
            }

            foreach (var obj in config.TopPlates)
            {
                var flattened = FlattenPanel(obj.Geometry, "Top");
                flattenedTopPlates.Add(new FlattenedComponent { Geom = flattened, BBox = flattened.GetBoundingBox(true) });
            }

            if (flattenedBackers.Count + flattenedLids.Count + flattenedTopPlates.Count == 0)
            {
                return new LiftLidFlattenedInfo { OverallBBox = BoundingBox.Empty };
            }

            // Calculate overall bbox in original position
            BoundingBox overallBBox = BoundingBox.Empty;
            Action<List<FlattenedComponent>> unionAll = list =>
            {
                foreach (var f in list)
                {
                    overallBBox.Union(f.BBox);
                }
            };
            unionAll(flattenedBackers);
            unionAll(flattenedLids);
            unionAll(flattenedTopPlates);

            // Translate to target position (align to left, top)
            var translate = Transform.Translation(leftAlignX - overallBBox.Min.X, topY - overallBBox.Max.Y, 0);

            Action<List<FlattenedComponent>> transformList = list =>
            {
                for (int i = 0; i < list.Count; i++)
                {
                    var item = list[i];
                    item.Geom.Transform(translate);
                    var newBBox = item.BBox;
                    newBBox.Transform(translate);
                    list[i] = new FlattenedComponent { Geom = item.Geom, BBox = newBBox };
                    AddToLayer(doc, item.Geom, layerIndex, "Top");
                }
            };

            transformList(flattenedBackers);
            transformList(flattenedLids);
            transformList(flattenedTopPlates);

            overallBBox.Transform(translate);

            // Build width groups
            var widthGroups = flattenedTopPlates.Select(f => f.BBox).ToList();
            var backerDepths = flattenedBackers.Select(f => f.BBox).ToList();
            var lidDepths = flattenedLids.Select(f => f.BBox).ToList();

            // FIXED: Union doesn't return value, so use temporary variable
            for (int i = 0; i < flattenedBackers.Count && i < flattenedLids.Count; i++)
            {
                var pairBBox = flattenedBackers[i].BBox;
                pairBBox.Union(flattenedLids[i].BBox);
                widthGroups.Add(pairBBox);
            }

            // Sort by Min.X
            widthGroups = widthGroups.OrderBy(b => b.Min.X).ToList();

            return new LiftLidFlattenedInfo
            {
                OverallBBox = overallBBox,
                WidthGroupBBoxes = widthGroups,
                BackerDepthBBoxes = backerDepths,
                LidDepthBBoxes = lidDepths
            };
        }

        private void DeferLiftLidTopDimensions(LiftLidTopComponents config, LiftLidFlattenedInfo info, int dimensionsLayerIndex)
        {
            var totalCount = config.TotalComponentCount;
            var panelType = totalCount > 1 ? "Tops" : "Top";
            var quantity = totalCount;

            if (totalCount == 1)
            {
                // Simple single-piece top
                deferredDimensions.Add(new PanelDimensionInfo
                {
                    BBox = info.OverallBBox,
                    PanelType = panelType,
                    Quantity = quantity,
                    IsChild = false,
                    DrawWidth = true,
                    DrawHeight = true,
                    LayerIndex = dimensionsLayerIndex
                });
            }
            else
            {
                // Multi-component lift lid top

                // Parent dimension (8" offset) - total width and height
                deferredDimensions.Add(new PanelDimensionInfo
                {
                    BBox = info.OverallBBox,
                    PanelType = panelType,
                    Quantity = quantity,
                    IsChild = false,
                    GroupBBox = info.OverallBBox,
                    DrawWidth = true,
                    DrawHeight = true,
                    LayerIndex = dimensionsLayerIndex
                });

                // Child dimensions (4" offset) - individual component widths
                foreach (var childBBox in info.WidthGroupBBoxes)
                {
                    deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = childBBox,
                        PanelType = panelType,
                        IsChild = true,
                        DrawWidth = true,
                        DrawHeight = false,
                        LayerIndex = dimensionsLayerIndex
                    });
                }

                // IMPROVED: Depth dimensions with collision avoidance
                // Strategy: Use different offsets for backer vs lift lid to prevent overlap
                // Also extend short dimensions (< 4") to give text more room

                for (int i = 0; i < info.BackerDepthBBoxes.Count && i < info.LidDepthBBoxes.Count; i++)
                {
                    var backerBBox = info.BackerDepthBBoxes[i];
                    var lidBBox = info.LidDepthBBoxes[i];

                    double backerDepth = backerBBox.Max.Y - backerBBox.Min.Y;
                    double lidDepth = lidBBox.Max.Y - lidBBox.Min.Y;

                    // Determine offsets to avoid collision
                    // If backer is small (< 4"), give it extra offset and extend its dimension line
                    double backerOffset = 5.0;  // Standard
                    double lidOffset = 5.0;     // Standard

                    bool backerIsSmall = backerDepth < 4.0;
                    bool lidIsSmall = lidDepth < 4.0;

                    if (backerIsSmall && !lidIsSmall)
                    {
                        // Backer is small, lid is normal
                        // Option 1: Extend backer offset further left
                        backerOffset = 9.0;  // Extra offset to avoid lid's text
                    }
                    else if (!backerIsSmall && lidIsSmall)
                    {
                        // Lid is small, backer is normal
                        lidOffset = 9.0;
                    }
                    else if (backerIsSmall && lidIsSmall)
                    {
                        // Both are small - stagger them
                        backerOffset = 9.0;
                        lidOffset = 5.0;
                    }

                    // Check vertical overlap potential
                    // If they're close vertically (within 3"), increase separation
                    double verticalGap = Math.Abs(backerBBox.Center.Y - lidBBox.Center.Y);
                    if (verticalGap < 3.0 && backerIsSmall)
                    {
                        backerOffset = 11.0;  // Even more offset if they're close
                    }

                    // Backer depth - with adjusted offset
                    deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = backerBBox,
                        PanelType = "TopBackerDepth",  // Special type to trigger custom offset
                        DrawWidth = false,
                        DrawHeight = true,
                        IsRightHeightDim = false,
                        LayerIndex = dimensionsLayerIndex,
                        CustomHeightOffset = backerOffset  // NEW: Pass custom offset
                    });

                    // Lift lid depth - with adjusted offset
                    deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = lidBBox,
                        PanelType = "TopLiftLidDepth",  // Special type
                        DrawWidth = false,
                        DrawHeight = true,
                        IsRightHeightDim = false,
                        LayerIndex = dimensionsLayerIndex,
                        CustomHeightOffset = lidOffset  // NEW: Pass custom offset
                    });
                }
            }

            // Defer leader for the category
            deferredLeaders.Add(new Tuple<BoundingBox, string, int, int>(
                info.OverallBBox, panelType, quantity, dimensionsLayerIndex));
        }

        private double ProcessPanelsHorizontalButt(RhinoDoc doc, List<RhinoObject> panels, string panelType, double leftAlignX, double currentY, double verticalSpacing, int panelsLayerIndex, int dimensionsLayerIndex, out double maxXReached)
        {
            maxXReached = leftAlignX;

            if (panels == null || panels.Count == 0)
                return currentY;


            // Store thickness
            double categoryThickness = 0;
            for (int i = 0; i < panels.Count; i++)
            {
                string key = $"{panelType}_{i}";
                double thickness = CalculateThickness(panels[i].Geometry);
                panelThicknesses[key] = thickness;
                categoryThickness = thickness;
            }

            double currentX = leftAlignX;
            double categoryMaxHeight = 0;
            var categoryBounds = new List<BoundingBox>();

            // Flatten and position each panel (edge-to-edge, no gaps)
            foreach (var panel in panels)
            {
                var panelFlat = FlattenPanel(panel.Geometry, panelType);
                var panelBox = panelFlat.GetBoundingBox(true);

                // Position panel: X at currentX, Y aligned to top at currentY
                var panelMoveX = currentX - panelBox.Min.X;
                var panelMoveY = currentY - panelBox.Max.Y;

                panelFlat.Transform(Transform.Translation(panelMoveX, panelMoveY, 0));
                panelBox = panelFlat.GetBoundingBox(true);

                AddToLayer(doc, panelFlat, panelsLayerIndex, panelType);
                allPanelBounds.Add(panelBox);
                categoryBounds.Add(panelBox);

                // Next panel starts where this one ends (edge-to-edge butting)
                currentX = panelBox.Max.X;

                double panelHeight = panelBox.Max.Y - panelBox.Min.Y;
                categoryMaxHeight = Math.Max(categoryMaxHeight, panelHeight);
            }

            // Sort categoryBounds by Min.X (left to right)
            categoryBounds.Sort((a, b) => a.Min.X.CompareTo(b.Min.X));

            // Calculate group bbox
            BoundingBox groupBBox = categoryBounds[0];
            for (int i = 1; i < categoryBounds.Count; i++)
            {
                groupBBox.Union(categoryBounds[i]);
            }

            // === DEFER DIMENSIONS (using Grok's improved logic) ===
            if (categoryBounds.Count > 1)
            {
                // Parent dimension (8" offset) - width only
                deferredDimensions.Add(new PanelDimensionInfo
                {
                    BBox = groupBBox,
                    PanelType = panelType,
                    Quantity = panels.Count,
                    IsChild = false,
                    GroupBBox = groupBBox,
                    DrawWidth = true,
                    DrawHeight = false,
                    LayerIndex = dimensionsLayerIndex
                });

                // Child dimensions (4" offset) - width only
                foreach (var bbox in categoryBounds)
                {
                    deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = bbox,
                        PanelType = panelType,
                        IsChild = true,
                        DrawWidth = true,
                        DrawHeight = false,
                        LayerIndex = dimensionsLayerIndex
                    });
                }

                // Height dimensions - check if all same height
                double firstHeight = categoryBounds[0].Max.Y - categoryBounds[0].Min.Y;
                bool allSameHeight = true;
                List<int> differentHeightIndices = new List<int>();

                for (int i = 1; i < categoryBounds.Count; i++)
                {
                    double currentHeight = categoryBounds[i].Max.Y - categoryBounds[i].Min.Y;
                    if (Math.Abs(currentHeight - firstHeight) > 0.01)
                    {
                        allSameHeight = false;
                        differentHeightIndices.Add(i);
                    }
                }

                if (allSameHeight)
                {
                    // Single left height for entire group
                    deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = groupBBox,
                        PanelType = panelType,
                        DrawWidth = false,
                        DrawHeight = true,
                        IsRightHeightDim = false,
                        LayerIndex = dimensionsLayerIndex
                    });
                }
                else
                {
                    // Left height for first panel
                    deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = categoryBounds[0],
                        PanelType = panelType,
                        DrawWidth = false,
                        DrawHeight = true,
                        IsRightHeightDim = false,
                        LayerIndex = dimensionsLayerIndex
                    });

                    // Right height for each different-height panel
                    foreach (int i in differentHeightIndices)
                    {
                        deferredDimensions.Add(new PanelDimensionInfo
                        {
                            BBox = categoryBounds[i],
                            PanelType = panelType,
                            DrawWidth = false,
                            DrawHeight = true,
                            IsRightHeightDim = true,
                            LayerIndex = dimensionsLayerIndex
                        });
                    }
                }
            }
            else if (categoryBounds.Count == 1)
            {
                // Single panel - normal dimensions (5" offset)
                deferredDimensions.Add(new PanelDimensionInfo
                {
                    BBox = categoryBounds[0],
                    PanelType = panelType,
                    Quantity = 1,
                    IsChild = false,
                    DrawWidth = true,
                    DrawHeight = true,
                    LayerIndex = dimensionsLayerIndex
                });
            }

            // Defer leader for the category
            if (categoryBounds.Count > 0)
            {
                var rightmostBox = categoryBounds[categoryBounds.Count - 1];
                deferredLeaders.Add(new Tuple<BoundingBox, string, int, int>(
                    rightmostBox, panelType, panels.Count, dimensionsLayerIndex));
            }

            maxXReached = currentX;
            currentY -= (categoryMaxHeight + verticalSpacing);

            return currentY;
        }

        private void SwitchToTopViewportAndCenter(RhinoDoc doc)
        {
            try
            {

                // Try to find and activate "Top" viewport tab
                RhinoView topView = null;
                foreach (var view in doc.Views)
                {
                    if (view.MainViewport.Name.Equals("Top", StringComparison.OrdinalIgnoreCase))
                    {
                        topView = view;
                        break;
                    }
                }

                if (topView != null)
                {
                    doc.Views.ActiveView = topView;
                }
                else
                {
                    var activeView = doc.Views.ActiveView;
                    if (activeView != null)
                    {
                        activeView.MainViewport.ChangeToParallelProjection(true);
                        activeView.MainViewport.SetCameraLocation(new Point3d(0, 0, 100), false);
                        activeView.MainViewport.SetCameraDirection(new Vector3d(0, 0, -1), false);
                        activeView.MainViewport.CameraUp = Vector3d.YAxis;
                    }
                }

                // Calculate bounding box of all panels with 10% padding
                if (allPanelBounds.Count > 0)
                {
                    BoundingBox overallBBox = allPanelBounds[0];
                    for (int i = 1; i < allPanelBounds.Count; i++)
                    {
                        overallBBox.Union(allPanelBounds[i]);
                    }

                    // Add 10% padding
                    double xPadding = (overallBBox.Max.X - overallBBox.Min.X) * 0.1;
                    double yPadding = (overallBBox.Max.Y - overallBBox.Min.Y) * 0.1;
                    overallBBox.Inflate(xPadding, yPadding, 0);

                    // Zoom to fit
                    var activeView = doc.Views.ActiveView;
                    if (activeView != null)
                    {
                        activeView.MainViewport.ZoomBoundingBox(overallBBox);
                        activeView.Redraw();
                    }
                }

                doc.Views.Redraw();
            }
            catch (Exception ex)
            {
            }
        }

        private int FindLayerIndex(RhinoDoc doc, Layer parentLayer, string sublayer1, string sublayer2)
        {
            for (int i = 0; i < doc.Layers.Count; i++)
            {
                var layer = doc.Layers[i];
                if (layer.Name == sublayer2)
                {
                    var parent = doc.Layers.FindId(layer.ParentLayerId);
                    if (parent != null && parent.Name == sublayer1)
                    {
                        var grandparent = doc.Layers.FindId(parent.ParentLayerId);
                        if (grandparent != null && grandparent.Id == parentLayer.Id)
                        {
                            return i;
                        }
                    }
                }
            }
            return 0;
        }

        private void AddPanelDimensions(RhinoDoc doc, PanelDimensionInfo info)
        {
            var bbox = info.BBox;
            double width = bbox.Max.X - bbox.Min.X;
            double height = bbox.Max.Y - bbox.Min.Y;

            // CRITICAL: Use the TOP surface Z coordinate for dimension placement
            double dimZ = bbox.Max.Z + 0.01;

            // Offsets - use custom if provided
            double widthDimOffset;
            double heightDimOffset;

            if (info.CustomHeightOffset.HasValue)
            {
                // Use custom offset (for collision avoidance)
                heightDimOffset = info.CustomHeightOffset.Value;
                widthDimOffset = 5.0;  // Default for width
            }
            else if (info.GroupBBox.HasValue)
            {
                widthDimOffset = 8.0;
                heightDimOffset = 5.0;
            }
            else if (info.IsChild)
            {
                widthDimOffset = 4.0;
                heightDimOffset = 5.0;
            }
            else
            {
                widthDimOffset = 5.0;
                heightDimOffset = 5.0;
            }

            var redColor = System.Drawing.Color.FromArgb(255, 0, 0);

            if (info.DrawWidth)
            {
                // Width dimension at TOP of panel
                var widthPt1 = new Point3d(bbox.Min.X, bbox.Max.Y, dimZ);
                var widthPt2 = new Point3d(bbox.Max.X, bbox.Max.Y, dimZ);
                var widthLinePt = new Point3d((bbox.Min.X + bbox.Max.X) / 2, bbox.Max.Y + widthDimOffset, dimZ);

                var widthPlane = new Plane(new Point3d(0, 0, dimZ), Vector3d.ZAxis);

                // Create dimension with proper 3D points
                var widthDim = LinearDimension.Create(
                    AnnotationType.Aligned,
                    doc.DimStyles.Current,
                    widthPlane,
                    Vector3d.XAxis,
                    widthPt1,
                    widthPt2,
                    widthLinePt,
                    0);

                var attr = new ObjectAttributes();
                attr.LayerIndex = info.LayerIndex;
                attr.ColorSource = ObjectColorSource.ColorFromObject;
                attr.ObjectColor = redColor;

                var widthGuid = doc.Objects.AddLinearDimension(widthDim, attr);
                if (widthGuid != Guid.Empty)
                {
                    var dimObj = doc.Objects.FindId(widthGuid);
                    if (dimObj != null)
                    {
                        var dimBBox = dimObj.Geometry.GetBoundingBox(true);
                        dimBBox.Inflate(3.0);
                        allDimensionBounds.Add(dimBBox);
                    }
                }
            }

            if (info.DrawHeight)
            {
                // Height dimension at TOP of panel
                Point3d defPoint1 = new Point3d(bbox.Min.X, bbox.Min.Y, dimZ);
                Point3d defPoint2 = new Point3d(bbox.Min.X, bbox.Max.Y, dimZ);
                Point3d dimLinePoint = new Point3d(bbox.Min.X - heightDimOffset, (bbox.Min.Y + bbox.Max.Y) / 2, dimZ);

                if (info.IsRightHeightDim)
                {
                    defPoint1 = new Point3d(bbox.Max.X, bbox.Min.Y, dimZ);
                    defPoint2 = new Point3d(bbox.Max.X, bbox.Max.Y, dimZ);
                    dimLinePoint = new Point3d(bbox.Max.X + heightDimOffset, (bbox.Min.Y + bbox.Max.Y) / 2, dimZ);
                }

                var heightPlane = new Plane(new Point3d(0, 0, dimZ), Vector3d.ZAxis);
                Vector3d horizontalDirection = Vector3d.XAxis;
                DimensionStyle dimStyle = doc.DimStyles.Current;
                double rotationInPlane = Math.PI / 2.0;

                var heightDim = LinearDimension.Create(
                    AnnotationType.Rotated,
                    dimStyle,
                    heightPlane,
                    horizontalDirection,
                    defPoint1,
                    defPoint2,
                    dimLinePoint,
                    rotationInPlane);

                var attr2 = new ObjectAttributes();
                attr2.LayerIndex = info.LayerIndex;
                attr2.ColorSource = ObjectColorSource.ColorFromObject;
                attr2.ObjectColor = redColor;

                var heightGuid = doc.Objects.AddLinearDimension(heightDim, attr2);
                if (heightGuid != Guid.Empty)
                {
                    var dimObj = doc.Objects.FindId(heightGuid);
                    if (dimObj != null)
                    {
                        var dimBBox = dimObj.Geometry.GetBoundingBox(true);
                        dimBBox.Inflate(3.0);
                        allDimensionBounds.Add(dimBBox);
                    }
                }
            }

        }

        private void AddPanelLeader(RhinoDoc doc, BoundingBox bbox, string panelType, int quantity, int layerIndex)
        {
            // Build label lines with Title Case
            var lines = new List<string>();

            string panelName = panelType;
            if (quantity > 1)
            {
                if (panelName.EndsWith("Shelf"))
                    panelName = panelName.Replace("Shelf", "Shelves");
                else if (panelName.EndsWith("Wall"))
                    panelName = panelName + "s";
                else
                    panelName = panelName + "s";
            }
            else
            {
                panelName = panelName + " Panel";
            }
            lines.Add("• " + panelName);

            if (quantity > 1)
            {
                lines.Add($"• (Qty: {quantity})");
            }

            double thickness = 0.75;
            foreach (var key in panelThicknesses.Keys)
            {
                if (key.StartsWith(panelType) || key == panelType)
                {
                    thickness = panelThicknesses[key];
                    break;
                }
            }

            if (Math.Abs(thickness - 0.75) > 0.001)
            {
                lines.Add("• " + FormatDimension(thickness, 4) + " Stock");
            }

            // Arrow target: 1" to the right, 40% down from top-right corner
            double panelHeight = bbox.Max.Y - bbox.Min.Y;
            Point3d arrowTarget = new Point3d(
                bbox.Max.X + 1.0,
                bbox.Max.Y - (panelHeight * 0.4),
                0);

            // Diagonal endpoint: 2.5" above the top of the panel
            double diagonalEndY = bbox.Max.Y + 2.5;

            // Calculate X position for diagonal end based on 45-degree angle
            double diagonalRise = diagonalEndY - arrowTarget.Y;
            Point3d diagonalEnd = new Point3d(
                arrowTarget.X + diagonalRise,
                diagonalEndY,
                0);

            // Jog endpoint: always 3" horizontal from diagonal end
            Point3d jogEnd = new Point3d(
                diagonalEnd.X + 3.0,
                diagonalEnd.Y,
                0);

            // Get text height from current dimension style with proper scaling
            DimensionStyle dimStyle = doc.DimStyles.Current;
            double baseTextHeight = dimStyle.TextHeight;
            double dimScale = dimStyle.DimensionScale;

            double textHeight = baseTextHeight * dimScale;
            double lineSpacing = textHeight * 1.5;
            double labelGap = textHeight * 1.2;

            // Leader line points: arrow -> diagonal end -> jog end
            var leaderPoints = new List<Point3d> { arrowTarget, diagonalEnd, jogEnd };
            var leaderLine = new PolylineCurve(leaderPoints);

            var leaderAttr = new ObjectAttributes();
            leaderAttr.LayerIndex = layerIndex;
            leaderAttr.ColorSource = ObjectColorSource.ColorFromObject;
            leaderAttr.ObjectColor = System.Drawing.Color.FromArgb(255, 0, 0);
            doc.Objects.AddCurve(leaderLine, leaderAttr);

            // Add arrowhead
            AddStreamlinedArrowhead(doc, arrowTarget, diagonalEnd, layerIndex);

            // Add text labels
            Point3d textStart = new Point3d(jogEnd.X + labelGap, jogEnd.Y, 0);

            for (int i = 0; i < lines.Count; i++)
            {
                double visualOffset = textHeight * 0.5;
                var textPt = new Point3d(textStart.X, textStart.Y - (i * lineSpacing) - visualOffset, 0);
                var textPlane = new Plane(textPt, Vector3d.ZAxis);

                var font = new Rhino.DocObjects.Font("Century Gothic Bold");

                var text = new TextEntity
                {
                    Plane = textPlane,
                    Text = lines[i],
                    TextHeight = textHeight,
                    Font = font,
                    TextHorizontalAlignment = TextHorizontalAlignment.Left,
                    TextVerticalAlignment = TextVerticalAlignment.BottomOfTop,
                    DimensionStyleId = dimStyle.Id
                };

                var textAttr = new ObjectAttributes();
                textAttr.LayerIndex = layerIndex;
                textAttr.ColorSource = ObjectColorSource.ColorFromObject;
                textAttr.ObjectColor = System.Drawing.Color.FromArgb(255, 0, 0);
                textAttr.Space = Rhino.DocObjects.ActiveSpace.ModelSpace;

                doc.Objects.AddText(text, textAttr);
            }

        }

        private void AddStreamlinedArrowhead(RhinoDoc doc, Point3d tip, Point3d direction, int layerIndex)
        {
            Vector3d dir = direction - tip;
            dir.Unitize();

            double arrowLength = 0.9;
            double arrowWidth = 0.24;

            Vector3d perpendicular = Vector3d.CrossProduct(dir, Vector3d.ZAxis);
            perpendicular.Unitize();

            Point3d base1 = tip + (dir * arrowLength) + (perpendicular * arrowWidth);
            Point3d base2 = tip + (dir * arrowLength) - (perpendicular * arrowWidth);

            var arrowCurve = new PolylineCurve(new Point3d[] { tip, base1, base2, tip });

            var redColor = System.Drawing.Color.FromArgb(255, 0, 0);

            // Try to create filled mesh
            try
            {
                var mesh = new Mesh();
                mesh.Vertices.Add(tip);
                mesh.Vertices.Add(base1);
                mesh.Vertices.Add(base2);
                mesh.Faces.AddFace(0, 1, 2);
                mesh.Normals.ComputeNormals();

                var meshAttr = new ObjectAttributes();
                meshAttr.LayerIndex = layerIndex;
                meshAttr.ColorSource = ObjectColorSource.ColorFromObject;
                meshAttr.ObjectColor = redColor;
                meshAttr.PlotColorSource = ObjectPlotColorSource.PlotColorFromObject;
                meshAttr.PlotColor = redColor;

                doc.Objects.AddMesh(mesh, meshAttr);
            }
            catch
            {
                // Fallback to hatch
                try
                {
                    var hatches = Hatch.Create(arrowCurve, 0, 0, 1.0, 0.001);
                    if (hatches != null && hatches.Length > 0)
                    {
                        var hatchAttr = new ObjectAttributes();
                        hatchAttr.LayerIndex = layerIndex;
                        hatchAttr.ColorSource = ObjectColorSource.ColorFromObject;
                        hatchAttr.ObjectColor = redColor;
                        doc.Objects.AddHatch(hatches[0], hatchAttr);
                    }
                }
                catch { }
            }

            // Add outline
            var curveAttr = new ObjectAttributes();
            curveAttr.LayerIndex = layerIndex;
            curveAttr.ColorSource = ObjectColorSource.ColorFromObject;
            curveAttr.ObjectColor = redColor;
            doc.Objects.AddCurve(arrowCurve, curveAttr);
        }

        private GeometryBase FlattenPanel(GeometryBase geometry, string panelName)
        {
            var flattened = geometry.Duplicate();
            var bbox = flattened.GetBoundingBox(true);
            double xSize = bbox.Max.X - bbox.Min.X;
            double ySize = bbox.Max.Y - bbox.Min.Y;
            double zSize = bbox.Max.Z - bbox.Min.Z;


            bool isVertical = zSize > Math.Min(xSize, ySize);
            bool isShelf = panelName.Contains("Shelf");

            // Rotate if vertical (but not shelves)
            if (isVertical && !isShelf)
            {

                Vector3d rotationAxis;
                if (panelName.Contains("Wall"))
                {
                    rotationAxis = Vector3d.YAxis;
                }
                else
                {
                    rotationAxis = Vector3d.XAxis;
                }

                var rotateTransform = Transform.Rotation(-Math.PI / 2.0, rotationAxis, bbox.Center);
                flattened.Transform(rotateTransform);
            }
            else if (isShelf)
            {
            }

            // IMPROVED: Flatten by projecting to Z=0 plane instead of scaling
            // This preserves the validity of complex geometry
            var currentBBox = flattened.GetBoundingBox(true);
            double zOffset = -currentBBox.Max.Z;  // Top at Z=0 (panel below)

            var flattenTransform = Transform.Translation(0, 0, zOffset);
            flattened.Transform(flattenTransform);

            // Validate the result
            if (flattened is Brep brep)
            {
                if (!brep.IsValid)
                {

                    // Try to repair the Brep
                    brep.Repair(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

                    if (brep.IsValid)
                    {
                        return brep;
                    }
                    else
                    {
                    }
                }
            }
            else if (flattened is Extrusion extrusion)
            {
                // Convert to Brep and validate
                var convertedBrep = extrusion.ToBrep(true);
                if (convertedBrep != null && convertedBrep.IsValid)
                {
                    return convertedBrep;
                }
                else if (convertedBrep != null)
                {
                    convertedBrep.Repair(RhinoDoc.ActiveDoc.ModelAbsoluteTolerance);

                    if (convertedBrep.IsValid)
                    {
                        return convertedBrep;
                    }
                }
            }

            return flattened;
        }

        private void AddToLayer(RhinoDoc doc, GeometryBase geometry, int layerIndex, string panelCategory = "")
        {
            var attr = new ObjectAttributes();
            attr.LayerIndex = layerIndex;

            try
            {
                Brep brep = geometry as Brep;

                if (brep == null)
                {
                    // Try conversion from other geometry types
                    if (geometry is Extrusion extrusion)
                    {
                        brep = extrusion.ToBrep(true);
                    }
                    else if (geometry is Surface surface)
                    {
                        brep = surface.ToBrep();
                    }
                    else
                    {
                        // Last resort: try to get as Brep directly
                        var geometryType = geometry.GetType().Name;


                        // Try casting to Brep (works for PolySurfaces)
                        brep = Brep.TryConvertBrep(geometry);

                        if (brep == null)
                        {
                            return;
                        }
                    }
                }

                if (brep != null && brep.IsValid)
                {
                    var guid = doc.Objects.AddBrep(brep, attr);
                    if (guid == Guid.Empty)
                    {
                    }
                    else
                    {

                        // Tag panel with category for Phase 3A
                        if (!string.IsNullOrEmpty(panelCategory))
                        {
                            var panelObj = doc.Objects.FindId(guid);
                            if (panelObj != null)
                            {
                                panelObj.Attributes.SetUserString("PanelCategory", panelCategory);
                                panelObj.CommitChanges();
                            }
                        }
                    }
                }
                else
                {
                    if (brep != null)
                    {
                    }
                }
            }
            catch (Exception ex)
            {
            }
        }

        private string FormatDimension(double inches, int denominatorPower)
        {
            int wholeInches = (int)Math.Floor(inches);
            double fraction = inches - wholeInches;

            int denominator = (int)Math.Pow(2, denominatorPower);
            int numerator = (int)Math.Round(fraction * denominator);

            if (numerator == 0)
            {
                return $"{wholeInches}\"";
            }
            else if (numerator == denominator)
            {
                return $"{wholeInches + 1}\"";
            }
            else
            {
                int gcd = GCD(numerator, denominator);
                numerator /= gcd;
                denominator /= gcd;

                if (wholeInches == 0)
                    return $"{numerator}/{denominator}\"";
                else
                    return $"{wholeInches}-{numerator}/{denominator}\"";
            }
        }

        private int GCD(int a, int b)
        {
            while (b != 0)
            {
                int temp = b;
                b = a % b;
                a = temp;
            }
            return a;
        }
    }
}