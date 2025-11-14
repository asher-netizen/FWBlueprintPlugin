using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace FWBlueprintPlugin.Services
{
    /// <summary>
    /// Handles panel flattening, layout, and deferred dimension data collection.
    /// </summary>
    internal class PanelArrangementService
    {
        private readonly RhinoDoc _doc;

        private Dictionary<string, double> _panelThicknesses = new Dictionary<string, double>();
        private List<BoundingBox> _panelBounds = new List<BoundingBox>();
        private List<PanelDimensionInfo> _deferredDimensions = new List<PanelDimensionInfo>();
        private List<PanelLeaderInfo> _deferredLeaders = new List<PanelLeaderInfo>();
        private int _panelMarkerDimStyleIndex = -1;

        private struct PanelCategoryEntry
        {
            public BoundingBox BBox;
            public Guid PanelId;
            public string Letter;
        }

        public PanelArrangementService(RhinoDoc doc)
        {
            _doc = doc ?? throw new ArgumentNullException(nameof(doc));
        }

        public void SetupLayersAndStyles(Layer parentLayer)
        {
            CreateLayerStructure(parentLayer);
            CreateDimensionStyle();
        }

        public PanelArrangementResult ArrangePanels(
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
            _panelThicknesses = new Dictionary<string, double>();
            _panelBounds = new List<BoundingBox>();
            _deferredDimensions = new List<PanelDimensionInfo>();
            _deferredLeaders = new List<PanelLeaderInfo>();

            try
            {
                int panelsLayerIndex = FindLayerIndex(parentLayer, "Blueprint", "Panels");
                int dimensionsLayerIndex = FindLayerIndex(parentLayer, "Blueprint", "Dimensions");

                double verticalSpacing = 12.0;
                double behindConsoleOffset = 300.0;
                double sheetSpacing = 100.0;

                var allTopComponents = topConfig.BackerPlates
                    .Concat(topConfig.LiftLids)
                    .Concat(topConfig.TopPlates)
                    .ToList();

                BoundingBox consoleBBox = BoundingBox.Empty;
                foreach (var obj in allTopComponents)
                {
                    consoleBBox.Union(obj.Geometry.GetBoundingBox(true));
                }

                double leftAlignX = consoleBBox.Min.X;
                double startY = consoleBBox.Max.Y + behindConsoleOffset;

                int topIdx = 0;
                foreach (var obj in allTopComponents)
                {
                    double thickness = CalculateThickness(obj.Geometry);
                    _panelThicknesses[$"Top_{topIdx}"] = thickness;
                    AssignPanelThickness(obj, thickness);
                    topIdx++;
                }

                double backThickness = CalculateThickness(backPanel.Geometry);
                _panelThicknesses["Back"] = backThickness;
                AssignPanelThickness(backPanel, backThickness);

                double bottomThickness = CalculateThickness(bottomPanel.Geometry);
                _panelThicknesses["Bottom"] = bottomThickness;
                AssignPanelThickness(bottomPanel, bottomThickness);

                var topInfo = FlattenLiftLidTop(topConfig, leftAlignX, startY, panelsLayerIndex);
                _panelBounds.Add(topInfo.OverallBBox);
                DeferLiftLidTopDimensions(topConfig, topInfo, dimensionsLayerIndex);

                double currentY = topInfo.OverallBBox.Min.Y - verticalSpacing;
                double maxX;

                currentY = ProcessPanelsHorizontalButt(
                    new List<RhinoObject> { backPanel },
                    "Back",
                    leftAlignX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out maxX);

                currentY = ProcessPanelsHorizontalButt(
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

                currentY = startY;
                double sheet2MaxX;

                currentY = ProcessPanelsHorizontalButt(
                    exteriorWalls,
                    "Exterior Wall",
                    sheet2StartX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out sheet2MaxX);

                currentY = ProcessPanelsHorizontalButt(
                    interiorWalls,
                    "Interior Wall",
                    sheet2StartX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out sheet2MaxX);

                currentY = ProcessPanelsHorizontalButt(
                    removableShelves,
                    "Removable Shelf",
                    sheet2StartX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out sheet2MaxX);

                double sheet3StartX = sheet2MaxX + sheetSpacing;
                currentY = startY;

                double permMaxX;
                currentY = ProcessPanelsHorizontalButt(
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
                    doors,
                    "Door",
                    sheet3StartX,
                    currentY,
                    verticalSpacing,
                    panelsLayerIndex,
                    dimensionsLayerIndex,
                    out doorMaxX);

                SwitchToTopViewportAndCenter();

                return new PanelArrangementResult
                {
                    Success = true,
                    PanelThicknesses = _panelThicknesses,
                    PanelBounds = _panelBounds,
                    DeferredDimensions = _deferredDimensions,
                    DeferredLeaders = _deferredLeaders
                };
            }
            catch
            {
                return new PanelArrangementResult { Success = false };
            }
        }

        private void CreateLayerStructure(Layer parentLayer)
        {
            var blueprintLayer = new Layer
            {
                Name = "Blueprint",
                ParentLayerId = parentLayer.Id,
                Color = System.Drawing.Color.Black
            };
            int blueprintIndex = _doc.Layers.Add(blueprintLayer);

            if (blueprintIndex < 0)
            {
                for (int i = 0; i < _doc.Layers.Count; i++)
                {
                    if (_doc.Layers[i].Name == "Blueprint" && _doc.Layers[i].ParentLayerId == parentLayer.Id)
                    {
                        blueprintIndex = i;
                        break;
                    }
                }
            }

            var blueprintLayerObj = _doc.Layers[blueprintIndex];

            var panelsLayer = new Layer
            {
                Name = "Panels",
                ParentLayerId = blueprintLayerObj.Id,
                Color = System.Drawing.Color.Black
            };
            _doc.Layers.Add(panelsLayer);

            var dimensionsLayer = new Layer
            {
                Name = "Dimensions",
                ParentLayerId = blueprintLayerObj.Id,
                Color = System.Drawing.Color.Red
            };
            _doc.Layers.Add(dimensionsLayer);
        }

        private void CreateDimensionStyle()
        {
            var styleName = BlueprintAnnotationStyles.Default;
            if (_doc.DimStyles.FindName(styleName) != null)
            {
                return;
            }

            RhinoApp.WriteLine($"[Blueprint Styles] Dimension style '{styleName}' is missing; ensure the resource model is available.");
        }

        private double CalculateThickness(GeometryBase geometry)
        {
            var bbox = geometry.GetBoundingBox(true);
            double xSize = bbox.Max.X - bbox.Min.X;
            double ySize = bbox.Max.Y - bbox.Min.Y;
            double zSize = bbox.Max.Z - bbox.Min.Z;

            return Math.Min(Math.Min(xSize, ySize), zSize);
        }

        private LiftLidFlattenedInfo FlattenLiftLidTop(
            LiftLidTopComponents config,
            double leftAlignX,
            double topY,
            int layerIndex)
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
                    _ = AddToLayer(item.Geom, layerIndex, "Top");
                }
            };

            transformList(flattenedBackers);
            transformList(flattenedLids);
            transformList(flattenedTopPlates);

            overallBBox.Transform(translate);

            var widthGroups = flattenedTopPlates.Select(f => f.BBox).ToList();
            var backerDepths = flattenedBackers.Select(f => f.BBox).ToList();
            var lidDepths = flattenedLids.Select(f => f.BBox).ToList();

            for (int i = 0; i < flattenedBackers.Count && i < flattenedLids.Count; i++)
            {
                var pairBBox = flattenedBackers[i].BBox;
                pairBBox.Union(flattenedLids[i].BBox);
                widthGroups.Add(pairBBox);
            }

            widthGroups = widthGroups.OrderBy(b => b.Min.X).ToList();

            return new LiftLidFlattenedInfo
            {
                OverallBBox = overallBBox,
                WidthGroupBBoxes = widthGroups,
                BackerDepthBBoxes = backerDepths,
                LidDepthBBoxes = lidDepths
            };
        }

        private void DeferLiftLidTopDimensions(
            LiftLidTopComponents config,
            LiftLidFlattenedInfo info,
            int dimensionsLayerIndex)
        {
            var totalCount = config.TotalComponentCount;
            var panelType = totalCount > 1 ? "Tops" : "Top";
            var quantity = totalCount;

            if (totalCount == 1)
            {
                _deferredDimensions.Add(new PanelDimensionInfo
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
                _deferredDimensions.Add(new PanelDimensionInfo
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

                foreach (var childBBox in info.WidthGroupBBoxes)
                {
                    _deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = childBBox,
                        PanelType = panelType,
                        IsChild = true,
                        DrawWidth = true,
                        DrawHeight = false,
                        LayerIndex = dimensionsLayerIndex
                    });
                }

                for (int i = 0; i < info.BackerDepthBBoxes.Count && i < info.LidDepthBBoxes.Count; i++)
                {
                    var backerBBox = info.BackerDepthBBoxes[i];
                    var lidBBox = info.LidDepthBBoxes[i];

                    double backerDepth = backerBBox.Max.Y - backerBBox.Min.Y;
                    double lidDepth = lidBBox.Max.Y - lidBBox.Min.Y;

                    double backerOffset = 5.0;
                    double lidOffset = 5.0;

                    bool backerIsSmall = backerDepth < 4.0;
                    bool lidIsSmall = lidDepth < 4.0;

                    if (backerIsSmall && !lidIsSmall)
                    {
                        backerOffset = 9.0;
                    }
                    else if (!backerIsSmall && lidIsSmall)
                    {
                        lidOffset = 9.0;
                    }
                    else if (backerIsSmall && lidIsSmall)
                    {
                        backerOffset = 9.0;
                        lidOffset = 5.0;
                    }

                    double verticalGap = Math.Abs(backerBBox.Center.Y - lidBBox.Center.Y);
                    if (verticalGap < 3.0 && backerIsSmall)
                    {
                        backerOffset = 11.0;
                    }

                    _deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = backerBBox,
                        PanelType = "TopBackerDepth",
                        DrawWidth = false,
                        DrawHeight = true,
                        IsRightHeightDim = false,
                        LayerIndex = dimensionsLayerIndex,
                        CustomHeightOffset = backerOffset
                    });

                    _deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = lidBBox,
                        PanelType = "TopLiftLidDepth",
                        DrawWidth = false,
                        DrawHeight = true,
                        IsRightHeightDim = false,
                        LayerIndex = dimensionsLayerIndex,
                        CustomHeightOffset = lidOffset
                    });
                }
            }

            _deferredLeaders.Add(new PanelLeaderInfo
            {
                BBox = info.OverallBBox,
                PanelType = panelType,
                Quantity = quantity,
                LayerIndex = dimensionsLayerIndex
            });
        }

        private double ProcessPanelsHorizontalButt(
            List<RhinoObject> panels,
            string panelType,
            double leftAlignX,
            double currentY,
            double verticalSpacing,
            int panelsLayerIndex,
            int dimensionsLayerIndex,
            out double maxXReached)
        {
            maxXReached = leftAlignX;

            if (panels == null || panels.Count == 0)
                return currentY;

            double categoryThickness = 0;
            for (int i = 0; i < panels.Count; i++)
            {
                string key = $"{panelType}_{i}";
                double thickness = CalculateThickness(panels[i].Geometry);
                _panelThicknesses[key] = thickness;
                AssignPanelThickness(panels[i], thickness);
                categoryThickness = thickness;
            }

            bool shouldLabelPanels = panels.Count > 1;

            double currentX = leftAlignX;
            double categoryMaxHeight = 0;
            var categoryEntries = new List<PanelCategoryEntry>();

            bool arrangeAsDoors = panelType.Equals("Door", StringComparison.OrdinalIgnoreCase);

            if (arrangeAsDoors)
            {
                categoryEntries = ArrangeDoorPanels(
                    panels,
                    panelType,
                    leftAlignX,
                    currentY,
                    panelsLayerIndex,
                    out maxXReached,
                    out categoryMaxHeight);
            }
            else
            {
                for (int panelIndex = 0; panelIndex < panels.Count; panelIndex++)
                {
                    var panel = panels[panelIndex];
                    var panelFlat = FlattenPanel(panel.Geometry, panelType);
                    var panelBox = panelFlat.GetBoundingBox(true);

                    var panelMoveX = currentX - panelBox.Min.X;
                    var panelMoveY = currentY - panelBox.Max.Y;

                    panelFlat.Transform(Transform.Translation(panelMoveX, panelMoveY, 0));
                    panelBox = panelFlat.GetBoundingBox(true);

                    var panelGuid = AddToLayer(panelFlat, panelsLayerIndex, panelType);

                    _panelBounds.Add(panelBox);
                    categoryEntries.Add(new PanelCategoryEntry
                    {
                        BBox = panelBox,
                        PanelId = panelGuid,
                        Letter = string.Empty
                    });

                    currentX = panelBox.Max.X;

                    double panelHeight = panelBox.Max.Y - panelBox.Min.Y;
                    categoryMaxHeight = Math.Max(categoryMaxHeight, panelHeight);
                }

                maxXReached = currentX;
            }

            if (categoryEntries.Count > 1)
            {
                categoryEntries.Sort((a, b) => a.BBox.Min.X.CompareTo(b.BBox.Min.X));
            }

            for (int i = 0; i < categoryEntries.Count; i++)
            {
                var entry = categoryEntries[i];
                string letter = ConvertIndexToLetter(i);
                entry.Letter = letter;

                if (entry.PanelId != Guid.Empty)
                {
                    var panelObj = _doc.Objects.FindId(entry.PanelId);
                    if (panelObj != null)
                    {
                        if (shouldLabelPanels)
                        {
                            panelObj.Attributes.Name = $"{panelType}: {letter}";
                            panelObj.Attributes.SetUserString("PanelLabel", letter);
                            panelObj.Attributes.SetUserString("PanelName", $"{panelType}: {letter}");
                        }
                        else
                        {
                            panelObj.Attributes.Name = panelType;
                            panelObj.Attributes.SetUserString("PanelName", panelType);
                            panelObj.Attributes.SetUserString("PanelLabel", string.Empty);
                        }

                        panelObj.CommitChanges();
                    }
                }

                if (shouldLabelPanels)
                {
                    AddPanelMarker(entry.BBox, letter, panelsLayerIndex);
                }

                categoryEntries[i] = entry;
            }

            var categoryBounds = categoryEntries.Select(entry => entry.BBox).ToList();

            BoundingBox groupBBox = categoryBounds[0];
            for (int i = 1; i < categoryBounds.Count; i++)
            {
                groupBBox.Union(categoryBounds[i]);
            }

            if (categoryBounds.Count > 1)
            {
                _deferredDimensions.Add(new PanelDimensionInfo
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

                for (int i = 0; i < categoryEntries.Count; i++)
                {
                    var entry = categoryEntries[i];
                    _deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = entry.BBox,
                        PanelType = $"{panelType}: {entry.Letter}",
                        IsChild = true,
                        DrawWidth = true,
                        DrawHeight = false,
                        LayerIndex = dimensionsLayerIndex
                    });
                }

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
                    _deferredDimensions.Add(new PanelDimensionInfo
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
                    _deferredDimensions.Add(new PanelDimensionInfo
                    {
                        BBox = categoryBounds[0],
                        PanelType = $"{panelType}: {categoryEntries[0].Letter}",
                        DrawWidth = false,
                        DrawHeight = true,
                        IsRightHeightDim = false,
                        LayerIndex = dimensionsLayerIndex
                    });

                    foreach (int i in differentHeightIndices)
                    {
                        _deferredDimensions.Add(new PanelDimensionInfo
                        {
                            BBox = categoryBounds[i],
                            PanelType = $"{panelType}: {categoryEntries[i].Letter}",
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
                _deferredDimensions.Add(new PanelDimensionInfo
                {
                    BBox = categoryBounds[0],
                    PanelType = $"{panelType}: {categoryEntries[0].Letter}",
                    Quantity = panels.Count,
                    IsChild = false,
                    DrawWidth = true,
                    DrawHeight = true,
                    LayerIndex = dimensionsLayerIndex
                });
            }

            if (categoryBounds.Count > 0)
            {
                var rightmostBox = categoryBounds[categoryBounds.Count - 1];
                _deferredLeaders.Add(new PanelLeaderInfo
                {
                    BBox = rightmostBox,
                    PanelType = panelType,
                    Quantity = panels.Count,
                    LayerIndex = dimensionsLayerIndex
                });
            }

            maxXReached = currentX;
            currentY -= (categoryMaxHeight + verticalSpacing);

            return currentY;
        }

        private List<PanelCategoryEntry> ArrangeDoorPanels(
            List<RhinoObject> panels,
            string panelType,
            double leftAlignX,
            double currentY,
            int panelsLayerIndex,
            out double maxXReached,
            out double categoryMaxHeight)
        {
            const double ColumnOverlapRatioThreshold = 0.35;
            const double FullSpanTolerance = 0.5;
            const double TopAlignmentTolerance = 0.25;

            var doorPanels = new List<DoorPanelInfo>();
            foreach (var panel in panels)
            {
                var flattened = FlattenPanel(panel.Geometry, panelType);
                var flattenedBox = flattened.GetBoundingBox(true);
                var originalBox = panel.Geometry.GetBoundingBox(true);

                doorPanels.Add(new DoorPanelInfo(panel, flattened, flattenedBox, originalBox));
            }

            if (doorPanels.Count == 0)
            {
                maxXReached = leftAlignX;
                categoryMaxHeight = 0;
                return new List<PanelCategoryEntry>();
            }

            SnapDoorPanelTops(doorPanels, TopAlignmentTolerance);
            var columns = BuildDoorColumns(doorPanels, ColumnOverlapRatioThreshold);
            double currentX = leftAlignX;
            var entries = new List<PanelCategoryEntry>();

            double groupTop = doorPanels.Max(p => p.SnappedTop);
            double groupBottom = doorPanels.Min(p => p.OriginalBBox.Min.Z);
            categoryMaxHeight = Math.Max(0, groupTop - groupBottom);

            foreach (var column in columns.OrderBy(c => c.OriginalMinX))
            {
                double columnWidth = column.OriginalWidth;
                if (columnWidth < 0.001)
                {
                    columnWidth = column.Panels.Max(p => p.FlattenedBBox.Max.X - p.FlattenedBBox.Min.X);
                }

                double columnTopY = column.Panels.Max(p => p.SnappedTop);

                foreach (var panelInfo in column.Panels.OrderByDescending(p => p.OriginalBBox.Max.Z))
                {
                    var panelBox = panelInfo.FlattenedBBox;
                    double relativeStart = panelInfo.OriginalBBox.Min.X - column.OriginalMinX;
                    if (relativeStart < 0)
                    {
                        relativeStart = 0;
                    }

                    double targetMinX = currentX + relativeStart;
                    double panelWidth = panelInfo.OriginalBBox.Max.X - panelInfo.OriginalBBox.Min.X;
                    double widthDiff = Math.Abs(panelWidth - columnWidth);
                    bool isTopPanel = Math.Abs(panelInfo.SnappedTop - columnTopY) <= TopAlignmentTolerance;

                    if (isTopPanel && widthDiff <= FullSpanTolerance)
                    {
                        targetMinX = currentX;
                    }

                    double verticalOffset = groupTop - panelInfo.SnappedTop;
                    double targetTopY = currentY - verticalOffset;

                    double moveX = targetMinX - panelBox.Min.X;
                    double moveY = targetTopY - panelBox.Max.Y;
                    var translation = Transform.Translation(moveX, moveY, 0);

                    panelInfo.Geometry.Transform(translation);
                    panelBox = panelInfo.Geometry.GetBoundingBox(true);

                    var panelGuid = AddToLayer(panelInfo.Geometry, panelsLayerIndex, panelType);
                    _panelBounds.Add(panelBox);

                    entries.Add(new PanelCategoryEntry
                    {
                        BBox = panelBox,
                        PanelId = panelGuid,
                        Letter = string.Empty
                    });
                }

                currentX += columnWidth;
            }

            maxXReached = currentX;

            if (entries.Count > 0)
            {
                double minY = entries.Min(e => e.BBox.Min.Y);
                double maxY = entries.Max(e => e.BBox.Max.Y);
                categoryMaxHeight = Math.Max(categoryMaxHeight, maxY - minY);
            }

            return entries;
        }

        private static void SnapDoorPanelTops(List<DoorPanelInfo> doorPanels, double tolerance)
        {
            var anchors = new List<double>();

            foreach (var panel in doorPanels.OrderByDescending(p => p.OriginalBBox.Max.Z))
            {
                double top = panel.OriginalBBox.Max.Z;
                double snapped = double.NaN;

                foreach (var anchor in anchors)
                {
                    if (Math.Abs(anchor - top) <= tolerance)
                    {
                        snapped = anchor;
                        break;
                    }
                }

                if (double.IsNaN(snapped))
                {
                    anchors.Add(top);
                    snapped = top;
                }

                panel.SnappedTop = snapped;
            }
        }

        private static List<DoorColumn> BuildDoorColumns(List<DoorPanelInfo> doorPanels, double overlapThreshold)
        {
            const double VerticalStackingThreshold = 0.5;
            const double MinOverlapAbsolute = 0.01;

            int count = doorPanels.Count;
            var disjoint = new DisjointSet(count);

            for (int i = 0; i < count; i++)
            {
                for (int j = i + 1; j < count; j++)
                {
                    var a = doorPanels[i];
                    var b = doorPanels[j];

                    double overlap = GetOverlap(
                        a.OriginalBBox.Min.X,
                        a.OriginalBBox.Max.X,
                        b.OriginalBBox.Min.X,
                        b.OriginalBBox.Max.X);

                    if (overlap <= MinOverlapAbsolute)
                    {
                        continue;
                    }

                    double minWidth = Math.Min(a.OriginalWidth, b.OriginalWidth);
                    if (minWidth < 0.001)
                    {
                        continue;
                    }

                    double ratio = overlap / minWidth;
                    double verticalDelta = Math.Abs(a.OriginalBBox.Center.Z - b.OriginalBBox.Center.Z);
                    if (ratio < overlapThreshold)
                    {
                        continue;
                    }

                    if (verticalDelta <= VerticalStackingThreshold)
                    {
                        continue;
                    }

                    disjoint.Union(i, j);
                }
            }

            var columnMap = new Dictionary<int, DoorColumn>();
            for (int i = 0; i < count; i++)
            {
                int root = disjoint.Find(i);
                if (!columnMap.TryGetValue(root, out var column))
                {
                    column = new DoorColumn(doorPanels[i]);
                    columnMap[root] = column;
                }
                else
                {
                    column.Add(doorPanels[i]);
                }
            }

            var ordered = columnMap.Values
                .OrderBy(c => c.OriginalMinX)
                .ToList();

            return ordered;
        }

        private static double GetOverlap(double aMin, double aMax, double bMin, double bMax)
        {
            double overlapMin = Math.Max(aMin, bMin);
            double overlapMax = Math.Min(aMax, bMax);
            return Math.Max(0, overlapMax - overlapMin);
        }

        private sealed class DisjointSet
        {
            private readonly int[] _parent;
            private readonly int[] _rank;

            public DisjointSet(int size)
            {
                _parent = new int[size];
                _rank = new int[size];
                for (int i = 0; i < size; i++)
                {
                    _parent[i] = i;
                    _rank[i] = 0;
                }
            }

            public int Find(int value)
            {
                if (_parent[value] != value)
                {
                    _parent[value] = Find(_parent[value]);
                }

                return _parent[value];
            }

            public void Union(int a, int b)
            {
                int rootA = Find(a);
                int rootB = Find(b);
                if (rootA == rootB)
                {
                    return;
                }

                if (_rank[rootA] < _rank[rootB])
                {
                    _parent[rootA] = rootB;
                }
                else if (_rank[rootA] > _rank[rootB])
                {
                    _parent[rootB] = rootA;
                }
                else
                {
                    _parent[rootB] = rootA;
                    _rank[rootA]++;
                }
            }
        }

        private sealed class DoorPanelInfo
        {
            public DoorPanelInfo(RhinoObject source, GeometryBase geometry, BoundingBox flattened, BoundingBox original)
            {
                Source = source;
                Geometry = geometry;
                FlattenedBBox = flattened;
                OriginalBBox = original;
                SnappedTop = original.Max.Z;
            }

            public RhinoObject Source { get; }
            public GeometryBase Geometry { get; }
            public BoundingBox FlattenedBBox { get; }
            public BoundingBox OriginalBBox { get; }
            public double OriginalWidth => OriginalBBox.Max.X - OriginalBBox.Min.X;
            public double SnappedTop { get; set; }
        }

        private sealed class DoorColumn
        {
            private readonly List<DoorPanelInfo> _panels = new List<DoorPanelInfo>();

            public DoorColumn(DoorPanelInfo initialPanel)
            {
                OriginalMinX = initialPanel.OriginalBBox.Min.X;
                OriginalMaxX = initialPanel.OriginalBBox.Max.X;
                _panels.Add(initialPanel);
            }

            public IReadOnlyList<DoorPanelInfo> Panels => _panels;
            public double OriginalMinX { get; private set; }
            public double OriginalMaxX { get; private set; }
            public double OriginalWidth => Math.Max(0, OriginalMaxX - OriginalMinX);

            public void Add(DoorPanelInfo panel)
            {
                _panels.Add(panel);
                OriginalMinX = Math.Min(OriginalMinX, panel.OriginalBBox.Min.X);
                OriginalMaxX = Math.Max(OriginalMaxX, panel.OriginalBBox.Max.X);
            }
        }


        private void SwitchToTopViewportAndCenter()
        {
            try
            {
                Rhino.Display.RhinoView topView = null;
                foreach (var view in _doc.Views)
                {
                    if (view.MainViewport.Name.Equals("Top", StringComparison.OrdinalIgnoreCase))
                    {
                        topView = view;
                        break;
                    }
                }

                if (topView != null)
                {
                    _doc.Views.ActiveView = topView;
                }
                else
                {
                    var activeView = _doc.Views.ActiveView;
                    if (activeView != null)
                    {
                        activeView.MainViewport.ChangeToParallelProjection(true);
                        activeView.MainViewport.SetCameraLocation(new Point3d(0, 0, 100), false);
                        activeView.MainViewport.SetCameraDirection(new Vector3d(0, 0, -1), false);
                        activeView.MainViewport.CameraUp = Vector3d.YAxis;
                    }
                }

                if (_panelBounds.Count > 0)
                {
                    BoundingBox overallBBox = _panelBounds[0];
                    for (int i = 1; i < _panelBounds.Count; i++)
                    {
                        overallBBox.Union(_panelBounds[i]);
                    }

                    double xPadding = (overallBBox.Max.X - overallBBox.Min.X) * 0.1;
                    double yPadding = (overallBBox.Max.Y - overallBBox.Min.Y) * 0.1;
                    overallBBox.Inflate(xPadding, yPadding, 0);

                    var activeView = _doc.Views.ActiveView;
                    if (activeView != null)
                    {
                        activeView.MainViewport.ZoomBoundingBox(overallBBox);
                        activeView.Redraw();
                    }
                }

                _doc.Views.Redraw();
            }
            catch
            {
            }
        }

        private static string ConvertIndexToLetter(int index)
        {
            const int alphabetSize = 26;
            if (index < 0)
            {
                return "A";
            }

            var result = string.Empty;
            int current = index;
            do
            {
                int remainder = current % alphabetSize;
                result = (char)('A' + remainder) + result;
                current = (current / alphabetSize) - 1;
            } while (current >= 0);

            return result;
        }

        private void AddPanelMarker(BoundingBox panelBox, string letter, int layerIndex)
        {
            try
            {
                var dimStyle = GetPanelMarkerDimStyle();
                var plane = Plane.WorldXY;
                plane.Origin = new Point3d(panelBox.Min.X + 0.75, panelBox.Max.Y - 1.25, 0);

                var textEntity = new TextEntity
                {
                    Plane = plane,
                    PlainText = letter,
                    Justification = TextJustification.MiddleLeft,
                    TextHeight = 0.6
                };
                if (dimStyle != null)
                {
                    textEntity.DimensionStyleId = dimStyle.Id;
                    textEntity.TextHeight = dimStyle.TextHeight;
                }

                if (dimStyle != null)
                {
                    string faceName = dimStyle.Font?.FaceName;
                    Rhino.DocObjects.Font fontToUse = null;

                    if (!string.IsNullOrEmpty(faceName))
                    {
                        try
                        {
                            fontToUse = Rhino.DocObjects.Font.FromQuartetProperties(faceName, true, false);
                        }
                        catch
                        {
                        }
                    }

                    if (fontToUse == null && dimStyle.Font != null && !string.IsNullOrEmpty(dimStyle.Font.FaceName))
                    {
                        try
                        {
                            fontToUse = Rhino.DocObjects.Font.FromQuartetProperties(dimStyle.Font.FaceName, true, false);
                        }
                        catch
                        {
                            fontToUse = dimStyle.Font;
                        }
                    }

                    if (fontToUse == null && dimStyle.Font != null)
                    {
                        fontToUse = dimStyle.Font;
                    }

                    if (fontToUse != null)
                    {
                        textEntity.Font = fontToUse;
                    }

                    string fontFaceForRtf = fontToUse?.FaceName ?? dimStyle?.Font?.FaceName ?? "Arial";
                    string escapedFont = fontFaceForRtf.Replace("\\", "\\\\").Replace("{", "\\{").Replace("}", "\\}");
                    string escapedLetter = letter.Replace("\\", "\\\\").Replace("{", "\\{").Replace("}", "\\}");
                    textEntity.RichText = $"{{\\rtf1\\ansi\\deff0{{\\fonttbl{{\\f0 {escapedFont};}}}}\\f0\\b {escapedLetter}}}";
                }

                var attr = new ObjectAttributes
                {
                    LayerIndex = layerIndex,
                    ColorSource = ObjectColorSource.ColorFromLayer
                };

                var textGuid = _doc.Objects.AddText(textEntity, attr);
                if (textGuid != Guid.Empty && _doc.Objects.FindId(textGuid) is TextObject textObject && textObject.Geometry is TextEntity createdText)
                {
                    createdText.TextHeight = dimStyle?.TextHeight ?? 0.6;
                    if (dimStyle != null)
                    {
                        createdText.DimensionStyleId = dimStyle.Id;
                    }
                    createdText.DimensionScale = 1.0;
                    textObject.CommitChanges();
                }
            }
            catch
            {
            }
        }

        private DimensionStyle GetPanelMarkerDimStyle()
        {
            const string styleName = "Panel Marker Label";
            if (_panelMarkerDimStyleIndex >= 0 && _panelMarkerDimStyleIndex < _doc.DimStyles.Count)
            {
                var existing = _doc.DimStyles[_panelMarkerDimStyleIndex];
                if (existing != null)
                {
                    EnsureMarkerStyleSettings(existing);
                    return existing;
                }
            }

            for (int i = 0; i < _doc.DimStyles.Count; i++)
            {
                if (string.Equals(_doc.DimStyles[i].Name, styleName, StringComparison.OrdinalIgnoreCase))
                {
                    _panelMarkerDimStyleIndex = i;
                    EnsureMarkerStyleSettings(_doc.DimStyles[i]);
                    return _doc.DimStyles[i];
                }
            }

            var source = BlueprintAnnotationDebug.ResolveDefaultStyle(_doc, "PanelArrangementService.GetPanelMarkerDimStyle")
                         ?? _doc.DimStyles.Current
                         ?? _doc.DimStyles[0];
            int newIndex = _doc.DimStyles.Add(styleName);
            var markerStyle = _doc.DimStyles[newIndex];
            markerStyle.CopyFrom(source);
            markerStyle.Name = styleName;
            markerStyle.DimensionScale = 1.0;
            markerStyle.TextHeight = 0.6;
            _doc.DimStyles.Modify(markerStyle, newIndex, true);
            _panelMarkerDimStyleIndex = newIndex;
            return markerStyle;
        }

        private void EnsureMarkerStyleSettings(DimensionStyle style)
        {
            bool updated = false;
            if (Math.Abs(style.TextHeight - 0.6) > 0.0001)
            {
                style.TextHeight = 0.6;
                updated = true;
            }
            if (Math.Abs(style.DimensionScale - 1.0) > 0.0001)
            {
                style.DimensionScale = 1.0;
                updated = true;
            }
            if (updated)
            {
                _doc.DimStyles.Modify(style, style.Index, true);
            }
        }

        private int FindLayerIndex(Layer parentLayer, string sublayer1, string sublayer2)
        {
            for (int i = 0; i < _doc.Layers.Count; i++)
            {
                var layer = _doc.Layers[i];
                if (layer.Name == sublayer2)
                {
                    var parent = _doc.Layers.FindId(layer.ParentLayerId);
                    if (parent != null && parent.Name == sublayer1)
                    {
                        var grandparent = _doc.Layers.FindId(parent.ParentLayerId);
                        if (grandparent != null && grandparent.Id == parentLayer.Id)
                        {
                            return i;
                        }
                    }
                }
            }
            return 0;
        }

        private GeometryBase FlattenPanel(GeometryBase geometry, string panelName)
        {
            var flattened = geometry.Duplicate();
            var bbox = flattened.GetBoundingBox(true);
            double xSize = bbox.Max.X - bbox.Min.X;
            double ySize = bbox.Max.Y - bbox.Min.Y;
            double zSize = bbox.Max.Z - bbox.Min.Z;

            bool isVertical = zSize > Math.Min(xSize, ySize);
            bool isShelf = !string.IsNullOrEmpty(panelName) &&
                           panelName.IndexOf("Shelf", StringComparison.OrdinalIgnoreCase) >= 0;

            if (isVertical && !isShelf)
            {
                Vector3d rotationAxis = (!string.IsNullOrEmpty(panelName) &&
                                          panelName.IndexOf("Wall", StringComparison.OrdinalIgnoreCase) >= 0)
                    ? Vector3d.YAxis
                    : Vector3d.XAxis;

                var rotateTransform = Transform.Rotation(-Math.PI / 2.0, rotationAxis, bbox.Center);
                flattened.Transform(rotateTransform);
            }

            var currentBBox = flattened.GetBoundingBox(true);
            double zOffset = -currentBBox.Max.Z;
            var flattenTransform = Transform.Translation(0, 0, zOffset);
            flattened.Transform(flattenTransform);

            if (flattened is Brep brep)
            {
                if (!brep.IsValid)
                {
                    brep = brep.DuplicateBrep();
                    brep.Repair(_doc.ModelAbsoluteTolerance);

                    if (brep.IsValid)
                    {
                        return brep;
                    }
                }
            }
            else if (flattened is Extrusion extrusion)
            {
                var convertedBrep = extrusion.ToBrep(true);
                if (convertedBrep != null)
                {
                    if (!convertedBrep.IsValid)
                    {
                        convertedBrep.Repair(_doc.ModelAbsoluteTolerance);
                    }

                    if (convertedBrep.IsValid)
                    {
                        return convertedBrep;
                    }
                }
            }

            return flattened;
        }

        private Guid AddToLayer(GeometryBase geometry, int layerIndex, string panelCategory = "")
        {
            var attr = new ObjectAttributes
            {
                LayerIndex = layerIndex
            };

            try
            {
                Brep brep = geometry as Brep;

                if (brep == null)
                {
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
                        brep = Brep.TryConvertBrep(geometry);
                        if (brep == null)
                        {
                            return Guid.Empty;
                        }
                    }
                }

                if (brep != null && brep.IsValid)
                {
                    var guid = _doc.Objects.AddBrep(brep, attr);
                    if (guid != Guid.Empty && !string.IsNullOrEmpty(panelCategory))
                    {
                        var panelObj = _doc.Objects.FindId(guid);
                        if (panelObj != null)
                        {
                            panelObj.Attributes.SetUserString("PanelCategory", panelCategory);
                            panelObj.CommitChanges();
                        }
                    }

                    return guid;
                }
            }
            catch
            {
            }

            return Guid.Empty;
        }

        private void AssignPanelThickness(RhinoObject panel, double thickness)
        {
            if (panel == null)
            {
                return;
            }

            try
            {
                panel.Attributes.SetUserString("PanelThickness", thickness.ToString(CultureInfo.InvariantCulture));
                panel.CommitChanges();
            }
            catch
            {
            }
        }
    }
}
