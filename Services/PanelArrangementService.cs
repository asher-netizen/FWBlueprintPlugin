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
            const string styleName = "Furniture Dim - CG";

            for (int i = 0; i < _doc.DimStyles.Count; i++)
            {
                if (_doc.DimStyles[i].Name == styleName)
                {
                    return;
                }
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
                var dimStyle = _doc.DimStyles.FindName("Furniture Dim - CG") ?? _doc.DimStyles.Current;
                double targetHeight = 0.6;

                var plane = Plane.WorldXY;
                plane.Origin = new Point3d(panelBox.Min.X + 0.75, panelBox.Max.Y - 1.25, 0);

                var textEntity = new TextEntity
                {
                    Plane = plane,
                    PlainText = letter,
                    Justification = TextJustification.MiddleLeft,
                    TextHeight = targetHeight
                };

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

                _doc.Objects.AddText(textEntity, attr);
            }
            catch
            {
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
