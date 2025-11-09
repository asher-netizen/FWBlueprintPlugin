using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using static FWBlueprintPlugin.Services.DimensionFormatting;

namespace FWBlueprintPlugin.Services
{
    /// <summary>
    /// Encapsulates logic for adding panel dimensions and leaders.
    /// </summary>
    internal class PanelDimensioningService
    {
        private readonly RhinoDoc _doc;
        private readonly Dictionary<string, double> _panelThicknesses;
        private readonly List<BoundingBox> _dimensionBounds = new List<BoundingBox>();

        public PanelDimensioningService(RhinoDoc doc, Dictionary<string, double> panelThicknesses)
        {
            _doc = doc ?? throw new ArgumentNullException(nameof(doc));
            _panelThicknesses = panelThicknesses ?? new Dictionary<string, double>();
        }

        public IReadOnlyList<BoundingBox> DimensionBounds => _dimensionBounds;

        public void AddPanelDimensions(PanelDimensionInfo info)
        {
            if (info == null)
            {
                return;
            }

            var bbox = info.BBox;
            double dimZ = bbox.Max.Z + 0.01;

            double widthDimOffset;
            double heightDimOffset;

            if (info.CustomHeightOffset.HasValue)
            {
                heightDimOffset = info.CustomHeightOffset.Value;
                widthDimOffset = 5.0;
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

            var redColor = Color.FromArgb(255, 0, 0);
            var dimStyle = _doc.DimStyles.Current;

            if (info.DrawWidth)
            {
                var widthPt1 = new Point3d(bbox.Min.X, bbox.Max.Y, dimZ);
                var widthPt2 = new Point3d(bbox.Max.X, bbox.Max.Y, dimZ);
                var widthLinePt = new Point3d((bbox.Min.X + bbox.Max.X) / 2, bbox.Max.Y + widthDimOffset, dimZ);

                var widthPlane = new Plane(new Point3d(0, 0, dimZ), Vector3d.ZAxis);

                var widthDim = LinearDimension.Create(
                    AnnotationType.Aligned,
                    dimStyle,
                    widthPlane,
                    Vector3d.XAxis,
                    widthPt1,
                    widthPt2,
                    widthLinePt,
                    0);

                var attr = new ObjectAttributes
                {
                    LayerIndex = info.LayerIndex,
                    ColorSource = ObjectColorSource.ColorFromObject,
                    ObjectColor = redColor
                };

                var widthGuid = _doc.Objects.AddLinearDimension(widthDim, attr);
                if (widthGuid != Guid.Empty)
                {
                    var dimObj = _doc.Objects.FindId(widthGuid);
                    if (dimObj != null)
                    {
                        var dimBBox = dimObj.Geometry.GetBoundingBox(true);
                        dimBBox.Inflate(3.0);
                        _dimensionBounds.Add(dimBBox);
                    }
                }
            }

            if (info.DrawHeight)
            {
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

                var attr = new ObjectAttributes
                {
                    LayerIndex = info.LayerIndex,
                    ColorSource = ObjectColorSource.ColorFromObject,
                    ObjectColor = redColor
                };

                var heightGuid = _doc.Objects.AddLinearDimension(heightDim, attr);
                if (heightGuid != Guid.Empty)
                {
                    var dimObj = _doc.Objects.FindId(heightGuid);
                    if (dimObj != null)
                    {
                        var dimBBox = dimObj.Geometry.GetBoundingBox(true);
                        dimBBox.Inflate(3.0);
                        _dimensionBounds.Add(dimBBox);
                    }
                }
            }
        }

        public void AddPanelLeader(BoundingBox bbox, string panelType, int quantity, int layerIndex)
        {
            var lines = new List<string>();

            string panelName = panelType;
            if (quantity > 1)
            {
                if (panelName.EndsWith("Shelf"))
                    panelName = panelName.Replace("Shelf", "Shelves");
                else if (panelName.EndsWith("Wall"))
                    panelName += "s";
                else
                    panelName += "s";
            }
            else
            {
                panelName += " Panel";
            }
            lines.Add("• " + panelName);

            if (quantity > 1)
            {
                lines.Add($"• (Qty: {quantity})");
            }

            double thickness = 0.75;
            foreach (var key in _panelThicknesses.Keys)
            {
                if (key.StartsWith(panelType) || key == panelType)
                {
                    thickness = _panelThicknesses[key];
                    break;
                }
            }

            if (Math.Abs(thickness - 0.75) > 0.001)
            {
                lines.Add("• " + FormatDimension(thickness, 4) + " Stock");
            }

            double panelHeight = bbox.Max.Y - bbox.Min.Y;
            Point3d arrowTarget = new Point3d(
                bbox.Max.X + 1.0,
                bbox.Max.Y - (panelHeight * 0.4),
                0);

            double diagonalEndY = bbox.Max.Y + 2.5;
            double diagonalRise = diagonalEndY - arrowTarget.Y;
            Point3d diagonalEnd = new Point3d(
                arrowTarget.X + diagonalRise,
                diagonalEndY,
                0);

            Point3d jogEnd = new Point3d(
                diagonalEnd.X + 3.0,
                diagonalEnd.Y,
                0);

            DimensionStyle dimStyle = _doc.DimStyles.Current;
            double baseTextHeight = dimStyle.TextHeight;
            double dimScale = dimStyle.DimensionScale;

            double textHeight = baseTextHeight * dimScale;
            double lineSpacing = textHeight * 1.5;
            double labelGap = textHeight * 1.2;

            var leaderPoints = new List<Point3d> { arrowTarget, diagonalEnd, jogEnd };
            var leaderLine = new PolylineCurve(leaderPoints);

            var leaderAttr = new ObjectAttributes
            {
                LayerIndex = layerIndex,
                ColorSource = ObjectColorSource.ColorFromObject,
                ObjectColor = Color.FromArgb(255, 0, 0)
            };
            _doc.Objects.AddCurve(leaderLine, leaderAttr);

            AddStreamlinedArrowhead(arrowTarget, diagonalEnd, layerIndex);

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
                    TextHeight = textHeight,
                    Font = font,
                    TextHorizontalAlignment = TextHorizontalAlignment.Left,
                    TextVerticalAlignment = TextVerticalAlignment.BottomOfTop,
                    DimensionStyleId = dimStyle.Id
                };
                text.RichText = lines[i];

                var textAttr = new ObjectAttributes
                {
                    LayerIndex = layerIndex,
                    ColorSource = ObjectColorSource.ColorFromObject,
                    ObjectColor = Color.FromArgb(255, 0, 0),
                    Space = ActiveSpace.ModelSpace
                };

                _doc.Objects.AddText(text, textAttr);
            }
        }

        private void AddStreamlinedArrowhead(Point3d tip, Point3d direction, int layerIndex)
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

            var redColor = Color.FromArgb(255, 0, 0);

            try
            {
                var mesh = new Mesh();
                mesh.Vertices.Add(tip);
                mesh.Vertices.Add(base1);
                mesh.Vertices.Add(base2);
                mesh.Faces.AddFace(0, 1, 2);
                mesh.Normals.ComputeNormals();

                var meshAttr = new ObjectAttributes
                {
                    LayerIndex = layerIndex,
                    ColorSource = ObjectColorSource.ColorFromObject,
                    ObjectColor = redColor,
                    PlotColorSource = ObjectPlotColorSource.PlotColorFromObject,
                    PlotColor = redColor
                };

                _doc.Objects.AddMesh(mesh, meshAttr);
            }
            catch
            {
                try
                {
                    var hatches = Hatch.Create(arrowCurve, 0, 0, 1.0, 0.001);
                    if (hatches != null && hatches.Length > 0)
                    {
                        var hatchAttr = new ObjectAttributes
                        {
                            LayerIndex = layerIndex,
                            ColorSource = ObjectColorSource.ColorFromObject,
                            ObjectColor = redColor
                        };
                        _doc.Objects.AddHatch(hatches[0], hatchAttr);
                    }
                }
                catch
                {
                }
            }

            var curveAttr = new ObjectAttributes
            {
                LayerIndex = layerIndex,
                ColorSource = ObjectColorSource.ColorFromObject,
                ObjectColor = redColor
            };
            _doc.Objects.AddCurve(arrowCurve, curveAttr);
        }

    }
}
