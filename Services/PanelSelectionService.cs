using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Input;
using Rhino.Input.Custom;
using System;
using System.Collections.Generic;
using System.Linq;
using Rhino.Geometry;

namespace FWBlueprintPlugin.Services
{
    /// <summary>
    /// Handles user-facing selection flows for the blueprint command.
    /// </summary>
    internal class PanelSelectionService
    {
        private readonly RhinoDoc _doc;

        public PanelSelectionService(RhinoDoc doc)
        {
            _doc = doc ?? throw new ArgumentNullException(nameof(doc));
        }

        public List<RhinoObject> SelectTopComponents()
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

        public LiftLidTopComponents DetectLiftLidComponents(List<RhinoObject> topComponents)
        {
            if (topComponents == null)
            {
                return null;
            }

            var config = new LiftLidTopComponents
            {
                TotalComponentCount = topComponents.Count
            };

            if (topComponents.Count == 1)
            {
                config.TopPlates = topComponents;
                return config;
            }

            var componentData = new List<(RhinoObject obj, BoundingBox bbox, double height, double width)>();
            foreach (var obj in topComponents)
            {
                var bbox = obj.Geometry.GetBoundingBox(true);
                double height = bbox.Max.Y - bbox.Min.Y;
                double width = bbox.Max.X - bbox.Min.X;
                componentData.Add((obj, bbox, height, width));
            }

            double minHeight = componentData.Min(c => c.height);
            const double heightTolerance = 0.1;

            var backerCandidates = componentData.Where(c => Math.Abs(c.height - minHeight) < heightTolerance).ToList();
            var remainingComponents = new List<RhinoObject>(topComponents);

            foreach (var backerData in backerCandidates)
            {
                config.BackerPlates.Add(backerData.obj);
                remainingComponents.Remove(backerData.obj);

                double backerWidth = backerData.width;
                double backerCenterX = (backerData.bbox.Min.X + backerData.bbox.Max.X) / 2;
                double backerMinY = backerData.bbox.Min.Y;
                double backerMaxY = backerData.bbox.Max.Y;

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

                    double widthDiff = Math.Abs(lidWidth - backerWidth);
                    bool widthMatches = widthDiff < 1.0;

                    double xDiff = Math.Abs(lidCenterX - backerCenterX);
                    bool xAligned = xDiff < 2.0;

                    double frontGap = Math.Abs(lidMinY - backerMaxY);
                    double backGap = Math.Abs(lidMaxY - backerMinY);
                    double minGap = Math.Min(frontGap, backGap);
                    bool adjacent = minGap < 1.0;

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
            }

            config.TopPlates = remainingComponents;
            return ConfirmTopConfiguration(config);
        }

        public RhinoObject SelectSinglePanel(string prompt)
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

        public List<RhinoObject> SelectMultiple(string prompt)
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

        private LiftLidTopComponents ConfirmTopConfiguration(LiftLidTopComponents config)
        {
            var originalColors = new Dictionary<Guid, System.Drawing.Color>();

            foreach (var obj in config.BackerPlates)
            {
                originalColors[obj.Id] = obj.Attributes.ObjectColor;
                obj.Attributes.ObjectColor = System.Drawing.Color.Blue;
                obj.Attributes.ColorSource = ObjectColorSource.ColorFromObject;
                obj.CommitChanges();
            }

            foreach (var obj in config.LiftLids)
            {
                originalColors[obj.Id] = obj.Attributes.ObjectColor;
                obj.Attributes.ObjectColor = System.Drawing.Color.Green;
                obj.Attributes.ColorSource = ObjectColorSource.ColorFromObject;
                obj.CommitChanges();
            }

            foreach (var obj in config.TopPlates)
            {
                originalColors[obj.Id] = obj.Attributes.ObjectColor;
                obj.Attributes.ObjectColor = System.Drawing.Color.Red;
                obj.Attributes.ColorSource = ObjectColorSource.ColorFromObject;
                obj.CommitChanges();
            }

            _doc.Views.Redraw();

            var gk = new GetString();
            gk.SetCommandPrompt("[Enter]=Correct and continue  [B]=Reselect backers  [L]=Reselect lift lids  [Esc]=Cancel");
            gk.AcceptNothing(true);
            gk.AddOption("Backers");
            gk.AddOption("LiftLids");

            var result = gk.Get();

            foreach (var kvp in originalColors)
            {
                var obj = _doc.Objects.FindId(kvp.Key);
                if (obj != null)
                {
                    obj.Attributes.ObjectColor = kvp.Value;
                    obj.Attributes.ColorSource = ObjectColorSource.ColorFromLayer;
                    obj.CommitChanges();
                }
            }
            _doc.Views.Redraw();

            if (result == GetResult.Cancel)
            {
                return null;
            }

            return config;
        }
    }
}
