**Change Log**

- **Files Created**  
  - `BlueprintModels.cs` centralizes DTOs (`PanelDimensionInfo`, `PanelLeaderInfo`, `EdgeFeature`, `PanelArrangementResult`, `LiftLidTopComponents`, `LiftLidFlattenedInfo`, `FlattenedComponent`).  
  - `Services/DimensionFormatting.cs`, `PanelSelectionService.cs`, `PanelArrangementService.cs`, `PanelDimensioningService.cs`, `EdgeDimensioningService.cs` establish the service layer for selection, layout, formatting, panel dimensions, and edge feature dimensions.  
  - `.todo/refactor-plan.md` tracks refactor milestones (all now checked off).

- **File Deleted**  
  - `EdgeFeatureDimensioning.cs` removed after migrating its logic into `Services/EdgeDimensioningService.cs`.

- **Files Modified**  
  - `FWBlueprintPluginCommand.cs` trimmed to an orchestrator that wires the new services and keeps only hide helpers.  
  - `FWBlueprint_Phase3_Additions.cs` now consumes shared models/helpers, instantiates `EdgeDimensioningService`, and delegates edge-dimension output there.  
  - Build artifacts (`bin`, `obj`) changed as part of compilation; omitted here per repository hygiene.

**Representative Diffs**

```diff
diff --git a/FWBlueprintPluginCommand.cs b/FWBlueprintPluginCommand.cs
@@
-using System;
-using System.Collections.Generic;
-using Rhino;
-using Rhino.Commands;
-using Rhino.DocObjects;
+using System;
+using System.Collections.Generic;
+using FWBlueprintPlugin.Services;
+using Rhino;
+using Rhino.Commands;
+using Rhino.DocObjects;
@@
-        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
-        {
-            // ... monolithic selection, flattening, dimensioning, edge helpers ...
-        }
+        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
+        {
+            var selectionService = new PanelSelectionService(doc);
+            var topComponents = selectionService.SelectTopComponents();
+            // ... selection early exits ...
+            var arrangementService = new PanelArrangementService(doc);
+            arrangementService.SetupLayersAndStyles(parentLayer);
+            var arrangementResult = arrangementService.ArrangePanels(/* ...categories... */);
+            if (!arrangementResult.Success)
+            {
+                return Result.Failure;
+            }
+            var dimensionService = new PanelDimensioningService(doc, arrangementResult.PanelThicknesses);
+            foreach (var dimInfo in arrangementResult.DeferredDimensions)
+            {
+                dimensionService.AddPanelDimensions(dimInfo);
+            }
+            foreach (var leaderInfo in arrangementResult.DeferredLeaders)
+            {
+                dimensionService.AddPanelLeader(leaderInfo.BBox, leaderInfo.PanelType, leaderInfo.Quantity, leaderInfo.LayerIndex);
+            }
+            ExtractAndCreate2DPanels(doc, parentLayer);
+            doc.Views.Redraw();
+            return Result.Success;
+        }
@@
-        private void AddPanelDimensions(...){...}
-        private void AddPanelLeader(...){...}
-        private void AddStreamlinedArrowhead(...){...}
-        private GeometryBase FlattenPanel(...){...}
-        private void AddToLayer(...){...}
-        private string FormatDimension(...){...}
-        private int GCD(...){...}
+        private static void HideObjects(...){...}
+        private static void HideObject(...){...}
```

```diff
diff --git a/FWBlueprint_Phase3_Additions.cs b/FWBlueprint_Phase3_Additions.cs
@@
-// edge helper class and inline FormatDimension usage
+using FWBlueprintPlugin.Services;
+using static FWBlueprintPlugin.Services.DimensionFormatting;
@@
-                var edgeCutouts = DetectEdgeFeaturesFromSegments_RayProtocol(doc, bbox, panelBoundary, segmentData, panelBrep, tol, idxDims);
+                var edgeCutouts = DetectEdgeFeaturesFromSegments_RayProtocol(doc, bbox, panelBoundary, segmentData, panelBrep, tol, idxDims, edgeDimensioningService);
@@
-        private List<Brep> DetectEdgeFeaturesFromSegments_RayProtocol(..., int dimensionsLayerIndex)
+    private List<Brep> DetectEdgeFeaturesFromSegments_RayProtocol(..., int dimensionsLayerIndex, EdgeDimensioningService edgeDimensioningService)
@@
-                AddEdgeFeatureDimensions(doc, bbox, allEdgeFeatures, dimensionsLayerIndex);
+                edgeDimensioningService.AddEdgeFeatureDimensions(bbox, allEdgeFeatures, dimensionsLayerIndex);
-        private class EdgeFeature { ... }
```

```diff
diff --git a/Services/DimensionFormatting.cs b/Services/DimensionFormatting.cs
+using System;
+
+namespace FWBlueprintPlugin.Services
+{
+    internal static class DimensionFormatting
+    {
+        public static string FormatDimension(double inches, int denominatorPower)
+        {
+            // fraction formatting shared by services and Phase 3A
+        }
+
+        private static int GreatestCommonDivisor(int a, int b)
+        {
+            while (b != 0)
+            {
+                int temp = b;
+                b = a % b;
+                a = temp;
+            }
+            return a;
+        }
+    }
+}
```

```diff
diff --git a/Services/PanelSelectionService.cs b/Services/PanelSelectionService.cs
+namespace FWBlueprintPlugin.Services
+{
+    internal class PanelSelectionService
+    {
+        // SelectTopComponents, DetectLiftLidComponents, SelectSinglePanel,
+        // SelectMultiple, ConfirmTopConfiguration (color feedback + prompt)
+    }
+}
```

```diff
diff --git a/Services/PanelArrangementService.cs b/Services/PanelArrangementService.cs
+namespace FWBlueprintPlugin.Services
+{
+    internal class PanelArrangementService
+    {
+        // CreateLayerStructure, CreateDimensionStyle
+        // ArrangePanels orchestrates FlattenLiftLidTop, ProcessPanelsHorizontalButt,
+        // defers dimensions/leaders and tracks panel thickness.
+        // Contains moved helpers: FlattenLiftLidTop, DeferLiftLidTopDimensions,
+        // ProcessPanelsHorizontalButt, SwitchToTopViewportAndCenter, CalculateThickness,
+        // FlattenPanel, AddToLayer, FindLayerIndex.
+    }
+}
```

```diff
diff --git a/Services/PanelDimensioningService.cs b/Services/PanelDimensioningService.cs
+namespace FWBlueprintPlugin.Services
+{
+    internal class PanelDimensioningService
+    {
+        // AddPanelDimensions, AddPanelLeader, AddStreamlinedArrowhead
+        // uses DimensionFormatting.FormatDimension and Rhino rich-text safe assignments.
+    }
+}
```

```diff
diff --git a/Services/EdgeDimensioningService.cs b/Services/EdgeDimensioningService.cs
+namespace FWBlueprintPlugin.Services
+{
+    internal class EdgeDimensioningService
+    {
+        // AddEdgeFeatureDimensions delegates to AddEdgeHoleDimensions,
+        // AddEdgeSlotDimensions, AddMixedEdgeDimensions,
+        // CreateRunningDimensionLine, GetEdgeParameters (Rectangle3d.BoundingBox),
+        // ValidateSegmentSum, LogSegments. Replaces deleted partial.
+    }
+}
```

```diff
diff --git a/BlueprintModels.cs b/BlueprintModels.cs
+namespace FWBlueprintPlugin
+{
+    internal class PanelDimensionInfo { ... }
+    internal class PanelLeaderInfo { ... }
+    internal class EdgeFeature { ... }
+    internal class PanelArrangementResult { ... }
+    internal class LiftLidTopComponents { ... }
+    internal class LiftLidFlattenedInfo { ... }
+    internal struct FlattenedComponent { ... }
+}
```

```diff
diff --git a/.todo/refactor-plan.md b/.todo/refactor-plan.md
+- [x] Extract panel selection logic into a new service class
+- [x] Extract panel arrangement / grouping logic into a new service class
+- [x] Extract dimensioning logic (edge + panel dimensions) into a new service class
+- [x] Update FWBlueprintPluginCommand to orchestrate the new services
+- [x] Ensure partial class helpers are moved or shared cleanly, deleting redundant code
+- [x] Validate build and run lint/tests if available
+- [x] Fix string comparison incompatibilities and Rhino text warnings after refactor
```

_For readability, the service file diffs above summarize moved methods; run `git diff` locally for the full 1:1 line output._

**Refactor Highlights**

- Moved `FlattenLiftLidTop`, `DeferLiftLidTopDimensions`, `ProcessPanelsHorizontalButt`, `SwitchToTopViewportAndCenter`, `AddToLayer`, and `FlattenPanel` from `FWBlueprintPluginCommand` into `PanelArrangementService` inside `FWBlueprintPlugin.Services`.
- Moved `AddPanelDimensions`, `AddPanelLeader`, `AddStreamlinedArrowhead`, and shared `FormatDimension` logic from the command into `PanelDimensioningService` and `DimensionFormatting`.
- Moved edge dimension routines (`AddEdgeHoleDimensions`, etc.) into `EdgeDimensioningService`; `EdgeFeature` POCO relocated to `BlueprintModels`.
- Updated namespaces/usings so the command and Phase 3 partial reference `FWBlueprintPlugin.Services`; `using static` added for `DimensionFormatting`.
- Addressed prior build warnings by consolidating `FormatDimension`, using Rhino `TextEntity.RichText`, and removing duplicated string helpers.

**Updated Layout**

- Root: `FWBlueprintPluginCommand.cs`, `FWBlueprint_Phase3_Additions.cs`, `BlueprintModels.cs`, `FWBlueprintPluginPlugin.cs`, project/solution files, existing partials, `.todo/refactor-plan.md`.
- `Services/`: `DimensionFormatting.cs`, `PanelSelectionService.cs`, `PanelArrangementService.cs`, `PanelDimensioningService.cs`, `EdgeDimensioningService.cs`.
- `EmbeddedResources/`, `Properties/`, `bin/`, `obj/` remain unchanged aside from build outputs.

**Main Command Signature**

```
protected override Result RunCommand(RhinoDoc doc, RunMode mode)
```

Builds clean, runs identical to pre-refactor, all Phase 2 logic preserved.
