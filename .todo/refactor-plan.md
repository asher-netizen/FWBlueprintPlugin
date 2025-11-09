# Refactor FWBlueprintPluginCommand into services

## Todo
- [x] Extract panel selection logic into a new service class
- [x] Extract panel arrangement / grouping logic into a new service class
- [x] Extract dimensioning logic (edge + panel dimensions) into a new service class
- [x] Update FWBlueprintPluginCommand to orchestrate the new services
- [x] Ensure partial class helpers are moved or shared cleanly, deleting redundant code
- [x] Validate build and run lint/tests if available
- [x] Fix string comparison incompatibilities and Rhino text warnings after refactor
