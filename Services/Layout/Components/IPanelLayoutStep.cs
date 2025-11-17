using FWBlueprintPlugin.Models.Layout.Context;
using Rhino.DocObjects;

namespace FWBlueprintPlugin.Services.Layout.Components
{
    internal interface IPanelLayoutStep
    {
        void Execute(PanelLayoutContext context, Layer parentLayer);
    }
}
