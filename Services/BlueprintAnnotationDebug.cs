using System;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;

namespace FWBlueprintPlugin.Services
{
    internal static class BlueprintAnnotationDebug
    {
        public static DimensionStyle ResolveDefaultStyle(RhinoDoc doc, string caller)
        {
            bool verbose = LoggingOptions.EnableVerboseLogging;

            if (doc == null)
            {
                RhinoApp.WriteLine($"[Blueprint Styles][{caller}] RhinoDoc is null; cannot resolve dimension style.");
                return null;
            }

            if (verbose)
            {
                RhinoApp.WriteLine($"[Blueprint Styles][{caller}] Document '{doc.Name}' has {doc.DimStyles.Count} dim style(s). Current='{doc.DimStyles.Current?.Name ?? "(null)"}'.");
            }

            var blueprintStyle = doc.DimStyles.FindName(BlueprintAnnotationStyles.Default);
            if (blueprintStyle != null)
            {
                if (verbose)
                {
                    RhinoApp.WriteLine($"[Blueprint Styles][{caller}] Using '{blueprintStyle.Name}' (Index={blueprintStyle.Index}, Id={blueprintStyle.Id}).");
                }
                return blueprintStyle;
            }

            var fallback = doc.DimStyles.Current ?? (doc.DimStyles.Count > 0 ? doc.DimStyles[0] : null);
            RhinoApp.WriteLine($"[Blueprint Styles][{caller}] '{BlueprintAnnotationStyles.Default}' missing. Fallback -> '{fallback?.Name ?? "(null)"}'.");
            return fallback;
        }

        public static void LogDimensionRequest(string caller, string dimensionType, DimensionStyle style, Point3d start, Point3d end, Point3d dimLinePoint, int layerIndex)
        {
            if (LoggingOptions.EnableVerboseLogging)
            {
                RhinoApp.WriteLine(
                    $"[Blueprint Styles][{caller}] Creating {dimensionType} dimension | layer={layerIndex} | style='{style?.Name ?? "(null)"}' (Id={style?.Id ?? Guid.Empty}) | start={FormatPoint(start)} end={FormatPoint(end)} dimLine={FormatPoint(dimLinePoint)}");
            }
        }

        public static void LogStyleImport(string message, bool always = false)
        {
            if (always || LoggingOptions.EnableVerboseLogging)
            {
                RhinoApp.WriteLine($"[Blueprint Styles][Import] {message}");
            }
        }

        private static string FormatPoint(Point3d pt) =>
            $"({pt.X:F3}, {pt.Y:F3}, {pt.Z:F3})";
    }
}
