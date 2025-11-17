using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using FWBlueprintPlugin.Infrastructure.Logging;
using Rhino;
using Rhino.DocObjects;
using Rhino.FileIO;

namespace FWBlueprintPlugin.Services
{
    internal static class BlueprintAnnotationStyles
    {
        public const string Default = "Blueprint Dim - Default";
        public const string Detail = "Blueprint Dim - Detail";

        public static readonly string[] All = { Default, Detail };
    }

    /// <summary>
    /// Ensures the required Blueprint dimension styles exist in the current document by importing them
    /// from an embedded Rhino model when needed.
    /// </summary>
    internal sealed class BlueprintAnnotationStyleProvider
    {
        private const string ResourceName = "FWBlueprintPlugin.EmbeddedResources.BlueprintAnnotationStyles.3dm";
        private static readonly IReadOnlyDictionary<string, string[]> StyleAliases =
            new Dictionary<string, string[]>(StringComparer.OrdinalIgnoreCase)
            {
                { BlueprintAnnotationStyles.Default, new[] { "Blueprint Dim - Defaut" } }
            };
        private readonly RhinoDoc _doc;

        public BlueprintAnnotationStyleProvider(RhinoDoc doc)
        {
            _doc = doc ?? throw new ArgumentNullException(nameof(doc));
        }

        public void EnsureStyles()
        {
            if (_doc == null)
            {
                return;
            }

            bool verbose = LoggingService.IsEnabled(LogLevel.Debug);

            if (verbose)
            {
                BlueprintAnnotationDebug.LogStyleImport($"Ensuring styles for doc '{_doc.Name}' with {_doc.DimStyles.Count} existing dim style(s): {DescribeDimStyles()}");
            }

            var missingStyles = GetMissingStyles();
            if (missingStyles.Count == 0)
            {
                if (verbose)
                {
                    BlueprintAnnotationDebug.LogStyleImport("All Blueprint styles already present; no import required.");
                }
                return;
            }

            if (verbose)
            {
                BlueprintAnnotationDebug.LogStyleImport($"Missing styles detected: {string.Join(", ", missingStyles)}");
            }

            string tempFilePath = ExtractResourceToTempFile();
            if (string.IsNullOrWhiteSpace(tempFilePath))
            {
                BlueprintAnnotationDebug.LogStyleImport("Unable to access embedded annotation styles (resource stream empty).", true);
                return;
            }

            try
            {
                long sizeBytes = new FileInfo(tempFilePath).Length;
                if (verbose)
                {
                    BlueprintAnnotationDebug.LogStyleImport($"Loading carrier model '{tempFilePath}' ({sizeBytes} bytes) to copy missing styles.");
                }

                int addedCount = CopyStylesFromCarrier(tempFilePath, missingStyles);
                if (verbose)
                {
                    BlueprintAnnotationDebug.LogStyleImport($"Attempted to add {missingStyles.Count} style(s); successfully added {addedCount}.");
                }

                var stillMissing = GetMissingStyles();
                if (stillMissing.Count > 0)
                {
                    BlueprintAnnotationDebug.LogStyleImport($"Import completed but styles still missing: {string.Join(", ", stillMissing)}", true);
                }
                else
                {
                    if (verbose)
                    {
                        BlueprintAnnotationDebug.LogStyleImport($"Import succeeded; document now has {_doc.DimStyles.Count} dim style(s): {DescribeDimStyles()}");
                    }
                }
            }
            finally
            {
                TryDeleteTempFile(tempFilePath);
            }
        }

        private List<string> GetMissingStyles()
        {
            var missing = new List<string>();
            foreach (var styleName in BlueprintAnnotationStyles.All)
            {
                if (_doc.DimStyles.FindName(styleName) == null)
                {
                    missing.Add(styleName);
                }
            }

            return missing;
        }

        private static string ExtractResourceToTempFile()
        {
            try
            {
                var assembly = typeof(BlueprintAnnotationStyleProvider).Assembly;
                using (var resourceStream = assembly.GetManifestResourceStream(ResourceName))
                {
                    if (resourceStream == null)
                    {
                        BlueprintAnnotationDebug.LogStyleImport("Embedded resource stream not found; check build output.", true);
                        return string.Empty;
                    }

                    string tempFilePath = Path.Combine(Path.GetTempPath(), $"FWBlueprintStyles_{Guid.NewGuid():N}.3dm");
                    using (var fileStream = File.Create(tempFilePath))
                    {
                        resourceStream.CopyTo(fileStream);
                    }

                    return tempFilePath;
                }
            }
            catch (Exception ex)
            {
                BlueprintAnnotationDebug.LogStyleImport($"Failed to extract embedded styles: {ex.Message}", true);
                return string.Empty;
            }
        }

        private int CopyStylesFromCarrier(string filePath, IEnumerable<string> requestedStyles)
        {
            int added = 0;
            File3dm carrier;
            try
            {
                carrier = File3dm.Read(filePath);
            }
            catch (Exception ex)
            {
                BlueprintAnnotationDebug.LogStyleImport($"Unable to read carrier model '{filePath}': {ex.Message}", true);
                return 0;
            }

            if (carrier == null)
            {
                BlueprintAnnotationDebug.LogStyleImport("Carrier model read returned null; aborting style copy.", true);
                return 0;
            }

            var carrierStyles = carrier.AllDimStyles;
            if (carrierStyles == null)
            {
                BlueprintAnnotationDebug.LogStyleImport("Carrier file did not expose AllDimStyles; no styles copied.", true);
                return 0;
            }

            foreach (var styleName in requestedStyles)
            {
                if (_doc.DimStyles.FindName(styleName) != null)
                {
                    continue;
                }

                DimensionStyle sourceStyle = FindCarrierStyle(carrierStyles, styleName);
                if (sourceStyle == null)
                {
                    BlueprintAnnotationDebug.LogStyleImport($"Carrier file did not contain style '{styleName}'.", true);
                    continue;
                }

                var clone = sourceStyle.Duplicate();
                if (clone == null)
                {
                    BlueprintAnnotationDebug.LogStyleImport($"Failed to duplicate style '{styleName}' from carrier file.", true);
                    continue;
                }

                clone.Name = styleName;
                clone.Id = Guid.NewGuid();

                int newIndex = _doc.DimStyles.Add(clone, true);
                if (newIndex >= 0)
                {
                    added++;
                    BlueprintAnnotationDebug.LogStyleImport($"Added style '{styleName}' at index {newIndex} (Id={clone.Id}).");
                }
                else
                {
                    BlueprintAnnotationDebug.LogStyleImport($"Rhino refused to add style '{styleName}'; DimStyles.Add returned {newIndex}.", true);
                }
            }

            return added;
        }

        private static DimensionStyle FindCarrierStyle(File3dmDimStyleTable table, string styleName)
        {
            if (table == null || string.IsNullOrWhiteSpace(styleName))
            {
                return null;
            }

            var direct = table.FindName(styleName);
            if (direct != null)
            {
                return direct;
            }

            if (StyleAliases.TryGetValue(styleName, out var aliases))
            {
                foreach (var alias in aliases)
                {
                    var aliasMatch = table.FindName(alias);
                    if (aliasMatch != null)
                    {
                        BlueprintAnnotationDebug.LogStyleImport($"Carrier style '{aliasMatch.Name}' matched requested '{styleName}' via alias '{alias}'.");
                        return aliasMatch;
                    }
                }
            }

            foreach (var style in table)
            {
                var candidateName = style?.Name;
                if (string.IsNullOrWhiteSpace(candidateName))
                {
                    continue;
                }

                if (string.Equals(candidateName.Trim(), styleName, StringComparison.OrdinalIgnoreCase))
                {
                    BlueprintAnnotationDebug.LogStyleImport($"Carrier style '{candidateName}' matched requested '{styleName}' via relaxed comparison.");
                    return style;
                }
            }

            var availableNames = string.Join(", ", table.Select(s => s?.Name ?? "(null)"));
            BlueprintAnnotationDebug.LogStyleImport($"Carrier styles available: {availableNames}", true);
            return null;
        }

        private static void TryDeleteTempFile(string path)
        {
            if (string.IsNullOrWhiteSpace(path))
            {
                return;
            }

            try
            {
                if (File.Exists(path))
                {
                    File.Delete(path);
                    BlueprintAnnotationDebug.LogStyleImport($"Deleted temporary style carrier '{path}'.");
                }
            }
            catch
            {
                BlueprintAnnotationDebug.LogStyleImport($"Failed to delete temporary file '{path}'. Manual cleanup may be required.", true);
            }
        }

        private string DescribeDimStyles()
        {
            if (_doc == null || _doc.DimStyles.Count == 0)
            {
                return "(none)";
            }

            var descriptions = new List<string>();
            for (int i = 0; i < _doc.DimStyles.Count; i++)
            {
                var style = _doc.DimStyles[i];
                descriptions.Add($"{style.Name}(Idx={i},Id={style.Id})");
            }
            return string.Join("; ", descriptions);
        }
    }
}
