namespace FWBlueprintPlugin.Models.Dimensioning
{
    /// <summary>
    /// Represents an edge cutout feature detected on a panel perimeter.
    /// </summary>
    internal class EdgeFeature
    {
        public string EdgeName { get; set; }
        public double StartPos { get; set; }
        public double EndPos { get; set; }
        public double Width { get; set; }
        public double Depth { get; set; }
        public string Type { get; set; }
        public double CenterPos { get; set; }
        public bool HasCurvature { get; set; }
        public double GapBefore { get; set; }
        public double GapAfter { get; set; }
        public double EdgeLength { get; set; }

        public EdgeFeature(
            string edgeName,
            double startPos,
            double endPos,
            double width,
            double depth,
            string type,
            bool hasCurvature,
            double gapBefore = 0,
            double gapAfter = 0,
            double edgeLength = 0)
        {
            EdgeName = edgeName;
            StartPos = startPos;
            EndPos = endPos;
            Width = width;
            Depth = depth;
            Type = type;
            CenterPos = startPos + (width / 2.0);
            HasCurvature = hasCurvature;
            GapBefore = gapBefore;
            GapAfter = gapAfter;
            EdgeLength = edgeLength;
        }
    }
}
