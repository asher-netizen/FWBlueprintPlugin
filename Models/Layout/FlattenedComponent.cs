using Rhino.Geometry;

namespace FWBlueprintPlugin.Models.Layout
{
    /// <summary>
    /// Stores a flattened geometry object and its bounding box.
    /// </summary>
    internal struct FlattenedComponent
    {
        public GeometryBase Geom { get; set; }
        public BoundingBox BBox { get; set; }
    }
}
