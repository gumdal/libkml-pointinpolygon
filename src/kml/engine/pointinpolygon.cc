// Implements a basic point-in-polygon test using libkml.

#include "pointinpolygon.h"

// See http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
// for details on how the ray casting algorithm works.
bool IsPointInPolygon(const PointPtr& point, const PolygonPtr& polygon)
{
  // Note: should check that point and polygon are well-formed, have
  // coordinates, etc.
  const Vec3 pt_vec = point->get_coordinates()->get_coordinates_array_at(0);
  double p_lon = pt_vec.get_longitude();
  double p_lat = pt_vec.get_latitude();

  const OuterBoundaryIsPtr& outer = polygon->get_outerboundaryis();
  const LinearRingPtr& ring = outer->get_linearring();
  const CoordinatesPtr& ring_coords = ring->get_coordinates();

  bool is_contained = false;

  size_t n = ring_coords->get_coordinates_array_size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    Vec3 vi = ring_coords->get_coordinates_array_at(i);
    double i_lat = vi.get_latitude();
    double i_lon = vi.get_longitude();
    Vec3 vj = ring_coords->get_coordinates_array_at(j);
    double j_lat = vj.get_latitude();
    double j_lon = vj.get_longitude();
    if ( ((i_lat > p_lat) != (j_lat > p_lat)) &&
        (p_lon < (j_lon - i_lon) * (p_lat - i_lat) / (j_lat - i_lat) + i_lat)) {
      is_contained = !is_contained;
    }
  }
  return is_contained;
}

bool IsPointInKMLPolygon(const char *filePath, kmldom::PointPtr& point)
{
    // Find examples here: http://code.google.com/p/libkml/source/browse/trunk/examples#examples%2Fhelloworld

    bool pointInPolygon = false;
/*    if (DistanceToPolygon(filePath, point)<0.0)
        pointInPolygon = true;*/
    

    return pointInPolygon;
}

float DistanceToPolygon(const char *filePath, kmldom::PointPtr& point)
{
    return 0.0;
}

/*int main(int argc, char** argv) {

  // Square polygon bounded +/- 1°.
  const std::string polygon_str(
      "<Polygon>"
      "<outerBoundaryIs>"
      "<LinearRing>"
      "<coordinates>"
      "-1, -1, 0"
      "1, -1, 0"
      "1, 1, 0"
      "-1, 1, 0"
      "-1, -1, 0"
      "</coordinates>"
      "</LinearRing>"
      "</outerBoundaryIs>"
      "</Polygon>");
  ElementPtr polygon_root = kmldom::Parse(polygon_str, NULL);
  const PolygonPtr polygon = kmldom::AsPolygon(polygon_root);
  
  const std::string inside(
      "<Point>"
      "<coordinates>0, 0, 0</coordinates>"
      "</Point>");
  ElementPtr point_inside = kmldom::Parse(inside, NULL);
  assert(IsPointInPolygon(kmldom::AsPoint(point_inside), polygon));

  const std::string outside(
      "<Point>"
      "<coordinates>2, 0, 0</coordinates>"
      "</Point>");
  ElementPtr point_outside = kmldom::Parse(outside, NULL);
  assert(!IsPointInPolygon(kmldom::AsPoint(point_outside), polygon));

  // Points on the edge are contained.
  const std::string edge(
      "<Point>"
      "<coordinates>-1, -1, 0</coordinates>"
      "</Point>");
  ElementPtr point_edge = kmldom::Parse(edge, NULL);
  assert(IsPointInPolygon(kmldom::AsPoint(point_edge), polygon));
}
*/