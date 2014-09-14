// Implements a basic point-in-polygon test using libkml.

#include "pointinpolygon.h"

#include <string>
#include "kml/base/file.h"

using kmlbase::Vec3;
using kmldom::CoordinatesPtr;
using kmldom::ElementPtr;
using kmldom::LinearRingPtr;
using kmldom::OuterBoundaryIsPtr;
using kmldom::PointPtr;
using kmldom::PolygonPtr;
using std::cout;
using std::endl;

using kmldom::ContainerPtr;
using kmldom::ElementPtr;
using kmldom::FeaturePtr;
using kmldom::GeometryPtr;
using kmldom::KmlPtr;
using kmldom::MultiGeometryPtr;
using kmldom::PlacemarkPtr;
using std::cout;
using std::endl;

void WalkGeometry(const GeometryPtr& geometry);
void WalkGeometryPointInPolygon(const GeometryPtr& geometry, const PointPtr& point, bool *outPointInPolygon);
void WalkFeature(const FeaturePtr& feature);
void WalkFeaturePointInPolygon(const FeaturePtr& feature, const PointPtr& point, bool *outPointInPolygon);
void WalkContainer(const ContainerPtr& container);
void WalkContainerPointInPolygon(const ContainerPtr& container, const PointPtr& point, bool *outPointInPolygon);
const FeaturePtr GetRootFeature(const ElementPtr& root);

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
            (p_lon < (j_lon - i_lon) * (p_lat - i_lat) / (j_lat - i_lat) + i_lon)) {
            is_contained = !is_contained;
        }
    }
    return is_contained;
}

#pragma mark - Walking through the geometry
void WalkGeometryPointInPolygon(const GeometryPtr& geometry, const PointPtr& point, bool *outPointInPolygon)
{
    if (!geometry || *outPointInPolygon) {
        return;
    }
    // Print the Geometry type.
    //    cout << "Found a";
    switch(geometry->Type()) {
        case kmldom::Type_Point:
            //            cout << " Point";
            break;
        case kmldom::Type_LineString:
            //            cout << " LineString";
            break;
        case kmldom::Type_LinearRing:
            //            cout << " LinearRing";
            break;
        case kmldom::Type_Polygon:
            //            cout << " Polygon";
        {
            kmldom::PolygonPtr polyGeo = kmldom::AsPolygon(geometry);
            if (point && outPointInPolygon)
            {
                if (IsPointInPolygon(point,  polyGeo))
                {
                    *outPointInPolygon = true;
                }
            }
        }
            break;
        case kmldom::Type_MultiGeometry:
            //            cout << " MultiGeometry";
            break;
        case kmldom::Type_Model:
            //            cout << " Model";
            break;
        default:  // KML has 6 types of Geometry.
            break;
    }
    //    cout << endl;
    // Recurse into <MultiGeometry>.
    if (const MultiGeometryPtr multigeometry =
        kmldom::AsMultiGeometry(geometry)) {
        for (size_t i = 0; i < multigeometry->get_geometry_array_size(); ++i) {
            WalkGeometry(multigeometry->get_geometry_array_at(i));
        }
    }
}

void WalkGeometry(const GeometryPtr& geometry)
{
    WalkGeometryPointInPolygon(geometry, NULL, NULL);
}

void WalkFeaturePointInPolygon(const FeaturePtr& feature, const PointPtr& point, bool *outPointInPolygon)
{
    if (feature) {
        if (const ContainerPtr container = kmldom::AsContainer(feature)) {
            WalkContainerPointInPolygon(container, point, outPointInPolygon);
        } else if (const PlacemarkPtr placemark = kmldom::AsPlacemark(feature)) {
            WalkGeometryPointInPolygon(placemark->get_geometry(), point, outPointInPolygon);
        }
    }
}

void WalkFeature(const FeaturePtr& feature)
{
    WalkFeaturePointInPolygon(feature, NULL, NULL);
}

void WalkContainerPointInPolygon(const ContainerPtr& container, const PointPtr& point, bool *outPointInPolygon)
{
    for (size_t i = 0; i < container->get_feature_array_size(); ++i) {
        WalkFeaturePointInPolygon(container->get_feature_array_at(i), point, outPointInPolygon);
    }
}

void WalkContainer(const ContainerPtr& container)
{
    WalkContainerPointInPolygon(container, NULL, NULL);
}

const FeaturePtr GetRootFeature(const ElementPtr& root) {
    const KmlPtr kml = kmldom::AsKml(root);
    if (kml && kml->has_feature()) {
        return kml->get_feature();
    }
    return kmldom::AsFeature(root);
}

#pragma mark -
bool IsPointInKMLPolygon(const char *filePath, kmldom::PointPtr& point)
{
    // Find examples here: http://code.google.com/p/libkml/source/browse/trunk/examples#examples%2Fhelloworld
    // How to walk through polygons: http://code.google.com/p/libkml/source/browse/trunk/examples/helloworld/printgeometry.cc
    
    bool pointInPolygon = false;
    /*    if (DistanceToPolygon(filePath, point)<0.0)
     pointInPolygon = true;*/
    
    std::string kml;
    kmlbase::File::ReadFileToString(filePath, &kml);
    //    std::cout<<"Output string: " <<kml;
    if (kml.length())
    {
        std::string errors;
        WalkFeaturePointInPolygon(GetRootFeature(kmldom::Parse(kml, &errors)), point, &pointInPolygon);
        if (!errors.empty()) {
            //            cout << filePath << ": parse error" << endl;
            //            cout << errors << endl;
            //            return 0;
        }
    }
    
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