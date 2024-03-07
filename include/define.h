#ifndef _DEFINE_H_
#define _DEFINE_H_

/* STL headers */ 
#include <list>
#include <vector>

/* CGAL */
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_2.h>
#include <CGAL/Point_3.h>
#include <CGAL/Line_2.h>
#include <CGAL/Line_3.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Segment_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/bounding_box.h>
#include <CGAL/Polygon_2.h>

#include <CGAL/HalfedgeDS_default.h>
#include <CGAL/HalfedgeDS_min_items.h>
#include <CGAL/HalfedgeDS_items_2.h>
#include <CGAL/HalfedgeDS_decorator.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

#include <CGAL/function_objects.h>
#include <CGAL/Join_input_iterator.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Combination_enumerator.h>
#include <CGAL/intersections.h>
#include <CGAL/random_convex_set_2.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/Random.h>
#include <CGAL/algorithm.h>

namespace MyCG
{
    /* CGAL */
    typedef CGAL::Exact_predicates_inexact_constructions_kernel     
                                                                    Kernel;
    typedef Kernel::FT                                        
                                                                    FT;
    typedef CGAL::Vector_2<Kernel>                                        
                                                                    Vector_2;
    typedef CGAL::Vector_3<Kernel>                                        
                                                                    Vector_3;
    typedef CGAL::Point_2<Kernel>
                                                                    Point_2;
    typedef CGAL::Point_3<Kernel>                                   
                                                                    Point_3;
    typedef CGAL::Segment_2<Kernel>
                                                                    Segment_2;
    typedef CGAL::Segment_3<Kernel>
                                                                    Segment_3;  
    typedef CGAL::Line_2<Kernel>
                                                                    Line_2;
    typedef CGAL::Line_3<Kernel>
                                                                    Line_3;
    typedef CGAL::Polygon_2<Kernel>
                                                                    Polygon_2;                                                        
    typedef CGAL::Vector_3<Kernel>                                  
                                                                    Vector;
    typedef CGAL::Polyhedron_3<Kernel,CGAL::Polyhedron_items_with_id_3>
                                                                    Polyhedron;
    typedef Kernel::Iso_cuboid_3                                    
                                                                    Bbox_3;                            
    typedef Kernel::Iso_rectangle_2                                    
                                                                    Bbox_2;
    typedef CGAL::Combination_enumerator<int>            
                                                                    Combination;
    typedef CGAL::HalfedgeDS_default<Kernel, CGAL::HalfedgeDS_items_3> 
                                                                    HDS;
    typedef CGAL::HalfedgeDS_decorator<HDS>
                                                                    HDS_decorator;

    /* OpenGL */
    typedef std::vector<float> 
                                                                    DataF;
    typedef std::vector<int>
                                                                    DataI;
    typedef std::vector<Point_2>                                    
                                                                    DataPoints_2;
    typedef std::vector<Point_3>                                    
                                                                    DataPoints_3;
    typedef std::vector<Segment_2> 
                                                                    DataSegments_2;
    typedef std::vector<Segment_3> 
                                                                    DataSegments_3; 
    typedef std::vector<Polygon_2>
                                                                    DataPolygon_2;                                                                                                                       
}

#endif // _DEFINE_H_