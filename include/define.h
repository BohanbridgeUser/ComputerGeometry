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

namespace MyCG
{
    /* CGAL */
    typedef CGAL::Exact_predicates_inexact_constructions_kernel     
                                                                    Kernel;
    typedef Kernel::FT                                        
                                                                    FT;
    typedef CGAL::Point_2<Kernel>
                                                                    Point_2;
    typedef CGAL::Point_3<Kernel>                                   
                                                                    Point_3;
    typedef CGAL::Segment_2<Kernel>
                                                                    Segment_2;
    typedef CGAL::Segment_3<Kernel>
                                                                    Segment_3;                                                                
    typedef CGAL::Vector_3<Kernel>                                  
                                                                    Vector;
    typedef CGAL::Polyhedron_3<Kernel,CGAL::Polyhedron_items_with_id_3>
                                                                    Polyhedron;
    typedef Kernel::Iso_cuboid_3                                    
                                                                    Bbox_3;                            
    typedef Kernel::Iso_rectangle_2                                    
                                                                    Bbox_2;

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
}

#endif // _DEFINE_H_