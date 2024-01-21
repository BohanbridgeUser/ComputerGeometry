#ifndef _MODEL_H_
#define _MODEL_H_

#include <algorithm>

#include <QString>

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/point_generators_2.h>

#include "define.h"

namespace MyCG
{
    class Model
    {
        public:
            /// @name Type Define
            /// @{
            /// @}
            /// @name Life Circle
            /// @{
                Model();
                ~Model();
            /// @}
            /// @name Operators
            /// @{
            /// @}
            /// @name Operations
            /// @{
                bool load_file(const std::string& filename);
                bool load_file_off(const std::string& filename);
                bool load_file_off2(const std::string& filename);
                bool load_file_int(const std::string& filename);
                bool load_file_chf(const std::string& filename);
                void clean();

                /* Generation */
                void generate_points();
                Combination generate_segments_from_points();
                void generate_segments_on_circle();
                void generate_convexhulls();

                void flip_empty();
            /// @}
            /// @name Access
            /// @{
                /* ConvexHull */
                DataPoints_2& Get_DataPoints_2() { return m_points_2; }
                DataSegments_2& Get_DataSegments_2() { return m_Segments_2; }

                /* Intersection */
                DataPoints_2& Get_Intersection_Points() { return m_intersection_points; }
                DataSegments_2& Get_Intersection_Interval() { return m_intersection_interval; }
                DataSegments_2& Get_Intersection_Segments() { return m_intersection_segments; }
                DataPoints_2& Get_Generation_Intersection_Points() { return m_generation_intersection_points; }

                /* Mesh */
                DataPoints_3& Get_DataPoints_3() { return m_points_3; }
                DataSegments_3& Get_DataSegments_3() { return m_Segments_3; }
                Polyhedron& Get_Polyhedron() { return m_mesh; }

                /* Generation ConvexHull */
                DataPoints_2& Get_Generation_ConvexHull_Points() { return m_generation_convexhull_points; }
                DataSegments_2& Get_Generation_ConvexHull_Segments() { return m_generation_convexhull_segments; }
                DataPoints_2& Get_Intersection_ConvexHull_Points() { return m_intersection_convexhull_points; }
                DataSegments_2& Get_Intersection_ConvexHull_Segments() { return m_intersection_convexhull; }

                /* Camera */
                Point_3& Get_Center() { return m_center; }
                double Get_Radius() { return m_radius; }
            /// @}
            /// @name Inquiry
            /// @{
                /* ConvexHull */
                int Get_Points2_Num_Vertices();
                int Get_Segments2_Num_Segments();

                /* Intersection */
                int Get_Intersection_Num_Segments();
                int Get_Intersection_Num_Points();
                int Get_Intersection_Num_Intervals();
                
                /* Mesh */
                int Get_Mesh_Num_Vertices();
                int Get_Mesh_Num_Facets();

                /* Camera */
                void calculate_Bbox_2();
            /// @}
        protected:
            /// @name Protected Static Member Variables
            /// @{
            /// @}
            /// @name Protected Member Variables
            /// @{
            /// @}
            /// @name Protected Operatiors
            /// @{
            /// @}
            /// @name Protected Operations
            /// @{
            /// @}
            /// @name Protected Access
            /// @{
            /// @}
            /// @name Protected Inquiry
            /// @{
            /// @}
        private:
            /// @name Private Static Member Variables
            /// @{                
                bool                    m_empty;

                Point_3                 m_center;  
                double                  m_radius;

                /* ConvexHull */
                DataPoints_2            m_points_2;
                DataSegments_2          m_Segments_2;

                /* Generation ConvexHull*/
                DataPoints_2            m_generation_convexhull_points;
                DataSegments_2          m_generation_convexhull_segments;
                DataPoints_2            m_intersection_convexhull_points;
                DataSegments_2          m_intersection_convexhull;

                /* Intersection */
                DataPoints_2            m_intersection_points;
                DataSegments_2          m_intersection_interval;
                DataSegments_2          m_intersection_segments;
                DataPoints_2            m_generation_intersection_points;

                /* Mesh */ 
                DataPoints_3            m_points_3;
                DataSegments_3          m_Segments_3;
                Polyhedron              m_mesh;
            /// @}
            /// @name Private Member Variables
            /// @{
            /// @}
            /// @name Private Operatiors
            /// @{
            /// @}
            /// @name Private Operations
            /// @{
            /// @}
            /// @name Private Access
            /// @{
            /// @}
            /// @name Private Inquiry
            /// @{
            /// @}
    };
}

#endif // _MODEL_H_