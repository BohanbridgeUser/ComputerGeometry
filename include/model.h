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
                void clean();
                void generate_points();
                void flip_empty();
            /// @}
            /// @name Access
            /// @{
                DataPoints_2& Get_DataPoints_2() { return m_points_2; }
                DataPoints_3& Get_DataPoints_3() { return m_points_3; }
                DataSegments_2& Get_DataSegments_2() { return m_Segments_2; }
                DataSegments_3& Get_DataSegments_3() { return m_Segments_3; }
                Polyhedron& Get_Polyhedron() { return m_mesh; }
                Point_3& Get_Center() { return m_center; }
                double Get_Radius() { return m_radius; }
            /// @}
            /// @name Inquiry
            /// @{
                int Get_Points2_Num_Vertices();
                int Get_Mesh_Num_Vertices();
                int Get_Mesh_Num_Facets();
                void calculate_Bbox();
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

                DataPoints_2            m_points_2;
                DataPoints_3            m_points_3;
                DataSegments_2          m_Segments_2;
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