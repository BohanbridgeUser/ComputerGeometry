#include "model.h"


namespace MyCG
{
    /// @brief public:
    /// @name Type Define
    /// @{
    /// @}
    /// @name Life Circle
    /// @{
        Model::Model()
        {
            m_empty = true;
        }
        Model::~Model(){}
    /// @}
    /// @name Operators
    /// @{
    /// @}
    /// @name Operations
    /// @{
        void Model::clean()
        {
            m_mesh.clear();
            m_points_2.clear();
            m_points_3.clear();
            m_Segments_2.clear();
            m_Segments_3.clear();
            m_empty = true;
        }

        bool Model::load_file(const std::string& filename)
        {
            clean();
            if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(filename, m_mesh))
                return false;
            m_empty = false;
            return !m_empty;
        }

        void Model::generate_points()
        {
            clean();

            typedef CGAL::Creator_uniform_2<double,Point_2>  Creator;
            m_points_2.reserve(100);
            CGAL::Random_points_in_square_2<Point_2,Creator> g(2.0);
            std::copy_n( g, 50, std::back_inserter(m_points_2));
        }
    
        void Model::flip_empty()
        {
            if(m_empty) m_empty = !m_empty;
        }
    /// @}
    /// @name Access
    /// @{
    /// @}
    /// @name Inquiry
    /// @{
        int Model::Get_Num_Vertices()
        {
            return m_mesh.size_of_vertices();
        }
        int Model::Get_Num_Facets()
        {
            return m_mesh.size_of_facets();
        }
        void Model::calculate_Bbox()
        {
            if(m_empty)
                return;
            Bbox_2 bbox = CGAL::bounding_box(m_points_2.begin(), m_points_2.end());
            m_center = Point_3((bbox.xmin() + bbox.xmax()) / 2.0,
                               (bbox.ymin() + bbox.ymax()) / 2.0,
                               0.f);
            Point_3 corner(bbox.xmin(), bbox.ymin(), 1.f);
            m_radius = std::sqrt(CGAL::squared_distance(m_center, corner));                   
        }
    /// @}
    
    /// @brief protected:
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
    
    /// @brief private:
    /// @name Private Static Member Variables
    /// @{
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


}