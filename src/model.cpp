#include "model.h"
#include <fstream>

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
            m_intersection_points.clear();
            m_Segments_2.clear();
            m_Segments_3.clear();
            m_intersection_segments.clear();
            m_intersection_interval.clear();
            m_intersection_segments.clear();
            m_generation_intersection_points.clear();
            m_generation_convexhull_points.clear();
            m_generation_convexhull_segments.clear();
            m_intersection_convexhull.clear();
            m_intersection_convexhull_points.clear();
            m_generation_polygon_points.clear();
            m_generation_polygon_segments.clear();
            m_generation_polygon_2.clear();
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

        bool Model::load_file_off(const std::string& filename)
        {
            clean();
            std::fstream file(filename);
            if(!file.is_open())
            {
                std::cout << "Error opening file" << std::endl;
                return false;
            }
            double loc_x, loc_y, loc_z;
            while(file >> loc_x >> loc_y >> loc_z)
            {
                m_points_2.push_back(Point_2(loc_x, loc_y));
            }
            return true;
        }

        bool Model::load_file_off2(const std::string& filename)
        {
            clean();
            std::fstream file(filename);
            if(!file.is_open())
            {
                std::cout << "Error opening file" << std::endl;
                return false;
            }
            double loc_x, loc_y, loc_z, loc_x2, loc_y2, loc_z2;
            while(file >> loc_x >> loc_y >> loc_z >> loc_x2 >> loc_y2 >> loc_z2)
            {
                m_points_2.push_back(Point_2(loc_x, loc_y));
                m_points_2.push_back(Point_2(loc_x2, loc_y2));
                m_Segments_2.push_back(Segment_2(Point_2(loc_x, loc_y), Point_2(loc_x2, loc_y2)));
            }
            return true;
        }

        bool Model::load_file_int(const std::string& filename)
        {
            clean();
            std::fstream file(filename);
            if(!file.is_open())
            {
                std::cout << "Error opening file" << std::endl;
                return false;
            }
            double loc_x, loc_y, loc_z, loc_x2, loc_y2, loc_z2;
            while(file >> loc_x >> loc_y >> loc_z >> loc_x2 >> loc_y2 >> loc_z2)
            {
                m_intersection_points.push_back(Point_2(loc_x, loc_y));
                m_intersection_points.push_back(Point_2(loc_x2, loc_y2));
                m_intersection_segments.push_back(Segment_2(Point_2(loc_x, loc_y), Point_2(loc_x2, loc_y2)));
            }
            return true;
        }

        bool Model::load_file_chf(const std::string& filename)
        {
            clean();

            std::ifstream file(filename);
            if(!file.is_open())
            {
                std::cout << "Error opening file" << std::endl;
                return false;
            }

            double loc_x, loc_y, loc_x2, loc_y2;
            while(file >> loc_x >> loc_y >> loc_x2 >> loc_y2 )
            {
                m_generation_convexhull_points.push_back(Point_2(loc_x, loc_y));
                m_generation_convexhull_segments.push_back(Segment_2(Point_2(loc_x, loc_y), Point_2(loc_x2, loc_y2)));
            }
            return true;
        }

        bool Model::load_file_tp(const std::string& filename)
        {
            clean();

            std::ifstream file(filename);
            if(!file.is_open())
            {
                std::cout << "Error opening file" << std::endl;
                return false;
            }

            double loc_x, loc_y;
            while(file >> loc_x >> loc_y)
                m_generation_polygon_points.push_back(Point_2(loc_x, loc_y));
            for(int i=0;i<m_generation_polygon_points.size()-1;++i)
                m_generation_polygon_segments.push_back(Segment_2(m_generation_polygon_points[i], m_generation_polygon_points[i+1]));
            m_generation_polygon_segments.push_back(Segment_2(m_generation_polygon_points.back(), m_generation_polygon_points.front()));
            return true;
        }

        /* Generation */
        void Model::generate_points()
        {
            clean();

            typedef CGAL::Creator_uniform_2<double,Point_2>  Creator;
            m_points_2.reserve(100);
            CGAL::Random_points_in_square_2<Point_2,Creator> g(2.0);
            std::copy_n( g, 10, std::back_inserter(m_points_2));
        }

        void Model::generate_segments_on_circle()
        {
            clean();

            typedef CGAL::Creator_uniform_2<double,Point_2>  Point_Creator;
            typedef CGAL::Random_points_on_segment_2<Point_2, Point_Creator> Generator_On_Segment;
            typedef CGAL::Random_points_on_circle_2<Point_2, Point_Creator> Generator_On_Circle;

            m_intersection_segments.reserve(100);
            Generator_On_Segment GS(Point_2(-1,0), Point_2(1,0));
            Generator_On_Circle GC(1);

            typedef CGAL::Creator_uniform_2<Point_2, Segment_2>  Seg_Creator;
            typedef CGAL::Join_input_iterator_2<Generator_On_Segment, Generator_On_Circle, Seg_Creator> 
                                                                 Seg_Iterator;
            Seg_Iterator segments(GS, GC, Seg_Creator());
            std::copy_n(segments, 100, std::back_inserter(m_intersection_segments));
        }

        Combination Model::generate_segments_from_points()
        {
            Combination  combination(35, 0, 49);
            return combination;
        }

        void Model::generate_convexhulls()
        {
            /* Default generate 7 convexhulls that contain 30 points */
            clean();
            
            static int count = 0;
            count = 1;
            std::ofstream outfile;
            std::ifstream file;
            if(count == 1)
            {
                file.open("../../res/TestPoints/ConvexHull.chf");
                if(!file.is_open())
                {
                    outfile.open("../../res/TestPoints/ConvexHulls.chf");
                    if(!outfile.is_open())
                        std::cout << "Error opening file" << std::endl;
                }else
                {
                    file.close();
                }
            }

            typedef CGAL::Creator_uniform_2<double, Point_2>  Creator;
            typedef CGAL::Random_points_in_square_2<Point_2, Creator> Point_Generator;
            CGAL::Random random;
            for(int i=0;i<7;++i)
            {
                double random_x = random.uniform_real(-5.0, 5.0);
                double random_y = random.uniform_real(-5.0, 5.0);
                CGAL::random_convex_set_2(30, std::back_inserter(m_generation_convexhull_points), Point_Generator(2.0));
                for(int j=i*30;j<(i+1)*30;++j)
                    m_generation_convexhull_points[j] = Point_2(m_generation_convexhull_points[j].x() + random_x,
                                                                m_generation_convexhull_points[j].y() + random_y);
            }
                
            int begin = 0;
            while(begin < 7 * 30)
            {
                for(int i=begin;i<begin+29;++i)
                    m_generation_convexhull_segments.push_back(Segment_2(m_generation_convexhull_points[i], 
                                                                m_generation_convexhull_points[(i+1)]));
                m_generation_convexhull_segments.push_back(Segment_2(m_generation_convexhull_points[begin+29], 
                                                            m_generation_convexhull_points[begin]));
                for(int i=begin;i<begin+29;++i)
                    outfile << m_generation_convexhull_segments[i] << std::endl;
                outfile << m_generation_convexhull_segments[begin+29] << std::endl;
                begin+=30;
            }
            outfile.close();
            file.close();
        }

        void Model::generate_polygon()
        {
            clean();

            static int count = 0;
            count = 1;
            std::ofstream outfile;
            std::ifstream file;
            if(count == 1)
            {
                outfile.open("../../res/TestPoints/Generation_Polygon.tp");
                if(!outfile.is_open())
                    std::cout << "Error opening file" << std::endl;
            }

            typedef std::list<Point_2> Container;
            // typedef CGAL::Creator_uniform_2<int, Point_2>             Creator;
            typedef CGAL::Random_points_in_square_2<Point_2> Point_generator;
            Container point_set;
            CGAL::Random rand;

            // int size = rand.get_int(4, 100);
            // CGAL::copy_n_unique(Point_generator(2.f), 
            //                     100, 
            //                     std::back_inserter(point_set));
            CGAL::random_polygon_2(12, 
                                   std::back_inserter(m_generation_polygon_2), 
                                   Point_generator(2.f));
            std::cout << "Simple : " << m_generation_polygon_2.is_simple() << std::endl;
            
            for(auto vertex_iter = m_generation_polygon_2.vertices_begin();vertex_iter != m_generation_polygon_2.vertices_end();++vertex_iter)
            {
                Point_2 p(vertex_iter->x(),vertex_iter->y());
                m_generation_polygon_points.push_back(p);
                outfile << m_generation_polygon_points.back() << std::endl;
            }

            for(auto edge_iter = m_generation_polygon_2.edges_begin();edge_iter != m_generation_polygon_2.edges_end();++edge_iter)
            {
                Point_2 p1(edge_iter->source().x(), edge_iter->source().y());
                Point_2 p2(edge_iter->target().x(), edge_iter->target().y());
                m_generation_polygon_segments.push_back(Segment_2(p1, p2));
            }

            outfile.close();
            file.close();
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
        /* ConvexHull */
        int Model::Get_Points2_Num_Vertices()
        {
            return m_points_2.size();
        }

        int Model::Get_Segments2_Num_Segments()
        {
            return m_Segments_2.size();
        }

        /* Intersection */
        int Model::Get_Intersection_Num_Segments()
        {
            return m_intersection_interval.size();
        }

        int Model::Get_Intersection_Num_Points()
        {
            return m_intersection_points.size();
        }

        int Model::Get_Intersection_Num_Intervals()
        {
            return m_intersection_interval.size();
        }

        

        /* Mesh */
        int Model::Get_Mesh_Num_Vertices()
        {
            return m_mesh.size_of_vertices();
        }
        
        int Model::Get_Mesh_Num_Facets()
        {
            return m_mesh.size_of_facets();
        }

        /* Camera */
        void Model::calculate_Bbox_2()
        {
            if(m_empty)
                return;
            DataPoints_2 temp_p;
            if(!m_points_2.empty())
                for(int i=0;i<m_points_2.size();++i)
                    temp_p.push_back(m_points_2[i]);
            if(!m_intersection_points.empty())
                for(int i=0;i<m_intersection_points.size();++i)
                    temp_p.push_back(m_intersection_points[i]);
            if(!m_intersection_interval.empty())
                for(int i=0;i<m_intersection_interval.size();++i)
                {
                    temp_p.push_back(m_intersection_interval[i].target());
                    temp_p.push_back(m_intersection_interval[i].source());
                }
            if(!m_intersection_segments.empty())
                for(int i=0;i<m_intersection_segments.size();++i)
                {
                    temp_p.push_back(m_intersection_segments[i].target());
                    temp_p.push_back(m_intersection_segments[i].source());
                }
            if(!m_Segments_2.empty())
                for(int i=0;i<m_Segments_2.size();++i)
                {
                    temp_p.push_back(m_Segments_2[i].target());
                    temp_p.push_back(m_Segments_2[i].source());
                }
            if(!m_generation_convexhull_points.empty())
                for(int i=0;i<m_generation_convexhull_points.size();++i)
                    temp_p.push_back(m_generation_convexhull_points[i]);
            if(!m_generation_convexhull_segments.empty())
                for(int i=0;i<m_generation_convexhull_segments.size();++i)
                {
                    temp_p.push_back(m_generation_convexhull_segments[i].target());
                    temp_p.push_back(m_generation_convexhull_segments[i].source());
                }
            if(!m_intersection_convexhull.empty())
                for(int i=0;i<m_intersection_convexhull.size();++i)
                {
                    temp_p.push_back(m_intersection_convexhull[i].target());
                    temp_p.push_back(m_intersection_convexhull[i].source());
                }
            if(!m_intersection_convexhull_points.empty())
                for(int i=0;i<m_intersection_convexhull_points.size();++i)
                    temp_p.push_back(m_intersection_convexhull_points[i]);
            if(!m_generation_polygon_points.empty())
                for(int i=0;i<m_generation_polygon_points.size();++i)
                    temp_p.push_back(m_generation_polygon_points[i]);
            if(!m_generation_polygon_segments.empty())
                for(int i=0;i<m_generation_polygon_segments.size();++i)
                {
                    temp_p.push_back(m_generation_polygon_segments[i].target());
                    temp_p.push_back(m_generation_polygon_segments[i].source());
                }

            Bbox_2 bbox = CGAL::bounding_box(temp_p.begin(), temp_p.end());
            double centerx = (bbox.xmin() + bbox.xmax()) / 2.0;
            double centery = (bbox.ymin() + bbox.ymax()) / 2.0;
            double centerz = 0;
            m_center = Point_3(centerx,centery,0.f);
            if(std::fabs(centerx) < 1e-5 || std::fabs(centery) < 1e-5)
                m_radius = 1.f;
            else
            {
                Point_3 corner(bbox.xmin(), bbox.ymin(), 1.f);
                m_radius = std::sqrt(CGAL::squared_distance(m_center, corner));  
            }             
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