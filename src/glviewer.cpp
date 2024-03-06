#include "glviewer.h"
#include <CGAL/Qt/CreateOpenGLContext.h>

/// @name Type Define
/// @{
/// @}

/// @name Life Circle
/// @{
    glViewer::glViewer(QWidget *parent) : QGLViewer(parent)
    {
        m_gl_display = nullptr;
        m_model = new Model;
    }

    glViewer::~glViewer()
    {
        delete m_model;
    }
/// @}

/// @name Operators
/// @{
/// @}

/// @name Operations
/// @{
    void glViewer::setdisplaymode(int mode)
    {
        if(m_gl_display != nullptr)
            m_gl_display->setdisplaymode(mode);
    }

    void glViewer::draw()
    {
        QGLViewer::draw();
        if(m_gl_display != nullptr)
            m_gl_display->render(this);
    }

    bool glViewer::load_file(const std::string& filename)
    {
        if(m_model->load_file(filename))
            return true;
        else
            return false;
        m_gl_display->check_buffer();
    }

    bool glViewer::load_file_off(const std::string& filename)
    {
        m_model->clean();
        m_gl_display->setdisplaymode(0);
        if(m_model->load_file_off(filename))
        {
            DataPoints_2& points_2 = m_model->Get_DataPoints_2();
            points_data_to_display(points_2, m_gl_display->Get_pos_points());
            m_model->flip_empty();
            adjustCamera();
            m_gl_display->check_buffer();
            return true;
        }
        else
            return false;
    }

    bool glViewer::load_file_off2(const std::string& filename)
    {
        m_model->clean();
        m_gl_display->setdisplaymode(0);
        m_gl_display->setdisplaymode(2);
        if(m_model->load_file_off2(filename))
        {
            DataPoints_2& points_2 = m_model->Get_DataPoints_2();
            DataSegments_2& segments_2 = m_model->Get_DataSegments_2();
            points_data_to_display(points_2, m_gl_display->Get_pos_points());
            segments_data_to_display(segments_2, m_gl_display->Get_pos_segments());
            m_model->flip_empty();
            adjustCamera();
            m_gl_display->check_buffer();
            return true;
        }
        else
            return false;
    }

    bool glViewer::load_file_int(const std::string& filename)
    {
        m_model->clean();
        m_gl_display->setdisplaymode(2);
        m_gl_display->setdisplaymode(3);
        if(m_model->load_file_int(filename))
        {
            DataPoints_2& points_2 = m_model->Get_Intersection_Points();
            DataSegments_2& segments_2 = m_model->Get_Intersection_Segments();
            points_data_to_display(points_2, m_gl_display->Get_pos_intersection_points());
            segments_data_to_display(segments_2, m_gl_display->Get_pos_intersection_lines());
            m_model->flip_empty();
            adjustCamera();
            m_gl_display->check_buffer();
            return true;
        }
        else
            return false;
    }

    bool glViewer::load_file_chf(const std::string& filename)
    {
        clean();
        for(int i=7;i<9;i++)
            m_gl_display->setdisplaymode(i);
        
        if(m_model->load_file_chf(filename))
        {
            std::cout << "open chf file !\n";
            DataPoints_2& points_2 = m_model->Get_Generation_ConvexHull_Points();
            DataSegments_2& segments_2 = m_model->Get_Generation_ConvexHull_Segments();
            points_data_to_display(points_2, m_gl_display->Get_pos_generation_convexhull_points());
            segments_data_to_display(segments_2, m_gl_display->Get_pos_generation_convexhull_lines());
            m_model->flip_empty();
            adjustCamera();
            m_gl_display->check_buffer();
            return true;
        }
        else
            return false;
    }

    bool glViewer::load_file_tp(const std::string& filename)
    {
        clean();
        for(int i=11;i<13;i++)
            m_gl_display->setdisplaymode(i);
        
        if(m_model->load_file_tp(filename))
        {
            std::cout << "open cp file !\n";
            DataPoints_2& points_2 = m_model->Get_Generation_Polygon_Points();
            DataSegments_2& segments_2 = m_model->Get_Generation_Polygon_Segments();
            points_data_to_display(points_2, m_gl_display->Get_pos_generation_polygon_points());
            segments_data_to_display(segments_2, m_gl_display->Get_pos_generation_polygon_lines());
            m_model->flip_empty();
            adjustCamera();
            m_gl_display->check_buffer();
            return true;
        }
        else
            return false;
    }

    void glViewer::adjustCamera() 
    {
        m_model->calculate_Bbox_2();
        Point_3& center = m_model->Get_Center();
        double radius = m_model->Get_Radius();
        setSceneCenter(CGAL::qglviewer::Vec(center.x(), center.y(), center.z()));
        setSceneRadius((float)(1.2 * radius));
        showEntireScene();
    }

    void glViewer::clean()
    {
        m_model->clean();
        m_gl_display->clean();
    }

    // Gernerator
    void glViewer::generate_points()
    {
        m_model->clean();
        m_gl_display->setdisplaymode(0);
        m_model->generate_points();
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        points_data_to_display(points_2, m_gl_display->Get_pos_points());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::generate_segments_from_points()
    {
        m_gl_display->setdisplaymode(2);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        std::cout << "Number of points: " << points_2.size() << std::endl;
        Combination combination = m_model->generate_segments_from_points();
        DataSegments_2& segments_2 = m_model->Get_DataSegments_2();
        for(int i=0;i<combination.number_of_elements()-1;i+=2)
            segments_2.push_back(Segment_2(points_2[combination[i]], points_2[combination[i+1]]));
        std::cout << "Number of segments: " << segments_2.size() << std::endl;
        segments_data_to_display(segments_2, m_gl_display->Get_pos_segments());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::generate_segments_on_circle()
    {
        m_model->clean();
        m_gl_display->setdisplaymode(5);
        m_model->generate_segments_on_circle();
        DataSegments_2& segments_2 = m_model->Get_Intersection_Segments();
        segments_data_to_display(segments_2, m_gl_display->Get_pos_intersection_lines());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::generate_convexhulls()
    {
        clean();
        for(int i=7;i<9;i++)
            m_gl_display->setdisplaymode(i);
        
        m_model->generate_convexhulls();
        DataPoints_2& points_2 = m_model->Get_Generation_ConvexHull_Points();
        DataSegments_2& segments_2 = m_model->Get_Generation_ConvexHull_Segments();
        points_data_to_display(points_2, m_gl_display->Get_pos_generation_convexhull_points());
        segments_data_to_display(segments_2, m_gl_display->Get_pos_generation_convexhull_lines());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::generate_polygon()
    {
        clean();
        for(int i=11;i<13;i++)
            m_gl_display->setdisplaymode(i);
        m_model->generate_polygon();
        DataPoints_2& points_2 = m_model->Get_Generation_Polygon_Points();
        DataSegments_2& segments_2 = m_model->Get_Generation_Polygon_Segments();
        points_data_to_display(points_2, m_gl_display->Get_pos_generation_polygon_points());
        segments_data_to_display(segments_2, m_gl_display->Get_pos_generation_polygon_lines());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    // Convexhull
    void glViewer::convexhull()
    {
        m_gl_display->setdisplaymode(1);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        DataSegments_2 convexsegments = MyCG::ConvexHull_2::ConvexHull_2_TriMethod(points_2);
        segments_data_to_display(convexsegments,m_gl_display->Get_pos_segments());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::convexhull_ee()
    {
        m_gl_display->setdisplaymode(1);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        DataSegments_2 convexsegments = MyCG::ConvexHull_2::ConvexHull_2_EE(points_2);
        segments_data_to_display(convexsegments,m_gl_display->Get_pos_segments());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::convexhull_jarvis_march()
    {
        m_gl_display->setdisplaymode(1);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        DataSegments_2 convexsegments = MyCG::ConvexHull_2::ConvexHull_2_Jarvis_March(points_2);
        segments_data_to_display(convexsegments, m_gl_display->Get_pos_segments());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::convexhull_graham_scan()
    {
        m_gl_display->setdisplaymode(1);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        DataSegments_2 convexsegments = MyCG::ConvexHull_2::ConvexHull_2_Graham_Scan(points_2);
        segments_data_to_display(convexsegments, m_gl_display->Get_pos_segments());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::convexhull_divide_and_conquer()
    {
        m_gl_display->setdisplaymode(1);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        DataSegments_2 convexsegments = MyCG::ConvexHull_2::ConvexHull_2_Divide_and_Conquer(points_2);
        segments_data_to_display(convexsegments,  m_gl_display->Get_pos_segments());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }   

    // Intersection
    void glViewer::interval()
    {
        for(int i=4;i<6;++i)
            m_gl_display->setdisplaymode(i);
        DataSegments_2& segments_2 = m_model->Get_Intersection_Segments();
        DataSegments_2 interval = MyCG::Intersection_2::Interval(segments_2);
        segments_data_to_display(interval, m_gl_display->Get_pos_intersection_interval());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::segments_intersection_2()
    {
        m_gl_display->setdisplaymode(3);
        DataSegments_2& segments_2 = m_model->Get_DataSegments_2();
        DataPoints_2 intersection_points = MyCG::Intersection_2::Intersection(segments_2);
        points_data_to_display(intersection_points, m_gl_display->Get_pos_generation_intersection_points());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::convexhulls_intersection(std::vector<std::vector<int>>& intersections)
    {
        for(int i=7;i<11;i++)
            m_gl_display->setdisplaymode(i);
        DataPoints_2& generate_points_2 = m_model->Get_Generation_ConvexHull_Points();
        DataSegments_2& generate_segments_2 = m_model->Get_Generation_ConvexHull_Segments();
        DataPoints_2& intersection_points_2 = m_model->Get_Intersection_ConvexHull_Points();
        DataSegments_2& intersection_segments_2 = m_model->Get_Intersection_ConvexHull_Segments();
        intersections = MyCG::Intersection_2::ConvexHull_Intersection(intersection_points_2, intersection_segments_2,
                                                                      generate_points_2, generate_segments_2);
    }

    void glViewer::convexhulls_edge_chasing()
    {
        for(int i=7;i<11;i++)
            m_gl_display->setdisplaymode(i);
        DataPoints_2& generate_points_2 = m_model->Get_Generation_ConvexHull_Points();
        DataSegments_2& generate_segments_2 = m_model->Get_Generation_ConvexHull_Segments();
        DataPoints_2& intersection_points_2 = m_model->Get_Intersection_ConvexHull_Points();
        DataSegments_2& intersection_segments_2 = m_model->Get_Intersection_ConvexHull_Segments();
        MyCG::Intersection_2::ConvexHull_Edge_Chasing(intersection_points_2, intersection_segments_2,
                                                      generate_points_2, generate_segments_2);
        points_data_to_display(intersection_points_2, m_gl_display->Get_pos_intersection_convexhull_points());
        segments_data_to_display(intersection_segments_2, m_gl_display->Get_pos_intersection_convexhull());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    // Triangulation
    void glViewer::monotone_triangulation()
    {
        for(int i=11;i<15;i++)
            m_gl_display->setdisplaymode(i);
        DataPoints_2& points_2 = m_model->Get_Generation_Polygon_Points();
        DataPoints_2& triangulations = m_model->Get_Triangulations();
        std::vector<DataPoints_2> triangulations_points;
        MyCG::Triangulation::Triangulation_Monotone(points_2, triangulations_points);
        DataSegments_2 triangulation_lines;
        DataPoints_2 triangulation_polygons;
        for(int i=0;i<triangulations_points.size();++i)
        {
            for(int j=0;j<triangulations_points[i].size()-1;++j)
            {
                Segment_2 segment(triangulations_points[i][j], triangulations_points[i][j+1]);
                triangulation_lines.push_back(segment);
                triangulation_polygons.push_back(triangulations_points[i][j]);
            }
            triangulation_lines.push_back(Segment_2(triangulations_points[i][triangulations_points[i].size()-1], triangulations_points[i][0]));
            triangulation_polygons.push_back(triangulations_points[i][triangulations_points[i].size()-1]);
        } 

        triangulations_data_to_display(triangulation_polygons, m_gl_display->Get_pos_triangulation_polygon());
        segments_data_to_display(triangulation_lines, m_gl_display->Get_pos_triangulation_monotone_lines());
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    // Voronoi
    void glViewer::voronoi_naive()
    {
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        HDS hds;
        MyCG::Voronoi::Voronoi_Naive(points_2, hds);
        DataPoints_2 voronoi_sites(points_2), voronoi_vertices;
        DataSegments_2 voronoi_edges;
        auto vertex_handle = hds.vertices_begin();
        do
        {
            voronoi_vertices.push_back(Point_2(vertex_handle->point().x(), vertex_handle->point().y()));
            vertex_handle++;
        } while (vertex_handle != hds.vertices_begin());
        
        auto edge_handle = hds.halfedges_begin();
        do
        {
            voronoi_edges.push_back(Segment_2(edge_handle->vertex()->point(), edge_handle->opposite()->vertex()->point()));
            edge_handle++;
        } while (edge_handle != hds.halfedges_begin());
        
        points_data_to_display(voronoi_sites, m_gl_display->Get_pos_voronoi_sites());
        points_data_to_display(voronoi_vertices, m_gl_display->Get_pos_voronoi_vertices());
        segments_data_to_display(voronoi_edges, m_gl_display->Get_pos_voronoi_edges());

        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::voronoi_incremental()
    {
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        HDS hds;
        MyCG::Voronoi::Voronoi_Incremental(points_2, hds);
        DataPoints_2 voronoi_sites(points_2), voronoi_vertices;
        DataSegments_2 voronoi_edges;
        auto vertex_handle = hds.vertices_begin();
        do
        {
            voronoi_vertices.push_back(Point_2(vertex_handle->point().x(), vertex_handle->point().y()));
            vertex_handle++;
        } while (vertex_handle != hds.vertices_begin());
        
        auto edge_handle = hds.halfedges_begin();
        do
        {
            voronoi_edges.push_back(Segment_2(edge_handle->vertex()->point(), edge_handle->opposite()->vertex()->point()));
            edge_handle++;
        } while (edge_handle != hds.halfedges_begin());
        
        points_data_to_display(voronoi_sites, m_gl_display->Get_pos_voronoi_sites());
        points_data_to_display(voronoi_vertices, m_gl_display->Get_pos_voronoi_vertices());
        segments_data_to_display(voronoi_edges, m_gl_display->Get_pos_voronoi_edges());
        
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::voronoi_divide_and_conquer()
    {
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        HDS hds;
        MyCG::Voronoi::Voronoi_Divide_and_Conquer(points_2, hds);
        DataPoints_2 voronoi_sites(points_2), voronoi_vertices;
        DataSegments_2 voronoi_edges;
        auto vertex_handle = hds.vertices_begin();
        do
        {
            voronoi_vertices.push_back(Point_2(vertex_handle->point().x(), vertex_handle->point().y()));
            vertex_handle++;
        } while (vertex_handle != hds.vertices_begin());
        
        auto edge_handle = hds.halfedges_begin();
        do
        {
            voronoi_edges.push_back(Segment_2(edge_handle->vertex()->point(), edge_handle->opposite()->vertex()->point()));
            edge_handle++;
        } while (edge_handle != hds.halfedges_begin());
        
        points_data_to_display(voronoi_sites, m_gl_display->Get_pos_voronoi_sites());
        points_data_to_display(voronoi_vertices, m_gl_display->Get_pos_voronoi_vertices());
        segments_data_to_display(voronoi_edges, m_gl_display->Get_pos_voronoi_edges());
        
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::voronoi_sweep_line()
    {
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        HDS hds;
        MyCG::Voronoi::Voronoi_Sweep_Line(points_2, hds);
        DataPoints_2 voronoi_sites(points_2), voronoi_vertices;
        DataSegments_2 voronoi_edges;
        auto vertex_handle = hds.vertices_begin();
        do
        {
            voronoi_vertices.push_back(Point_2(vertex_handle->point().x(), vertex_handle->point().y()));
            vertex_handle++;
        } while (vertex_handle != hds.vertices_begin());
        
        auto edge_handle = hds.halfedges_begin();
        do
        {
            voronoi_edges.push_back(Segment_2(edge_handle->vertex()->point(), edge_handle->opposite()->vertex()->point()));
            edge_handle++;
        } while (edge_handle != hds.halfedges_begin());
        
        points_data_to_display(voronoi_sites, m_gl_display->Get_pos_voronoi_sites());
        points_data_to_display(voronoi_vertices, m_gl_display->Get_pos_voronoi_vertices());
        segments_data_to_display(voronoi_edges, m_gl_display->Get_pos_voronoi_edges());
        
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

/// @}

/// @name Access
/// @{
/// @}

/// @name Inquiry
/// @{
    int glViewer::Get_Intersection_Num_Segments() { return m_model->Get_Intersection_Num_Segments(); }
    int glViewer::Get_Intersection_Num_Points() { return m_model->Get_Intersection_Num_Points(); }
    int glViewer::Get_Intersection_Num_Intervals() { return m_model->Get_Intersection_Num_Intervals(); }
    int glViewer::Get_Points2_Num_Vertices() { return m_model->Get_Points2_Num_Vertices(); }
    int glViewer::Get_Mesh_Num_Vertices() { return m_model->Get_Mesh_Num_Vertices();}
    int glViewer::Get_Mesh_Num_Facets() { return m_model->Get_Mesh_Num_Facets(); }
/// @}

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
    void glViewer::initializeGL()
    {
        QGLViewer::initializeGL();
        setBackgroundColor(QColor(255,255,255));
        setSceneRadius(2.0);
    }

    void glViewer::points_data_to_display(DataPoints_2 const& rpoints, DataF& dis_points)
    {
        dis_points.resize(3*rpoints.size());
        for(int i=0;i<rpoints.size();++i)
        { 
            dis_points[3*i]   = rpoints[i].x();
            dis_points[3*i+1] = rpoints[i].y();
            dis_points[3*i+2] = 0.0;
        }
    }

    void glViewer::segments_data_to_display(DataSegments_2 const& rsegments, DataF& loc_segments_vertex)
    {
        loc_segments_vertex.resize(6*rsegments.size());
        for(int i=0;i<rsegments.size();++i)
        {
            loc_segments_vertex[6*i  ] = rsegments[i].point(1).x();
            loc_segments_vertex[6*i+1] = rsegments[i].point(1).y();
            loc_segments_vertex[6*i+2] = 0.0f;
            loc_segments_vertex[6*i+3] = rsegments[i].point(2).x();
            loc_segments_vertex[6*i+4] = rsegments[i].point(2).y();
            loc_segments_vertex[6*i+5] = 0.0f;
        }
    }

    void glViewer::triangulations_data_to_display(DataPoints_2 const& rpoints, DataF& dis_points)
    {
        dis_points.resize(3*rpoints.size());
        for(int i=0;i<rpoints.size();++i)
        { 
            dis_points[3*i]   = rpoints[i].x();
            dis_points[3*i+1] = rpoints[i].y();
            dis_points[3*i+2] = 0.0;
        }
    }
/// @}

/// @name Private Access
/// @{  
/// @}

/// @name Private Inquiry
/// @{
/// @}