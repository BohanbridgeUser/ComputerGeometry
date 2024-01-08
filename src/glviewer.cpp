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
            points_data_to_display(points_2);
            m_model->flip_empty();
            adjustCamera();
            m_gl_display->check_buffer();
            return true;
        }
        else
            return false;
    }

    void glViewer::generate_points()
    {
        m_model->clean();
        m_gl_display->setdisplaymode(0);
        m_model->generate_points();
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        points_data_to_display(points_2);
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::convexhull()
    {
        m_gl_display->setdisplaymode(1);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        DataSegments_2 convexsegments = MyCG::ConvexHull_2::ConvexHull_2_TriMethod(points_2);
        segments_data_to_display(convexsegments);
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::convexhull_ee()
    {
        m_gl_display->setdisplaymode(1);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        DataSegments_2 convexsegments = MyCG::ConvexHull_2::ConvexHull_2_EE(points_2);
        segments_data_to_display(convexsegments);
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::convexhull_jarvis_march()
    {
        m_gl_display->setdisplaymode(1);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        DataSegments_2 convexsegments = MyCG::ConvexHull_2::ConvexHull_2_Jarvis_March(points_2);
        segments_data_to_display(convexsegments);
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::convexhull_graham_scan()
    {
        m_gl_display->setdisplaymode(1);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        DataSegments_2 convexsegments = MyCG::ConvexHull_2::ConvexHull_2_Graham_Scan(points_2);
        segments_data_to_display(convexsegments);
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }

    void glViewer::convexhull_divide_and_conquer()
    {
        m_gl_display->setdisplaymode(1);
        DataPoints_2& points_2 = m_model->Get_DataPoints_2();
        DataSegments_2 convexsegments = MyCG::ConvexHull_2::ConvexHull_2_Divide_and_Conquer(points_2);
        segments_data_to_display(convexsegments);
        m_model->flip_empty();
        adjustCamera();
        m_gl_display->check_buffer();
    }   

    void glViewer::adjustCamera() {
        m_model->calculate_Bbox();
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
/// @}

/// @name Access
/// @{
/// @}

/// @name Inquiry
/// @{
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

    void glViewer::points_data_to_display(DataPoints_2 const& rpoints)
    {
        DataF& dis_points = m_gl_display->Get_pos_points();
        dis_points.resize(3*rpoints.size());
        for(int i=0;i<rpoints.size();++i)
        { 
            dis_points[3*i]   = rpoints[i].x();
            dis_points[3*i+1] = rpoints[i].y();
            dis_points[3*i+2] = 0.0;
        }
    }

    void glViewer::segments_data_to_display(DataSegments_2 const& rsegments)
    {
        DataF& loc_segments_vertex = m_gl_display->Get_pos_segments();
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
/// @}

/// @name Private Access
/// @{  
/// @}

/// @name Private Inquiry
/// @{
/// @}