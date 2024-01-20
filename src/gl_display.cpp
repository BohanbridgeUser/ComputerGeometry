#include "gl_display.h"
#include <CGAL/Qt/CreateOpenGLContext.h>
#include <QMatrix4x4>

/// @brief public:
/// @name Type Define
/// @{
/// @}
/// @name Life Circle
/// @{
    Gl_Display::Gl_Display()
    {
        gl_init = false;
        gl_buffer_init = false;
        for(int i=0;i<SIZE_OF_MODE;++i)
            m_mode[i] = false;
    }

    Gl_Display::~Gl_Display()
    {
        for(int i=0;i<SIZE_OF_VAO;++i)
            vao[i].destroy();
        for(int i=0;i<SIZE_OF_VBO;++i)
            vbo[i].destroy();
    }
/// @}
/// @name Operators
/// @{
/// @}
/// @name Operations
/// @{
    void Gl_Display::setdisplaymode(int mode)
    {
        m_mode[mode] = true;
    }

    void Gl_Display::clean()
    {
        pos_points.clear();
        pos_segments.clear();
        pos_generation_intersection_points.clear();
        pos_intersection_points.clear();
        pos_intersection_lines.clear();
        pos_intersection_interval.clear();
        pos_generation_convexhull_points.clear();
        pos_generation_convexhull_lines.clear();
        pos_intersection_convexhull.clear();
        gl_buffer_init = false;
        for(int i=0;i<SIZE_OF_MODE;++i)
            m_mode[i] = false;
    }
/// @}
/// @name OpenGL
/// @{
    void Gl_Display::render(CGAL::QGLViewer *viewer)
    {
        if(!gl_init)
            initializeGL();
        if(!gl_buffer_init)
            initialize_buffers();
        
        attrib_buffers(viewer);
        gl->glEnable(GL_DEPTH_TEST);

        for(int i=0;i<SIZE_OF_MODE;++i)
        {
            if(m_mode[i] == true)
                render_functions[i]();
        }
    }

    void Gl_Display::render_points()
    {
        gl->glEnable(GL_POINT_SMOOTH);
        vao[POINTS_VAO].bind();
        rendering_programs[VERTEX_SHADER].bind();
        rendering_programs[VERTEX_SHADER].setUniformValue(point_size_location, options.Point_Size);
        rendering_programs[VERTEX_SHADER].setUniformValue(points_colorLocation, options.points_color);
        gl->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(pos_points.size() / 3));
        rendering_programs[VERTEX_SHADER].release();
        vao[POINTS_VAO].release();
    }

    void Gl_Display::render_generation_intersections()
    {
        gl->glEnable(GL_POINT_SMOOTH);
        vao[GENERATION_INTERSECTION_LOCATION].bind();
        rendering_programs[VERTEX_SHADER].bind();
        rendering_programs[VERTEX_SHADER].setUniformValue(generation_intersection_points_size_location, options.Generation_Points_Size);
        rendering_programs[VERTEX_SHADER].setUniformValue(generation_intersection_points_color_location, options.generation_points_color);
        gl->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(pos_generation_intersection_points.size() / 3));
        rendering_programs[VERTEX_SHADER].release();
        vao[GENERATION_INTERSECTION_LOCATION].release();
    }

    void Gl_Display::render_segments()
    {
        gl->glEnable(GL_LINE_SMOOTH);
        vao[SEGMENTS_VAO].bind();
        rendering_programs[SEGMENT_SHADER].bind();
        rendering_programs[SEGMENT_SHADER].setUniformValue(segment_colorlocation, options.segments_color);
        gl->glLineWidth(options.Segment_Size);
        gl->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(pos_segments.size() / 3));
        rendering_programs[SEGMENT_SHADER].release();
        vao[SEGMENTS_VAO].release();
    }

    void Gl_Display::render_generation_segments()
    {
        gl->glEnable(GL_LINE_SMOOTH);
        vao[SEGMENTS_VAO].bind();
        rendering_programs[SEGMENT_SHADER].bind();
        rendering_programs[SEGMENT_SHADER].setUniformValue(segment_colorlocation, options.generation_segments_color);
        gl->glLineWidth(options.Generation_Segments_Size);
        gl->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(pos_segments.size() / 3));
        rendering_programs[SEGMENT_SHADER].release();
        vao[SEGMENTS_VAO].release();
    }

    void Gl_Display::render_intersection_points()
    {
        gl->glEnable(GL_POINT_SMOOTH);
        vao[INTERSECTION_POINTS_VAO].bind();
        rendering_programs[VERTEX_SHADER].bind();
        rendering_programs[VERTEX_SHADER].setUniformValue(intersection_points_size_location, options.Intersection_Point_Size);
        rendering_programs[VERTEX_SHADER].setUniformValue(intersection_points_color_location, options.intersection_points_color);
        gl->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(pos_intersection_points.size() / 3));
        rendering_programs[VERTEX_SHADER].release();
        vao[INTERSECTION_POINTS_VAO].release();
    }

    void Gl_Display::render_intersection_lines()
    {
        gl->glEnable(GL_LINE_SMOOTH);
        vao[INTERSECTION_LINES_VAO].bind();
        rendering_programs[SEGMENT_SHADER].bind();
        rendering_programs[SEGMENT_SHADER].setUniformValue(intersection_line_color_location, options.intersection_line_color);
        gl->glLineWidth(options.Intersection_Line_Size);
        gl->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(pos_intersection_lines.size()/3));
        rendering_programs[SEGMENT_SHADER].release();
        vao[INTERSECTION_LINES_VAO].release();
    }

    void Gl_Display::render_intersection_interval()
    {
        gl->glEnable(GL_LINE_SMOOTH);
        vao[INTERSECTION_INTERVAL_VAO].bind();
        rendering_programs[SEGMENT_SHADER].bind();
        rendering_programs[SEGMENT_SHADER].setUniformValue(intersection_interval_color, options.intersection_interval_color);
        gl->glLineWidth(options.Intersection_Interval_Size);
        gl->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(pos_intersection_interval.size() / 3));
        rendering_programs[SEGMENT_SHADER].release();
        vao[INTERSECTION_INTERVAL_VAO].release();
    }

    void Gl_Display::render_generation_convexhull_points()
    {
        gl->glEnable(GL_POINT_SMOOTH);
        vao[GENERATION_CONVEXHULL_POINTS_VAO].bind();
        rendering_programs[VERTEX_SHADER].bind();
        rendering_programs[VERTEX_SHADER].setUniformValue(generation_convexhull_points_size_location, options.Generation_ConvexHull_Point_Size);
        rendering_programs[VERTEX_SHADER].setUniformValue(generation_convexhull_points_color_location, options.generation_convexhull_point_color);
        gl->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(pos_generation_convexhull_points.size() / 3));
        rendering_programs[VERTEX_SHADER].release();
    }

    void Gl_Display::render_generation_convexhull_lines()
    {
        gl->glEnable(GL_LINE_SMOOTH);
        vao[GENERATION_CONVEXHULL_LINES_VAO].bind();
        rendering_programs[SEGMENT_SHADER].bind();
        rendering_programs[SEGMENT_SHADER].setUniformValue(generation_convexhull_lines_color_location, options.generation_convexhull_line_color);
        gl->glLineWidth(options.Generation_ConvexHull_Line_Size);
        gl->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(pos_generation_convexhull_lines.size() / 3));
        rendering_programs[SEGMENT_SHADER].release();
    }

    void Gl_Display::render_intersection_convexhull()
    {
        gl->glEnable(GL_LINE_SMOOTH);
        vao[INTERSECTION_CONVEXHULL_VAO].bind();
        rendering_programs[SEGMENT_SHADER].bind();
        rendering_programs[SEGMENT_SHADER].setUniformValue(intersection_convexhull_color_location, options.intersection_convexhull_color);
        gl->glLineWidth(options.Intersection_ConvexHull_Size);
        gl->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(pos_intersection_convexhull.size() / 3));
        rendering_programs[SEGMENT_SHADER].release();
    }

    void Gl_Display::render_intersection_convexhull_points()
    {
        gl->glEnable(GL_POINT_SMOOTH);
        vao[INTERSECTION_CONVEXHULL_POINTS_VAO].bind();
        rendering_programs[VERTEX_SHADER].bind();
        rendering_programs[VERTEX_SHADER].setUniformValue(intersection_convexhull_points_size_location, options.Intersection_Convexhull_Point_Size);
        rendering_programs[VERTEX_SHADER].setUniformValue(intersection_convexhull_points_color_location, options.intersection_convexhull_point_color);
        gl->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(pos_intersection_convexhull_points.size() / 3));
        rendering_programs[VERTEX_SHADER].release();
    }

    void Gl_Display::initializeGL()
    {
        gl = new QOpenGLFunctions();
        gl->initializeOpenGLFunctions();

        compile_shaders();
        gl_init = true;
    }

    void Gl_Display::compile_shaders()
    {
        for(int i=0;i<SIZE_OF_VAO;++i)
            vao[i].create();

        for(int i=0;i<SIZE_OF_VBO;++i)
            vbo[i].create();

        /* points shader */
        QOpenGLShader *vertex_vs = new QOpenGLShader(QOpenGLShader::Vertex);
        if(!vertex_vs->compileSourceFile("../../shader/points.vs"))
            std::cerr << "Error while compiling points vertex shader" << std::endl;
        QOpenGLShader *vertex_fs = new QOpenGLShader(QOpenGLShader::Fragment);
        if(!vertex_fs->compileSourceFile("../../shader/points.fs"))
            std::cerr << "Error while compiling points fragment shader" << std::endl;
            
        if(!rendering_programs[VERTEX_SHADER].addShader(vertex_vs))
            std::cerr << "Error while adding points vertex shader" << std::endl;
        if(!rendering_programs[VERTEX_SHADER].addShader(vertex_fs))
            std::cerr << "Error while adding points fragment shader" << std::endl;
        if(!rendering_programs[VERTEX_SHADER].link())
            std::cerr << "Error while linking points program" << std::endl;
        rendering_programs[VERTEX_SHADER].bind();

        /* segments shader */
        QOpenGLShader *segment_vs = new QOpenGLShader(QOpenGLShader::Vertex);
        if(!segment_vs->compileSourceFile("../../shader/segments.vs"))
            std::cerr << "Error while compiling segments vertex shader" << std::endl;
        QOpenGLShader *segment_fs = new QOpenGLShader(QOpenGLShader::Fragment);
        if(!segment_fs->compileSourceFile("../../shader/segments.fs"))
            std::cerr << "Error while compiling segments fragment shader" << std::endl;

        if(!rendering_programs[SEGMENT_SHADER].addShader(segment_vs))
            std::cerr << "Error while adding segments vertex shader" << std::endl;
        if(!rendering_programs[SEGMENT_SHADER].addShader(segment_fs))
            std::cerr << "Error while adding segments fragment shader" << std::endl;
        if(!rendering_programs[SEGMENT_SHADER].link()) 
            std::cerr << "Error while linking segments program" << std::endl;
        rendering_programs[SEGMENT_SHADER].bind();

        render_functions.emplace_back(std::bind(&Gl_Display::render_points,this));
        render_functions.emplace_back(std::bind(&Gl_Display::render_segments,this));
        render_functions.emplace_back(std::bind(&Gl_Display::render_generation_segments,this));
        render_functions.emplace_back(std::bind(&Gl_Display::render_generation_intersections,this));
        render_functions.emplace_back(std::bind(&Gl_Display::render_intersection_points,this));
        render_functions.emplace_back(std::bind(&Gl_Display::render_intersection_lines,this));
        render_functions.emplace_back(std::bind(&Gl_Display::render_intersection_interval,this));
        render_functions.emplace_back(std::bind(&Gl_Display::render_generation_convexhull_points,this));
        render_functions.emplace_back(std::bind(&Gl_Display::render_generation_convexhull_lines,this));
        render_functions.emplace_back(std::bind(&Gl_Display::render_intersection_convexhull,this));
        render_functions.emplace_back(std::bind(&Gl_Display::render_intersection_convexhull_points,this));
    }

    void Gl_Display::check_buffer()
    {
        gl_buffer_init = false;
    }

    void Gl_Display::init_buffer(QOpenGLShaderProgram& program,
                                 QOpenGLVertexArrayObject& vao,
                                 QOpenGLBuffer& vbo,
                                 const DataF& data,
                                 const char* attribute_name)
    {
        vao.bind();
        vbo.bind();
        vbo.allocate(data.data(), data.size()*sizeof(float));

        program.bind();
        int Location = program.attributeLocation(attribute_name);
        program.enableAttributeArray(Location);
        program.setAttributeBuffer(Location, GL_FLOAT, 0, 3);
        
        program.release();
        vbo.release();
        vao.release();
    }

    void Gl_Display::initialize_buffers()
    {
        if(!gl_buffer_init)
        {
            init_buffer(rendering_programs[VERTEX_SHADER],vao[POINTS_VAO],
                              vbo[POINTS_LOCATION], pos_points, "pos_vertex");
            init_buffer(rendering_programs[SEGMENT_SHADER],vao[SEGMENTS_VAO],
                              vbo[SEGMENTS_LOCATION], pos_segments, "vertex");
            init_buffer(rendering_programs[VERTEX_SHADER],vao[INTERSECTION_POINTS_VAO],
                              vbo[INTERSECTION_POINTS_LOCATION], pos_intersection_points, "pos_vertex");
            init_buffer(rendering_programs[SEGMENT_SHADER],vao[INTERSECTION_LINES_VAO],
                              vbo[INTERSECTION_LINES_LOCATION], pos_intersection_lines, "vertex");
            init_buffer(rendering_programs[SEGMENT_SHADER],vao[INTERSECTION_INTERVAL_VAO],
                              vbo[INTERSECTION_INTERVAL_LOCATION], pos_intersection_interval, "vertex");
            init_buffer(rendering_programs[VERTEX_SHADER],vao[GENERATION_INTERSECTION_VAO],
                              vbo[GENERATION_INTERSECTION_LOCATION], pos_generation_intersection_points, "pos_vertex");
            init_buffer(rendering_programs[VERTEX_SHADER],vao[GENERATION_CONVEXHULL_POINTS_VAO],
                              vbo[GENERATION_CONVEXHULL_POINTS_LOCATION], pos_generation_convexhull_points, "pos_vertex");
            init_buffer(rendering_programs[SEGMENT_SHADER],vao[GENERATION_CONVEXHULL_LINES_VAO],
                              vbo[GENERATION_CONVEXHULL_LINES_LOCATION], pos_generation_convexhull_lines, "vertex");
            init_buffer(rendering_programs[SEGMENT_SHADER],vao[INTERSECTION_CONVEXHULL_VAO],
                              vbo[INTERSECTION_CONVEXHULL_LOCATION], pos_intersection_convexhull, "vertex");
        }
        gl_buffer_init = true;
    }

    void Gl_Display::attrib_buffers(CGAL::QGLViewer *viewer)
    {
        QMatrix4x4 mvp_matrix;
        QMatrix4x4 mvMatrix;
        QMatrix4x4 vpMatrix;
        double mat[16];
        viewer->camera()->getModelViewProjectionMatrix(mat);
        for (int i = 0; i < 16; i++)
            mvp_matrix.data()[i] = (float)mat[i];
        for (int i = 0; i < 16; i++)
        mvMatrix.data()[i] = (float)mat[i];
        int mat1[4];
        viewer->camera()->getViewport(mat1);
        vpMatrix.fill(0.);
        vpMatrix.data()[0] = (float)mat1[2] * 0.5;
        vpMatrix.data()[3] = (float)mat1[2] * 0.5 + mat1[0];
        vpMatrix.data()[5] = (float)mat1[3] * 0.5;
        vpMatrix.data()[7] = (float)mat1[3] * 0.5 + mat1[1];
        vpMatrix.data()[10] = 0.5;
        vpMatrix.data()[11] = 0.5;
        vpMatrix.data()[15] = 1.;

        rendering_programs[VERTEX_SHADER].bind();
        points_colorLocation = rendering_programs[VERTEX_SHADER].uniformLocation("color");
        point_size_location = rendering_programs[VERTEX_SHADER].uniformLocation("Point_Size");
        mvpLocation = rendering_programs[VERTEX_SHADER].uniformLocation("mvp_matrix");
        rendering_programs[VERTEX_SHADER].setUniformValue(mvpLocation, mvp_matrix);
        rendering_programs[VERTEX_SHADER].release();

        rendering_programs[SEGMENT_SHADER].bind();
        segment_mvpLocation = rendering_programs[SEGMENT_SHADER].uniformLocation("mvp_matrix");
        segment_colorlocation = rendering_programs[SEGMENT_SHADER].uniformLocation("color");
        rendering_programs[SEGMENT_SHADER].setUniformValue(segment_mvpLocation, mvp_matrix);
        rendering_programs[SEGMENT_SHADER].release();

        rendering_programs[VERTEX_SHADER].bind();
        intersection_points_mvplocation = rendering_programs[VERTEX_SHADER].uniformLocation("mvp_matrix");
        intersection_points_color_location = rendering_programs[VERTEX_SHADER].uniformLocation("color");
        intersection_points_size_location = rendering_programs[VERTEX_SHADER].uniformLocation("Point_Size");
        rendering_programs[VERTEX_SHADER].setUniformValue(intersection_points_mvplocation, mvp_matrix);
        rendering_programs[VERTEX_SHADER].release();

        rendering_programs[SEGMENT_SHADER].bind();
        intersection_line_mvplocation = rendering_programs[SEGMENT_SHADER].uniformLocation("mvp_matrix");
        intersection_line_color_location = rendering_programs[SEGMENT_SHADER].uniformLocation("color");
        rendering_programs[SEGMENT_SHADER].setUniformValue(intersection_line_mvplocation, mvp_matrix);
        rendering_programs[SEGMENT_SHADER].release();

        rendering_programs[SEGMENT_SHADER].bind();
        intersection_interval_mvplocation = rendering_programs[SEGMENT_SHADER].uniformLocation("mvp_matrix");
        intersection_interval_color = rendering_programs[SEGMENT_SHADER].uniformLocation("color");
        rendering_programs[SEGMENT_SHADER].setUniformValue(intersection_interval_mvplocation, mvp_matrix);
        rendering_programs[SEGMENT_SHADER].release();

        rendering_programs[VERTEX_SHADER].bind();
        generation_intersection_points_mvplocation = rendering_programs[VERTEX_SHADER].uniformLocation("mvp_matrix");
        generation_intersection_points_color_location = rendering_programs[VERTEX_SHADER].uniformLocation("color");
        generation_intersection_points_size_location = rendering_programs[VERTEX_SHADER].uniformLocation("Point_Size");
        rendering_programs[VERTEX_SHADER].setUniformValue(generation_intersection_points_mvplocation, mvp_matrix);
        rendering_programs[VERTEX_SHADER].release();

        rendering_programs[VERTEX_SHADER].bind();
        generation_convexhull_points_mvplocation = rendering_programs[VERTEX_SHADER].uniformLocation("mvp_matrix");
        generation_convexhull_points_color_location = rendering_programs[VERTEX_SHADER].uniformLocation("color");
        generation_convexhull_points_size_location = rendering_programs[VERTEX_SHADER].uniformLocation("Point_Size");
        rendering_programs[VERTEX_SHADER].setUniformValue(generation_convexhull_points_mvplocation, mvp_matrix);
        rendering_programs[VERTEX_SHADER].release();

        rendering_programs[SEGMENT_SHADER].bind();
        generation_convexhull_lines_mvplocation = rendering_programs[SEGMENT_SHADER].uniformLocation("mvp_matrix");
        generation_convexhull_lines_color_location = rendering_programs[SEGMENT_SHADER].uniformLocation("color");
        rendering_programs[SEGMENT_SHADER].setUniformValue(generation_convexhull_lines_mvplocation, mvp_matrix);
        rendering_programs[SEGMENT_SHADER].release();

        rendering_programs[SEGMENT_SHADER].bind();
        intersection_convexhull_mvplocation = rendering_programs[SEGMENT_SHADER].uniformLocation("mvp_matrix");
        intersection_convexhull_color_location = rendering_programs[SEGMENT_SHADER].uniformLocation("color");
        rendering_programs[SEGMENT_SHADER].setUniformValue(intersection_convexhull_mvplocation, mvp_matrix);
        rendering_programs[SEGMENT_SHADER].release();

        rendering_programs[VERTEX_SHADER].bind();
        intersection_convexhull_points_mvplocation = rendering_programs[VERTEX_SHADER].uniformLocation("mvp_matrix");
        intersection_convexhull_points_color_location = rendering_programs[VERTEX_SHADER].uniformLocation("color");
        intersection_convexhull_points_size_location = rendering_programs[VERTEX_SHADER].uniformLocation("Point_Size");
        rendering_programs[VERTEX_SHADER].setUniformValue(intersection_convexhull_points_mvplocation, mvp_matrix);
        rendering_programs[VERTEX_SHADER].release();
    }

/// @}
/// @name Access
/// @{
/// @}
/// @name Inquiry
/// @{
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



