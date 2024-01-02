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
        mvpLocation = rendering_programs[SEGMENT_SHADER].uniformLocation("mvp_matrix");
        segment_colorlocation = rendering_programs[SEGMENT_SHADER].uniformLocation("color");
        rendering_programs[SEGMENT_SHADER].setUniformValue(mvpLocation, mvp_matrix);
        rendering_programs[SEGMENT_SHADER].release();
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



