#ifndef _GL_DISPLAY_H_
#define _GL_DISPLAY_H_

/* STL headers */
#include <functional>

/* QT headers */
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <CGAL/Qt/qglviewer.h>

/* My headers */
#include "define.h"

struct Draw_Options
{
    float Point_Size = 5.5f;
    QColor points_color = QColor(255.f, 0, 0.0f, 255.f);

    float Segment_Size = 2.0f;
    QColor segments_color = QColor(255.f, 0.f, 255.f);
};
class Gl_Display
{
    public:
        /// @name Type Define
        /// @{
            typedef MyCG::Kernel            Kernel;
            typedef MyCG::FT                FT;
            typedef MyCG::Point_2           Point_2;
            typedef MyCG::Point_3           Point_3;
            typedef MyCG::Vector            Vector;
            typedef MyCG::Polyhedron        Polyhedron;
            
            typedef MyCG::DataF             DataF;
            typedef MyCG::DataI             DataI;
            typedef MyCG::DataPoints_2      DataPoints_2;
            typedef MyCG::DataPoints_3      DataPoints_3;
            typedef MyCG::DataSegments_2    DataSegments_2;
            typedef MyCG::DataSegments_3    DataSegments_3;

            enum Draw_Mode{
                POINTS = 0,
                LINES,
                MESH,
                SIZE_OF_MODE
            };
        /// @}
        /// @name Life Circle
        /// @{
            Gl_Display();
            ~Gl_Display();
        /// @}
        /// @name Operators
        /// @{
        /// @}
        /// @name Operations
        /// @{
            void clean();
        /// @}
        /// @name OpenGL
        /// @{
            void setdisplaymode(int mode);

            void initializeGL();
            void init_buffer(QOpenGLShaderProgram& program,
                                    QOpenGLVertexArrayObject& vao,
                                    QOpenGLBuffer& vbo,
                                    const DataF& data,
                                    const char* attirbute_name);
            void initialize_buffers();
            void compile_shaders();
            void check_buffer();
            void attrib_buffers(CGAL::QGLViewer *viewer);
            void compute_elements();
            void render(CGAL::QGLViewer *viewer);
            void render_points();
            void render_segments();
        /// @}
        /// @name Access
        /// @{
            DataF& Get_pos_points() { return pos_points; }
            DataF& Get_pos_segments() { return pos_segments; }
        /// @}
        /// @name Inquiry
        /// @{
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
        /// @}
        /// @name Private Member Variables
        /// @{
            /* check */
            bool                          gl_init;
            bool                          gl_buffer_init;
              
            /* options */             
            Draw_Options                  options;
            bool                          m_mode[SIZE_OF_MODE];

            /* uniform variables indexes */
            int                           mvpLocation;
            int                           point_size_location;
            int                           points_colorLocation;
            int                           segment_mvpLocation;
            int                           segment_colorlocation;

            /* data */
            DataF                         pos_points;
            DataF                         pos_segments;

            /* OpenGL Buffers */
            enum VBO{
                POINTS_LOCATION = 0,
                SEGMENTS_LOCATION,
                SIZE_OF_VBO
            };
            enum VAO{
                POINTS_VAO = 0,
                SEGMENTS_VAO,
                SIZE_OF_VAO
            };
            enum SHADER{
                VERTEX_SHADER = 0,
                SEGMENT_SHADER,
                SIZE_OF_SHADER
            };

            std::vector<std::function<void()>>  render_functions;
            QOpenGLFunctions                    *gl;
            QOpenGLVertexArrayObject            vao[SIZE_OF_VAO];
            QOpenGLBuffer                       vbo[SIZE_OF_VBO];
            QOpenGLShaderProgram                rendering_programs[SIZE_OF_SHADER];
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

#endif  // _GL_DISPLAY_H_