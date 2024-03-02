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
    QColor points_color = QColor(255.f, 0.f, 0.f);

    float Segment_Size = 2.0f;
    QColor segments_color = QColor(255.f, 0.f, 255.f);

    float Generation_Segments_Size = 1.0f;
    QColor generation_segments_color = QColor(127.f, 255.f, 212.f);

    float Generation_Points_Size = 8.0f;
    QColor generation_points_color = QColor(155.f, 48.f, 255.f);

    float Intersection_Point_Size = 5.5f;
    QColor intersection_points_color = QColor(0.f, 255.f, 255.f);

    float Intersection_Interval_Size = 2.0f;
    QColor intersection_interval_color = QColor(0.f, 255.f, 0.f);

    float Intersection_Line_Size = 3.0f;
    QColor intersection_line_color = QColor(0.f, 0.f, 255.f);

    float Generation_ConvexHull_Point_Size = 4.0f;
    QColor generation_convexhull_point_color = QColor(162.f, 0, 124.f);

    float Generation_ConvexHull_Line_Size = 2.0f;
    QColor generation_convexhull_line_color = QColor(0.f, 178.f, 191.f);

    float Intersection_Convexhull_Point_Size = 10.0f;
    QColor intersection_convexhull_point_color = QColor(255.f, 0.f, 0.f);

    float Intersection_ConvexHull_Size = 2.0f;
    QColor intersection_convexhull_color = QColor(220.f, 216.f, 0.f);

    float Generation_Polygon_Point_Size = 4.0f;
    QColor generation_polygon_point_color = QColor(220.f, 216.f, 0.f);
    float Generation_Polygon_Segment_Size = 1.0f;
    QColor generation_polygon_color = QColor(123.f, 104.f, 238.f);

    float Triangulation_Monotone_Segment_Size = 2.0f;
    QColor triangulation_monotone_segment_color = QColor(255.f, 0.f, 0.f);
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
                CONVEXHULL_POINTS = 0,
                CONVEXHULL_LINES,
                GENERATION_SEGMENTS,
                GENERATION_POINTS,
                INTERSECTION_POINTS,
                INTERSECTION_LINES,
                INTERSECTION_INTERVAL,
                GENERATION_CONVEXHULL_POINTS,
                GENERATION_CONVEXHULL_LINES,
                INTERSECTION_CONVEXHULL_POINTS,
                INTERSECTION_CONVEXHULL_LINES,
                GENERATION_POLYGON_POINTS,
                GENERATION_POLYGON_LINES,
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
            void render(CGAL::QGLViewer *viewer);
            void render_points();
            void render_generation_intersections();
            void render_segments();
            void render_generation_segments();
            void render_intersection_points();
            void render_intersection_lines();
            void render_intersection_interval();
            void render_generation_convexhull_points();
            void render_generation_convexhull_lines();
            void render_intersection_convexhull_points();
            void render_intersection_convexhull();
            void render_generation_polygon_points();
            void render_generation_polygon_segments();
            void render_triangulation_monotone_segments();
        /// @}
        /// @name Access
        /// @{
            DataF& Get_pos_points() { return pos_points; }
            DataF& Get_pos_segments() { return pos_segments; }
            DataF& Get_pos_generation_intersection_points() { return pos_generation_intersection_points; }
            DataF& Get_pos_intersection_points() { return pos_intersection_points; }
            DataF& Get_pos_intersection_lines() { return pos_intersection_lines; }
            DataF& Get_pos_intersection_interval() { return pos_intersection_interval; }
            DataF& Get_pos_generation_convexhull_points() { return pos_generation_convexhull_points; }
            DataF& Get_pos_generation_convexhull_lines() { return pos_generation_convexhull_lines; }
            DataF& Get_pos_intersection_convexhull() { return pos_intersection_convexhull; }
            DataF& Get_pos_intersection_convexhull_points() { return pos_intersection_convexhull_points; }
            DataF& Get_pos_generation_polygon_points() { return pos_generation_polygon_points; }
            DataF& Get_pos_generation_polygon_lines() { return pos_generation_polygon_lines; }
            DataF& Get_pos_triangulation_monotone_lines() { return pos_triangulation_monotone_lines; }
            
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
            int                           generation_intersection_points_size_location;
            int                           generation_intersection_points_color_location;
            int                           generation_intersection_points_mvplocation;
            int                           intersection_points_size_location;
            int                           intersection_points_color_location;
            int                           intersection_points_mvplocation;
            int                           intersection_line_color_location;
            int                           intersection_line_mvplocation;
            int                           intersection_interval_color;
            int                           intersection_interval_mvplocation;
            int                           generation_convexhull_points_size_location;
            int                           generation_convexhull_points_color_location;
            int                           generation_convexhull_points_mvplocation;
            int                           generation_convexhull_lines_color_location;
            int                           generation_convexhull_lines_mvplocation;
            int                           intersection_convexhull_color_location;
            int                           intersection_convexhull_mvplocation;
            int                           intersection_convexhull_points_size_location;
            int                           intersection_convexhull_points_color_location;
            int                           intersection_convexhull_points_mvplocation;
            int                           generation_polygon_points_size_location;
            int                           generation_polygon_points_color_location;
            int                           generation_polygon_points_mvplocation;
            int                           generation_polygon_lines_color_location;
            int                           generation_polygon_lines_mvplocation;
            int                           triangulation_monotone_segment_color_location;
            int                           triangulation_monotone_segment_mvplocation;

            /* data */
            DataF                         pos_points;
            DataF                         pos_segments;
            DataF                         pos_generation_intersection_points;
            DataF                         pos_intersection_points;
            DataF                         pos_intersection_lines;
            DataF                         pos_intersection_interval;
            DataF                         pos_generation_convexhull_points;
            DataF                         pos_generation_convexhull_lines;
            DataF                         pos_intersection_convexhull;
            DataF                         pos_intersection_convexhull_points;
            DataF                         pos_generation_polygon_points;
            DataF                         pos_generation_polygon_lines;
            DataF                         pos_triangulation_monotone_lines;
            /* OpenGL Buffers */
            enum VBO{
                POINTS_LOCATION = 0,
                SEGMENTS_LOCATION,
                GENERATION_INTERSECTION_LOCATION,
                INTERSECTION_POINTS_LOCATION,
                INTERSECTION_LINES_LOCATION,
                INTERSECTION_INTERVAL_LOCATION,
                GENERATION_CONVEXHULL_POINTS_LOCATION,
                GENERATION_CONVEXHULL_LINES_LOCATION,
                INTERSECTION_CONVEXHULL_LOCATION,
                INTERSECTION_CONVEXHULL_POINTS_LOCATION,
                GENERATION_POLYGON_POINTS_LOCATION,
                GENERATION_POLYGON_LINES_LOCATION,
                TRIANGULATION_MONOTONE_LINES_LOCATION,
                SIZE_OF_VBO
            };
            enum VAO{
                POINTS_VAO = 0,
                SEGMENTS_VAO,
                GENERATION_INTERSECTION_VAO,
                INTERSECTION_POINTS_VAO,
                INTERSECTION_LINES_VAO,
                INTERSECTION_INTERVAL_VAO,
                GENERATION_CONVEXHULL_POINTS_VAO,
                GENERATION_CONVEXHULL_LINES_VAO,
                INTERSECTION_CONVEXHULL_VAO,
                INTERSECTION_CONVEXHULL_POINTS_VAO,
                GENERATION_POLYGON_POINTS_VAO,
                GENERATION_POLYGON_LINES_VAO,
                TRIANGULATION_MONOTONE_LINES_VAO,
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
            void attrib_buffer_vertex(int& psl, int& pcl, int& mvpl, QMatrix4x4& mvp_matrix);
            void attrib_buffer_segment(int& scl, int& mvpl, QMatrix4x4& mvp_matrix);
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