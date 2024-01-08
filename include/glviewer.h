#ifndef _GLVIEWER_H_
#define _GLVIEWER_H_

/* QT headers */ 
#include <QMap>
#include <QtGlobal>
#include <CGAL/Qt/qglviewer.h>

/* My headers */ 
#include "define.h"
#include "gl_display.h"
#include "model.h"
#include "myalgorithm.h"

class glViewer : public CGAL::QGLViewer
{
    Q_OBJECT

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

            typedef MyCG::Model             Model;
        /// @}
        /// @name Life Circle
        /// @{
            glViewer(QWidget *parent = 0);
            ~glViewer();
        /// @}
        /// @name Operators
        /// @{
        /// @}
        /// @name Operations of gl_display
        /// @{
            void draw();  
            void setdisplaymode(int mode);  
            void set_gl_display(Gl_Display* gl_display) { m_gl_display = gl_display; }
        /// @}
        /// @name Operations of model
        /// @{ 
            void generate_points();
            void convexhull();
            void convexhull_ee();
            void convexhull_jarvis_march();
            void convexhull_graham_scan();
            void convexhull_divide_and_conquer();
            void clean();
        /// @}
        /// @name Operations
        /// @{ 
            bool load_file(const std::string& filename);
            bool load_file_off(const std::string& filename);
            void adjustCamera();
        /// @}
        /// @name Access
        /// @{
        /// @}
        /// @name Inquiry
        /// @{
            int Get_Points2_Num_Vertices();
            int Get_Mesh_Num_Vertices();
            int Get_Mesh_Num_Facets();
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
            Gl_Display*                     m_gl_display;
            Model*                          m_model;
        /// @}
        /// @name Private Operatiors
        /// @{
        /// @}
        /// @name Private Operations
        /// @{
            void initializeGL();
            void points_data_to_display(DataPoints_2 const& rpoints);
            void segments_data_to_display(DataSegments_2 const& rsegments);
        /// @}
        /// @name Private Access
        /// @{
        /// @}
        /// @name Private Inquiry
        /// @{
        /// @}
};


#endif // _GLVIEWER_H_