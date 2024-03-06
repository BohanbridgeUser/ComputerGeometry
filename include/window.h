#ifndef _WINDOW_H_
#define _WINDOW_H_

/* QT headers */
#include <QWidget>

/* My headers */
#include "gl_display.h"
#include "glviewer.h"
#include "ui_CGUI.h"

class MainWindow : public QMainWindow, public Ui_MainWindow
{
    Q_OBJECT

private:
    Gl_Display* m_gl_display;

public:
    MainWindow();
    ~MainWindow();

    void update();

protected:
    public slots:
        void on_action_Open_triggered();
        void on_action_Quit_triggered();

        void on_actionClear_triggered();

        /* Generation */
        void on_actionPoints_triggered();
        void on_actionOn_Circle_triggered();
        void on_actionFrom_points_triggered();
        void on_actionPolygon_triggered();

        /* My function */
        // ConvexHull
        void on_actionConvexHull_EP_triggered();
        void on_actionConvexHull_EE_2_triggered();
        void on_actionConvexHull_Jarvis_March_2_triggered();
        void on_actionConvexHull_Graham_Scan_2_triggered();
        void on_actionConvexHull_Divide_and_Conquer_2_triggered();

        // Intersection
        void on_actionInterval_triggered();
        void on_actionSegments_2_triggered();
        void on_actionConvexHulls_triggered();
        void on_actionConvexHull_2_triggered();
        void on_actionConvexHull_Edge_Chasing_triggered();

        //Triangulation
        void on_actionMonotone_triggered();

        // Voronoi
        void on_actionNaive_3_triggered();
        void on_actionIncremental_triggered();
        void on_actionDivide_and_Conquer_2_triggered();
        void on_actionSweep_Line_2();
};


#endif // _WINDOW_H_