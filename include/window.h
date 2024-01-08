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

        /* My function */
        void on_actionPoints_triggered();
        void on_actionConvexHull_EP_triggered();
        void on_actionConvexHull_EE_2_triggered();
        void on_actionConvexHull_Jarvis_March_2_triggered();
        void on_actionConvexHull_Graham_Scan_2_triggered();
        void on_actionConvexHull_Divide_and_Conquer_2_triggered();
};


#endif // _WINDOW_H_