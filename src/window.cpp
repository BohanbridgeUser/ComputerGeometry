#include "window.h"
#include <string>

/// @brief public:
/// @name Type Define
/// @{
/// @}
/// @name Life Circle
/// @{
    MainWindow::MainWindow()
    {
        setupUi(this);

        m_gl_display = new Gl_Display;
        m_glviewer->set_gl_display(m_gl_display);

        std::string info = "Window Initialized\n";
        m_information->setPlainText(QString::fromStdString(info));

        menuView->addAction(dockWidget_info->toggleViewAction());
    }

    MainWindow::~MainWindow()
    {
        delete m_gl_display;
    }
/// @}
/// @name Operations
/// @{
    void MainWindow::update()
    {
        m_glviewer->repaint();
    }

/// @}
/// @name Menu Operations
/// @{
    void MainWindow::on_action_Open_triggered()
    {
        QString filename = QFileDialog::getOpenFileName(this, tr("Open File"), ".");
        int lastDotIndex = filename.lastIndexOf(".");
        QString suffix = filename.mid(lastDotIndex + 1);
        if(filename.isEmpty())
        {
            QMessageBox::warning(this, tr("Error"), tr("No file selected"));
            std::string info = "Failed load File [ " + filename.toStdString() + " ]!\n";
            m_information->setPlainText(QString::fromStdString(info));
            return;
        }

        if(suffix == "obj")
        {
            std::string info = "Loading file: " + filename.toStdString() + "\n";
            m_information->setPlainText(QString::fromStdString(info));
            if(m_glviewer->load_file(filename.toStdString()))
            {
                std::string info = "File [ " + filename.toStdString() + " ] loaded.\n";
                std::string details = "  Load mesh with " + std::to_string(m_glviewer->Get_Mesh_Num_Vertices()) 
                                    + " vertices and " + std::to_string(m_glviewer->Get_Mesh_Num_Facets())
                                    + " faces.\n";
                m_information->setPlainText(QString::fromStdString(info+details));    
            }
            else
            {
                std::string info = "Failed load File [ " + filename.toStdString() + " ]!\n";
                m_information->setPlainText(QString::fromStdString(info));
            }   
        }
        else if(suffix == "off")
        {
            std::string info = "Loading file: " + filename.toStdString() + "\n";
            m_information->setPlainText(QString::fromStdString(info));
            if(m_glviewer->load_file_off(filename.toStdString()))
            {
                std::string info = "File [ " + filename.toStdString() + " ] loaded.\n";
                std::string details = "  Load Points with " + std::to_string(m_glviewer->Get_Points2_Num_Vertices()) 
                                    + " vertices \n";
                m_information->setPlainText(QString::fromStdString(info+details));    
            }
            else
            {
                std::string info = "Failed load File [ " + filename.toStdString() + " ]!\n";
                m_information->setPlainText(QString::fromStdString(info));
            }   
        }
        else if(suffix == "off2")
        {
            std::string info = "Loading file: " + filename.toStdString() + "\n";
            m_information->setPlainText(QString::fromStdString(info));
            if(m_glviewer->load_file_off2(filename.toStdString()))
            {
                std::string info = "File [ " + filename.toStdString() + " ] loaded.\n";
                std::string details = "  Load Points with " + std::to_string(m_glviewer->Get_Points2_Num_Vertices()) 
                                    + " vertices \n";
                m_information->setPlainText(QString::fromStdString(info+details));    
            }
            else
            {
                std::string info = "Failed load File [ " + filename.toStdString() + " ]!\n";
                m_information->setPlainText(QString::fromStdString(info));
            }  
        }
        else if(suffix == "int")
        {
            std::string info = "Loading file: " + filename.toStdString() + "\n";
            m_information->setPlainText(QString::fromStdString(info));
            if(m_glviewer->load_file_int(filename.toStdString()))
            {
                std::string info = "File [ " + filename.toStdString() + " ] loaded.\n";
                std::string details = "  Load Points with " + std::to_string(m_glviewer->Get_Intersection_Num_Points()) 
                                    + " vertices \n";
                m_information->setPlainText(QString::fromStdString(info+details));    
            }
            else
            {
                std::string info = "Failed load File [ " + filename.toStdString() + " ]!\n";
                m_information->setPlainText(QString::fromStdString(info));
            }   
        }
        else if(suffix == "chf")
        {
            std::string info = "Loading file: " + filename.toStdString() + "\n";
            m_information->setPlainText(QString::fromStdString(info));
            if(m_glviewer->load_file_chf(filename.toStdString()))
            {
                std::string info = "File [ " + filename.toStdString() + " ] loaded.\n";
                std::string details = "  Load Points with vertices \n";
                m_information->setPlainText(QString::fromStdString(info+details));    
            }
            else
            {
                std::string info = "Failed load File [ " + filename.toStdString() + " ]!\n";
                m_information->setPlainText(QString::fromStdString(info));
            }   
        }
        else
        {
            QMessageBox::warning(this, tr("Error"), tr("File format not supported"));
            std::string info = "Failed load File [ " + filename.toStdString() + " ]!\n";
            m_information->setPlainText(QString::fromStdString(info));
            return;
        }
        update();
    }
    
    void MainWindow::on_action_Quit_triggered()
    {
        close();
    }

    void MainWindow::on_actionClear_triggered()
    {
        m_glviewer->clean();
        update();
    }

    /* Generation */
    void MainWindow::on_actionPoints_triggered()
    {
        m_glviewer->generate_points();
        update();
    }

    void MainWindow::on_actionOn_Circle_triggered()
    {
        m_glviewer->generate_segments_on_circle();
        update();
    }

    void MainWindow::on_actionFrom_points_triggered()
    {
        m_glviewer->generate_segments_from_points();
        update();
    }

    void MainWindow::on_actionConvexHulls_triggered()
    {
        m_glviewer->generate_convexhulls();
        update();
    }

    // ConvexHull
    void MainWindow::on_actionConvexHull_EP_triggered()
    {
        m_glviewer->convexhull();
        update();
    }

    void MainWindow::on_actionConvexHull_EE_2_triggered()
    {
        m_glviewer->convexhull_ee();
        update();
    }

    void MainWindow::on_actionConvexHull_Jarvis_March_2_triggered()
    {
        m_glviewer->convexhull_jarvis_march();
        update();
    }

    void MainWindow::on_actionConvexHull_Graham_Scan_2_triggered()
    {
        m_glviewer->convexhull_graham_scan();
        update();
    }

    void MainWindow::on_actionConvexHull_Divide_and_Conquer_2_triggered()
    {
        m_glviewer->convexhull_divide_and_conquer();
        update();
    }

    // Intersection
    void MainWindow::on_actionInterval_triggered()
    {
        m_glviewer->interval();
        update();
    }

    void MainWindow::on_actionSegments_2_triggered()
    {
        m_glviewer->segments_intersection_2();
        update();
    }

    void MainWindow::on_actionConvexHull_2_triggered()
    {
        std::vector<std::vector<int>> intersections(7, std::vector<int>(7, 0));
        m_glviewer->convexhulls_intersection(intersections);
        std::string info;
        for(int i=0;i<intersections.size();++i)
            for(int j=i+1;j<intersections[i].size();++j)
                if(intersections[i][j] != 0)
                    info += "convexhull " + std::to_string(i) + " and " + std::to_string(j) + " intersect\n";
        m_information->setPlainText(QString::fromStdString(info)); 
        update();
    }

    void MainWindow::on_actionConvexHull_Edge_Chasing_triggered()
    {
        m_glviewer->convexhulls_edge_chasing();
        update();
    }

/// @}

