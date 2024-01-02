#include <QtGui>
#include <QSurfaceFormat>

#include "window.h"

#include <CGAL/Qt/resources.h>
int main(int argc, char** argv)
{
  // Read command lines arguments
  QApplication application(argc,argv);

  CGAL_QT_INIT_RESOURCES;

  QSurfaceFormat glFormat;
	glFormat.setProfile(QSurfaceFormat::CoreProfile);
	QSurfaceFormat::setDefaultFormat(glFormat);

  MainWindow mainWindow;
  mainWindow.show();

  // Run main loop
  return application.exec();
}

