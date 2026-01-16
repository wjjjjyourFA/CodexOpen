#include <QApplication>

#include "tools/mmpose_inference_gui/mainwidget.h"

int main(int argc, char* argv[]) {
  QApplication a(argc, argv);

  MainWidget w;
  w.show();
  
  return a.exec();
}
