#include <QtWidgets/QApplication>
#include "window.h"

int main(int argc, char **argv)
{
    QApplication app (argc, argv);

    Window window("../config/ropods.yaml");
    window.show();

    return app.exec();
}
