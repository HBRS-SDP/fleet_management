#include <QtWidgets/QApplication>
#include "window.h"

int main(int argc, char **argv)
{
    QApplication app (argc, argv);

    Window window("../config/ccu_config.yaml");
    window.show();

    return app.exec();
}
