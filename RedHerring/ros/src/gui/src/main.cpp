/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
 * Includes
 *****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/main_window.hpp"

/*****************************************************************************
 * Defines
 *****************************************************************************/
#define GUI_WIDTH_MULTIPLIER  0.45
#define GUI_HEIGHT_MULTIPLIER 0.85

/*****************************************************************************
 * Main
 *****************************************************************************/
int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    redgui::MainWindow w(argc,argv);

    /* Sets the GUI size to ~85% of your screens height and ~45% the width */
    auto qrect = QApplication::desktop()->screenGeometry();
    int gui_width = qrect.width() * GUI_WIDTH_MULTIPLIER;
    int gui_height = qrect.height() * GUI_HEIGHT_MULTIPLIER;
    w.setFixedSize(gui_width,gui_height);

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    w.show();


    /* start app and return exit code */
    return app.exec();
}
