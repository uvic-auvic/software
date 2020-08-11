/**
 * @file /include/redgui/main_window.hpp
 *
 * @brief Qt based gui for redgui.
 *
 * @date November 2010
 **/
#ifndef REDGUI_MAIN_WINDOW_H
#define REDGUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "controls_tab.hpp"
#include "communications_tab.hpp"
#include "system_tab.hpp"
#include "motors_tab.hpp"
#include "camera_tab.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace redgui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the main window here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void ReadSettings(); // Load up qt program settings at startup
    void WriteSettings(); // Save qt program settings when closing

    void closeEvent(QCloseEvent *event); // Overloaded function
    void init();

public Q_SLOTS:
    /******************************************
     * Auto-connections (connectSlotsByName())
    /******************************************/
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check);
    void on_checkbox_use_environment_stateChanged(int state);
    void enableControls(bool enable);

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    tab::controls_tab controls_tab;
    tab::communications_tab communications_tab;
    tab::system_tab system_tab;
    tab::motors_tab motors_tab;
    tab::camera_tab camera_tab;
};

}  // namespace redgui

//Q_DECLARE_METATYPE(thrusters::thrusterValues)
#endif // redgui_MAIN_WINDOW_H
