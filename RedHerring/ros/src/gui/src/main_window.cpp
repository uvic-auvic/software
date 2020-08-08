/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <QtGui>
#include <QCheckBox>
#include <QMessageBox>
#include <iostream>
#include <stdio.h>

#include "../include/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace redgui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
    , controls_tab(&ui, &qnode)
    , communications_tab(&ui, &qnode)
    , system_tab(&ui, &qnode)
    , motors_tab(&ui, &qnode)
    , camera_tab(&ui, &qnode)
{

    /* Initializes this MainWindow */
    this->init();

    /* Initialize all the tabs */
    controls_tab.init();
    communications_tab.init();
    system_tab.init();
    motors_tab.init();
    camera_tab.init();
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation
*****************************************************************************/

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void MainWindow::init()
{
    /* Setup all of the UI elements and connects their internal signal/slots */
    ui.setupUi(this);

    /* Connects main window signals and slots to the various triggers */
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(enableControls(bool)), this, SLOT(enableControls(bool)));

    /* Setup the UI based off previous settings  */
    ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));

    /* Ensures the first tab is the first thing shown  */
    ui.tab_manager->setCurrentIndex(0);

}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/
void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>REDGUI</h2><p>The GUI used to configure and monitor RedHerring.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "redgui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if (checked)
    {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    }

}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "redgui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

/*****************************************************************************
 * Implementation [Slots]
/*****************************************************************************/

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void MainWindow::on_button_connect_clicked(bool check)
{
    if ( ui.checkbox_use_environment->isChecked() ) // Use roscore running on the same computer
    {
        if ( !qnode.init() )
        {
            QMessageBox::information(this, "RedGui", "Couldn't find the ROS Master");
        }

        else
        {
            ui.button_connect->setEnabled(false);
        }
    }

    else // Use a roscore running on a different machine
    {

        if ( !qnode.init(ui.line_edit_master->text().toStdString(), ui.line_edit_host->text().toStdString()) )
        {
            QMessageBox::information(this, "RedGui", "Couldn't find the ROS Master");
        }

        else
        {
            ui.button_connect->setEnabled(false);
            ui.line_edit_master->setReadOnly(true);
            ui.line_edit_host->setReadOnly(true);
            //ui.line_edit_topic->setReadOnly(true);
            ui.GeneralControlsGroupBox->setEnabled(true);

            ui.throttleLockoutCheckBox->setEnabled(true);
            ui.control_tab->setEnabled(true);
            ui.JoystickGroupBox->setEnabled(true);

            qnode.throttleLockoutChange(true);
            ui.forwardFactor->setValue(qnode.sensitivityData("forwardSensitivity"));
            ui.pitchFactor->setValue(qnode.sensitivityData("pitchSensitivity"));
            ui.rollFactor->setValue(qnode.sensitivityData("rollSensitivity"));
            ui.yawFactor->setValue(qnode.sensitivityData("yawSensitivity"));
            ui.ascentFactor->setValue(qnode.sensitivityData("ascentSensitivity"));
            qnode.sensitivityPublish();


            if(qnode.whichControlMode() == 2)
                ui.toggleModeButton->setText("AUV mode");
            /* XXXXX */
            //ui.videoRecordPushButton->setEnabled(false); /* XXXXXX
            //                                              * Disabled for demonstration days so people don't save gigs of video data and overload
           /* XXXXX                                       * the harddrive. Comment out for normal use to give the option to record video.
                                                          * XXXXXX*/
        }
    }
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
    bool enabled = (state == 0);
    ui.line_edit_master->setEnabled(enabled);
    ui.line_edit_host->setEnabled(enabled);
    //ui.line_edit_topic->setEnabled(enabled);
}

void MainWindow::enableControls(bool enable)
{
    controls_tab.enableControls(enable);
}

}  // namespace redgui

