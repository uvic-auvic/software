/***************************************************************
 * @file /src/qtab.cpp
 * @brief Tab Manager. Take out as much of the implementation
 *        code from MainWindow as possible
 * @date February 2017
/***************************************************************/

/***************************************************************
 * Includes
/***************************************************************/
#include <string>
#include <QString>
#include <QtGui/QMainWindow>

#include "../include/camera_tab.hpp"
#include "../include/config.hpp"

/***************************************************************
 * Namespaces
/***************************************************************/
using namespace redgui;
using namespace tab;

/*************************************************************
 * Implementation [Control Tab]
/*************************************************************/

/*************************************************************
 * @Name     ReadSettings
 * @Args     filename
 * @function reads and stores settings from the file passed
 *************************************************************/
void camera_tab::ReadSettings(std::string filename)
{
    // used to remove the annoying warning
    filename = "test";

}

/*************************************************************
 * @Name     WriteSettings
 * @Args     none
 * @function write the current config settings to the default file
 *************************************************************/
void camera_tab::WriteSettings()
{

}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void camera_tab::init()
{
    /* GUI Cam Video Record */
    //TODO: reconnect once camera is operational
    //QObject::connect(ui.videoRecordPushButton, SIGNAL(released()), this, SLOT(updateVideoRecordPushButton()));
    QObject::connect(qnode, SIGNAL(processedImage(QImage)), this, SLOT(updatePlayerUI(QImage)));
    QObject::connect(qnode, SIGNAL(processedImage2(QImage)), this, SLOT(updatePlayerUI2(QImage)));
}

/*************************************************************
 * Controls Tab Implementation
/*************************************************************/

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void camera_tab::updatePlayerUI(QImage img){
    if(!img.isNull()){
        ui->CameraOneFeed->setAlignment(Qt::AlignCenter);
        ui->CameraOneFeed->setPixmap(QPixmap::fromImage(img).scaled(ui->CameraOneFeed->geometry().width(), ui->CameraOneFeed->geometry().height(), Qt::KeepAspectRatio, Qt::FastTransformation));
    }
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void camera_tab::updatePlayerUI2(QImage img){
    if(!img.isNull()){
        ui->CameraTwoFeed->setAlignment(Qt::AlignCenter);
        ui->CameraTwoFeed->setPixmap(QPixmap::fromImage(img).scaled(ui->CameraTwoFeed->geometry().width(), ui->CameraTwoFeed->geometry().height(), Qt::KeepAspectRatio, Qt::FastTransformation));

    }
}
