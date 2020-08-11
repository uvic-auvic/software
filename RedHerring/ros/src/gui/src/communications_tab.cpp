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

#include "../include/communications_tab.hpp"
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
void communications_tab::ReadSettings(std::string filename)
{
    // used to remove the annoying warning
    filename = "test";

}

/*************************************************************
 * @Name     WriteSettings
 * @Args     none
 * @function write the current config settings to the default file
 *************************************************************/
void communications_tab::WriteSettings()
{

}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void communications_tab::init()
{
  ui->view_logging->setModel(qnode->loggingModel());
  //QObject::connect(qnode, SIGNAL(LoggingUpdated()), this, SLOT(updateLoggingView()));
}

/*************************************************************
 * Controls Tab Implementation
/*************************************************************/

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void communications_tab::updateLoggingView()
{
  ui->view_logging->scrollToBottom();
}


