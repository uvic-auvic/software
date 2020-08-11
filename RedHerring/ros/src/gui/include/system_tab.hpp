/***************************************************************
 * @file /include/redgui/qtab.hpp
 * @brief Tab Manager. Take out as much of the implementation
 *        code from MainWindow as possible
 * @date February 2017
/***************************************************************/

/***************************************************************
 * Header Guard
/***************************************************************/
#ifndef REDGUI_TAB_SYSTEM
#define REDGUI_TAB_SYSTEM

/***************************************************************
 * Includes
/***************************************************************/
#include <ros/ros.h>
#include <string>
#include <QString>

#include "qnode.hpp"
#include "ui_main_window.h"

/***************************************************************
 * Defines
/***************************************************************/
#define CONFIG_FILE_SYSTEM "system_tab.conf"

/***************************************************************
 * Namespaces
/***************************************************************/
namespace redgui {

  namespace tab {

/*************************************************************
 * Interface [Control Tab]
/*************************************************************/
class system_tab : public QObject{
Q_OBJECT

public :
  system_tab(const Ui::MainWindowDesign * p_ui, QNode * p_qnode)
      : ui(p_ui)
      , qnode(p_qnode)
        {}
  void init();
  void ReadSettings(std::string filename = CONFIG_FILE_SYSTEM);
  void WriteSettings();

public Q_SLOTS:
  /*************************************************************
   * MainWindow Interface
  /*************************************************************/
  void updateUCDieTemperatureLCD(float temp);
  void updateOILTemperatureLCD(float temp);

private:
  const Ui::MainWindowDesign * ui;
  QNode * qnode;
  const char * default_config_file = "communications_tab.conf";
};

  } // end of tab namespace
} // end of redgui namespace

#endif // end of header guard
