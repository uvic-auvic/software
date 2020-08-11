/***************************************************************
 * @file /include/redgui/qtab.hpp
 * @brief Tab Manager. Take out as much of the implementation
 *        code from MainWindow as possible
 * @date February 2017
/***************************************************************/

/***************************************************************
 * Header Guard
/***************************************************************/
#ifndef REDGUI_TAB_MOTORS
#define REDGUI_TAB_MOTORS

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
#define CONFIG_FILE_MOTORS "motors_tab.conf"

/***************************************************************
 * Namespaces
/***************************************************************/
namespace redgui {

  namespace tab {

/*************************************************************
 * Interface [Control Tab]
/*************************************************************/
class motors_tab : public QObject{
Q_OBJECT

public :
  motors_tab(const Ui::MainWindowDesign * p_ui, QNode * p_qnode)
      : ui(p_ui)
      , qnode(p_qnode)
        {}
  void init();
  void ReadSettings(std::string filename = CONFIG_FILE_MOTORS);
  void WriteSettings();

public Q_SLOTS:
  /*************************************************************
   * MainWindow Interface
  /*************************************************************/
  void updateForwardSensitivity(int value);
  void updatePitchSensitivity(int value);
  void updateRollSensitivity(int value);
  void updateYawSensitivity(int value);
  void updateAscentSensitivity(int value);
  void updateForwardThrusterBar(float thrustValue);
  void updatePitchThrusterBar(float thrustValue);
  void updateRollThrusterBar(float thrustValue);
  void updateYawThrusterBar(float thrustValue);
  void updateAscentThrusterBar(float ascentCommand);

private:
  const Ui::MainWindowDesign * ui;
  QNode * qnode;
  const char * default_config_file = "communications_tab.conf";
};

  } // end of tab namespace
} // end of redgui namespace

#endif // end of header guard
