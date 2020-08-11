/***************************************************************
 * @file /include/redgui/qtab.hpp
 * @brief Tab Manager. Take out as much of the implementation
 *        code from MainWindow as possible
 * @date February 2017
/***************************************************************/

/***************************************************************
 * Header Guard
/***************************************************************/
#ifndef REDGUI_TAB_CONTROL
#define REDGUI_TAB_CONTROL

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
#define CONFIG_FILE_CONTROL "control.conf"

/***************************************************************
 * Namespaces
/***************************************************************/
namespace redgui {

  namespace tab {

/*************************************************************
 * Interface [Control Tab]
/*************************************************************/
class controls_tab : public QObject{
Q_OBJECT

public :
  controls_tab(const Ui::MainWindowDesign * p_ui, QNode * p_qnode)
      : ui(p_ui)
      , qnode(p_qnode)
        {}

  void init();
  void ReadSettings(std::string filename = CONFIG_FILE_CONTROL);
  void WriteSettings();
  void enableControls(bool enable);

public Q_SLOTS:
  /*************************************************************
   * MainWindow Interface
  /*************************************************************/
  /* Slider/text handling slots */
  void updateForwardTrimSliderText(int value);
  void updatePitchTrimSliderText(int value);
  void updateYawTrimSliderText(int value);
  void updateRollTrimSliderText(int value);

  /* Slider handling slots */
  void updateForwardTrimSliderPos(QString qs);
  void updatePitchTrimSliderPos(QString qs);
  void updateYawTrimSliderPos(QString qs);
  void updateRollTrimSliderPos(QString qs);

  /* Checkbox handling slots */
  void on_forwardInvertCheckBox_stateChanged(int state);
  void on_pitchInvertCheckBox_stateChanged(int state);
  void on_yawInvertCheckBox_stateChanged(int state);
  void on_rollInvertCheckBox_stateChanged(int state);

  /* Toggle Buttons and checkboxes */
  void update_ROVtoAUVPushButton();
  void on_lightsCheckBox_stateChanged(int state);
  void on_throttleLockoutCheckBox_stateChanged(int state);

private:
  const Ui::MainWindowDesign * ui;
  QNode * qnode;
  const char * default_config_file = "controls_tab.conf";

};


  } // end of tab namespace
} // end of redgui namespace

#endif // end of header guard
