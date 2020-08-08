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

#include "../include/motors_tab.hpp"
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
void motors_tab::ReadSettings(std::string filename)
{
    // used to remove the annoying warning
    filename = "test";

}

/*************************************************************
 * @Name     WriteSettings
 * @Args     none
 * @function write the current config settings to the default file
 *************************************************************/
void motors_tab::WriteSettings()
{

}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::init()
{
    /* Thruster Sensitivities */
    QObject::connect(ui->forwardFactor, SIGNAL(valueChanged(int)), this, SLOT(updateForwardSensitivity(int)));
    QObject::connect(ui->pitchFactor, SIGNAL(valueChanged(int)), this, SLOT(updatePitchSensitivity(int)));
    QObject::connect(ui->rollFactor, SIGNAL(valueChanged(int)), this, SLOT(updateRollSensitivity(int)));
    QObject::connect(ui->yawFactor, SIGNAL(valueChanged(int)), this, SLOT(updateYawSensitivity(int)));
    QObject::connect(ui->ascentFactor, SIGNAL(valueChanged(int)), this, SLOT(updateAscentSensitivity(int)));


    /* Thruster Values */
    QObject::connect(qnode, SIGNAL(thrusterForwardSignal(float)), this, SLOT(updateForwardThrusterBar(float)));
    QObject::connect(qnode, SIGNAL(thrusterPitchSignal(float)), this, SLOT(updatePitchThrusterBar(float)));
    QObject::connect(qnode, SIGNAL(thrusterRollSignal(float)), this, SLOT(updateRollThrusterBar(float)));
    QObject::connect(qnode, SIGNAL(thrusterYawSignal(float)), this, SLOT(updateYawThrusterBar(float)));
    QObject::connect(qnode, SIGNAL(thrusterAscentSignal(float)), this, SLOT(updateAscentThrusterBar(float)));
}

/*************************************************************
 * Controls Tab Implementation
/*************************************************************/

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::updateForwardSensitivity(int value)
{
    ui->forwardFactor->setValue(value);
    qnode->forwardSensitivity(value);
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::updatePitchSensitivity(int value)
{
    ui->pitchFactor->setValue(value);
    qnode->pitchSensitivity(value);
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::updateRollSensitivity(int value)
{
    ui->rollFactor->setValue(value);
    qnode->rollSensitivity(value);
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::updateYawSensitivity(int value)
{
    ui->yawFactor->setValue(value);
    qnode->yawSensitivity(value);
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::updateAscentSensitivity(int value)
{
    ui->ascentFactor->setValue(value);
    qnode->ascentSensitivity(value);
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::updateForwardThrusterBar(float thrustValue)
{
    int value = (thrustValue);
    ui->forwardThrusterBar->setValue(value);
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::updatePitchThrusterBar(float thrustValue)
{
    int value = (thrustValue);
    ui->pitchThrusterBar->setValue(value);
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::updateRollThrusterBar(float thrustValue)
{
    int value = (thrustValue);
    ui->rollThrusterBar->setValue(value);
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::updateYawThrusterBar(float thrustValue)
{
    int value = (thrustValue);
    ui->yawThrusterBar->setValue(value);
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void motors_tab::updateAscentThrusterBar(float thrustValue)
{
    int value = (thrustValue);
    ui->ascentThrusterBar->setValue(value);
}
