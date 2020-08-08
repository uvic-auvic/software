/***************************************************************
 * @file /src/qtab.cpp
 * @brief Tab Manager. Take out as much of the implementation
 *        code from MainWindow as possible
 * @date February 2017
/***************************************************************/

/***************************************************************
 * Includes
/***************************************************************/
#include <ros/ros.h>
#include <string>
#include <QString>
#include <QtGui/QMainWindow>

#include "../include/controls_tab.hpp"
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
void controls_tab::ReadSettings(std::string filename)
{
    // used to remove the annoying warning
    filename = "test";

}

/*************************************************************
 * @Name     WriteSettings
 * @Args     none
 * @function write the current config settings to the default file
 *************************************************************/
void controls_tab::WriteSettings()
{

}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::init() {
    /* Connect the text input*/
    QObject::connect(ui->forwardTrimHorizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updateForwardTrimSliderText(int)));
    QObject::connect(ui->pitchTrimHorizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updatePitchTrimSliderText(int)));
    QObject::connect(ui->yawTrimHorizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updateYawTrimSliderText(int)));
    QObject::connect(ui->rollTrimHorizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(updateRollTrimSliderText(int)));

    /* Connect the Sliders*/
    QObject::connect(ui->forwardTrimLineEdit, SIGNAL(textChanged(QString)), this, SLOT(updateForwardTrimSliderPos(QString)));
    QObject::connect(ui->pitchTrimLineEdit, SIGNAL(textChanged(QString)), this, SLOT(updatePitchTrimSliderPos(QString)));
    QObject::connect(ui->yawTrimLineEdit, SIGNAL(textChanged(QString)), this, SLOT(updateYawTrimSliderPos(QString)));
    QObject::connect(ui->rollTrimLineEdit, SIGNAL(textChanged(QString)), this, SLOT(updateRollTrimSliderPos(QString)));

    /* Lights */
    //TODO: re-enable when its properly configured
    //QObject::connect(ui.lightsPushButton, SIGNAL(toggled(bool)), this, SLOT(on_lightsPushButton_toggled(bool)));

    /* Control Mode */
    QObject::connect(ui->toggleModeButton, SIGNAL(released()), this, SLOT(update_ROVtoAUVPushButton()));

    /* Enable Tab and all the GroupBoxes*/
    ui->control_tab->setEnabled(true);
    ui->toggleModeButton->setEnabled(true);
    this->enableControls(true);


}

/*************************************************************
 * Controls Tab Implementation
/*************************************************************/

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::updateForwardTrimSliderText(int value)
{
    ui->forwardTrimLineEdit->setText(QString("%1").arg(value));
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::updatePitchTrimSliderText(int value)
{
    ui->pitchTrimLineEdit->setText(QString("%1").arg(value));
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::updateYawTrimSliderText(int value)
{
    ui->yawTrimLineEdit->setText(QString("%1").arg(value));
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::updateRollTrimSliderText(int value)
{
    ui->rollTrimLineEdit->setText(QString("%1").arg(value));
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::updateForwardTrimSliderPos(QString qs)
{
    QRegExp re("[-+]?[0-9]*");
    if(re.exactMatch(qs))
    {
        int value;
        bool ok;
        value = qs.toInt(&ok, 10);
        if(ok && value <= ui->forwardTrimHorizontalSlider->maximum() && value >= ui->forwardTrimHorizontalSlider->minimum())
        {
            ui->forwardTrimHorizontalSlider->setValue(value);
        }
        else
        {
            ROS_WARN("Only numerical values between %d and %d are acceptable!", ui->forwardTrimHorizontalSlider->minimum(), ui->forwardTrimHorizontalSlider->maximum());
            ui->forwardTrimLineEdit->setText((QString("%1").arg(ui->forwardTrimHorizontalSlider->value())));
        }
    }
    else
    {
        ROS_WARN("Only numerical values between %d and %d are acceptable!", ui->forwardTrimHorizontalSlider->minimum(), ui->forwardTrimHorizontalSlider->maximum());
        ui->forwardTrimLineEdit->setText((QString("%1").arg(ui->forwardTrimHorizontalSlider->value())));
    }
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::updatePitchTrimSliderPos(QString qs)
{
    QRegExp re("[-+]?[0-9]*");
    if(re.exactMatch(qs))
    {
        int value;
        bool ok;
        value = qs.toInt(&ok, 10);
        if(ok && value <= ui->pitchTrimHorizontalSlider->maximum() && value >= ui->pitchTrimHorizontalSlider->minimum())
        {
            ui->pitchTrimHorizontalSlider->setValue(value);
        }
        else
        {
            ROS_WARN("Only numerical values between %d and %d are acceptable!", ui->pitchTrimHorizontalSlider->minimum(), ui->pitchTrimHorizontalSlider->maximum());
            ui->pitchTrimLineEdit->setText((QString("%1").arg(ui->pitchTrimHorizontalSlider->value())));
        }
    }
    else
    {
        ROS_WARN("Only numerical values between %d and %d are acceptable!", ui->pitchTrimHorizontalSlider->minimum(), ui->pitchTrimHorizontalSlider->maximum());
        ui->pitchTrimLineEdit->setText((QString("%1").arg(ui->pitchTrimHorizontalSlider->value())));
    }
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::updateYawTrimSliderPos(QString qs)
{
    QRegExp re("[-+]?[0-9]*");
    if(re.exactMatch(qs))
    {
        int value;
        bool ok;
        value = qs.toInt(&ok, 10);
        if(ok && value <= ui->yawTrimHorizontalSlider->maximum() && value >= ui->yawTrimHorizontalSlider->minimum())
        {
            ui->yawTrimHorizontalSlider->setValue(value);
        }
        else
        {
            ROS_WARN("Only numerical values between %d and %d are acceptable!", ui->yawTrimHorizontalSlider->minimum(), ui->yawTrimHorizontalSlider->maximum());
            ui->yawTrimLineEdit->setText((QString("%1").arg(ui->yawTrimHorizontalSlider->value())));
        }
    }
    else
    {
        ROS_WARN("Only numerical values between %d and %d are acceptable!", ui->yawTrimHorizontalSlider->minimum(), ui->yawTrimHorizontalSlider->maximum());
        ui->yawTrimLineEdit->setText((QString("%1").arg(ui->yawTrimHorizontalSlider->value())));
    }
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::updateRollTrimSliderPos(QString qs)
{
    QRegExp re("[-+]?[0-9]*");
    if(re.exactMatch(qs))
    {
        int value;
        bool ok;
        value = qs.toInt(&ok, 10);
        if(ok && value <= ui->rollTrimHorizontalSlider->maximum() && value >= ui->rollTrimHorizontalSlider->minimum())
        {
            ui->rollTrimHorizontalSlider->setValue(value);
        }
        else
        {
            ROS_WARN("Only numerical values between %d and %d are acceptable!", ui->rollTrimHorizontalSlider->minimum(), ui->rollTrimHorizontalSlider->maximum());
            ui->rollTrimLineEdit->setText((QString("%1").arg(ui->rollTrimHorizontalSlider->value())));
        }
    }
    else
    {
        ROS_WARN("Only numerical values between %d and %d are acceptable!", ui->rollTrimHorizontalSlider->minimum(), ui->rollTrimHorizontalSlider->maximum());
        ui->rollTrimLineEdit->setText((QString("%1").arg(ui->rollTrimHorizontalSlider->value())));
    }
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::on_forwardInvertCheckBox_stateChanged(int state)
{
    qnode->forwardInvertChange((bool)(state != 0));
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::on_pitchInvertCheckBox_stateChanged(int state)
{
    qnode->pitchInvertChange((bool)(state != 0));
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::on_rollInvertCheckBox_stateChanged(int state)
{
    qnode->rollInvertChange((bool)(state != 0));
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::on_yawInvertCheckBox_stateChanged(int state)
{
    qnode->yawInvertChange((bool)(state != 0));
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::update_ROVtoAUVPushButton()
{
    switch (qnode->whichControlMode())
    {
        /* Currently in ROV mode and switching to AUV mode */
        case redgui::ROV:
                ui->toggleModeButton->setText("AUV mode");
                qnode->updateControlMode(redgui::AUV);
                break;

        /* In any other mode, switch back to ROV Mode */
        default:
                ui->toggleModeButton->setText("ROV mode");
                qnode->updateControlMode(redgui::ROV);
                break; //uneccesary but just to keep if uniform. The compiler will remove it anyway
    }
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::on_lightsCheckBox_stateChanged(int state)
{
    qnode->lightChange((bool)(state != 0));
}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void controls_tab::on_throttleLockoutCheckBox_stateChanged(int state)
{
    bool enabled;
    if(state == 0){
        QString unlocked = "QProgressBar::chunk {background:  rgb(48, 181, 8);} QProgressBar {text-align: center;}";
        ui->ascentThrusterBar->setStyleSheet(unlocked);
        ui->yawThrusterBar->setStyleSheet(unlocked);
        ui->pitchThrusterBar->setStyleSheet(unlocked);
        ui->forwardThrusterBar->setStyleSheet(unlocked);
        ui->rollThrusterBar->setStyleSheet(unlocked);
        enabled = false;
    }
    else
    {
        QString locked =  "QProgressBar::chunk {background: rgb(172, 172, 172);} QProgressBar {text-align: center;}";
        ui->ascentThrusterBar->setStyleSheet(locked);
        ui->yawThrusterBar->setStyleSheet(locked);
        ui->pitchThrusterBar->setStyleSheet(locked);
        ui->forwardThrusterBar->setStyleSheet(locked);
        ui->rollThrusterBar->setStyleSheet(locked);
        enabled = true;
    }
    qnode->throttleLockoutChange(enabled);
}

void controls_tab::enableControls(bool enable)
{
        ui->JoystickGroupBox->setEnabled(enable);
        ui->GeneralControlsGroupBox->setEnabled(enable);
        ui->throttleLockoutCheckBox->setEnabled(enable);
}




