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

#include "../include/system_tab.hpp"
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
void system_tab::ReadSettings(std::string filename)
{
    // used to remove the annoying warning
    filename = "test";

}

/*************************************************************
 * @Name     WriteSettings
 * @Args     none
 * @function write the current config settings to the default file
 *************************************************************/
void system_tab::WriteSettings()
{

}

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void system_tab::init()
{
    QObject::connect(qnode, SIGNAL(UCDieTemperatureSignal(float)), this, SLOT(updateUCDieTemperatureLCD(float)));
    QObject::connect(qnode, SIGNAL(OILTemperatureSignal(float)), this, SLOT(updateOILTemperatureLCD(float)));
}

/*************************************************************
 * Controls Tab Implementation
/*************************************************************/

/*************************************************************
 * @Name     init
 * @Args     none
 * @function setup up all the signal & slot connections
 *************************************************************/
void system_tab::updateUCDieTemperatureLCD(float value)
{
    value = value*100;
    ui->UCDieTemperatureLCD->display(value);
    if(value > 90)
    {
        // Critical temperature reached!! Display RED
        ui->UCDieTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(191,0,0)}");
    }
    else if(value > 70)
    {
        // Warning temperature reached!! Display ORANGE
        ui->UCDieTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(215,165,0)}");
    }
    else if(value > 10)
    {
        // Running cool~ Display BLUE
        ui->UCDieTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(24,0,176)}");
    }
    else
    {
        // Running cold. Display BLACK
        ui->UCDieTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: black}");
    } 
}

void system_tab::updateOILTemperatureLCD(float value)
{
    value = value*100;
    ui->oilTemperatureLCD->display(value);
    if(value > 90)
    {
        // Critical temperature reached!! Display RED
        ui->oilTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(191,0,0)}");
    }
    else if(value > 70)
    {
        // Warning temperature reached!! Display ORANGE
        ui->oilTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(215,165,0)}");
    }
    else if(value > 10)
    {
        // Running cool~ Display BLUE
        ui->oilTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: rgb(24,0,176)}");
    }
    else
    {
        // Running cold. Display BLACK
        ui->oilTemperatureLCD->setStyleSheet("QLCDNumber{ background-color: black}");
    }
}


