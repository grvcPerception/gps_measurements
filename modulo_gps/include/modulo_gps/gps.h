
/** 
 * @file  gps.h
 * @brief Fichero de cabecera para funciones de gestióin del GPS+IMU
 * @author Carlos Amores
 * @date 2013, 2014, 2015
 */
#ifndef GPS_H
#define	GPS_H


// Librerias 
#include <iostream>
#include <time.h>
#include <stdlib.h> 
#include <stdio.h>
// ROS
#include "ros/ros.h"
#include "modulo_gps/constant_gps.h"
#include "modulo_gps/GPS_Management.h"
#include "sensor_msgs/NavSatFix.h"



/**
 * \struct insData
 * \brief Estructura para almacenamiento de información recibida del GPS+IMU
 */
typedef struct{
    float latitude;
    float longitude;
    float altitude;
    float roll;
    float pitch;
    float yaw;
}insData;


bool exitModule, readyToPublish, teachActive;
insData insdata;
bool connect();
bool disconnect();
bool configure();
bool sendData();
bool recvData();
bool isAlive();
bool checkStateGPS();
void initModuleVariables();

#endif	/* GPS_H */

