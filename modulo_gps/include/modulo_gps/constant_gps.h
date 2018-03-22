
/** 
 * @file  constant_gps.h
 * @brief Colección de constantes necesarias para el tratamiento de GPS+IMU
 * @author Carlos Amores
 * @date 2013, 2014, 2015
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* CONSTANT_H */


// Errores
#define SERIAL_ERROR 0 ///< Indica un error en la gestión del puerto serie
#define FRAME_ERROR  1 ///< Indica un error por trama inválida recibida


// Funcion: ALIGNMENTMODE
#define ALIGNMENTMODE_UNAIDED "UNAIDED" ///< Indica el modo de alineamiento UNAIDED
#define ALIGNMENTMODE_AIDED_STATIC "AIDED_STATIC" ///< Indica el modo de alineamiento AIDED_STATIC
#define ALIGNMENTMODE_AIDED_TRANSFER "AIDED_TRANSFER" ///< Indica el modo de alineamiento AIDED_TRANSFER

//Funcion: APPLYVEHICLEBODYROTATION
#define APPLYVEHICLEBODYROTATION_ENABLE "ENABLE" ///< Indica el modo de configuración de ejes ENABLE
#define APPLYVEHICLEBODYROTATION_DISABLE "DISABLE" ///< Indica el modo de configuración de ejes DISABLE

// Funcion: CANCONFIG
#define CANCONFIG_ENABLE "ENABLE" ///< Indica la opción para configuración de CAN ENABLE
#define CANCONFIG_DISABLE "DISABLE" ///< Indica la opción para configuración de CAN DISABLE
#define CANCONFIG_PORT_CAN1 "CAN1" ///< Indica la opción para configuración de CAN CAN1
#define CANCONFIG_PORT_CAN2 "CAN2" ///< Indica la opción para configuración de CAN CAN2
#define CANCONFIG_SOURCE_GPS "GPS" ///< Indica la opción para configuración de CAN GPS
#define CANCONFIG_SOURCE_INSGPS "INSGPS" ///< Indica la opción para configuración de CAN GPS+IMU


//Funcion: EXTHDGOFFSET
/*Sin constantes definidas*/

// Funcion: FRESET
#define FRESET_STANDARD "STANDARD" ///< Indica la opción para resetear STANDARD
#define FRESET_COMMAND "COMMAND" ///< Indica la opción para resetear COMMAND
#define FRESET_GPSALMANAC "GPSALMANAC" ///< Indica la opción para resetear GPSALMANAC
#define FRESET_GPSEPHEM "GPSEPHEM"   ///< Indica la opción para resetear GPSEPHEM
#define FRESET_MODEL "MODEL"  ///< Indica la opción para resetear MODEL
#define FRESET_CLKCALIBRATION "CLKCALIBRATION"  ///< Indica la opción para resetear CLKCALIBRATION
#define FRESET_SBASALMANAC "SBASALMANAC" ///< Indica la opción para resetear SBASALMANAC
#define FRESET_LAST_POSITION "LAST_POSITION" ///< Indica la opción para resetear LAST_POSITION
#define FRESET_VEHICLE_BODY_R "VEHICLE_BODY_R" ///< Indica la opción para resetear VEHICLE_BODY_R
#define FRESET_INS_LEVER_ARM "INS_LEVER_ARM" ///< Indica la opción para resetear INS_LEVER_ARM

//Funcion: INSCOMMAND
#define INSCOMMAND_ENABLE "ENABLE" ///< Indica la opción para comandos de IMU ENABLE
#define INSCOMMAND_DISABLE "DISABLE" ///< Indica la opción para comandos de IMU DISABLE

//Funcion: INSPHASEUPDATE
#define INSPHASEUPDATE_ENABLE "ENABLE" ///< Indica la opción para la actualización fase a ENABLE
#define INSPHASEUPDATE_DISABLE "DISABLE" ///< Indica la opción para la actualización fase a DISABLE

//Funcion: INSZUPT
/*Sin constantes definidas*/

//Funcion: INSZUPTCONTROL
#define INSZUPTCONTROL_ENABLE "ENABLE" ///< Indica la opción para la actualización del control de la IMU a ENABLE
#define INSZUPTCONTROL_DISABLE "DISABLE" ///< Indica la opción para la actualización del control de la IMU a DISABLE

// Funcion: NMEATALKER
#define NMEATALKER_GP "GP" ///< Indica la opción para NMEATALKER a GP
#define NMEATALKER_AUTO "AUTO" ///< Indica la opción para NMEATALKER a AUTO

//Funcion: RVBCALIBRATE
#define RVBCALIBRATE_ENABLE "ENABLE" ///< Indica la opción para RVBCALIBRATE a ENABLE
#define RVBCALIBRATE_DISABLE "DISABLE" ///< Indica la opción para RVBCALIBRATE a DISABLE
#define RVBCALIBRATE_RESET "RESET" ///< Indica la opción para RVBCALIBRATE a RESET

//Funcion: SETIMUORIENTATION
#define SETIMUORIENTATION_AUTO   "0"  ///< Indica la orientación de la IMU respecto al vehículo automática
#define SETIMUORIENTATION_X_UP   "1" ///< Indica la orientación de la IMU respecto al vehículo con eje X hacia arriba
#define SETIMUORIENTATION_X_DOWN "2" ///< Indica la orientación de la IMU respecto al vehículo con eje X hacia abajo
#define SETIMUORIENTATION_Y_UP   "3" ///< Indica la orientación de la IMU respecto al vehículo con eje Y hacia arriba
#define SETIMUORIENTATION_Y_DOWN "4" ///< Indica la orientación de la IMU respecto al vehículo con eje Y hacia abajo
#define SETIMUORIENTATION_Z_UP   "5" ///< Indica la orientación de la IMU respecto al vehículo con eje Z hacia arriba
#define SETIMUORIENTATION_Z_DOWN "6" ///< Indica la orientación de la IMU respecto al vehículo con eje Z hacia abajo

//Funcion: SETIMUTOANTOFFSET
/*Sin constantes definidas*/

//Funcion: SETIMUTOANTOFFSET2
/*Sin constantes definidas*/

//Funcion: SETINITATTITUDE
/*Sin constantes definidas*/

//Funcion: SETINITAZIMUTH
/*Sin constantes definidas*/

//Funcion: SETINSOFFSET
/*Sin constantes definidas*/

//Funcion: SETMARK1OFFSET
/*Sin constantes definidas*/

//Funcion: SETWHEELPARAMETERS
/*Sin constantes definidas*/

//Funcion: VEHICLEBODYROTATION
/*Sin constantes definidas*/


//Tipo de tramas
#define TT_ERROR -1  ///< Indica tipo de trama de ERRPR
#define TT_INSPVASA 0 ///< Indica tipo de trama de INSPVASA
#define TT_CORRIMUDATASA 1 ///< Indica tipo de trama de CORRIMUDATA
#define TT_BESTGPSPOSA 2 ///< Indica tipo de trama de BESTGPSPOS
#define TT_INSPOSA 3 ///< Indica tipo de trama de INSPOSA
#define TT_HEADINGA 4 ///< Indica tipo de trama de HEADINGA
#define TT_GPSVELA 5 ///< Indica tipo de trama de GPSVELA
#define TT_BESTLEVERARMA 6 ///< Indica tipo de trama de BESTLEVERALARMA
#define TT_INSPVAA 7 ///< Indica tipo de trama de INSPVAA
#define TT_CLOCKMODEL 8 ///< Indica tipo de trama de CLOCKMODEL


// Estados del GPS+IMU / ERRORES

#define GPS_GLOBAL_ERROR 0
#define DATA_RCV_FAILED 1 
#define CONFIG_OPEN_SERIALPORT_ERROR 2
#define CONFIG_CONFIG_SERIALPORT_ERROR 3
#define CONFIG_ALIGNMENT_MODE_ERROR 4
#define CONFIG_INITIAL_AZIMUTH_ERROR 5
#define CONFIG_ANTOFFSET_ERROR 6
#define INS_INACTIVE 7
#define INS_ALIGNING 8
#define INS_SOLUTION_NOT_GOOD 9
#define INS_BAD_GPS_AGREEMENT 10
#define INS_ALIGNMENT_COMPLETE 11
#define INSUFFICIENT_OBS 12
#define NO_CONVERGENCE 13
#define SINGULARITY 14
#define COV_TRACE 15
#define TEST_DIST 16
#define COLD_START 17
#define V_H_LIMIT 18
#define VARIANCE 19
#define RESIDUALS 20
#define DELTA_POS 21
#define NEGATIVE_VAR 22
#define INTEGRITY_WARNING 23
#define IMU_UNPLUGGED 24
#define PENDING 25
#define INVALID_FIX 26
#define UNAUTHORIZED_STATE 27




