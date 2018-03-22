
/** 
 * @file  PortSerial.h
 * @brief Declara el tipo de la clase "PortSerial"
 * - La clase implementa la comunicación por puerto serie como capa SW que 
 * abstraiga del uso de funciones de bajo nivel
 * @author Carlos Amores
 * @date 2013, 2014, 2015
 */


#ifndef PORT_SERIAL_H
#define	PORT_SERIAL_H

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <unistd.h>

#define SERIAL_OK 0 ///< Indica el correcto estado de la conexión serie
#define SERIAL_BAD 1 ///< Indica un mal funcionamiento del enlace serie
#define SERIAL_TIMEOUT 2 ///< Indica que no se ha recibido nada en un determinado timeout
#define FRAME_FAILED 3 ///< Indica la recepción de una trama inválida via serie

/**
 * \class PortSerial
 * \brief Clase que representa una capa de alto nivel para la gestión de puerto serie
 */
class PortSerial {
private:
  int descriptorSerie;
  struct termios oldtio, newtio;
public:
  PortSerial();
  bool send(char* bufferOut, int tam);
  int recv(char* bufferIn, int tam, int timeout);
  bool openSerial(char* nombre);
  void closeSerial();
  void clean();
  void configura(int velocity);
};

#endif
