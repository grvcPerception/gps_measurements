/**
  @file PortSerial.cpp
  @brief Implementación de la clase PortSerial
  @author Carlos Amores
  @date 2013,2014,2015
*/

#include "modulo_gps/PortSerial.h"

using namespace std;

/**
 * Constructor de la clase (vacío)
 */
PortSerial::PortSerial()
{

}

/**
 * Método público que realiza la apertura del puerto serie
 * @param[in] nombre Dirección de registro del puerto (/dev/tty...)
 * @return Booleano que indica si la operación se ha realizado con éxito
 */
bool PortSerial::openSerial(char* nombre){
  this->descriptorSerie = open(nombre, O_RDWR | O_NOCTTY | O_NDELAY);
  if (descriptorSerie == -1){
    return false;
  }else{
    return true;
  }
}

/**
 * Método público que cerra la conexión serie
 */
void PortSerial::closeSerial(){
	close(descriptorSerie);
}

/**
 * Método público que limpia el buffer de recepción serie
 */
void PortSerial::clean(){
	tcflush(descriptorSerie, TCIFLUSH);
}

/**
 * Método público que realiza la configuración del puerto serie
 * @param[in] speed Velocidad de apertura del puerto
 */
void PortSerial::configura(int speed){
       fcntl(descriptorSerie, F_SETFL, 0);

       tcgetattr(descriptorSerie,&oldtio);
       newtio.c_cflag = speed | CS8 | CLOCAL | CREAD;

       newtio.c_iflag = IGNPAR ;
       newtio.c_oflag = 0;
       newtio.c_lflag = 0;


       newtio.c_cc[VTIME]= 0;
       newtio.c_cc[VMIN]= 1;

       tcflush(descriptorSerie, TCIFLUSH);
       tcsetattr(descriptorSerie,TCSANOW,&newtio);
}

/**
 * Método público utilizado para la escritura por el canal serie
 * @param[in] data Datos a enviar
 * @param[in] tamano Tamaño de los datos a enviar
 * @return Booleano que indica si la operación se ha realizado con éxito
 */
bool PortSerial::send(char* data, int tamano)
{
	return write(descriptorSerie,data,tamano);
}

/**
 * Método público utilizado para la lectura por el canal serie
 * @param[io] data Puntero a la estructura donde guardar los datos
 * @param[in] tamano Cantidad de bytes a leer
 * @param[in] timeout Timeout que se dispara en la lectura
 * @return Valor entero que devuelve la cantidad de bytes leídos o -1 en caso de
 * que hayan ocurrido errores
 */
int PortSerial::recv(char* data,int tamano, int timeout)
{

	int res;

	struct timeval Timeout;
	fd_set readfs;
	FD_ZERO(&readfs);
	FD_SET(descriptorSerie, &readfs);

	Timeout.tv_usec = 0;  /* milisegundos */
	Timeout.tv_sec  = timeout;  /* segundos */


	if(select(descriptorSerie+1, &readfs, NULL, NULL, &Timeout)>0)
	{
		res = read(descriptorSerie,data,tamano);
		if(res<=0)
			return SERIAL_BAD;
	}
	else
		return SERIAL_TIMEOUT;

	return SERIAL_OK;

}



