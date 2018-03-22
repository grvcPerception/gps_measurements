
/** 
 * @file  CksFrame.h
 * @brief Colección de funciones para cálculo del estándar CRC32
 * @author Carlos Amores
 * @date 2013, 2014, 2015
 */

#define CRC32_POLYNOMIAL 0xEDB88320 ///< Constante para cáculo polinomial CRC32

unsigned long CRC32Value(int i);
unsigned long CalculateBlockCRC32(unsigned long ulCount,unsigned char *ucBuffer);
