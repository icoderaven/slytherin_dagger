/**
 * \file crc32.h
 * This file contains function prototypes for CRC32 related functions
 * 
 * \author Michael Schwerin
 * \date 2007
 */
#ifndef CRC32_H_
#define CRC32_H_

void crc32_build_table();
void crc32_start(unsigned int *crc);
void crc32_update(unsigned char inchar, unsigned int *crc);
void crc32_finish(unsigned int *crc);
unsigned int calcCRC(unsigned char *inStr, int len);

#endif /*CRC32_H_*/
