/**
 * This file implements a 32-bit CRC algorithm based on lookup tables for speed.
 * The polynomial used is the same as in MPEG2 and various network standards (0x04C11DB7)
 *
 * The calling code must insure that all parameters are valid as these functions
 * do not do bounds checking on the passed parameters for the sake of speed
 *
 * \file crc32.c
 * \author Michael Schwerin
 */
#include "crc32.h"

/// storage location for the crc lookup table
static unsigned int crc32tbl[256];
/// polynomial used in MPEG2, IEEE 802.3, V.42
static const unsigned int crc32poly = 0x04C11DB7;

/**
 * Perform a complete CRC computation over a given length of data including the
 * initialization and finishing procedures.
 * \param inStr Points to the data over which the CRC will be calculated (no NULL checking)
 * \param len The number of bytes over which the CRC is to be calculated
 * \return The value which was calculated
 */
unsigned int calcCRC(unsigned char *inStr, int len) {
	unsigned int workCRC;
	int j;
	crc32_start(&workCRC);
	for(j = 0; j < len; j++) {
		crc32_update(inStr[j], &workCRC);
	}
	crc32_finish(&workCRC);
	return workCRC;
}

/**
 * initialize the table of crc values for each of the 256 character values
 */
void crc32_build_table() {
	unsigned int i, j, work;
	
	for(i = 0; i <= 0xFF; i++) {
		// first do a bit reflection of the working value and store the result in the 8 MSBs of the crc32tbl entry
		// xxxx xxxx xxxx xxxx xxxx xxxx abcd efgh -> hgfe dcba 0000 0000 0000 0000 0000 0000
		work = i;
		crc32tbl[i] = 0;   // zero the result location since we only use != to update it
		for(j = 0; j < 8; j++) {
			if(work & 1)   // if lsb is set then set the corresponding bit in the result
				crc32tbl[i] |= 1 << (31-j);
			work >>= 1;
		}
		
		// now apply the polynomial
		for(j = 0; j < 8; j++)
			// if MSB is set then crc32tbl[i] = (crc32tbl[i] << 1) XOR crc32poly
			//          otherwise crc32tbl[i] = (crc32tbl[i] << 1)
			crc32tbl[i] = (crc32tbl[i] << 1) ^ (crc32tbl[i] & (1<<31) ? crc32poly : 0);
		
		// now reflect the full 32 bit result
		work = crc32tbl[i];
		crc32tbl[i] = 0;   // zero the result location since we only use != to update it
		for(j = 0; j < 32; j++) {
			if(work & 1)   // if lsb is set then set the corresponding bit in the result
				crc32tbl[i] |= 1 << (31-j);
			work >>= 1;    // shift the next highest bit into the lsb
		}
	}
}

/**
 * perform the per block crc calculation initialization (just initializes the crc to 0xFFFFFFFF)
 * \param crc a pointer to the location of the working crc value (not NULL tested)
 */
void crc32_start(unsigned int *crc) {
	*crc = 0xFFFFFFFF;
}

/**
 * update the working crc value based on the passed character
 * \param inchar the character to compute the crc update on
 * \param crc a pointer to the location of the working crc value (not NULL tested)
 */
void crc32_update(unsigned char inchar, unsigned int *crc) {
 	*crc = (*crc >> 8) ^ crc32tbl[(*crc & 0xFF) ^ inchar];
}
 
/**
 * perform the per block crc calculation finishing steps (just XOR the crc with 0xFFFFFFFF)
 * This structure makes it easier to drop in a replacement crc algorithm
 * \param crc a pointer to the location ofthe working crc value (not NULL tested)
 */
void crc32_finish(unsigned int *crc) {
	*crc ^= 0xFFFFFFFF;
}
