/*
 * swd_write.h
 *
 * In debug builds, use dbprintf to print to STM Cube console.
 * In release builds, dbprintf and stdio.h are ommitted to reduce code size.
 * See https://www.youtube.com/watch?v=sPzQ5CniWtw
 *
 * NOTE: This is only availabe on Cortex-M3 and higher, not on M0. BIG SAD!
 *
 *  Created on: May 16, 2021
 *      Author: Grant Geyer
 */

#ifndef __SWD_WRITE_H__
#define __SWD_WRITE_H__

#ifdef DEBUG
	#include <stdio.h>
	#define dbprintf(...) printf(__VA_ARGS__)
#else
	#define dbprintf(...)
#endif

int _write(int file, char *ptr, int len);

#endif /* __SWD_WRITE_H__ */
