#ifndef __GRAPHICS_H
#define __GRAPHICS_H
#include <inttypes.h>

// structures
typedef struct _point {
	uint8_t x;
	uint8_t y;
} point;


// definitions
#define	NCHARS		96
//#define NCHARS		1
#define CHAROFFS	32
#define CHARBITW	6
#define BIT0		1
#define BIT1		2
#define BIT2		4
#define BIT3		8
#define BIT4		16
#define BIT5		32
#define BIT6		64
#define BIT7		128

#endif//__GRAPHICS_H
