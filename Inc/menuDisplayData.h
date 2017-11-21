#ifndef MENUDISPLAYDATA_H
#define MENUDISPLAYDATA_H
#include <inttypes.h>

#define RIGHTPAGE	(0x80U * 1)
#define LEFTPAGE	(0x80U * 0)
#define NCOLUMNS 	61
#define NPAGES		4
#define NSIDES		2

struct Page {
	uint8_t column[NCOLUMNS];
};

/*
union ScDisDat {
	struct _pf{
		struct Page dc1[NPAGES];
		struct Page dc2[NPAGES];
	} pageformat;
	uint8_t byteformat[NSIDES * NPAGES * NCOLUMNS];
};
*/



#endif//MENUDISPLAYDATA_H
