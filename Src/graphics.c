#include "graphics.h"
#include "menuScreen.h"
#include "menuDisplayData.h"
#include <inttypes.h>

//uint8_t galphabet[NCHARS][CHARBITW] = {{0x7E, 0x08, 0x08, 0x08, 0x7E, 0x00}};
extern uint8_t galphabet[NCHARS][CHARBITW];


	void putChar(char c, uint8_t * scrptr) {
		int i, g;			// local index variables
		g = c - CHAROFFS;	// index the alphabet

		// put char into character buffer (overwriting)
		for(i = 0; i < CHARBITW; ++i) {
			scrptr[i] |= galphabet[g][i];
		}
	}

	void putHLine(uint8_t hbits, uint8_t len, uint8_t * scrptr) {
		// stack allocations
		int i;
		for(i = 0; i < len; ++i) {
			scrptr[i] |= hbits; 
		}
	}

	void putVLine(uint8_t hstartbit, uint8_t len, uint8_t * scrptr) {
		// stack allocations
		int i, b, d;

		// top-down starting position of line
		switch(hstartbit) {
			case BIT0:	b = 0xFF; i = 8; break;
			case BIT1: 	b = 0x7F; i = 7; break;
			case BIT2:  b = 0x3F; i = 6; break;
			case BIT3:  b = 0x1F; i = 5; break;
			case BIT4:  b = 0x0F; i = 4; break;
			case BIT5:  b = 0x07; i = 3; break;
			case BIT6:  b = 0x03; i = 2; break;
			case BIT7:  b = 0x01; i = 1; break;
			default: b = 0; i = 0;
		}
		
		// len > length()
		if(len >= i) {
			for(i = 0; i < len; ++i) {
				scrptr[i] |= b;
			}

		} else {
			d = i - len;
			switch(d) {
				case 1: b &= (0xFE >> (8 - i)); break;
				case 2: b &= (0xFC >> (8 - i)); break;
				case 3: b &= (0xF8 >> (8 - i)); break;
				case 4: b &= (0xF0 >> (8 - i)); break;
				case 5: b &= (0xE0 >> (8 - i)); break;
				case 6: b &= (0xC0 >> (8 - i)); break;
				case 7: b &= (0x80 >> (8 - i)); break;
			}
		}
	}

	void writePage(uint8_t page[], uint8_t which) {
		// discover what page is being written
		//uint8_t pagenum = (which & 0x07);
		int i;

		// write that page of X side
		if(which & RIGHTPAGE) {
			Comright(MENUSCREEN_COL_ADDR_R + 0x00);
			for(i = 0; i < NCOLUMNS; ++i) {
				Writeright(page[i]);
			}

		} else {// LEFTPAGE
			Comleft(MENUSCREEN_COL_ADDR_R + 0x00);
			for(i = 0; i < NCOLUMNS; ++i) {
				Writeleft(page[i]);
			}
		}
	}
