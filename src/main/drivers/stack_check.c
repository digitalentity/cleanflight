/*
 * stack_check.c
 *
 *  Created on: 23 Aug 2016
 *      Author: martinbudden
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef STACK_CHECK

#include "build/debug.h"

#define STACK_FILL_CHAR 0xa5

extern char _estack; // end of stack, declared in .LD file
extern char _Min_Stack_Size; // declared in .LD file

/*
 * The ARM processor uses a full descending stack. This means the stack pointer holds the address
 * of the last stacked item in memory. When the processor pushes a new item onto the stack,
 * it decrements the stack pointer and then writes the item to the new memory location.
 *
 *
 * RAM layout is generally as below, although some targets vary
 *
 * F1 Boards
 * RAM is origin 0x20000000 length 20K that is:
 * 0x20000000 to 0x20005000
 *
 * F3 Boards
 * RAM is origin 0x20000000 length 40K that is:
 * 0x20000000 to 0x2000a000
 *
 * F4 Boards
 * RAM is origin 0x20000000 length 128K that is:
 * 0x20000000 to 0x20020000
 *
 */
void taskStackCheck(void)
{
    char * const stackHighMem = &_estack;
    const uint32_t stackSize = (uint32_t)&_Min_Stack_Size;
    char * const stackLowMem = stackHighMem - stackSize;
    const char * const stackCurrent = (char *)&stackLowMem;

    char *p;
	for (p = stackLowMem; p < stackCurrent; ++p) {
		if (*p != STACK_FILL_CHAR) {
			break;
		}
	}
#ifdef DEBUG_STACK
	debug[0] = (uint32_t)stackHighMem & 0xffff;
	debug[1] = (uint32_t)stackLowMem & 0xffff;
	debug[2] = (uint32_t)stackCurrent & 0xffff;
	debug[3] = (uint32_t)p & 0xffff; // watermark
#endif
}
#endif
