/*
 * debug.c
 *
 *  Created on: May 13, 2024
 *      Author: Asus
 */


#include "debug.h"
#include "main.h"



#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stdlib.h"

extern UART_HandleTypeDef huart3;


void DebugPrintf(const char *fmt, ...) {
    static char buffer[256]; // Adjust buffer size as needed
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void DebugPrintBinary(uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
    	DebugPrintf("%d", (byte >> i) & 1);
    }

}



void print_binary(uint8_t num, uint8_t bits) {
    for (int i = bits - 1; i >= 0; i--) {
        DebugPrintf("%u", (num >> i) & 1);
    }
}


void print_ksz8863_static_mac_entry(Ksz8863StaticMacEntry entry) 
{
	DebugPrintf("Ksz8863StaticMacEntry:\n");

    DebugPrintf("  fidH: %u (", entry.fidH);
    print_binary(entry.fidH, 2);
    DebugPrintf(")\n");

    DebugPrintf("  reserved: %u (", entry.reserved);
    print_binary(entry.reserved, 6);
    DebugPrintf(")\n");

    DebugPrintf("  forwardPorts: %u (", entry.forwardPorts);
    print_binary(entry.forwardPorts, 3);
    DebugPrintf(")\n");

    DebugPrintf("  valid: %u (", entry.valid);
    print_binary(entry.valid, 1);
    DebugPrintf(")\n");

    DebugPrintf("  override: %u (", entry.override);
    print_binary(entry.override, 1);
    DebugPrintf(")\n");

    DebugPrintf("  useFid: %u (", entry.useFid);
    print_binary(entry.useFid, 1);
    DebugPrintf(")\n");

    DebugPrintf("  fidL: %u (", entry.fidL);
    print_binary(entry.fidL, 2);
    DebugPrintf(")\n");

    DebugPrintf("\n");
}
