/*
 * debug.h
 *
 *  Created on: May 13, 2024
 *      Author: Asus
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_
#include <stdint.h>

#include "ksz8863_driver.h"

void DebugPrintf(const char *fmt, ...);
void DebugPrintBinary(uint8_t byte);
void print_ksz8863_static_mac_entry(Ksz8863StaticMacEntry entry) ;

#endif /* INC_DEBUG_H_ */
