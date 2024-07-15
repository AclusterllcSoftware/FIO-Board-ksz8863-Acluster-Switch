#include "ethernet.h"

#include "debug.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stdlib.h"


// Function to print MAC address as bytes
void printMacAddrAsBytes(const MacAddr *mac)
{
    DebugPrintf("MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac->b[0], mac->b[1], mac->b[2], mac->b[3], mac->b[4], mac->b[5]);
}

// Function to print MAC address as words
void printMacAddrAsWords(const MacAddr *mac)
{
	DebugPrintf("MAC as words: %04X:%04X:%04X\n",
           mac->w[0], mac->w[1], mac->w[2]);
}

void macCopyAddr(MacAddr *destMacAddr, const MacAddr *srcMacAddr)
{
    memcpy(destMacAddr, srcMacAddr, sizeof(MacAddr));
}

int macCompAddr(const MacAddr *macAddr1, const MacAddr *macAddr2)
{
    return memcmp(macAddr1, macAddr2, sizeof(MacAddr)) == 0;
}


