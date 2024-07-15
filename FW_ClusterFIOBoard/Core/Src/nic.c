#include "nic.h"

#include "ethernet.h"


#include "debug.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stdlib.h"


// Function to print SwitchFdbEntry
void printSwitchFdbEntry(const SwitchFdbEntry *entry) 
{
	DebugPrintf("Switch FDB Entry:\n");
    DebugPrintf("  MAC Address: ");
    printMacAddrAsBytes(&entry->macAddr);
    DebugPrintf("\n");
    DebugPrintf("  Source Port: %u\n", entry->srcPort);
    DebugPrintf("  Destination Ports: 0x%08X\n", entry->destPorts);
    DebugPrintf("  Override: %s\n", entry->override ? "true" : "false");
}


// Function to get the error message string
const char* getErrorMessage(error_t error) 
{
    switch (error) {
        case NO_ERROR:
            return "No error.";
        case ERROR_INVALID_ENTRY:
            return "Invalid entry.";
        case ERROR_END_OF_TABLE:
            return "End of table.";
        default:
            return "Unknown error.";
    }
}

// Function to print the error message
void printErrorMessage(error_t error) {
    printf("Error: %s\n", getErrorMessage(error));
}
