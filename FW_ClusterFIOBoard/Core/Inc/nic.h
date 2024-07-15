#ifndef INC_NIC_H_
#define INC_NIC_H_

#include "ethernet.h"
#include <stdbool.h>
#include <stdint.h>
/**
 * @brief Link state
 **/

typedef enum
{
   NIC_LINK_STATE_DOWN = 0,
   NIC_LINK_STATE_UP   = 1,
   NIC_LINK_STATE_AUTO = 2
} NicLinkState;


/**
 * @brief Link speed
 **/

typedef enum
{
   NIC_LINK_SPEED_UNKNOWN = 0,
   NIC_LINK_SPEED_10MBPS  = 10000000,
   NIC_LINK_SPEED_100MBPS = 100000000,
   NIC_LINK_SPEED_1GBPS   = 1000000000
} NicLinkSpeed;


/**
 * @brief Duplex mode
 **/

typedef enum
{
   NIC_UNKNOWN_DUPLEX_MODE = 0,
   NIC_HALF_DUPLEX_MODE    = 1,
   NIC_FULL_DUPLEX_MODE    = 2
} NicDuplexMode;


/**
 * @brief Switch port state
 **/

typedef enum
{
   SWITCH_PORT_STATE_UNKNOWN    = 0,
   SWITCH_PORT_STATE_DISABLED   = 1,
   SWITCH_PORT_STATE_BLOCKING   = 2,
   SWITCH_PORT_STATE_LISTENING  = 3,
   SWITCH_PORT_STATE_LEARNING   = 4,
   SWITCH_PORT_STATE_FORWARDING = 5
} SwitchPortState;


/**
 * @brief Forwarding database entry
 **/

typedef struct
{
  MacAddr macAddr;
  uint8_t srcPort;
  uint32_t destPorts;
  bool override;
} SwitchFdbEntry;


/**
 * @brief VLAN entry
 **/

typedef struct
{
   uint16_t vlanId;
   bool valid;
   uint16_t fid;
   uint32_t ports;
} SwitchVlanEntry;

// Define the enum error_t
typedef enum {
   NO_ERROR = 0,
   ERROR_INVALID_ENTRY,
   ERROR_END_OF_TABLE,
	ERROR_TABLE_FULL,
   ERROR_NOT_FOUND_ENTRY
} error_t;



void printSwitchFdbEntry(const SwitchFdbEntry *entry) ;

const char* getErrorMessage(error_t error);
void printErrorMessage(error_t error);

#endif /* INC_NIC_H_ */
