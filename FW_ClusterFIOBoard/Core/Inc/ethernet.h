#ifndef INC_ETHERNET_H_
#define INC_ETHERNET_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>




/**
 * @brief MAC address
 **/

typedef struct
{
   union {
      uint8_t b[6];
      uint16_t w[3];
   };
} MacAddr;



/**
 * @brief Ethernet frame header
 **/

typedef struct
{
   MacAddr destAddr; //0-5
   MacAddr srcAddr;  //6-11
   uint16_t type;    //12-13
   uint8_t data[];   //14
} EthHeader;


/**
 * @brief LLC header
 **/

typedef struct
{
   uint8_t dsap;    //0
   uint8_t ssap;    //1
   uint8_t control; //2
} LlcHeader;


/**
 * @brief VLAN tag
 **/

typedef struct
{
   uint16_t tci;  //0-1
   uint16_t type; //2-3
} VlanTag;





/**
 * @brief MAC filter table entry
 **/

typedef struct
{
   MacAddr addr;    ///<MAC address
   uint8_t refCount; ///<Reference count for the current entry
   bool addFlag;
   bool deleteFlag;
} MacFilterEntry;





void printMacAddrAsBytes(const MacAddr *mac);
void printMacAddrAsWords(const MacAddr *mac);


void macCopyAddr(MacAddr *destMacAddr, const MacAddr *srcMacAddr);
int macCompAddr(const MacAddr *macAddr1, const MacAddr *macAddr2);

#endif /* INC_ETHERNET_H_ */
