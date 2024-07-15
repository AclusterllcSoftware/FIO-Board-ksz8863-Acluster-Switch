

//Dependencies


#include "ksz8863_driver.h"
#include "ksz8863.h"
#include "nic.h"



// Tail tag rules (host to KSZ8863)


const uint8_t ksz8863IngressTailTag[3] =
{
   KSZ8863_TAIL_TAG_NORMAL_ADDR_LOOKUP,
   KSZ8863_TAIL_TAG_DEST_PORT1,
   KSZ8863_TAIL_TAG_DEST_PORT2
};


// KSZ8863 Ethernet switch initialization


error_t ksz8863Init()
{
  uint8_t port;

  uint8_t temp;

  //Debug message
  DebugPrintf("Initializing KSZ8863...\r\n");



  //Wait for the interface to be ready
  do
  {
     //Read CHIP_ID0 register
     temp = ksz8863ReadSwitchReg(KSZ8863_CHIP_ID0);

     //The returned data is invalid until the serial interface is ready
  } while(temp != KSZ8863_CHIP_ID0_FAMILY_ID_DEFAULT);

  //Enable tail tag feature
  temp = ksz8863ReadSwitchReg(KSZ8863_GLOBAL_CTRL1);
  temp |= KSZ8863_GLOBAL_CTRL1_TAIL_TAG_EN;
  ksz8863WriteSwitchReg(KSZ8863_GLOBAL_CTRL1, temp);

  //Loop through the ports
  for(port = KSZ8863_PORT1; port <= KSZ8863_PORT2; port++)
  {

   //Enable transmission, reception and address learning
   ksz8863SetPortState(port, SWITCH_PORT_STATE_FORWARDING);

  }
   ksz8863SetPowerManagementMode(NORMAL_MODE);
   ksz8863SetCpuClockSelection(CLOCK_125_MHZ);
   ksz8863EnablePassAllFrame(false);
   ksz8863EnableAging(true);
   
   for (uint8_t port = 1; port <=2 ; port++)
   {
      ksz8863PowerUpPort(port);
      ksz8863EnableAutoNegotiationPort(port);
      ksz8863SetForceLinkSpeed(port, NIC_LINK_SPEED_100MBPS); // set linkspeed
      ksz8863SetForcedDuplexMode(port, true); // set full duplex mode
      ksz8863TurnOnPortLed(port);
      ksz8863EnableFlowControlAdvertisementOnPort(port, true);
      ksz8863EnableAdvertise100BTFullDuplexCapabilityOnPort(port, true);
      ksz8863EnableAdvertise100BTHalfDuplexCapabilityOnPort(port, true);
      ksz8863EnableAdvertise10BTFullDuplexCapabilityOnPort(port, true);
      ksz8863EnableAdvertise10BTHalfDuplexCapabilityOnPort(port, true);
   }

  // Dump switch registers for debugging purpose
  // ksz8863DumpSwitchReg();

   ksz8863SwitchStart();

  //Successful initialization
  return NO_ERROR;
}



// Enable interrupts


void ksz8863EnableIrq()
{
   // will be implemented if required
}


// Disable interrupts


void ksz8863DisableIrq()
{
   // will be implemented if required
}

// enable pass all frame (for debugging)

void ksz8863EnablePassAllFrame(bool flag)
{
	uint8_t temp;
   // read register
   temp = ksz8863ReadSwitchReg(KSZ8863_GLOBAL_CTRL1);

   if(flag)
   {
      temp |= KSZ8863_GLOBAL_CTRL1_PASS_ALL_FRAMES;
   }
   else
   {
      temp &= ~KSZ8863_GLOBAL_CTRL1_PASS_ALL_FRAMES;
   }

   ksz8863WriteSwitchReg(KSZ8863_GLOBAL_CTRL1, temp);
}

//*****************  aging related features ****************************//

// enable or disable age function in chip

void ksz8863EnableAging(bool flag)
{
	uint8_t temp;
   // read register
   temp = ksz8863ReadSwitchReg(KSZ8863_GLOBAL_CTRL1);

   if(flag)
   {
      temp |= KSZ8863_GLOBAL_CTRL1_AGING_EN;
   }
   else
   {
      temp &= ~KSZ8863_GLOBAL_CTRL1_AGING_EN;
   }

   ksz8863WriteSwitchReg(KSZ8863_GLOBAL_CTRL1, temp);
}



// enable or disable fast aging (800us) function in chip

void ksz8863EnableFastAging(bool flag)
{
	uint8_t temp;
   // read register
   temp = ksz8863ReadSwitchReg(KSZ8863_GLOBAL_CTRL1);

   if(flag)
   {
      temp |= KSZ8863_GLOBAL_CTRL1_FAST_AGE_EN;
   }
   else
   {
      temp &= ~KSZ8863_GLOBAL_CTRL1_FAST_AGE_EN;
   }

   ksz8863WriteSwitchReg(KSZ8863_GLOBAL_CTRL1, temp);
}


//********************************************************************* */
//// tail tag code
//// Add tail tag to Ethernet frame
//// Decode tail tag from incoming Ethernet frame
//********************************************************************* */

// turn on switch operation

void ksz8863SwitchStart()
{
   uint8_t temp;

   // read register
   temp = ksz8863ReadSwitchReg(KSZ8863_CHIP_ID1);
   DebugPrintBinary(temp);
   DebugPrintf("\n");
   //
   temp |= KSZ8863_CHIP_ID1_START_SWITCH;
   ksz8863WriteSwitchReg(KSZ8863_CHIP_ID1, temp);

   DebugPrintBinary(temp);
   DebugPrintf("\n");

   DebugPrintf("start switch\n");
}

// turn off switch operation 


void ksz8863SwitchStop()
{
   uint8_t temp;

   // read register
   temp = ksz8863ReadSwitchReg(KSZ8863_CHIP_ID1);

   //
   temp &=  ~ KSZ8863_CHIP_ID1_START_SWITCH;
   ksz8863WriteSwitchReg(KSZ8863_CHIP_ID1, temp);
}


// Function to set the CPU interface clock selection
void ksz8863SetCpuClockSelection(CpuClockSelection clockSelection) 
{
   uint8_t temp;

   temp = ksz8863ReadSwitchReg(KSZ8863_GLOBAL_CTRL9);

    // Clear the bits 6:7 (masking with 0x3F)
    temp &= ~KSZ8863_GLOBAL_CTRL9_CPU_IF_CLK_SEL;

    // Set the new clock selection value (shifting clockSelection into bits 6:7)
    temp |= (clockSelection << 6);

    // Write the updated value back to the register
    ksz8863WriteSwitchReg(KSZ8863_GLOBAL_CTRL9, temp);
}

///************** All about link******************** */

// Get link state


bool ksz8863GetLinkState(uint8_t port)
{
   uint16_t status;
   bool linkState;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {

	 //Read port status 0 register
	 status = ksz8863ReadSwitchReg(KSZ8863_PORTn_STAT0(port));

	 //Retrieve current link state
	 linkState = (status & KSZ8863_PORTn_STAT0_LINK_GOOD) ? true : false;


   }
   else
   {
      //The specified port number is not valid
      linkState = false;
   }

   //Return link status
   DebugPrintf("link state of port %d : %d\n", port, linkState);
   return linkState;
}




// get link speed 

uint32_t ksz8863GetLinkSpeed(uint8_t port)
{
   uint16_t status;
   uint32_t linkSpeed;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      //Read port status 1 register
      status = ksz8863ReadSwitchReg(KSZ8863_PORTn_STAT1(port));

      //Retrieve current link speed
      if((status & KSZ8863_PORTn_STAT1_OP_SPEED) != 0)
      {
         linkSpeed = NIC_LINK_SPEED_100MBPS;
      }
      else
      {
         linkSpeed = NIC_LINK_SPEED_10MBPS;
      }
   }
   else if(port == KSZ8863_PORT3)
   {
      //Read global control 4 register
      status = ksz8863ReadSwitchReg(KSZ8863_GLOBAL_CTRL4);

      //Retrieve host interface speed
      if((status & KSZ8863_GLOBAL_CTRL4_SW_MII_10BT) != 0)
      {
         linkSpeed = NIC_LINK_SPEED_10MBPS;
      }
      else
      {
         linkSpeed = NIC_LINK_SPEED_100MBPS;
      }
   }
   else
   {
      //The specified port number is not valid
      linkSpeed = NIC_LINK_SPEED_UNKNOWN;
   }
   DebugPrintf("link speed %d\n", linkSpeed);
   //Return link status
   return linkSpeed;
}


// Get duplex mode

NicDuplexMode ksz8863GetDuplexMode(uint8_t port)
{
   uint16_t status;
   NicDuplexMode duplexMode;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      //Read port status 1 register
      status = ksz8863ReadSwitchReg(KSZ8863_PORTn_STAT1(port));

      //Retrieve current duplex mode
      if((status & KSZ8863_PORTn_STAT1_OP_DUPLEX) != 0)
      {
         duplexMode = NIC_FULL_DUPLEX_MODE;
      }
      else
      {
         duplexMode = NIC_HALF_DUPLEX_MODE;
      }
   }
   else if(port == KSZ8863_PORT3)
   {
      //Read global control 4 register
      status = ksz8863ReadSwitchReg(KSZ8863_GLOBAL_CTRL4);

      //Retrieve host interface duplex mode
      if((status & KSZ8863_GLOBAL_CTRL4_SW_MII_HALF_DUPLEX_MODE) != 0)
      {
         duplexMode = NIC_HALF_DUPLEX_MODE;
      }
      else
      {
         duplexMode = NIC_FULL_DUPLEX_MODE;
      }
   }
   else
   {
      //The specified port number is not valid
      duplexMode = NIC_UNKNOWN_DUPLEX_MODE;
   }

   DebugPrintf("duplexMode port: %d --> %d\n", port, duplexMode);

   //Return duplex mode
   return duplexMode;
}


// Set port state

void ksz8863SetPortState(uint8_t port, SwitchPortState state)
{
   uint8_t temp;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      //Read port control 2 register
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL2(port));

      //Update port state
      switch(state)
      {
      //Listening state
      case SWITCH_PORT_STATE_LISTENING:
         temp &= ~KSZ8863_PORTn_CTRL2_TRANSMIT_EN;
         temp |= KSZ8863_PORTn_CTRL2_RECEIVE_EN;
         temp |= KSZ8863_PORTn_CTRL2_LEARNING_DIS;
         break;

      //Learning state
      case SWITCH_PORT_STATE_LEARNING:
         temp &= ~KSZ8863_PORTn_CTRL2_TRANSMIT_EN;
         temp &= ~KSZ8863_PORTn_CTRL2_RECEIVE_EN;
         temp &= ~KSZ8863_PORTn_CTRL2_LEARNING_DIS;
         break;

      //Forwarding state
      case SWITCH_PORT_STATE_FORWARDING:
         temp |= KSZ8863_PORTn_CTRL2_TRANSMIT_EN;
         temp |= KSZ8863_PORTn_CTRL2_RECEIVE_EN;
         temp &= ~KSZ8863_PORTn_CTRL2_LEARNING_DIS;
         break;

      //Disabled state
      default:
         temp &= ~KSZ8863_PORTn_CTRL2_TRANSMIT_EN;
         temp &= ~KSZ8863_PORTn_CTRL2_RECEIVE_EN;
         temp |= KSZ8863_PORTn_CTRL2_LEARNING_DIS;
         break;
      }

      //Write the value back to port control 2 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL2(port), temp);
   }
}


// Get port state

SwitchPortState ksz8863GetPortState( uint8_t port)
{
   uint8_t temp;
   SwitchPortState state;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      //Read port control 2 register
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL2(port));

      //Check port state
      if((temp & KSZ8863_PORTn_CTRL2_TRANSMIT_EN) == 0 &&
         (temp & KSZ8863_PORTn_CTRL2_RECEIVE_EN) == 0 &&
         (temp & KSZ8863_PORTn_CTRL2_LEARNING_DIS) != 0)
      {
         //Disabled state
         state = SWITCH_PORT_STATE_DISABLED;
      }
      else if((temp & KSZ8863_PORTn_CTRL2_TRANSMIT_EN) == 0 &&
         (temp & KSZ8863_PORTn_CTRL2_RECEIVE_EN) != 0 &&
         (temp & KSZ8863_PORTn_CTRL2_LEARNING_DIS) != 0)
      {
         //Listening state
         state = SWITCH_PORT_STATE_LISTENING;
      }
      else if((temp & KSZ8863_PORTn_CTRL2_TRANSMIT_EN) == 0 &&
         (temp & KSZ8863_PORTn_CTRL2_RECEIVE_EN) == 0 &&
         (temp & KSZ8863_PORTn_CTRL2_LEARNING_DIS) == 0)
      {
         //Learning state
         state = SWITCH_PORT_STATE_LEARNING;
      }
      else if((temp & KSZ8863_PORTn_CTRL2_TRANSMIT_EN) != 0 &&
         (temp & KSZ8863_PORTn_CTRL2_RECEIVE_EN) != 0 &&
         (temp & KSZ8863_PORTn_CTRL2_LEARNING_DIS) == 0)
      {
         //Forwarding state
         state = SWITCH_PORT_STATE_FORWARDING;
      }
      else
      {
         //Unknown state
         state = SWITCH_PORT_STATE_UNKNOWN;
      }
   }
   else
   {
      //The specified port number is not valid
      state = SWITCH_PORT_STATE_DISABLED;
   }

   DebugPrintf("port %d state : %d\n", port, state);
   //Return port state
   return state;
}


// check if the port auto negotiation completed or not

bool ksz8863IsAutoNegotiationCompletedOnPort(uint8_t port)
{
   uint16_t status;
   bool ANCompleted;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {

	 //Read port status 0 register
	 status = ksz8863ReadSwitchReg(KSZ8863_PORTn_STAT0(port));

	 //Retrieve current link state
	 ANCompleted = (status & KSZ8863_PORTn_STAT0_AN_DONE) ? true : false;


   }
   else
   {
      //The specified port number is not valid
      ANCompleted = false;
   }

   //Return link status
   DebugPrintf("In port %d AN is  %d \n", port, ANCompleted);
   return ANCompleted;
}






//// IGMP MLD

/// ******************* static and dynamic MAC Table Database ******************************///

// Add a new entry to the static MAC table

error_t ksz8863AddStaticFdbEntry(const SwitchFdbEntry *entry)
{
   error_t error;
   uint8_t i;
   uint8_t j;
   uint8_t *p;
   SwitchFdbEntry currentEntry;
   Ksz8863StaticMacEntry newEntry;

   DebugPrintf("add static Table entry\n");

   //Keep track of the first free entry
   j = KSZ8863_STATIC_MAC_TABLE_SIZE;

   //Loop through the static MAC table
   for(i = 0; i < KSZ8863_STATIC_MAC_TABLE_SIZE; i++)
   {
      //Read current entry
      error = ksz8863GetStaticFdbEntry(i, &currentEntry);

      //Valid entry?
      if(!error)
      {
         //Check whether the table already contains the specified MAC address
         if(macCompAddr(&currentEntry.macAddr, &entry->macAddr))
         {
            j = i;
            break;
         }
      }
      else
      {
         //Keep track of the first free entry
         if(j == KSZ8863_STATIC_MAC_TABLE_SIZE)
         {
            j = i;
         }
      }
   }

   DebugPrintf("keep track of j: %d\n", j);

   //Any entry available?
   if(j < KSZ8863_STATIC_MAC_TABLE_SIZE)
   {
      // Format MAC entry
      newEntry.reserved = 0;
      newEntry.fidH = 0;
      newEntry.fidL = 0;
      newEntry.useFid = 0;
      newEntry.override = entry->override;
      newEntry.valid = 1;
      newEntry.macAddr = entry->macAddr;

/*
      // for testing
      newEntry.reserved = 0x3F;
      newEntry.fidH = 0x3;
      newEntry.fidL = 0x3;
      newEntry.useFid = 0;
      newEntry.override = 1;
      newEntry.valid = 0;
      newEntry.macAddr = entry->macAddr;
      newEntry.forwardPorts =  7 & KSZ8863_PORT_MASK;
*/
/************* test case  ****************
       111111 11
       11 0 1 0  111

      000000 10
      10 0 0 0 011

       000111 01
       00 1 0 1 010
******************************************* */
      //Set the relevant forward ports
      
      newEntry.forwardPorts = entry->destPorts & KSZ8863_PORT_MASK;
      

      //Point to the MAC entry
      p = (uint8_t *) &newEntry;

      //Write indirect data registers
      for(i = 0; i < sizeof(Ksz8863StaticMacEntry); i++)
      {
         DebugPrintBinary(p[i]);
         DebugPrintf("\n");
         ksz8863WriteSwitchReg(KSZ8863_INDIRECT_DATA7 + i, p[i]);
      }

      //Select the static MAC address table
      ksz8863WriteSwitchReg(KSZ8863_INDIRECT_CTRL0,
         KSZ8863_INDIRECT_CTRL0_WRITE |
         KSZ8863_INDIRECT_CTRL0_TABLE_SEL_STATIC_MAC);

      //Trigger the write operation
      ksz8863WriteSwitchReg(KSZ8863_INDIRECT_CTRL1, j);
      // ksz8863WriteSwitchReg(KSZ8863_INDIRECT_CTRL1, 0);

      //Successful processing
      error = NO_ERROR;
      DebugPrintf("Successful processing entry\n");
  }
  else
  {
     //The static MAC table is full
     error = ERROR_TABLE_FULL;
     DebugPrintf("Table full\n");
  }

   //Return status code
   return error;
}


// Remove an entry from the static MAC table


error_t ksz8863DeleteStaticFdbEntry(const SwitchFdbEntry *entry)
{
  error_t error;
  uint16_t i;
  uint16_t j;
  SwitchFdbEntry currentEntry;

  //Loop through the static MAC table
  for(j = 0; j < KSZ8863_STATIC_MAC_TABLE_SIZE; j++)
  {
     //Read current entry
     error = ksz8863GetStaticFdbEntry( j, &currentEntry);

     //Valid entry?
     if(!error)
     {
        //Check whether the table contains the specified MAC address
        if(macCompAddr(&currentEntry.macAddr, &entry->macAddr))
        {
           break;
        }
     }
  }

  //Any matching entry?
  if(j < KSZ8863_STATIC_MAC_TABLE_SIZE)
  {
     //Clear indirect data registers
     for(i = 0; i < sizeof(Ksz8863StaticMacEntry); i++)
     {
        ksz8863WriteSwitchReg( KSZ8863_INDIRECT_DATA7 + i, 0);
     }

     //Select the static MAC address table
     ksz8863WriteSwitchReg( KSZ8863_INDIRECT_CTRL0, KSZ8863_INDIRECT_CTRL0_WRITE |
        KSZ8863_INDIRECT_CTRL0_TABLE_SEL_STATIC_MAC);

     //Trigger the write operation
     ksz8863WriteSwitchReg(KSZ8863_INDIRECT_CTRL1, j);

     //Successful processing
     DebugPrintf("Successfully deleted entry\n");
     error = NO_ERROR;
  }
  else
  {
     //The static MAC table does not contain the specified address
     DebugPrintf("static MAC table does not contain the specified address\n");
     error = ERROR_NOT_FOUND_ENTRY;
  }

  //Return status code
  return error;
}


// Read an entry from the static MAC table


error_t ksz8863GetStaticFdbEntry(uint16_t index, SwitchFdbEntry *entry)
{
  error_t error;
  uint16_t i;
  uint8_t *p;
  Ksz8863StaticMacEntry currentEntry;
  DebugPrintf("staticFdb index: %d\n", index);

  //Check index parameter
  if(index < KSZ8863_STATIC_MAC_TABLE_SIZE)
  {
     //Select the static MAC address table
     ksz8863WriteSwitchReg(KSZ8863_INDIRECT_CTRL0, KSZ8863_INDIRECT_CTRL0_READ | KSZ8863_INDIRECT_CTRL0_TABLE_SEL_STATIC_MAC);

     //Trigger the read operation
     ksz8863WriteSwitchReg(KSZ8863_INDIRECT_CTRL1, index);

     //Point to the MAC entry
     p = (uint8_t *) &currentEntry;

   //   DebugPrintf("size of mac entry %d\n", sizeof(Ksz8863StaticMacEntry));

     //Read indirect data registers
      DebugPrintf("static table\n" );

     for(i = 0; i < sizeof(Ksz8863StaticMacEntry); i++)
     {
        p[i] = ksz8863ReadSwitchReg(KSZ8863_INDIRECT_DATA7 + i);
      //  DebugPrintf("Reg: %d -- %02X\n", i, p[i] );
      //   if(i < 2)
      //   {
      //   DebugPrintBinary(p[i]);
      //   DebugPrintf("\n");
      //   }
     }

//     print_ksz8863_static_mac_entry(currentEntry);

     DebugPrintf("\nerror of table:: %d\n", error);
    //Valid entry?
    if(currentEntry.valid)
    {
       //Copy MAC entry
       entry->macAddr = currentEntry.macAddr;
       entry->srcPort = 0;
       entry->destPorts = currentEntry.forwardPorts & KSZ8863_PORT_MASK;
       entry->override = currentEntry.override;

       //Successful processing
       printSwitchFdbEntry(entry);
       error = NO_ERROR;
       DebugPrintf("valid entry\n");
    }
    else
    {
       //The entry is not valid
       error = ERROR_INVALID_ENTRY;
       DebugPrintf("Invalid entry\n");
    }
  }
  else
  {
     //The end of the table has been reached
     error = ERROR_END_OF_TABLE;
     DebugPrintf("error end for the table\n");
  }

   printErrorMessage(error);

  //Return status code
  return error;
}


// Flush static MAC table


void ksz8863FlushStaticFdbTable()
{
  uint8_t i;
  uint8_t temp;
  uint8_t state[3];

  //Loop through the ports
  for(i = KSZ8863_PORT1; i <= KSZ8863_PORT3; i++)
  {
     //Save the current state of the port
     state[i - 1] = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL2(i));

     //Turn off learning capability
     ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL2(i), state[i - 1] | KSZ8863_PORTn_CTRL2_LEARNING_DIS);
  }

  //All the entries associated with a port that has its learning capability
  //being turned off will be flushed
  temp = ksz8863ReadSwitchReg(KSZ8863_GLOBAL_CTRL0);
  temp |= KSZ8863_GLOBAL_CTRL0_FLUSH_STATIC_MAC_TABLE;
  ksz8863WriteSwitchReg(KSZ8863_GLOBAL_CTRL0, temp);

  //Loop through the ports
  for(i = KSZ8863_PORT1; i <= KSZ8863_PORT3; i++)
  {
     //Restore the original state of the port
     ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL2(i), state[i - 1]);
  }
}



//  Read an entry from the dynamic MAC table
// TODO: SINGLE CALL

error_t ksz8863GetDynamicFdbEntry(uint16_t index, SwitchFdbEntry *entry)
{
  error_t error;
  uint16_t i;
  uint16_t n;
  uint8_t *p;
  Ksz8863DynamicMacEntry currentEntry;

  //Check index parameter
  if(index < KSZ8863_DYNAMIC_MAC_TABLE_SIZE)
  {
     //Read the MAC entry at the specified index
     do
     {
        //Select the dynamic MAC address table
        ksz8863WriteSwitchReg(KSZ8863_INDIRECT_CTRL0,
           KSZ8863_INDIRECT_CTRL0_READ |
           KSZ8863_INDIRECT_CTRL0_TABLE_SEL_DYNAMIC_MAC |
           (MSB(index) & KSZ8863_INDIRECT_CTRL0_ADDR_H));

        //Trigger the read operation
        ksz8863WriteSwitchReg(KSZ8863_INDIRECT_CTRL1, LSB(index));

        //Point to the MAC entry
        p = (uint8_t *) &currentEntry;

        //Read indirect data registers
        for(i = 0; i < sizeof(Ksz8863DynamicMacEntry); i++)
        {
           p[i] = ksz8863ReadSwitchReg(KSZ8863_INDIRECT_DATA8 + i);
        }

        //Retry until the entry is ready
     } while(currentEntry.dataNotReady);

     //Check whether there are valid entries in the table
     if(!currentEntry.macEmpty)
     {
        //Retrieve the number of valid entries
        n = ((currentEntry.numValidEntriesH << 8) | currentEntry.numValidEntriesL) + 1;
        DebugPrintf("number of valid entry in dynamic Table: %d\n", n);
     }
     else
     {
        //The table is empty
        DebugPrintf("no entry found..table is empty\n");
        n = 0;
     }

     //Valid entry?
     if(index < n)
     {
        //Copy MAC entry
        entry->macAddr = currentEntry.macAddr;
        entry->srcPort = currentEntry.sourcePort + 1;
        entry->destPorts = 0;
        entry->override = false;

        //Successful processing
        DebugPrintf("entry found in dynamic table......\n");
        error = NO_ERROR;
     }
     else
     {
        //The end of the table has been reached
        DebugPrintf("The end of the table has been reached\n");
        error = ERROR_END_OF_TABLE;
     }
  }
  else
  {
     //The end of the table has been reached
     DebugPrintf("The end of the table has been reached\n");
     error = ERROR_END_OF_TABLE;
  }

  //Return status code
  return error;
}



// Flush dynamic MAC table

void ksz8863FlushDynamicFdbTable(uint8_t port)
{
  uint8_t i;
  uint8_t temp;
  uint8_t state[3];

  //Loop through the ports
  for(i = KSZ8863_PORT1; i <= KSZ8863_PORT3; i++)
  {
     //Matching port number?
     if(i == port || port == 0)
     {
        //Save the current state of the port
        state[i - 1] = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL2(i));

        //Turn off learning capability
        ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL2(i), state[i - 1] | KSZ8863_PORTn_CTRL2_LEARNING_DIS);
     }
  }

  //All the entries associated with a port that has its learning capability
  //being turned off will be flushed
  temp = ksz8863ReadSwitchReg(KSZ8863_GLOBAL_CTRL0);
  temp |= KSZ8863_GLOBAL_CTRL0_FLUSH_DYNAMIC_MAC_TABLE;
  ksz8863WriteSwitchReg(KSZ8863_GLOBAL_CTRL0, temp);

  //Loop through the ports
  for(i = KSZ8863_PORT1; i <= KSZ8863_PORT3; i++)
  {
     //Matching port number?
     if(i == port || port == 0)
     {
        //Restore the original state of the port
        ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL2(i), state[i - 1]);
     }
  }
}


// Set forward ports for unknown multicast packets


void ksz8863SetUnknownMcastFwdPorts(uint8_t enable, uint32_t forwardPorts)
{
   //Not implemented
}



/// ******************** Basic Register Read & write operation **************************************//


//  Write switch register

void ksz8863WriteSwitchReg(uint8_t address, uint8_t data)
{

	uint8_t tData[3];
	uint8_t rData[1];
	tData[0] = KSZ8863_SPI_CMD_WRITE;
	tData[1] = address;
	tData[2] = data;
	ksz_csLOW();
	ksz_SPI_Write(tData, 3);
//	ksz_SPI_Read(rData, 1);
	ksz_csHIGH();

//	return rData[0];
}



// Read switch register
uint8_t ksz8863ReadSwitchReg(uint8_t address)
{
   uint8_t tData[2];
   uint8_t rData[1];

   ksz_csLOW();

   tData[0] = KSZ8863_SPI_CMD_READ;
   tData[1] = address;

   ksz_SPI_Write(tData, 2);
   ksz_SPI_Read(rData, 1);
   ksz_csHIGH();
   return rData[0];
}


//  Dump switch registers for debugging purpose


void ksz8863DumpSwitchReg()
{
   uint16_t i;

   //Loop through switch registers
   for(i = 0; i < 256; i++)
   {
//      Display current switch register

	   uint8_t data = ksz8863ReadSwitchReg(i);

	   DebugPrintf("Reg: %03X -- %03X ----", i,data );
	   DebugPrintBinary(data);
	   DebugPrintf("\r\n");
   }

   //Terminate with a line feed
   DebugPrintf("\r\n");
}


/// ****************** All about switch MAC address ***********************************///


// Function to read the switch's MAC address
MacAddr   ksz8863GetSwitchMacAddr(void)
{
	MacAddr macAddr;
    for (int i = 0; i < 6; i++)
    {
        macAddr.b[i] = ksz8863ReadSwitchReg(KSZ8863_MAC_ADDR0 + i);
    }

    printMacAddrAsBytes(&macAddr);
    return macAddr;
}


// Function to write the switch's MAC address
void ksz8863SetSwitchMacAddr(MacAddr *macAddr)
{
	for (int i = 0; i < 6; i++)
	{
		ksz8863WriteSwitchReg(KSZ8863_MAC_ADDR0 + i, macAddr->b[i]);
	}
}


// Function to read the Station1's MACA1 
MacAddr   ksz8863GetStation1MacAddr(void)
{
	MacAddr macAddr;
    for (int i = 0; i < 6; i++)
    {
        macAddr.b[i] = ksz8863ReadSwitchReg(KSZ8863_MACA1 + i);
    }

    printMacAddrAsBytes(&macAddr);
    return macAddr;
}


// Function to write the Station1's MACA1 
void ksz8863SetStation1MacAddr(MacAddr *macAddr)
{
	for (int i = 0; i < 6; i++)
	{
		ksz8863WriteSwitchReg(KSZ8863_MACA1 + i, macAddr->b[i]);
	}
}


// Function to read the Station2's MACA2 
MacAddr   ksz8863GetStation2MacAddr(void)
{
	MacAddr macAddr;
    for (int i = 0; i < 6; i++)
    {
        macAddr.b[i] = ksz8863ReadSwitchReg(KSZ8863_MACA2 + i);
    }

    printMacAddrAsBytes(&macAddr);
    return macAddr;
}


// Function to write the Station2's MACA2 
void ksz8863SetStation2MacAddr(MacAddr *macAddr)
{
	for (int i = 0; i < 6; i++)
	{
		ksz8863WriteSwitchReg(KSZ8863_MACA2 + i, macAddr->b[i]);
	}
}



/// ********************** power management ********************************** ///


// Function to read power management mode
PowerManagementMode  ksz8863GetPowerManagementMode() 
{
    uint8_t status;
    // Read the power management register
    status = ksz8863ReadSwitchReg(KSZ8863_PWR_MGMT_LED_MODE);

    // Mask and return only the power management mode bits
    return (PowerManagementMode)(status & KSZ8863_PWR_MGMT_LED_MODE_POWER_MGMT_MODE );
}


// Function to set power management mode
void  ksz8863SetPowerManagementMode(PowerManagementMode mode) 
{
    if (mode > POWER_SAVING_MODE) {
        DebugPrintf("Invalid power management mode\n");
        return;
    }
    uint8_t reg_value;
    // Read the current register value
    reg_value = ksz8863ReadSwitchReg(KSZ8863_PWR_MGMT_LED_MODE);

    // Clear the power management mode bits and set the new mode
    reg_value = (reg_value & 0xFC)  | (mode & KSZ8863_PWR_MGMT_LED_MODE_POWER_MGMT_MODE );
    // Write the new register value back
   ksz8863WriteSwitchReg(KSZ8863_PWR_MGMT_LED_MODE, reg_value);

}

// power down port [n]

void  ksz8863PowerDownPort(uint8_t port)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL13(port));

      temp |= KSZ8863_PORTn_CTRL13_POWER_DOWN;

      //Write the value back to port control 13 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL13(port), temp);
   }

}


// power up port [n]

void  ksz8863PowerUpPort(uint8_t port)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL13(port));

      temp &= ~KSZ8863_PORTn_CTRL13_POWER_DOWN;

      //Write the value back to port control 13 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL13(port), temp);
   }

}


/// ****************** Link Core feature *************************************///

// enable auto negotiation on port [n]

void  ksz8863EnableAutoNegotiationPort(uint8_t port)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL12(port));

      temp |= KSZ8863_PORTn_CTRL12_AN_EN;

      //Write the value back to port control 12 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL12(port), temp);
   }

}

// disable auto negotiation on port [n]

void  ksz8863DisableAutoNegotiationPort(uint8_t port)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL12(port));

      temp &= ~KSZ8863_PORTn_CTRL12_AN_EN;

      //Write the value back to port control 12 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL12(port), temp);
   }

}


// restart auto negotiation feature in port[n]

void  ksz8863RestartAutoNegotiationPort(uint8_t port)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL13(port));

      temp |= KSZ8863_PORTn_CTRL13_RESTART_AN;

      //Write the value back to port control 13 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL13(port), temp);
   }

}




// set link speed forcefully if AN disabled
void ksz8863SetForceLinkSpeed(uint8_t port, NicLinkSpeed linkSpeed)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL12(port));


      if( linkSpeed == NIC_LINK_SPEED_100MBPS)
      {
        temp |= KSZ8863_PORTn_CTRL12_FORCE_SPEED; 
      }
      else if (linkSpeed == NIC_LINK_SPEED_10MBPS)
      {
         temp &= ~KSZ8863_PORTn_CTRL12_FORCE_SPEED; 
      }
      

      //Write the value back to port control 12 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL12(port), temp);
   }

}


// set Duplex mode
// enable true means forced full-duplex if (1) AN is disabled or (2) AN is enabled but failed.
// enable false means forced half-duplex if (1) AN is disabled or (2) AN is enabled but failed
void ksz8863SetForcedDuplexMode(uint8_t port, bool enable)
{
    uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL12(port));


      if( enable == true)
      {
         // enable full duplex
        temp |= KSZ8863_PORTn_CTRL12_FORCE_DUPLEX; 
      }
      else
      {
         // enable half duplex
         temp &= ~KSZ8863_PORTn_CTRL12_FORCE_DUPLEX; 
      }
      

      //Write the value back to port control 12 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL12(port), temp);
   }

}




// turn off all led port[n]

void  ksz8863TurnOffPortLed(uint8_t port)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL13(port));

      temp |= KSZ8863_PORTn_CTRL13_LED_OFF;

      //Write the value back to port control 13 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL13(port), temp);
   }

}


// power up port [n]

void   ksz8863TurnOnPortLed(uint8_t port)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL13(port));

      temp &= ~KSZ8863_PORTn_CTRL13_LED_OFF;

      //Write the value back to port control 13 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL13(port), temp);
   }

}

/// ******************* Basic function ****************************///

// TODO: MACRO FUNCTION
uint8_t LSB(uint16_t value)
{
   return (uint8_t)(value & 0xFF);
}

uint8_t MSB(uint16_t value)
{
   return (uint8_t) ((value >> 8) & 0xFF);
}


// ***********port advertising related all features*****************************



// enable advertising flow control capability on port [n]

void  ksz8863EnableFlowControlAdvertisementOnPort(uint8_t port, bool enable)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      //Read the value port control 12 register
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL12(port));
      
      if(enable == true)
      {
         temp |= KSZ8863_PORTn_CTRL12_ADV_FLOW_CTRL;
      }
      else
      {
          temp &= ~KSZ8863_PORTn_CTRL12_ADV_FLOW_CTRL;
      }


      //Write the value back to port control 12 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL12(port), temp);
   }

}



// enable advertise 100BT full duplex capability on port [n]
// if enable value is true advertisement capability will be enabled and 
// if enable value is false advertisement capability will be disabled

void  ksz8863EnableAdvertise100BTFullDuplexCapabilityOnPort(uint8_t port, bool enable)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      //Read the value port control 12 register
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL12(port));
      
      if(enable == true)
      {
         temp |= KSZ8863_PORTn_CTRL12_ADV_100BT_FD;
      }
      else
      {
          temp &= ~KSZ8863_PORTn_CTRL12_ADV_100BT_FD;
      }


      //Write the value back to port control 12 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL12(port), temp);
   }

}



// enable advertise 100BT half duplex capability on port [n]
// if enable value is true advertisement capability will be enabled and 
// if enable value is false advertisement capability will be disabled

void  ksz8863EnableAdvertise100BTHalfDuplexCapabilityOnPort(uint8_t port, bool enable)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      //Read the value port control 12 register
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL12(port));
      
      if(enable == true)
      {
         temp |= KSZ8863_PORTn_CTRL12_ADV_100BT_HD;
      }
      else
      {
          temp &= ~KSZ8863_PORTn_CTRL12_ADV_100BT_HD;
      }


      //Write the value back to port control 12 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL12(port), temp);
   }

}






// enable advertise 10BT full duplex capability on port [n]
// if enable value is true advertisement capability will be enabled and 
// if enable value is false advertisement capability will be disabled

void  ksz8863EnableAdvertise10BTFullDuplexCapabilityOnPort(uint8_t port, bool enable)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      //Read the value port control 12 register
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL12(port));
      
      if(enable == true)
      {
         temp |= KSZ8863_PORTn_CTRL12_ADV_10BT_FD;
      }
      else
      {
          temp &= ~KSZ8863_PORTn_CTRL12_ADV_10BT_FD;
      }


      //Write the value back to port control 12 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL12(port), temp);
   }

}



// enable advertise 10BT half duplex capability on port [n]
// if enable value is true advertisement capability will be enabled and 
// if enable value is false advertisement capability will be disabled

void  ksz8863EnableAdvertise10BTHalfDuplexCapabilityOnPort(uint8_t port, bool enable)
{
   uint8_t temp;

   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {
      //Read the value port control 12 register
      temp = ksz8863ReadSwitchReg(KSZ8863_PORTn_CTRL12(port));
      
      if(enable == true)
      {
         temp |= KSZ8863_PORTn_CTRL12_ADV_10BT_HD;
      }
      else
      {
          temp &= ~KSZ8863_PORTn_CTRL12_ADV_10BT_HD;
      }


      //Write the value back to port control 12 register
      ksz8863WriteSwitchReg(KSZ8863_PORTn_CTRL12(port), temp);
   }

}



//// ********************* get Link partner related info ********************************//

// this function helps to get link's partner flow control capability
bool ksz8863IsPartnerFlowControlCapable(uint8_t port)
{
   uint16_t status;
   bool capable;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {

	 //Read port status 0 register
	 status = ksz8863ReadSwitchReg(KSZ8863_PORTn_STAT0(port));

	 //Retrieve current link state
	 capable = (status & KSZ8863_PORTn_STAT0_LP_FLOW_CTRL_CAPABLE) ? true : false;


   }
   else
   {
      //The specified port number is not valid
      capable = false;
   }

   if(capable)
   {
      DebugPrintf("link partner is flow control capable\n");
   }
   else
   {
       DebugPrintf("link partner is not flow control capable\n");
   }
   
   
   return capable;
}





// this function helps to get link's partner 100BT full duplex capability

bool ksz8863IsPartner100BTFullDuplexCapable(uint8_t port)
{
   uint16_t status;
   bool capable;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {

	 //Read port status 0 register
	 status = ksz8863ReadSwitchReg(KSZ8863_PORTn_STAT0(port));

	 //Retrieve current link state
	 capable = (status & KSZ8863_PORTn_STAT0_LP_100BTX_FD_CAPABLE) ? true : false;


   }
   else
   {
      //The specified port number is not valid
      capable = false;
   }

   if(capable)
   {
      DebugPrintf("link partner is 100BT Full Duplex capable\n");
   }
   else
   {
      DebugPrintf("link partner is not 100BT Full Duplex capable\n");
   }
   
   
   return capable;
}



// this function helps to get link's partner 100BT half duplex capability

bool ksz8863IsPartner100BTHalfDuplexCapable(uint8_t port)
{
   uint16_t status;
   bool capable;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {

	 //Read port status 0 register
	 status = ksz8863ReadSwitchReg(KSZ8863_PORTn_STAT0(port));

	 //Retrieve current link state
	 capable = (status & KSZ8863_PORTn_STAT0_LP_100BTX_HF_CAPABLE ) ? true : false;


   }
   else
   {
      //The specified port number is not valid
      capable = false;
   }

   if(capable)
   {
      DebugPrintf("link partner is 100BT Half Duplex capable\n");
   }
   else
   {
      DebugPrintf("link partner is not 100BT Half Duplex capable\n");
   }
   
   
   return capable;
}





// this function helps to get link's partner 100BT full duplex capability

bool ksz8863IsPartner10BTFullDuplexCapable(uint8_t port)
{
   uint16_t status;
   bool capable;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {

	 //Read port status 0 register
	 status = ksz8863ReadSwitchReg(KSZ8863_PORTn_STAT0(port));

	 //Retrieve current link state
	 capable = (status & KSZ8863_PORTn_STAT0_LP_100BTX_FD_CAPABLE) ? true : false;


   }
   else
   {
      //The specified port number is not valid
      capable = false;
   }

   if(capable)
   {
      DebugPrintf("link partner is 100BT Full Duplex capable\n");
   }
   else
   {
      DebugPrintf("link partner is not 100BT Full Duplex capable\n");
   }
   
   
   return capable;
}





// this function helps to get link's partner 10BT half duplex capability

bool ksz8863IsPartner10BTHalfDuplexCapable(uint8_t port)
{
   uint16_t status;
   bool capable;

   //Check port number
   if(port >= KSZ8863_PORT1 && port <= KSZ8863_PORT2)
   {

	 //Read port status 0 register
	 status = ksz8863ReadSwitchReg(KSZ8863_PORTn_STAT0(port));

	 //Retrieve current link state
	 capable = (status & KSZ8863_PORTn_STAT0_LP_10BT_HD_CAPABLE) ? true : false;


   }
   else
   {
      //The specified port number is not valid
      capable = false;
   }

   if(capable)
   {
      DebugPrintf("link partner is 10BT half Duplex capable\n");
   }
   else
   {
      DebugPrintf("link partner is not 10BT half Duplex capable\n");
   }
   
   
   return capable;
}

