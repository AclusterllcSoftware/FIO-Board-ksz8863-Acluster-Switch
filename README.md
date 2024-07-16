# FIO-Board-ksz8863-Acluster-Switch
This project provides an advanced driver for the KSZ8863 Ethernet switch, facilitating comprehensive control and management of Ethernet switching functionalities. The driver offers a wide range of features including initialization, port state management, link state monitoring, and MAC address configuration, designed for high-performance network applications.

# Features
## Initialization and Configuration
- **Initialization:** Seamless initialization of the KSZ8863 switch.
- **CPU Clock Selection:** Configure the CPU clock for optimal performance.
- **Pass-All-Frame Mode:** Enable or disable pass-all-frame mode for versatile frame handling.
- **Aging Control:** Control aging of dynamic entries for improved network stability.

## Port and Link Management
- **Port State Management:** Enable/disable ports, set port states, and manage auto-negotiation.
- **Link State Monitoring:** Retrieve link state, speed, and duplex mode for each port.
- **Port Power Management:** Power down or power up individual ports to save energy.
- **LED Control:** Turn port LEDs on or off for visual status indication.

## MAC Address and Database Management
- **MAC Address Configuration:** Set and get MAC addresses for the switch and individual stations.
- **Static FDB Management:** Add, delete, and retrieve static FDB entries.
- **Dynamic FDB Management:** Retrieve dynamic FDB entries and flush dynamic FDB table.


## Advanced Features
- **Flow Control Advertisement:** Enable or disable flow control advertisement on ports.
- ****Duplex Capability Advertisement:** Advertise full and half duplex capabilities for 10BT and 100BT modes.
- **Link Partner Capability Detection:** Detect the capabilities of link partners for enhanced compatibility.

## Power Management
- **Power Management Mode:** Get and set the power management mode for the switch.
- **Port Power Control:** Power down or power up individual ports to reduce power consumption.
- **Auto-Negotiation Control:** Enable, disable, or restart auto-negotiation on ports.
