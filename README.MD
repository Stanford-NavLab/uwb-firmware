# Ultra WideBand Radio Firmware for Multi-Agent Ranging Applications

## Table of Contents

[Overview](#building-from-source) <br/>
[UWB Configuration](#uwb-configuration) <br/>
[LCD](#lcd) <br/>
[Flashing the Binary](#flashing-the-binary) <br/>
[Building from Source](#building-from-source) <br/>
[Interfacing with the iBQR](#interfacing-with-linux) <br/>
[Calibration](#calibration)<br/>
[Firmware Definitions](#firmware-definitions) <br/>

### Overview

This software is a modification of the firmware supplied by Decawave for their EVK1000 evaluation board. The main enhancement provided by this software is network coordination which enables the UWBs to take turns ranging with each other. UWBs can be added and removed from the network in an ad-hoc manner and the network will reconfigure to optimize ranging cycle frequency. Additionally, ranging measurements are broadcast to the network so each UWB is aware of the ranging measurements between each UWB pair in its proximity &mdash eliminating the need for a secondary communication channel. 
<br/>
<br/>
Using the UWBs is simple. Once a UWB has been flashed with the firmware and the switches have been configured, simply apply power to two ore more UWBs and they will begin ranging with each other. The ranging data can then be accessed via USB. The `uwb_interface` ROS package to written to read the USB ranging data and publish it to a ROS node.
<br/>
<br/>
As much of the original firmware functionality, as possible, has been preserved so much of the information provided in the EVK1000 User Manual still applies to this software. The software and documentation for the EVK1000 evaluation kit and DW1000 transceiver can be found at the links below. It can also be found in the [Decawave directory](Decawave).
<br/>
<br/>
[EVK1000 Product Page](https://www.decawave.com/product/evk1000-evaluation-kit/) <br/>
[Decawave Software Page](https://www.decawave.com/software/) <br/>
[Decawave Documentation Page](https://www.decawave.com/product-documentation/) <br/>

### UWB Configuration

The EVB1000 allows for a numer of operational configurations via the S1 microswitches. To ensure proper operattions, each UWB in a network must be configured identically. The table below summarizes the configurable and required S1 switch positions. The EVB1000 also has the S2 and S3 microswitches as well as 5 jumpers. The required configuration for these items are in the images below. 

Switch | Function             | OFF              | ON               | Notes
------ | ------------------   | --------------   | --------------   | --------
S1-1   |                      |                  | Required         |
S1-2   |                      | Required         |                  |
S1-3   |                      |                  | Required         | 
S1-4   | Onboard LCD          | OFF              | ON               | Consumes more power and reduces ranging cycle frequency when ON
S1-5   | Data Rate / Preamble | 110 kbps / 1024  | 6.8 Mbps / 128   | OFF increases max range but reduces ranging cycle frequency
S1-6   | PRF                  | 16               | 64               | 
S1-7   | Channel              | 2 (3993.6 MHz)   | 5 (6489.6 MHz)   | OFF increases range and signal penetration but ON may have less interference      
S1-8   |                      | Required         |                  | 

<img src="img/EVK1000 config front.PNG" width="480">

<img src="img/EVK1000 config rear.PNG" width="480">

### LCD

The onboard LCD is toggled via the S1-4 microswitch. The LCD is expected to be off during nominal operations since it increases power consumption and decreases ranging cycle frequency. If enabled, the LCD will display a series of screens on boot. 

```
GGRG UWB RANGING
TDMA Version 1.0
```
This screen displays the firmware version number.

```
MAX NETWORK SIZE
XX
```

This screen displays the maximum number of UWBs that can be connected simultaneously. All UWBs in a network should have this set to the same value. See [Firmware Definitions](#firmware-definitions) for how to modify this value.

```
SLOT DURATION
XXXXX us
```

Network coordination is acheived by assigning timeslots to UWBs. The slot duration value is the length in time of a single timeslot. The network will acheive no more than one ranging measurement per slot. This value is a function of the maximum network size and the settings configured by microswitches S1-4 through S1-7. 

```
TX DELAY: XXXXX
RX DELAY: XXXXX
```
The TX and RX delay values displayed are the calibration values set for the current configuration of microswitches S1-4 through S1-7.

```
ADDR STATUS
NXX FLXXX XX.XXm
```

This is the final screen that will be displayed by the UWB. The address (ADDR) and status will be displayed on the top line. Status will be either SEARCHING or CONNECTED to show network connection status. The second line will display the most recent range measurement in meters along with some information related to network coordination. NXX is the number of neighboring UWBs (within communication distance of the UWB). NXX with alternate with HXX which is the number of hidden UWBs (not within communication distance of the UWB, but within communication distance of a neighboring UWB). FLXXX is the TDMA framelength.

### Flashing the Binary

There are two ways to flash the binary, the ST-LINK Utility and CoIDE. The ST-LINK Utility can be used to flash an already compiled binary and CoIDE can both compile and flash the binary. A compiled copy of the firmware (EVK1000.bin) can be found in the root directory of this repository. Instructions for flashing with the ST-LINK Utility are in this section. Instructions for flashing with CoIDE are found in [Building from Source](#building-from-source)  

**_NOTE:_** Calibration data for each UWB must be set in the firmware before compilation. The precompiled binary is meant for general use and will only provide a reasonable guess for what the calibration values might be. See [Calibration](#calibration) for more information

**_NOTE:_** The STM32 ST-Link Utility can be downloaded from the [ST Link Utility Software Page](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-programmers/stsw-link004.html#get-software) and is also included in this repository in the [STLINK directory](STLINK).

To flash the EVK1000, first connect the ST-LINK/V2 to the 20 pin JTAG connector (J4) with the connector tab aligned with the white silkscreen marking (facing away from the center of the board). Connect the ST-LINK/V2 to your computer USB. 

**_NOTE:_** The ST-LINK/V2 may not properly connect to the ST-LINK Utility software if connected through a USB hub. 

<img src="img/STLINK connection.PNG" width="480">

Next open the ST-LINK Utility. Click the *Program verify* button, then *Browse* in the window that pops up. Navigate to and select the firmware file and press *Start*. 
<br/>
<br/>
 

<img src="img/STLINK software.PNG" width="480">

### Building from Source

To build the software from the source code, open the [DecaRanging.coproj file](DecaRanging.coproj) file with CoIDE. Download and install the [GNU Tools ARM Embedded 6-2017-q2-update](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads), click *Project* in the top menu, and click *Select Toolchain Path* from the dropdown. Click *Browse* in the window that pops up and navigate to and select the *GNU Tools Arm Embedded\6 2017-q2-update\bin* directory.

<img src="img/CoIDE toolchain.PNG" width="480">

Then simply click the *Build* button and the EVK1000/Debug/bin/ directory will be created and the compiled binary will be placed inside. The binary can then be flashed according to [Flashing the Binary](#flashing-the-binary) or by pressing the *Download Code to Flash* button in CoIDE.

**_NOTE:_** Calibration data for each UWB must be set in the firmware before compilation. See [Calibration](#calibration) for more information

<img src="img/CoIDE.PNG" width="480">

### Interfacing with Linux

The UWB ranging data is accessable via USB. The `uwb_interface` ROS package for interfacing with UWB can be found in the `tx2-ros-examples` repository. It is run using the following:

```
rosrun uwb_interface uwb_interface_node -p /dev/ttyAMC0
```

**_NOTE:_** Roscore must be running first. <br/>
**_NOTE:_** The argument after `-p` is the path to the USB port connection and may not be the same as listed above. See the `tx2-ros-examples` repository for more information.

### Calibration

The ranging algorithm depends on RX and TX timestamps to accurately determine the time-of-flight (from which distance is derived) of the communications signals. For the distance calculations to be accurate, the signal propogation time from the antenna to the DW1000 transceiver (RX delay) and from the DW1000 to the antenna (TX delay) must be accounted for. The UWB firmware is responsible for loading the calibration values into the DW1000 transceiver at boot, so the proper settings should be set when building the firmware.

**_NOTE:_** Step-by-step instructions for performing the RX and TX delay calibration can be found in the `uwb_delay_calibration` package of the `tx2-ros-examples` repository

The firmware definitions that control which values are loaded into the DW1000 are found near the top of [src/applicaiton/application_definitions.h](src/applicaiton/application_definitions.h). The default (uncalibrated) values are shown below

```
#define SET_TXRX_DELAY              0

//S1 5-6-7 OFF-OFF-OFF
#define TX_ANT_DELAY_000            0
#define RX_ANT_DELAY_000            0
//S1 5-6-7 OFF-OFF-ON
#define TX_ANT_DELAY_001            0
#define RX_ANT_DELAY_001            0
//S1 5-6-7 OFF-ON-OFF
#define TX_ANT_DELAY_010            0
#define RX_ANT_DELAY_010            0
//S1 5-6-7 OFF-ON-ON
#define TX_ANT_DELAY_011            0
#define RX_ANT_DELAY_011            0
//S1 5-6-7 ON-OFF-OFF
#define TX_ANT_DELAY_100            0
#define RX_ANT_DELAY_100            0
//S1 5-6-7 ON-OFF-ON
#define TX_ANT_DELAY_101            0
#define RX_ANT_DELAY_101            0
//S1 5-6-7 ON-ON-OFF
#define TX_ANT_DELAY_110            0
#define RX_ANT_DELAY_110            0
//S1 5-6-7 ON-ON-ON
#define TX_ANT_DELAY_111            0
#define RX_ANT_DELAY_111            0

```

For the calibration definitions to be applied, `SET_TXRX_DELAY` must be set to `1`. If set to `0`, the firmware will provide a reasonable guess of what the calibration values might be. For previously calibrated UWBs, the [calibration](calibration) directory contains text files with the calibration values which can be copied and pasted into the firmware prior to building. 

The files in the calibration directory are listed by UWB address. Each UWB's address can be found by turning the LCD on and powering the UWB after the firmware has been loaded. After several seconds, either the long address (16 characters) or the short address (last 4 characters of the long address) will be displayed on the top row of the LCD.      

**_NOTE:_** The calibration values differ for each combintaion of switch settings, S1-5, S1-6, and S1-7. 
When building and flashing the firmware be sure to copy/paste all of the calibration settings from the calibration file. After final calibration, make sure any changes are saved to the calibration files and uploaded to the repository.

### Firmware Definitions

Near the top of the [src/applicaiton/application_definitions.h](src/applicaiton/application_definitions.h) file is a section marked *Definitions expected to be modified by end-user*. Most of them are detailed in the [Calibration](#calibration) section. The rest are described here.

`UWB_LIST_SIZE` is used to specify the maximum network size. Setting this number to a larger value will increase the allowable network size but will decrease the ranging cycle frequency, regardless of how many UWBs are actually connected. To maximise ranging cycle frequency, set this value no larger than the expected network size. The default value is 10. Due to hardware and firmware limitations, it should not be set greater than 80. 

`USING_64BIT_ADDR` is used to specify whether to use 64-bit (long) or 16-bit (short) addresses. In general, 16-bit addresses should be used. It requires less data to be transmitted, increasing ranging cycle frequency, and decreasing power consumption. The only case where 64-bit addresses must be used is when two or more UWBs share the same short address.