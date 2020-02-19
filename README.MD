# Ultra WideBand Radio Firmware for Multi-Agent Ranging Applications

This software is a modification of the firmware supplied by Decawave for their EVK1000 evaluation board. The new feature provided by this software is the ability for anchors to range with multiple tags and tags to range with multiple anchors. As much as possible of the original functionality has been preserved so most of the information provided in the EVK1000 User Manual still applies to this software.

## Useful Links

The software and documentation for the EVK1000 evaluation kit and DW1000 transceiver can be found at the links below. The software and documentation can also be foud in the [Decawave directory](Decawave).

[EVK1000 Product Page](https://www.decawave.com/product/evk1000-evaluation-kit/).
[Decawave Software Page](https://www.decawave.com/software/)
[Decawave Documentation Page](https://www.decawave.com/product-documentation/)

## Getting Started

### UWB Configuration

The EVB1000 allows for a numer of operational configurations via microswitches and jumpers. For default operations, ensure the EVB1000 is configured as shown in the images below. Consult the EVK1000 documentation for more configuration options.

<img src="STLINK config front.PNG" width="100">
<img src="STLINK config rear.PNG" width="100">

### Flashing the Binary

To flash the EVK1000, first connect the ST-LINK/V2 to the 20 pin JTAG connector (J4) with the connector tab aligned with the white silkscreen marking (facing away from the center of the board). Connect the ST-LINK/V2 to your computer USB. 

**_NOTE:_** The ST-LINK/V2 may not properly connect to yhe ST-LINK Utility software if connected to a USB hub. 

<img src="STLINK connection.PNG" width="100">

Next open the ST-LINK Utility. Click the *Program verify* button, then *Browse* in the window that pops up. Select the firmware file and press *Start*. A compiled copy of the firmware (EVK1000.bin) can be found in the root directory of this repository. The STM32 ST-Link Utility can be downloaded from the [ST Link Utility Software Page](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-programmers/stsw-link004.html#get-software) and is also included in this repository in the [STLINK directory](STLINK directory).
 

<img src="STLINK software.PNG" width="100">

### Building from Source

//mention the toolchain used

### Interfacing with the iBQR