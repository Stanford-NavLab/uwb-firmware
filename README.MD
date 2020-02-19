# Ultra WideBand Radio Firmware for Multi-Agent Ranging Applications

This software is a modification of the firmware supplied by Decawave for their EVK1000 evaluation board. The new feature provided by this software is the ability for anchors to range with multiple tags and tags to range with multiple anchors. As much as possible of the original functionality has been preserved so most of the information provided in the EVK1000 User Manual still applies to this software 
<br/>
<br/>
The software and documentation for the EVK1000 evaluation kit and DW1000 transceiver can be found at the links below. The software and documentation can also be found in the [Decawave directory](Decawave).

[EVK1000 Product Page](https://www.decawave.com/product/evk1000-evaluation-kit/) <br/>
[Decawave Software Page](https://www.decawave.com/software/) <br/>
[Decawave Documentation Page](https://www.decawave.com/product-documentation/) <br/>

## Getting Started

### UWB Configuration

The EVB1000 allows for a numer of operational configurations via microswitches and jumpers. For default operations, ensure the EVB1000 is configured as shown in the images below. Consult the EVK1000 documentation for more configuration options.

<img src="img/STLINK config front.PNG" width="480">
<img src="img/STLINK config rear.PNG" width="480">

### Flashing the Binary

To flash the EVK1000, first connect the ST-LINK/V2 to the 20 pin JTAG connector (J4) with the connector tab aligned with the white silkscreen marking (facing away from the center of the board). Connect the ST-LINK/V2 to your computer USB. 

**_NOTE:_** The ST-LINK/V2 may not properly connect to the ST-LINK Utility software if connected to a USB hub. 

<img src="img/STLINK connection.PNG" width="480">

Next open the ST-LINK Utility. Click the *Program verify* button, then *Browse* in the window that pops up. Navigate to and select the firmware file and press *Start*. 
<br/>
<br/>
A compiled copy of the firmware (EVK1000.bin) can be found in the root directory of this repository. The STM32 ST-Link Utility can be downloaded from the [ST Link Utility Software Page](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-programmers/stsw-link004.html#get-software) and is also included in this repository in the [STLINK directory](STLINK).
 

<img src="img/STLINK software.PNG" width="480">

### Building from Source

To build the software from the source code, open the [DecaRanging.coproj file](DecaRanging.coproj) file with CoIDE. Simply click the *Build* button and the EVK1000/Debug/bin/ directory will be created and the compiled binary will be placed inside. The binary can then be flashed according to [Flashing the Binary](#flashing-the-binary) or by pressing the *Download Code to Flash* button in CoIDE.

<img src="img/STLINK software.PNG" width="480">

### Interfacing with the iBQR