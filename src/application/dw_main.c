/*! ----------------------------------------------------------------------------
 *  @file    dw_main.c
 *  @brief   main loop for the DecaRanging application
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
/* Includes */
#include "compiler.h"
#include "port.h"

#include "instance.h"
#include "tdma_handler.h"

#include "deca_types.h"

#include "deca_spi.h"
#include "stdio.h"
#include "deca_regs.h"

extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);

#define SOFTWARE_VER_STRING    "TDMA Version 1.0" //

int instance_mode = DISCOVERY;

uint8 s1switch = 0;
int chan, tagaddr, ancaddr, prf;

#define LCD_BUFF_LEN (100)
uint8 dataseq[LCD_BUFF_LEN];
uint8 dataseq1[LCD_BUFF_LEN];

int ranging = 0;

uint32 inittestapplication(uint8 mode_switch)
{
    uint32 devID ;
    int result;

    port_set_dw1000_slowrate();  //max SPI before PLLs configured is ~4M

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid();
    if(DWT_DEVICE_ID != devID) //if the read of device ID fails, the DW1000 could be asleep
    {
        port_wakeup_dw1000();

        devID = instancereaddeviceid() ;
        // SPI not working or Unsupported Device ID
        if(DWT_DEVICE_ID != devID)
            return(-1) ;
        //clear the sleep bit - so that after the hard reset below the DW does not go into sleep
        dwt_softreset();
    }

    //reset the DW1000 by driving the RSTn line low
    reset_DW1000();


    result = instance_init() ;
    if (0 > result) return(-1) ; // Some failure has occurred



    port_set_dw1000_fastrate();
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }

    instance_init_s();
	int dr_mode = decarangingmode(mode_switch);
	instance_data_t* inst = instance_get_local_structure_ptr(0);

    chan = inst->chConfig[dr_mode].channelNumber ;
    prf = (inst->chConfig[dr_mode].pulseRepFreq == DWT_PRF_16M)? 16 : 64 ;

    instance_config(&inst->chConfig[dr_mode]) ;                  // Set operating channel etc

    instance_init_timings();

    inst->mode =  DISCOVERY;

    return devID;
}

void initLCD(void)
{
    uint8 initseq[9] = { 0x39, 0x14, 0x55, 0x6D, 0x78, 0x38 /*0x3C*/, 0x0C, 0x01, 0x06 };
    uint8 command = 0x0;
    int j = 100000;

    writetoLCD( 9, 0,  initseq); //init seq
    while(j--);

    command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);
    command = 0x1 ;  //clear screen
    writetoLCD( 1, 0,  &command);
}

void setLCDline1(uint8 mode_switch)
{
	uint8 command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);

	sprintf((char*)&dataseq[0], "DecaRanging  %02x", mode_switch);
	writetoLCD( 40, 1, dataseq); //send some data

	sprintf((char*)&dataseq1[0], "                 ");
	writetoLCD( 16, 1, dataseq1); //send some data
}

/*
 * @fn configure_continuous_txspectrum_mode
 * @brief   test application for production to check the TX power in various modes
**/
void configure_continuous_txspectrum_mode(uint8 mode_switch)
{
	if(port_is_switch_on(TA_SW1_4) == S1_SWITCH_ON)
	{
		//LCD display ON

		uint8 command = 0x2 ;  //return cursor home
		writetoLCD(1, 0,  &command);
		sprintf((char*)&dataseq[0], "Conti TX %s:%d:%d ", (mode_switch & SWS1_SHF_MODE) ? "S" : "L", chan, prf);
		writetoLCD(LCD_BUFF_LEN, 1, dataseq); //send some data
		memcpy(dataseq, (const uint8 *) "Spectrum Test   ", LCD_BUFF_LEN);
		writetoLCD(LCD_BUFF_LEN, 1, dataseq); //send some data
	}

	//configure DW1000 into Continuous TX mode
	instance_starttxtest(0x1000);
	//measure the power
	//Spectrum Analyser set:
	//FREQ to be channel default e.g. 3.9936 GHz for channel 2
	//SPAN to 1GHz
	//SWEEP TIME 1s
	//RBW and VBW 1MHz
	//measure channel power

	//user has to reset the board to exit mode
	while(1)
	{
		Sleep(2);
	}

}

/*
 * @fn      main()
 * @brief   main entry point
**/
int dw_main(void)
{
    int i = 0;
    double range_result = 0;

    //LCD variables
    bool enableLCD = FALSE;
	int toggle = 0;
	int toggle_counter = 0;
	int toggle_step = 5;
	uint8 command = 0x0;

	instance_data_t* inst = instance_get_local_structure_ptr(0);

	//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
	instanceConfig_t cfg1 = {
            2,              // channel
			3,              // preambleCode
            DWT_PRF_16M,    // prf
            DWT_BR_110K,    // datarate
            DWT_PLEN_1024,  // preambleLength
            DWT_PAC32,      // pacSize
            1,       // non-standard SFD
            (1025 + 64 - 32) //SFD timeout
        };
	inst->chConfig[0] = cfg1;
	instanceConfig_t cfg2 = {
			2,              // channel
			3,             // preambleCode
			DWT_PRF_16M,    // prf
			DWT_BR_6M8,    // datarate
			DWT_PLEN_128,   // preambleLength
			DWT_PAC8,       // pacSize
			0,       // non-standard SFD
			(129 + 8 - 8) //SFD timeout
		};
	inst->chConfig[1] = cfg2;
	instanceConfig_t cfg3 = {
			2,              // channel
			9,             // preambleCode
			DWT_PRF_64M,    // prf
			DWT_BR_110K,    // datarate
			DWT_PLEN_1024,  // preambleLength
			DWT_PAC32,      // pacSize
			1,       // non-standard SFD
			(1025 + 64 - 32) //SFD timeout
		};
	inst->chConfig[2] = cfg3;
	instanceConfig_t cfg4 = {
			2,              // channel
			9,             // preambleCode
			DWT_PRF_64M,    // prf
			DWT_BR_6M8,    // datarate
			DWT_PLEN_128,   // preambleLength
			DWT_PAC8,       // pacSize
			0,       // non-standard SFD
			(129 + 8 - 8) //SFD timeout
		};
	inst->chConfig[3] = cfg4;
	instanceConfig_t cfg5 = {
			5,              // channel
			3,             // preambleCode
			DWT_PRF_16M,    // prf
			DWT_BR_110K,    // datarate
			DWT_PLEN_1024,  // preambleLength
			DWT_PAC32,      // pacSize
			1,       // non-standard SFD
			(1025 + 64 - 32) //SFD timeout
		};
	inst->chConfig[4] = cfg5;
	instanceConfig_t cfg6 = {
			5,              // channel
			3,             // preambleCode
			DWT_PRF_16M,    // prf
			DWT_BR_6M8,    // datarate
			DWT_PLEN_128,   // preambleLength
			DWT_PAC8,       // pacSize
			0,       // non-standard SFD
			(129 + 8 - 8) //SFD timeout
		};
	inst->chConfig[5] = cfg6;
	instanceConfig_t cfg7 = {
			5,              // channel
			9,             // preambleCode
			DWT_PRF_64M,    // prf
			DWT_BR_110K,    // datarate
			DWT_PLEN_1024,  // preambleLength
			DWT_PAC32,      // pacSize
			1,       // non-standard SFD
			(1025 + 64 - 32) //SFD timeout
		};
	inst->chConfig[6] = cfg7;
	instanceConfig_t cfg8 = {
			5,              // channel
			9,             // preambleCode
			DWT_PRF_64M,    // prf
			DWT_BR_6M8,    // datarate
			DWT_PLEN_128,   // preambleLength
			DWT_PAC8,       // pacSize
			0,       // non-standard SFD
			(129 + 8 - 8) //SFD timeout
		};
	inst->chConfig[7] = cfg8;


    led_off(LED_ALL); //turn off all the LEDs

    peripherals_init();

    s1switch = port_is_boot1_on(0) << 1
    		| port_is_switch_on(TA_SW1_3) << 2
    		| port_is_switch_on(TA_SW1_4) << 3
    		| port_is_switch_on(TA_SW1_5) << 4
		    | port_is_switch_on(TA_SW1_6) << 5
    		| port_is_switch_on(TA_SW1_7) << 6
    		| port_is_switch_on(TA_SW1_8) << 7;


    if(port_is_switch_on(TA_SW1_4) == S1_SWITCH_ON)
	{
		//display ON
    	enableLCD = TRUE;
	}

    if(enableLCD == TRUE)
    {
		spi_peripheral_init(1);
    }
    else
    {
    	spi_peripheral_init(0);
    }


    Sleep(1000); //wait for LCD to power on


    if(enableLCD == TRUE)
	{
    	initLCD();

		memset(dataseq, 0x0, sizeof(dataseq));
		writetoLCD(1, 0, dataseq);
		memcpy(dataseq, (const uint8 *) "GGRG UWB RANGING", LCD_BUFF_LEN);
		writetoLCD(40, 1, dataseq); //send some data
		memcpy(dataseq, (const uint8 *) SOFTWARE_VER_STRING, LCD_BUFF_LEN);
		writetoLCD(16, 1, dataseq); //send some data
	}



    Sleep(1000);

    port_DisableEXT_IRQ(); 	//disable DW1000 IRQ until we configure the application

#ifdef USB_SUPPORT
    // enable the USB functionality
    usb_init();
    Sleep(1000);
    usb_run();
#endif

    //run DecaRanging application

	led_off(LED_ALL);

#ifdef USB_SUPPORT //this is set in the port.h file
	usb_printconfig(16, (uint8 *)SOFTWARE_VER_STRING, s1switch);
#endif

	if(inittestapplication(s1switch) == (uint32)-1)
	{
		if(enableLCD == TRUE)
		{
			led_on(LED_ALL); //to display error....
			dataseq[0] = 0x2 ;  //return cursor home
			writetoLCD(1, 0,  &dataseq[0]);
			memset(dataseq, ' ', LCD_BUFF_LEN);
			memcpy(dataseq, (const uint8 *) "ERROR           ", LCD_BUFF_LEN);
			writetoLCD( 40, 1, dataseq); //send some data
			memcpy(dataseq, (const uint8 *) "INIT FAIL       ", LCD_BUFF_LEN);
			writetoLCD( 16, 1, dataseq); //send some data
		}
		return 0; //error
	}

	tdma_init_s(inst->durationSlotMax_us);	//call after instance_init_timings() to get slot duration

	if(enableLCD == TRUE)
	{
		dataseq[0] = 0x2 ;  //return cursor home
		writetoLCD( 1, 0,  dataseq);
		memset(dataseq, ' ', LCD_BUFF_LEN);
		memcpy(dataseq, (const uint8 *) "MAX NETWORK SIZE", LCD_BUFF_LEN);
		writetoLCD(40, 1, dataseq); //send some data
		memset(dataseq, ' ', LCD_BUFF_LEN);
		sprintf((char*)&dataseq[0], "%d", UWB_LIST_SIZE);
		writetoLCD(16, 1, dataseq); //send some data

		Sleep(2000);

		dataseq[0] = 0x2 ;  //return cursor home
		writetoLCD( 1, 0,  dataseq);
		memset(dataseq, ' ', LCD_BUFF_LEN);
		memcpy(dataseq, (const uint8 *) "SLOT DURATION   ", LCD_BUFF_LEN);
		writetoLCD(40, 1, dataseq); //send some data
		memset(dataseq, ' ', LCD_BUFF_LEN);
		sprintf((char*)&dataseq[0], "%llu us", inst->durationSlotMax_us);
		writetoLCD(16, 1, dataseq); //send some data

		Sleep(2000);




		dataseq[0] = 0x2 ;  //return cursor home
		writetoLCD( 1, 0,  dataseq);
		memset(dataseq, ' ', LCD_BUFF_LEN);
		sprintf((char*)&dataseq[0], "TX DELAY: %.5u ", inst->txAntennaDelay);
		writetoLCD(40, 1, dataseq); //send some data
		memset(dataseq, ' ', LCD_BUFF_LEN);
		sprintf((char*)&dataseq[0], "RX DELAY: %.5u ", inst->rxAntennaDelay);
		writetoLCD(16, 1, dataseq); //send some data

		Sleep(2000);

	}

	//test EVB1000 - used in EVK1000 production
	if((s1switch & SWS1_TXSPECT_MODE) == SWS1_TXSPECT_MODE) //to test TX power
	{
		//this function does not return!
		configure_continuous_txspectrum_mode(s1switch);
	}

	//sleep for 5 seconds displaying "Decawave"
	i=30;
	while(i--)
	{
		if(enableLCD == TRUE)
		{
			if (i & 1) led_off(LED_ALL);
			else    led_on(LED_ALL);
		}
		Sleep(200);
	}
	i = 0;
	led_off(LED_ALL);

	if(enableLCD == TRUE)
	{
		command = 0x2 ;  //return cursor home
		writetoLCD( 1, 0,  &command);
		memset(dataseq, ' ', LCD_BUFF_LEN);
	}

    if(enableLCD == TRUE)
	{
		memset(dataseq, ' ', LCD_BUFF_LEN);
		memset(dataseq1, ' ', LCD_BUFF_LEN);
	}

    port_EnableEXT_IRQ();

    // main loop
    while(1)
    {
    	bool updateLCD = FALSE;
    	//TODO reenable optimization in the compiler settings!!!
		instance_run(); //run the state machine!!!
		instance_mode = inst->mode;

		if(inst->canPrintUSB == TRUE)
		{
			if(instancenewrange())
			{
				int n;
				updateLCD = TRUE;
				//send the new range information to LCD and/or USB
				double idist = instance_get_idist(inst->newRangeUWBIndex);
				int rng = (int)(idist*1000.0);
				int rng_raw = (int)(instance_get_idistraw(inst->newRangeUWBIndex)*1000.0);

				uint64 saddr = instance_get_addr();
				uint64 aaddr = instancenewrangeancadd();
				uint64 taddr = instancenewrangetagadd();

				if(enableLCD == TRUE)
				{
					//only update range on display if this UWB is one of the UWBs involved in the range measurement
					if(memcmp(&saddr, &aaddr, sizeof(uint64)) == 0 || memcmp(&saddr, &taddr, sizeof(uint64)) == 0){
						range_result = idist;
					}
				}

				//self address, ranging anchor address, ranging tag address, range TODO
//				n = sprintf((char*)&dataseq[0], "%016llX %016llX %016llX %08X %08X", saddr, aaddr, taddr, rng, rng_raw);
//				n = sprintf((char*)&dataseq[0], "%016llX,%016llX,%016llX,%08X,%08X", saddr, aaddr, taddr, rng, rng_raw);
//				n = sprintf((char*)&dataseq[0], "%016llX %016llX %016llX %08X", saddr, aaddr, taddr, rng);
				n = sprintf((char*)&dataseq[0], "RANGE_COMPLETE,%04llX,%04llX", taddr, aaddr);
//				n = sprintf((char*)&dataseq[0], "%08d,xxxx,xxxx", rng);


				send_usbmessage(&dataseq[0], n);
				usb_run();
			}
        }

        //only write to LCD if we aren't in the middle of  ranging messages
        //the sleep messages embedded in the LCD calls mess up the timing otherwise
		if(enableLCD == TRUE && inst->canPrintLCD == TRUE)
		{

        	toggle_step = 750;
        	memset(dataseq, ' ', LCD_BUFF_LEN);
			memset(dataseq1, ' ', LCD_BUFF_LEN);
			uint64 addr = instance_get_addr();

			uint8 num_neighbors = instfindnumneighbors(inst);
			uint8 num_hidden = instfindnumhidden(inst);
			char status[10];

			if(instance_mode == DISCOVERY)
			{
				strcpy(status, "SEARCHING");
				range_result = 0;
			}
			else
			{
				strcpy(status, "CONNECTED");
			}

			if(toggle_counter <= toggle_step)
			{
				if(toggle == 2)
				{
					updateLCD = TRUE;
				}
				toggle = 1;
			}
			else if(toggle_counter <= toggle_step*2)
			{
				if(toggle == 1)
				{
					updateLCD = TRUE;
				}
				toggle = 2;
			}

			toggle_counter++;
			if(toggle_counter > toggle_step*2)
			{
				toggle_counter = 0;
			}

			if(updateLCD == TRUE)
			{
				dataseq[0] = 0x2 ;  //return cursor home
				writetoLCD( 1, 0,  dataseq);

				struct TDMAHandler *tdma_handler = tdma_get_local_structure_ptr();
				uint8 framelength = tdma_handler->uwbListTDMAInfo[0].framelength;
				if(toggle == 1)
				{
					if(inst->addrByteSize == 8)
					{
						sprintf((char*)&dataseq[0], "%s       ", status);
						sprintf((char*)&dataseq1[0], "N%02u FL%03u %05.2fm", num_neighbors, framelength, range_result);
					}
					else
					{
						sprintf((char*)&dataseq[0], "%llX %s", addr, status);
						sprintf((char*)&dataseq1[0], "N%02u FL%03d %05.2fm", num_neighbors, framelength, range_result);
					}
				}
				else //if(toggle == 2)
				{
					if(inst->addrByteSize == 8)
					{
						sprintf((char*)&dataseq[0], "%llX", addr);
						sprintf((char*)&dataseq1[0], "H%02u FL%03u %05.2fm", num_neighbors, framelength, range_result);
					}
					else
					{

						sprintf((char*)&dataseq[0], "%llX %s", addr, status);
						sprintf((char*)&dataseq1[0], "H%02u FL%03d %05.2fm", num_hidden, framelength, range_result);
					}

				}

				writetoLCD(40, 1, dataseq); //send some data
				writetoLCD(16, 1, dataseq1); //send some data
			}
		}
    }

    return 0;
}



