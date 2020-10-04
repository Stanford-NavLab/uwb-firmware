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

extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);

							// "1234567890123456" - 16 bytes long LCD
#define SOFTWARE_VER_STRING    "TDMA Version 1.0" //

#define SWS1_TXSPECT_MODE	0x80  //Continuous TX spectrum mode
#define SWS1_ANC_MODE 		0x08  //anchor mode
#define SWS1_SHF_MODE		0x10  //short frame mode (6.81M) (switch S1-5)
#define SWS1_64M_MODE		0x20  //64M PRF mode (switch S1-6)
#define SWS1_CH5_MODE		0x40  //channel 5 mode (switch S1-7)

int dr_mode = 0;
int instance_mode = DISCOVERY;

uint8 s1switch = 0;
int chan, tagaddr, ancaddr, prf;

#define LCD_BUFF_LEN (100)
uint8 dataseq[LCD_BUFF_LEN];
uint8 dataseq1[LCD_BUFF_LEN];

int ranging = 0;

//Configuration for DecaRanging Modes (8 default use cases selectable by the switch S1 on EVK)
instanceConfig_t chConfig[8] ={
                    //mode 1 - S1: 7 off, 6 off, 5 off
                    {
                        2,              // channel
						3,              // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 2
                    {
                        2,              // channel
                        3,             // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 3
                    {
                        2,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 4
                    {
                        2,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 5
                    {
                        5,              // channel
                        3,             // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 6
                    {
                        5,              // channel
                        3,             // preambleCode
                        DWT_PRF_16M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    },
                    //mode 7
                    {
                        5,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_110K,    // datarate
                        DWT_PLEN_1024,  // preambleLength
                        DWT_PAC32,      // pacSize
                        1,       // non-standard SFD
                        (1025 + 64 - 32) //SFD timeout
                    },
                    //mode 8
                    {
                        5,              // channel
                        9,             // preambleCode
                        DWT_PRF_64M,    // prf
                        DWT_BR_6M8,    // datarate
                        DWT_PLEN_128,   // preambleLength
                        DWT_PAC8,       // pacSize
                        0,       // non-standard SFD
                        (129 + 8 - 8) //SFD timeout
                    }
};


//uint32 inittestapplication(uint8 s1switch);


int decarangingmode(uint8 mode_switch)
{
    int mode = 0;

    if(mode_switch & SWS1_SHF_MODE)
    {
        mode = 1;
    }

    if(mode_switch & SWS1_64M_MODE)
    {
        mode = mode + 2;
    }
    if(mode_switch & SWS1_CH5_MODE)
    {
        mode = mode + 4;
    }

    return mode;
}

uint32 inittestapplication(uint8 mode_switch)
{
    uint32 devID ;
    int result;

    port_set_dw1000_slowrate();  //max SPI before PLLs configured is ~4M

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    devID = instancereaddeviceid() ;
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

    result = tdma_init_s();

    port_set_dw1000_fastrate();
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }

    if(mode_switch & SWS1_ANC_MODE)
    {
//        instance_mode = ANCHOR;

        led_on(LED_PC6);

    }
    else
    {
//        instance_mode = TAG;
        led_on(LED_PC7);
    }

    instance_init_s();
	dr_mode = decarangingmode(mode_switch);

    chan = chConfig[dr_mode].channelNumber ;
    prf = (chConfig[dr_mode].pulseRepFreq == DWT_PRF_16M)? 16 : 64 ;

    instance_config(&chConfig[dr_mode]) ;                  // Set operating channel etc

    instancesettagsleepdelay(POLL_SLEEP_DELAY, BLINK_SLEEP_DELAY); //set the Tag sleep time

    instance_init_timings();

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
    uint64 range_addr = 0;
    int canSleep;

    //LCD variables
    bool enableLCD = FALSE;
	int toggle = 0;
	int toggle_counter = 0;
	int toggle_step = 5;
//    uint8 dataseq[LCD_BUFF_LEN];
//    bool new_range = FALSE;
	uint8 command = 0x0;

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
		memcpy(dataseq, (const uint8 *) SOFTWARE_VER_STRING, LCD_BUFF_LEN); // Also set at line #26 (TODO Should make this from single value !!!)
		writetoLCD(16, 1, dataseq); //send some data
	}

    Sleep(1000);

    port_DisableEXT_IRQ(); 	//disable DW1000 IRQ until we configure the application

#ifdef USB_SUPPORT
    // enable the USB functionality
    usb_init();
    Sleep(1000);
    usb_run();
    Sleep(10000); //TODO remove this for the final build
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
			writetoLCD( LCD_BUFF_LEN, 1, dataseq); //send some data
			memcpy(dataseq, (const uint8 *) "INIT FAIL       ", LCD_BUFF_LEN);
			writetoLCD( LCD_BUFF_LEN, 1, dataseq); //send some data
		}
		return 0; //error
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

//	if(enableLCD == TRUE)
//	{
//
//		memcpy(&dataseq[0], (const uint8 *) " DISCOVERY MODE ", LCD_BUFF_LEN);
//		writetoLCD(LCD_BUFF_LEN, 1, dataseq); //send some data
//		sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
//		writetoLCD(LCD_BUFF_LEN, 1, dataseq); //send some data
//
//		command = 0x2 ;  //return cursor home
//		writetoLCD( 1, 0,  &command);
//	}



    if(enableLCD == TRUE)
	{
		memset(dataseq, ' ', LCD_BUFF_LEN);
		memset(dataseq1, ' ', LCD_BUFF_LEN);
	}

    port_EnableEXT_IRQ();

    //TODO remove
    instance_data_t* inst1 = instance_get_local_structure_ptr(0);
    inst1->testTimer = portGetTickCntMicro();

    // main loop
    while(1)
    {
    	bool updateLCD = FALSE;
    	//TODO reenable optimization in the compiler settings!!!
		instance_data_t* inst = instance_get_local_structure_ptr(0);
		canSleep = instance_run(); //run the state machine!!!
		instance_mode = inst->mode; //TODO modify how the rest of this works with DISCOVER, TAG, and ANCHOR!

        if(instancenewrange())
        {
        	updateLCD = TRUE;
//        	int n, rng, rng_raw;
//            uint64 aaddr, taddr;
//            ranging = 1;
            //send the new range information to LCD and/or USB
            range_result = instance_get_idist(inst->newRangeUWBIndex);
            range_addr = instance_get_uwbaddr(inst->newRangeUWBIndex);


            //set_rangeresult(range_result);
            if(enableLCD == TRUE)
			{
#if (DELAY_CALIB_OUTPUT == 1)
				dataseq[0] = 0x2 ;  //return cursor home
				writetoLCD( 1, 0,  dataseq);

				dataseq[0] = 0x2 ;  //return cursor home
				writetoLCD( 1, 0,  dataseq);
				memset(dataseq, ' ', LCD_BUFF_LEN);
				memset(dataseq1, ' ', LCD_BUFF_LEN);

				toggle_step = 5;

				if(toggle <= toggle_step)
				{
					sprintf((char*)&dataseq[0], "ADDRESS - SELF  ");
					sprintf((char*)&dataseq1[0], "%llX", instance_get_addr());
				}
				else if(toggle <= toggle_step*2)
				{
					sprintf((char*)&dataseq[0], "RANGING WITH    ");
					if(inst->mode == TAG)
					{
						sprintf((char*)&dataseq1[0], "%.3u ANCHORS     ", instfindnumactiveuwbinlist(inst));
					}
					else if(inst->mode == ANCHOR)
					{
						sprintf((char*)&dataseq1[0], "%.3u TAGS        ", instfindnumactiveuwbinlist(inst));
					}
				}
				else
				{
					sprintf((char*)&dataseq[0], "TX DELAY: %.5u ", inst->txAntennaDelay);
					sprintf((char*)&dataseq1[0], "RX DELAY: %.5u ", inst->rxAntennaDelay);
				}
				toggle++;
				if(toggle > toggle_step*3)
				{
					toggle = 0;
				}

				writetoLCD( 40, 1, dataseq); //send some data
				writetoLCD( 16, 1, dataseq1); //send some data
#else

//				dataseq[0] = 0x2 ;  //return cursor home
//				writetoLCD( 1, 0,  dataseq);
//
//				memset(dataseq, ' ', LCD_BUFF_LEN);
//				memset(dataseq1, ' ', LCD_BUFF_LEN);
//
//				sprintf((char*)&dataseq[0], "LAST: %4.2f m", range_result);
//				writetoLCD( 40, 1, dataseq); //send some data
//				sprintf((char*)&dataseq1[0], "%llX", instance_get_uwbaddr(inst->newRangeUWBIndex));
//				writetoLCD( 16, 1, dataseq1); //send some data
#endif
			}

//            aaddr = instancenewrangeancadd();
//            taddr = instancenewrangetagadd();
//            rng = (int) (range_result*1000);
//            rng_raw = (int) (instance_get_idistraw(inst->newRangeUWBIndex)*1000);



//            n = sprintf((char*)&dataseq[0], "RANGE_COMPLETE,%llX,%llX", taddr, aaddr);
//            n = sprintf((char*)&dataseq[0], "RANGE_COMPLETE,%llX,%llX", taddr, aaddr);

			if(instance_mode == TAG)
			{
				uint64 aaddr = instancenewrangeancadd();
				uint64 taddr = instancenewrangetagadd();
//				int n = sprintf((char*)&dataseq[0], "RANGE_COMPLETE,%llX,%llX", taddr, aaddr);
				int n = sprintf((char*)&dataseq[0], "RANGE_COMPLETE,%04llX,%04llX", taddr, aaddr);
				send_usbmessage(&dataseq[0], n);
				usb_run();

			}
			else
			{
//				n = sprintf((char*)&dataseq[0], "RANGE_COMPLETE,%llX,%llX", taddr, aaddr);
			}

//            if(instance_mode == TAG)
//            {
//                n = sprintf((char*)&dataseq[0], "t %llX %llX %08X %08X", aaddr, taddr, rng, rng_raw);
//
//            }
//            else
//            {
////                n = sprintf((char*)&dataseq[0], "a %llX %llX %08X %08X", aaddr, taddr, rng, rng_raw);
//                n = sprintf((char*)&dataseq[0], "RANGE_COMPLETE,%llX,%llX", taddr, aaddr);
//            }

#ifdef USB_SUPPORT //this is set in the port.h file
           if(instance_mode == TAG)
           {
//        	   send_usbmessage(&dataseq[0], n);
           }
//           send_usbmessage(&dataseq[0], n);
#endif
        }

        if(enableLCD == TRUE)
		{
        	toggle_step = 750;
        	memset(dataseq, ' ', LCD_BUFF_LEN);
			memset(dataseq1, ' ', LCD_BUFF_LEN);
			uint64 addr = instance_get_addr();

			uint8 num_neighbors = instfindnumactiveneighbors(inst);
			uint8 num_hidden = instfindnumactivehidden(inst);
			char status[10];

			if(instance_mode == DISCOVERY)
			{
				strcpy(status, "SEARCHING");
				range_addr = 0x0000;
				range_result = 0;
			}
			else
			{
				strcpy(status, "CONNECTED");
			}
			dataseq[0] = 0x2 ;  //return cursor home
			writetoLCD( 1, 0,  dataseq);

			//TODO only update the display if something has changed!
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


			//TODO only update the display if something has changed!
			if(updateLCD == TRUE)
			{
				if(toggle == 1)
				{
					if(inst->addrByteSize == 8)
					{
						sprintf((char*)&dataseq[0], "%s       ", status);
						sprintf((char*)&dataseq1[0], "N:%02u H:%02u %05.2fm", num_neighbors, num_hidden, range_result);
					}
					else
					{
						struct TDMAHandler *tdma_handler = tdma_get_local_structure_ptr();
						uint8 framelength = tdma_handler->uwbListTDMAInfo[0].framelength;
						sprintf((char*)&dataseq[0], "%llX %s", addr, status);
//						sprintf((char*)&dataseq1[0], "N:%02u %04llX:%05.2fm", num_neighbors, range_addr, range_result);
						sprintf((char*)&dataseq1[0], "N:%02u %04d:%05.2fm", num_neighbors, framelength, range_result);
					}
				}
				else //if(toggle == 2)
				{
					if(inst->addrByteSize == 8)
					{
						sprintf((char*)&dataseq[0], "%llX", addr);
						sprintf((char*)&dataseq1[0], "N:%02u H:%02u %05.2fm", num_neighbors, num_hidden, range_result);
					}
					else
					{
						struct TDMAHandler *tdma_handler = tdma_get_local_structure_ptr();
						uint8 framelength = tdma_handler->uwbListTDMAInfo[0].framelength;
						sprintf((char*)&dataseq[0], "%llX %s", addr, status);
//						sprintf((char*)&dataseq1[0], "H:%02u %04llX:%05.2fm", num_hidden, range_addr, range_result);
						sprintf((char*)&dataseq1[0], "H:%02u %04d:%05.2fm", num_hidden, framelength, range_result);
					}

				}

				writetoLCD(40, 1, dataseq); //send some data
				writetoLCD(16, 1, dataseq1); //send some data
			}
		}

#ifdef USB_SUPPORT //this is set in the port.h file
//        usb_run();
#endif

        if(canSleep)__WFI();
    }


    return 0;
}



