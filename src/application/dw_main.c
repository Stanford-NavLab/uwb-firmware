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

#include "deca_types.h"

#include "deca_spi.h"
#include "stdio.h"

extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);

							// "1234567890123456" - 16 bytes long LCD
#define SOFTWARE_VER_STRING    "Version 3.11    " //

#define SWS1_TXSPECT_MODE	0x80  //Continuous TX spectrum mode
#define SWS1_ANC_MODE 		0x08  //anchor mode
#define SWS1_SHF_MODE		0x10  //short frame mode (6.81M) (switch S1-5)
#define SWS1_64M_MODE		0x20  //64M PRF mode (switch S1-6)
#define SWS1_CH5_MODE		0x40  //channel 5 mode (switch S1-7)

int dr_mode = 0;
int instance_mode = ANCHOR;

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


uint32 inittestapplication(uint8 s1switch);


int decarangingmode(uint8 s1switch)
{
    int mode = 0;

    if(s1switch & SWS1_SHF_MODE)
    {
        mode = 1;
    }

    if(s1switch & SWS1_64M_MODE)
    {
        mode = mode + 2;
    }
    if(s1switch & SWS1_CH5_MODE)
    {
        mode = mode + 4;
    }

    return mode;
}

uint32 inittestapplication(uint8 s1switch)
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

    port_set_dw1000_fastrate();
    devID = instancereaddeviceid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }

    if(s1switch & SWS1_ANC_MODE)
    {
        instance_mode = ANCHOR;

        led_on(LED_PC6);

    }
    else
    {
        instance_mode = TAG;
        led_on(LED_PC7);
    }

    instance_init_s(instance_mode);
    dr_mode = decarangingmode(s1switch);

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

void setLCDline1(uint8 s1switch)
{
	uint8 command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);

	sprintf((char*)&dataseq[0], "DecaRanging  %02x", s1switch);
	writetoLCD( 40, 1, dataseq); //send some data

	sprintf((char*)&dataseq1[0], "                 ");
	writetoLCD( 16, 1, dataseq1); //send some data
}

/*
 * @fn configure_continuous_txspectrum_mode
 * @brief   test application for production to check the TX power in various modes
**/
void configure_continuous_txspectrum_mode(uint8 s1switch)
{
    uint8 command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);
	sprintf((char*)&dataseq[0], "Conti TX %s:%d:%d ", (s1switch & SWS1_SHF_MODE) ? "S" : "L", chan, prf);
	writetoLCD( 40, 1, dataseq); //send some data
	memcpy(dataseq, (const uint8 *) "Spectrum Test   ", 16);
	writetoLCD( 16, 1, dataseq); //send some data

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
    int toggle = 1;
    double range_result = 0;
    double avg_result = 0;
    int canSleep;

    led_off(LED_ALL); //turn off all the LEDs

    peripherals_init();

    spi_peripheral_init();

    Sleep(1000); //wait for LCD to power on

    initLCD();

    memset(dataseq, 0x0, sizeof(dataseq));
    memcpy(dataseq, (const uint8 *) "DECAWAVE        ", 16);
    writetoLCD( 40, 1, dataseq); //send some data
    memcpy(dataseq, (const uint8 *) SOFTWARE_VER_STRING, 16); // Also set at line #26 (Should make this from single value !!!)
    writetoLCD( 16, 1, dataseq); //send some data

    Sleep(1000);

    port_DisableEXT_IRQ(); 	//disable DW1000 IRQ until we configure the application

#ifdef USB_SUPPORT
    // enable the USB functionality
    usb_init();
    Sleep(1000);
#endif

    s1switch = port_is_boot1_on(0) << 1 // is_switch_on(TA_SW1_2) << 2
    		| port_is_switch_on(TA_SW1_3) << 2
    		| port_is_switch_on(TA_SW1_4) << 3
    		| port_is_switch_on(TA_SW1_5) << 4
		    | port_is_switch_on(TA_SW1_6) << 5
    		| port_is_switch_on(TA_SW1_7) << 6
    		| port_is_switch_on(TA_SW1_8) << 7;

    if(port_is_switch_on(TA_SW1_3) == S1_SWITCH_OFF)
    {
        int j = 1000000;
        uint8 command;

        memset(dataseq, 0, LCD_BUFF_LEN);

        while(j--);
        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);

        memcpy(dataseq, (const uint8 *) "DECAWAVE   ", 12);
        writetoLCD( 40, 1, dataseq); //send some data
#ifdef USB_SUPPORT //this is set in the port.h file
        memcpy(dataseq, (const uint8 *) "USB to SPI ", 12);
#else
#endif
        writetoLCD( 16, 1, dataseq); //send some data

        j = 1000000;

        while(j--);

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
#ifdef USB_SUPPORT //this is set in the port.h file
        // Do nothing in foreground -- allow USB application to run, I guess on the basis of USB interrupts?
        while (1)       // loop forever
        {
            usb_run();
        }
#endif
        return 1;
    }
    else //run DecaRanging application
    {
        uint8 dataseq[LCD_BUFF_LEN];
        uint8 command = 0x0;

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
        memset(dataseq, ' ', LCD_BUFF_LEN);
        memcpy(dataseq, (const uint8 *) "DECAWAVE   RANGE", 16);
        writetoLCD( 16, 1, dataseq); //send some data

        led_off(LED_ALL);

#ifdef USB_SUPPORT //this is set in the port.h file
        usb_printconfig(16, (uint8 *)SOFTWARE_VER_STRING, s1switch);
#endif

        if(inittestapplication(s1switch) == (uint32)-1)
        {
            led_on(LED_ALL); //to display error....
            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  &dataseq[0]);
            memset(dataseq, ' ', LCD_BUFF_LEN);
            memcpy(dataseq, (const uint8 *) "ERROR   ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            memcpy(dataseq, (const uint8 *) "  INIT FAIL ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
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
            if (i & 1) led_off(LED_ALL);
            else    led_on(LED_ALL);

            Sleep(200);
        }
        i = 0;
        led_off(LED_ALL);
        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);

        memset(dataseq, ' ', LCD_BUFF_LEN);

        if(s1switch & SWS1_ANC_MODE)
        {
            instance_mode = ANCHOR;

            led_on(LED_PC6);
        }
        else
        {
            instance_mode = TAG;
            led_on(LED_PC7);
        }

        if(instance_mode == TAG)
        {
			memcpy(&dataseq[2], (const uint8 *) " TAG BLINK  ", 12);

			writetoLCD( 40, 1, dataseq); //send some data
			sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
			writetoLCD( 16, 1, dataseq); //send some data
        }
        else
        {
            memcpy(&dataseq[2], (const uint8 *) "  AWAITING  ", 12);
            writetoLCD( 40, 1, dataseq); //send some data
            memcpy(&dataseq[2], (const uint8 *) "    POLL    ", 12);
            writetoLCD( 16, 1, dataseq); //send some data
        }

        command = 0x2 ;  //return cursor home
        writetoLCD( 1, 0,  &command);
    }

    memset(dataseq, ' ', LCD_BUFF_LEN);
    memset(dataseq1, ' ', LCD_BUFF_LEN);

    port_EnableEXT_IRQ();
    // main loop
    while(1)
    {
		instance_data_t* inst = instance_get_local_structure_ptr(0);
		canSleep = instance_run();

        //if delayed TX scheduled but did not happen after expected time then it has failed... (has to be < slot period)
        //if anchor just go into RX and wait for next message from tags/anchors
        //if tag handle as a timeout
        if((inst->monitor == 1) && ((portGetTickCnt() - inst->timeofTx) > inst->finalReplyDelay_ms))
        {
			inst->wait4ack = 0;

			if(instance_mode == TAG)
			{
				inst_processrxtimeout(inst);
			}
			else //if(instance_mode == ANCHOR)
			{
				dwt_forcetrxoff();	//this will clear all events
				//enable the RX
				inst->testAppState = TA_RXE_WAIT ;
			}
			inst->monitor = 0;
        }

        if(instancenewrange())
        {
        	int n, l = 0, /*txl = 0, rxl = 0,*/ aaddr, taddr, txa, rxa, rng, rng_raw;
            ranging = 1;
            //send the new range information to LCD and/or USB
            range_result = instance_get_idist();
            avg_result = instance_get_adist();
            //set_rangeresult(range_result);
            dataseq[0] = 0x2 ;  //return cursor home
            writetoLCD( 1, 0,  dataseq);

            memset(dataseq, ' ', LCD_BUFF_LEN);
            memset(dataseq1, ' ', LCD_BUFF_LEN);
            sprintf((char*)&dataseq[1], "LAST: %4.2f m", range_result);
            writetoLCD( 40, 1, dataseq); //send some data

            sprintf((char*)&dataseq1[1], "AVG8: %4.2f m", avg_result);

            writetoLCD( 16, 1, dataseq1); //send some data

            l = instance_get_lcount();
            //txl = instance_get_txl();
            //rxl = instance_get_rxl();
            aaddr = instancenewrangeancadd();
            taddr = instancenewrangetagadd();
            txa =  instancetxantdly();
            rxa =  instancerxantdly();
            rng = (int) (range_result*1000);
            rng_raw = (int) (instance_get_idistraw()*1000);

            if(instance_mode == TAG)
            {
                //n = sprintf((char*)&dataseq[0], "ia%04x t%04x %04x %04x %04x %04x %04x %02x %02x t", aaddr, taddr, rng, rng_raw, l, txa, rxa, txl, rxl);
            	n = sprintf((char*)&dataseq[0], "ia%04x t%04x %08x %08x %04x %04x %04x t", aaddr, taddr, rng, rng_raw, l, txa, rxa);
            }
            else
            {
                //n = sprintf((char*)&dataseq[0], "ia%04x t%04x %04x %04x %04x %04x %04x %02x %02x a", aaddr, taddr, rng, rng_raw, l, txa, rxa, txl, rxl);
            	//n = sprintf((char*)&dataseq[0], "ia%04x t%04x %08x %08x %04x %04x %04x %2.2f a", aaddr, taddr, rng, rng_raw, l, txa, rxa, instance_data[0].clockOffset);
            	n = sprintf((char*)&dataseq[0], "ia%04x t%04x %08x %08x %04x %04x %04x a", aaddr, taddr, rng, rng_raw, l, txa, rxa);
            }
#ifdef USB_SUPPORT //this is set in the port.h file
            send_usbmessage(&dataseq[0], n);
#endif
        }

        if(ranging == 0)
        {
            if(instance_mode != ANCHOR)
            {
                if(instancesleeping())
                {
                    dataseq[0] = 0x2 ;  //return cursor home
                    writetoLCD( 1, 0,  dataseq);
                    if(toggle)
                    {
                        toggle = 0;
                        memcpy(&dataseq[0], (const uint8 *) "    AWAITING    ", 16);
                        writetoLCD( 40, 1, dataseq); //send some data
                        memcpy(&dataseq[0], (const uint8 *) "    RESPONSE    ", 16);
                        writetoLCD( 16, 1, dataseq); //send some data
                    }
                    else
                    {
                        toggle = 1;
                        memcpy(&dataseq[2], (const uint8 *) "   TAG BLINK    ", 16);

                        writetoLCD( 40, 1, dataseq); //send some data
                        sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
                        writetoLCD( 16, 1, dataseq); //send some data
                    }
                }

                if(instanceanchorwaiting() == 2)
                {
                    ranging = 1;
                    dataseq[0] = 0x2 ;  //return cursor home
                    writetoLCD( 1, 0,  dataseq);
                    memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
                    writetoLCD( 40, 1, dataseq); //send some data
                    sprintf((char*)&dataseq[0], "%016llX", instance_get_anchaddr());
                    writetoLCD( 16, 1, dataseq); //send some data
                }
            }
            else //if(instance_mode == ANCHOR)
            {
                if(instanceanchorwaiting())
                {
                    toggle+=2;

                    if(toggle > 300000)
                    {
                        dataseq[0] = 0x2 ;  //return cursor home
                        writetoLCD( 1, 0,  dataseq);
                        if(toggle & 0x1)
                        {
                            toggle = 0;
                            memcpy(&dataseq[0], (const uint8 *) "    AWAITING    ", 16);
                            writetoLCD( 40, 1, dataseq); //send some data
                            memcpy(&dataseq[0], (const uint8 *) "      POLL      ", 16);
                            writetoLCD( 16, 1, dataseq); //send some data
                        }
                        else
                        {
                            toggle = 1;
                            memcpy(&dataseq[0], (const uint8 *) " DISCOVERY MODE ", 16);
                            writetoLCD( 40, 1, dataseq); //send some data
                            sprintf((char*)&dataseq[0], "%llX", instance_get_addr());
                            writetoLCD( 16, 1, dataseq); //send some data
                        }
                    }

                }
                else if(instanceanchorwaiting() == 2)
                {
                    dataseq[0] = 0x2 ;  //return cursor home
                    writetoLCD( 1, 0,  dataseq);
                    memcpy(&dataseq[0], (const uint8 *) "    RANGING WITH", 16);
                    writetoLCD( 40, 1, dataseq); //send some data
                    sprintf((char*)&dataseq[0], "%llX", instance_get_tagaddr());
                    writetoLCD( 16, 1, dataseq); //send some data
                }
            }
        }
#ifdef USB_SUPPORT //this is set in the port.h file
        usb_run();
#endif

        if(canSleep)__WFI();
    }


    return 0;
}



