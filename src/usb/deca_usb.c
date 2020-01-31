/*! ----------------------------------------------------------------------------
 * @file	deca_usb.c
 * @brief	DecaWave USB application state machine for USB to SPI feature
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

/* Includes */
#include "deca_usb.h"

#include "instance.h"
#include "deca_types.h"
#include "deca_spi.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

/** @defgroup USB_VCP_Private_Variables
  * @{
  */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

#define SOFTWARE_VER_STRINGUSB "EVB1000 USB2SPI 2.0"


typedef enum applicationModes{STAND_ALONE, USB_TO_SPI, USB_PRINT_ONLY, NUM_APP_MODES} APP_MODE;

int application_mode = STAND_ALONE;
int localSPIspeed = -1;

//USB to SPI data buffers
int local_buff_length = 0;
uint8_t local_buff[10000];
uint16_t local_buff_offset = 0;
int tx_buff_length = 0;
uint8_t tx_buff[10000];
int local_have_data = 0;

int version_size;
uint8* version;
int s1configswitch;
extern int ranging;

extern uint32_t APP_Rx_length;
extern uint32 inittestapplication(uint8 s1switch);
extern void setLCDline1(uint8 s1switch);


uint16_t DW_VCP_Init     (void) { return USBD_OK; }
uint16_t DW_VCP_DeInit   (void) { return USBD_OK; }
uint16_t VCP_COMConfig(uint8_t Conf) { return USBD_OK; }

/**
  * @brief  DW_VCP_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
uint16_t DW_VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
   return USBD_OK;
}

//example functions to interface to USB VCOM
/**
  * @brief  DW_VCP_DataTx
  *         CDC received data to be send over USB IN endpoint are managed in
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else VCP_FAIL
  */
#pragma GCC optimize ("O3")
uint16_t DW_VCP_DataTx (uint8_t* Buf, uint32_t Len)
{
	int i = 0;

	i = APP_Rx_ptr_in ;

	if ((APP_Rx_length+Len) < APP_RX_DATA_SIZE)
	{
		int l;
		/* Get the data to be sent */
		for (l = 0; l < Len; l++)
		{
			APP_Rx_Buffer[i++] = Buf[l];
			/* Increment the in pointer */
			if (i>=APP_RX_DATA_SIZE)
				i=0;
		}
	}

	APP_Rx_ptr_in = i;

	return USBD_OK;
}


/**
  * @brief  DW_VCP_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         until exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (i.e. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
//#pragma GCC optimize ("O3")
uint16_t DW_VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
  uint32_t i;

  // ZS: This is where PC (USB Tx) data is received
  for (i = 0; i < Len; i++)
  {
	  if((i + local_buff_offset) < 10000)
	  {
		  local_buff[i + local_buff_offset] = Buf[i];
	  }
  }

  local_buff_length = Len + local_buff_offset;
  //

  if((local_buff[2] + (local_buff[3]<<8)) == local_buff_length)
  {
	  local_buff_offset = 0;
	  local_have_data = 1;
  }
  else
  {
	   if((local_buff_length == 5) && (local_buff[0] == 100))
	   {
		   local_buff_offset = 0;
		   local_have_data = 1;
	   }
	   else if((Len == 5) && (Buf[0] == 100)) //reset the buffer
	   {
		   local_buff_offset = 0;
		   local_have_data = 1;
		   for (i = 0; i < Len; i++)
		   {
			   local_buff[i + local_buff_offset] = Buf[i];
		   }
	   }
	   //we have received a antenna calibration or tx power configuration command
	   else if(local_buff_length == 6)
	   {
		   if(((local_buff[0] == 0x5) && (local_buff[5] == 0x5))
			   || ((local_buff[0] == 0x7) && (local_buff[5] == 0x7)))
		   {
			   local_buff_offset = 0;
			   local_have_data = 1;
		   }
	   }
	   //we have received a S1 configuration command
	   else if((local_buff_length == 3) && (local_buff[0] == 0x6) && (local_buff[2] == 0x6))
	   {
	       local_buff_offset = 0;
	   	   local_have_data = 1;
	   }
	   else
	   {
		   local_buff_offset += Len;
	   }
  }



  return USBD_OK;
}



void configSPIspeed(int high)
{
	if(localSPIspeed != high)
	{
		localSPIspeed = high;

		if(high)
			port_set_dw1000_fastrate();  //max SPI
		else
			port_set_dw1000_slowrate();  //max SPI before PLLs configured is ~4M
	}
}
#pragma GCC optimize ("O3")
int process_usbmessage(void)
{
	int result = 0;
	switch(application_mode)
	{
		case STAND_ALONE:
		{
			if(local_buff_length == 5)
			{
				//d (from "deca")
				if((local_buff[0] == 100) && (result == 0)) //d (from "deca")
				{
					if(local_buff[4] == 63)
					{
						int i = sizeof(SOFTWARE_VER_STRINGUSB);
						//change mode to  USB_TO_SPI and send a reply "y"
						tx_buff[0] = 121;
						memcpy(&tx_buff[1], SOFTWARE_VER_STRINGUSB, i);
						tx_buff[i+2] = 0;
						tx_buff_length = i + 2;
						application_mode = USB_TO_SPI;
						result = 2;
						//led_off(LED_ALL);
						led_on(LED_PC7); //turn on LED to indicate connection to PC application
					}
				}
			}
		}
		break;


		case USB_TO_SPI:
		{
			//first byte specifies the SPI speed and SPI read/write operation
			//bit 0 = 1 for write, 0 for read
			//bit 1 = 1 for high speed, 0 for low speed

			//<STX>   <ETX>
			//
			//to read from the device (e.g. 4 bytes), total length is = 1 + 1 + length_of_command (2) + length_of_data_to_read (2) + header length (1/2) + 1
			//
			//to write to the device (e.g. 4 bytes),  total length is = 1 + 1 + length_of_command (2) + length_of_data_to_write (2) + header length (1/2) + data length + 1
			//
			//LBS comes first:   0x2, 0x2, 0x7, 0x0, 0x04, 0x00, 0x3

			if(local_buff_length)
			{
				//0x2 = STX - start of SPI transaction data
				if(local_buff[0] == 0x2)
				{
					//configure SPI speed
					configSPIspeed(((local_buff[1]>>1) & 0x1));

					if((local_buff[1] & 0x1) == 0) //SPI read
					{
						int msglength = local_buff[2] + (local_buff[3]<<8);
						int datalength = local_buff[4] + (local_buff[5]<<8);

						//led_on(LED_PC6);
						tx_buff[0] = 0x2;
						tx_buff[datalength+2] = 0x3;

						//max data we can read in a single SPI transaction is 4093 as the USB/VCP tx buffer is only 4096 bytes long
						if((local_buff[msglength-1] != 0x3) || (datalength > 4093))
						{
							tx_buff[1] = 0x1; // if no ETX (0x3) indicate error
						}
						else
						{
							// do the read from the SPI
							readfromspi(msglength-7, &local_buff[6], datalength, &tx_buff[2]);  // result is stored in the buffer

							tx_buff[1] = 0x0; // no error
						}

						tx_buff_length = datalength + 3;
						result = 2;
					}

					if((local_buff[1] & 0x1) == 1) //SPI write
					{
						int msglength = local_buff[2] + (local_buff[3]<<8);
						int datalength = local_buff[4] + (local_buff[5]<<8);
						int headerlength = msglength - 7 - datalength;

						if(local_buff_length == msglength) //we got the whole message (sent from the PC)
						{
							local_buff_offset = 0;

							led_off(LED_PC6);
							tx_buff[0] = 0x2;
							tx_buff[2] = 0x3;

							if(local_buff[msglength-1] != 0x3)
							{
								tx_buff[1] = 0x1; // if no ETX (0x3) indicate error
							}
							else
							{
								// do the write to the SPI
								writetospi(headerlength, &local_buff[6], datalength, &local_buff[6+headerlength]);  // result is stored in the buffer

								tx_buff[1] = 0x0; // no error
							}

							tx_buff_length = 3;
							result = 2;
						}
						else //wait for the whole message
						{
							led_on(LED_PC6);
						}
					}
				}

				if((local_buff[0] == 100) && (result == 0)) //d (from "deca")
				{
					if(local_buff[4] == 63)
					{
						int i = sizeof(SOFTWARE_VER_STRINGUSB);
						//change mode to  USB_TO_SPI and send a reply "y"
						tx_buff[0] = 121;
						memcpy(&tx_buff[1], SOFTWARE_VER_STRINGUSB, i);
						tx_buff[i+2] = 0;
						tx_buff_length = i + 2;
						application_mode = USB_TO_SPI;
						result = 2;
						//led_off(LED_ALL);
						led_on(LED_PC7); //turn on LED to indicate connection to PC application
					}

				}

				if((local_buff[0] == 114) && (result == 0)) //r - flush the USB buffers...
				{
					DCD_EP_Flush(&USB_OTG_dev, CDC_IN_EP);
					result = 0;
				}
			}
		}
		break;

		case USB_PRINT_ONLY:
			if(local_buff_length && (result == 0))
			{
				if((local_buff[0] == 0x5) && (local_buff[5] == 0x5))
				{
					uint16 txantennadelay = local_buff[1] + (local_buff[2]<<8);
					uint16 rxantennadelay = local_buff[3] + (local_buff[4]<<8);
					instanceconfigantennadelays(txantennadelay, rxantennadelay);
				}
				if((local_buff[0] == 0x7) && (local_buff[5] == 0x7))
				{
					//not used in EVK
				}
				if((local_buff[0] == 0x6) && (local_buff[2] == 0x6))
				{
					uint8 switchS1 = local_buff[1];

					//disable DW1000 IRQ
					port_DisableEXT_IRQ(); //disable IRQ until we configure the device
					//turn DW1000 off
					dwt_forcetrxoff();
					//re-configure the instance
					inittestapplication(switchS1);
					//save the new setting
					s1configswitch = switchS1;
					//set the LDC
					setLCDline1(switchS1);
					//enable DW1000 IRQ
					port_EnableEXT_IRQ(); //enable IRQ before starting

					ranging = 0;

				}
				//d (from "deca")
				if(local_buff[0] == 100) //d (from "deca")
				{
					if(local_buff[4] == 63)
					{
						int i = sizeof(SOFTWARE_VER_STRINGUSB);
						//send a reply "n"
						tx_buff[0] = 110;
						memcpy(&tx_buff[1], SOFTWARE_VER_STRINGUSB, i);
						tx_buff[i+2] = 0;
						tx_buff_length = i + 2;
						result = 2;
					}
					if(local_buff[4] == 36) //"$"
					{
						//send a reply "n"
						tx_buff[0] = 110;
						memcpy(&tx_buff[1], version, version_size);
						tx_buff[version_size+1] = s1configswitch & 0xff;
						tx_buff[version_size+2] = '\r';
						tx_buff[version_size+3] = '\n';
						tx_buff_length = version_size + 4;
						result = 2;
					}
					if(local_buff[4] == 33) //"!"
					{
						//send back the DW1000 partID and lotID
						uint32 partID = dwt_getpartid();
						uint32 lotID = dwt_getlotid();
						tx_buff[0] = 110;
						memcpy(&tx_buff[1], &partID, 4);
						memcpy(&tx_buff[5], &lotID, 4);
						tx_buff[9] = '\r';
						tx_buff[10] = '\n';
						tx_buff_length = 11;
						result = 2;
					}
				}
			}
			break;

		default:
			break;
	}

	return result;
}
#pragma GCC optimize ("O3")
void send_usbmessage(uint8 *string, int len)
{
	if(local_have_data == 0)
	{
		memcpy(&tx_buff[0], string, len);
		tx_buff[len] = '\r';
		tx_buff[len+1] = '\n';
		tx_buff_length = len + 2;

		local_have_data = 2;
	}
}
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

void usb_printconfig(int size, uint8* string, int s1switch)
{
	application_mode = USB_PRINT_ONLY;

	s1configswitch = s1switch;
	version_size = size;
	version = string;
}


int usb_init(void)
{
	uint32 devID = 0;

	memset(local_buff, 0, sizeof(local_buff));
	led_off(LED_ALL); //to display error....

	// enable/initialise the USB functionality
	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);

	port_set_dw1000_slowrate();  //max SPI before PLLs configured is ~4M
    localSPIspeed = 0;

    //this is called here to wake up the device (i.e. if it was in sleep mode before the restart)
    port_wakeup_dw1000();

    devID = dwt_readdevid() ;
    if(DWT_DEVICE_ID != devID)
    {
        // SPI not working or Unsupported Device ID
    	led_on(LED_ALL); //to display error....
    }

    return 0;
}


#pragma GCC optimize ("O3")
void usb_run(void)
{
	static int i;
	// My assumption is we will get a trigger when USB is connected to a host and we can support host command to disable ARM
	// control of DW1000 (e.g. running DecaRanging) and operates as USB to SPI converter for PC DecaRanging Applications.
	// (If this does not work we use a switch on an IO port to select between USB2SPI pass-through and native-DecaRanging functions.

    // Do nothing in foreground -- allow USB application to run, I guess on the basis of USB interrupts?

    // loop forever, doing nothing except giving a bit of a LED flash every so often
    {
    	//if(application_mode == STAND_ALONE)
		{
    		i++ ;
			if (i == 0x0D0000) led_on(LED_PC8);
			if (i == 0x0E0000) led_off(LED_PC8);
			if (i == 0x0F0000) led_on(LED_PC8);
			if (i == 0x100000)
			{
				led_off(LED_PC8);
				i = 0;
				//send_usbmessage("test ", 5);
			}
		}

        if(local_have_data == 1)
        {
        	local_have_data = process_usbmessage();
        }
        else if(local_have_data == 2) //have data to send (over USB)
        {
        	DW_VCP_DataTx(tx_buff, tx_buff_length);
			local_have_data = 0;
        }

	  }
}



