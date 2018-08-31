/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    usb_esp32.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "usb_esp32.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

USB_ESP32_DATA usb_esp32Data;
/* Static buffers, suitable for DMA transfer */
#define USB_ESP32_MAKE_BUFFER_DMA_READY  __attribute__((coherent)) __attribute__((aligned(16)))

static uint8_t USB_ESP32_MAKE_BUFFER_DMA_READY writeBuffer[USB_ESP32_USB_CDC_COM_PORT_SINGLE_WRITE_BUFFER_SIZE];
static uint8_t writeString[] = "Hello World\r\n";
static uint8_t USB_ESP32_MAKE_BUFFER_DMA_READY readBuffer [USB_ESP32_USB_CDC_COM_PORT_SINGLE_READ_BUFFER_SIZE];
static uint8_t readString[8];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************


/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE USB_ESP32_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void * pData,
    uintptr_t userData
)
{
    USB_ESP32_DATA * appDataObject;
    appDataObject = (USB_ESP32_DATA *)userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent.  */
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:
            /* This means that the host has sent some data*/
            appDataObject->readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
			readString[appDataObject->readProcessedLen] = readBuffer[0];
			if (appDataObject->readProcessedLen < 8)
			{
            	appDataObject->readProcessedLen++;
			}
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void USB_ESP32_USBDeviceEventHandler ( USB_DEVICE_EVENT event, void * eventData, uintptr_t context )
{
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch ( event )
    {
        case USB_DEVICE_EVENT_SOF:
            break;

        case USB_DEVICE_EVENT_RESET:

            usb_esp32Data.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData;
            if ( configuredEventData->configurationValue == 1)
            {
                /* Register the CDC Device application event handler here.
                 * Note how the usb_esp32Data object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, USB_ESP32_USBDeviceCDCEventHandler, (uintptr_t)&usb_esp32Data);

                /* Mark that the device is now configured */
                usb_esp32Data.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(usb_esp32Data.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(usb_esp32Data.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
  Function:
    static void USB_TX_Task (void)
    
   Remarks:
    Feeds the USB write function. 
*/
static void USB_TX_Task (void)
{
    if(!usb_esp32Data.isConfigured)
    {
        usb_esp32Data.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
    }
    else
    {
        /* Schedule a write if data is pending 
         */
        if ((usb_esp32Data.writeLen > 0)/* && (usb_esp32Data.writeTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID)*/)
        {
            USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                                 &usb_esp32Data.writeTransferHandle,
                                 writeBuffer, 
                                 usb_esp32Data.writeLen,
                                 USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
        }
    }
}

/******************************************************************************
  Function:
    static void USB_RX_Task (void)
    
   Remarks:
    Reads from the USB. 
*/
static void USB_RX_Task(void)
{
    if(!usb_esp32Data.isConfigured)
    {
        usb_esp32Data.readTransferHandle  = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        usb_esp32Data.readProcessedLen    = 0;
    }
    else
    {
        /* Schedule a read if none is pending and all previously read data
           has been processed
         */
        if((usb_esp32Data.readProcessedLen < 8) && (usb_esp32Data.readTransferHandle  == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID))
        {
            USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0,
                                 &usb_esp32Data.readTransferHandle, 
                                 readBuffer,
                                 USB_ESP32_USB_CDC_COM_PORT_SINGLE_READ_BUFFER_SIZE);
        };
    }
}

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void USB_ESP32_Initialize ( void )

  Remarks:
    See prototype in usb_esp32.h.
 */

void USB_ESP32_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    usb_esp32Data.state = USB_ESP32_STATE_INIT;


    /* Device Layer Handle  */
    usb_esp32Data.deviceHandle = USB_DEVICE_HANDLE_INVALID ;

    /* Device configured status */
    usb_esp32Data.isConfigured = false;

    /* Initial get line coding state */
    usb_esp32Data.getLineCodingData.dwDTERate   = 9600;
    usb_esp32Data.getLineCodingData.bParityType =  0;
    usb_esp32Data.getLineCodingData.bParityType = 0;
    usb_esp32Data.getLineCodingData.bDataBits   = 8;

    /* Read Transfer Handle */
    usb_esp32Data.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read data */
    usb_esp32Data.readProcessedLen = 0;

    /* Write Transfer Handle */
    usb_esp32Data.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
    
    /*Initialize the write data */
    usb_esp32Data.writeLen = sizeof(writeString);
	memcpy(writeBuffer, writeString, usb_esp32Data.writeLen);
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void USB_ESP32_Tasks ( void )

  Remarks:
    See prototype in usb_esp32.h.
 */

void USB_ESP32_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( usb_esp32Data.state )
    {
        /* Application's initial state. */
        case USB_ESP32_STATE_INIT:
        {
            bool appInitialized = true;
       

            /* Open the device layer */
            if (usb_esp32Data.deviceHandle == USB_DEVICE_HANDLE_INVALID)
            {
                usb_esp32Data.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                                               DRV_IO_INTENT_READWRITE );
                appInitialized &= ( USB_DEVICE_HANDLE_INVALID != usb_esp32Data.deviceHandle );
            }
        
            if (appInitialized)
            {

                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(usb_esp32Data.deviceHandle,
                                           USB_ESP32_USBDeviceEventHandler, 0);
            
                usb_esp32Data.state = USB_ESP32_STATE_SERVICE_TASKS;
            }
            break;
        }

        case USB_ESP32_STATE_SERVICE_TASKS:
        {
            USB_RX_Task();
            USB_TX_Task();
        
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
