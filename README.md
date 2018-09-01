# usb_esp32
<br>

## ソースコードの改良した主な箇所

<br>

改良したソースコードは、usb_esp32.h、usb_esp32.cの2つ。変更箇所は/*** add ***/ のコメントを入れている。

<br>

### usb_esp32.h

<br>

#### 状態と、変数を追加

<br>
<pre>
/*** 略 ***/

/*** add ***/
// ESP32→USB 通信 の状態
typedef enum
{
    ESP32_TO_USB_STATE_INIT=0,      // UARTデータ受信待ち
    ESP32_TO_USB_STATE_SEND,        // USBデータ送信中
} ESP32_TO_USB_STATES;

// USB→ESP32 通信 の状態
typedef enum
{
    USB_TO_ESP32_STATE_INIT=0,      // USBデータ受信開始
    USB_TO_ESP32_STATE_WAIT,        // USBデータ受信待ち、UARTデータ送信完了待ち
    USB_TO_ESP32_STATE_SEND,        // UARTデータ送信中
} USB_TO_ESP32_STATES;
/*** add ***/

/*** 略 ***/

typedef struct
{

    /*** 略 ***/

    /*** add ***/
    ESP32_TO_USB_STATES esp32_to_usb_State;   // ESP32→USB 通信の状態
    USB_TO_ESP32_STATES usb_to_esp32_State;   // USB→ESP32 通信の状態
    DRV_HANDLE handleUSART0;                  // UARTのハンドル
    /*** add ***/
} USB_ESP32_DATA;
</pre>

### usb_esp32.c

#### グローバル変数追加

usbからの読み込みバッファ、書き込みバッファは既存のものを流用する。

<pre>
/*** add ***/
static uint16_t readBufferSize;                                                 // usbから読み込んだデータのサイズ
static uint16_t writeBufferSize;                                                // usbに書き込むデータのサイズ
static uint8_t sendBuffer[USB_ESP32_USB_CDC_COM_PORT_SINGLE_READ_BUFFER_SIZE];  // uart送信データバッファ
static uint16_t sendBufferIdx;                                                  // uart送信データバッファのインデックス
static uint16_t sendBufferSize;                                                 // uart送信データバッファのサイズ
static uint8_t recvBuffer[USB_ESP32_USB_CDC_COM_PORT_SINGLE_WRITE_BUFFER_SIZE]; // uart受信データバッファ
static uint16_t recvBufferSize;                                                 // uart受信データバッファのサイズ
/*** add ***/
</pre>

初期処理追加。UARTの受信、送信コールバック設定。ハンドル取得。

<pre>
void USB_ESP32_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( usb_esp32Data.state )
    {
        /* Application's initial state. */
        case USB_ESP32_STATE_INIT:
        {
            /*** 略 ***/
          
            /*** add ***/
            DRV_USART_ByteTransmitCallbackSet(USB_DEVICE_INDEX_0, callbackSend);  // UART 送信コールバック
            DRV_USART_ByteReceiveCallbackSet(USB_DEVICE_INDEX_0, callbackRecv);   // UART 受信コールバック
            if (usb_esp32Data.handleUSART0 == DRV_HANDLE_INVALID)
            {
                // UART ハンドル取得
                usb_esp32Data.handleUSART0 = DRV_USART_Open(DRV_USART_INDEX_0, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
                appInitialized &= (DRV_HANDLE_INVALID != usb_esp32Data.handleUSART0);
            }
            /*** add ***/
          
            /*** 略 ***/
        }
    /*** 略 ***/
    }
/*** 略 ***/
}
</pre>
<br>

#### USBライブラリからのイベントコールバック

<br>

受信完了時( USB_DEVICE_CDC_EVENT_READ_COMPLETE ) と送信完了時 ( USB_DEVICE_CDC_EVENT_WRITE_COMPLETE )

<pre>
USB_DEVICE_CDC_EVENT_RESPONSE USB_ESP32_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void * pData,
    uintptr_t userData
)
{
    /*** 略 ***/
    
    switch ( event )
    {
        /*** 略 ***/
        
        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:
            /* This means that the host has sent some data*/
            /*** add ***/
            // 元のコードをコメントアウト
            //appDataObject->readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
			      //readString[appDataObject->readProcessedLen] = readBuffer[0];
			      //if (appDataObject->readProcessedLen < 8)
			      //{
            //	  appDataObject->readProcessedLen++;
			      //}
            // UART 送信状態にする
            usb_esp32Data.usb_to_esp32_State = USB_TO_ESP32_STATE_SEND;
            /*** add ***/
            break;
            
        /*** 略 ***/
        
        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the host has sent some data*/
            appDataObject->writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            /*** add ***/
            // UARTデータ受信待ち 状態にする。
            usb_esp32Data.esp32_to_usb_State = ESP32_TO_USB_STATE_INIT;
            /*** add ***/
            break;
        /*** 略 ***/
    }
    /*** 略 ***/
}
</pre>

<br>

#### USB送信定期処理

<br>

UART から受信したデータをUSB送信する。

<pre>
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
        /*** add ***/
        // 元の処理をコメントアウト
        //if ((usb_esp32Data.writeLen > 0)/* && (usb_esp32Data.writeTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID)*/)
        //{
        //    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
        //                         &usb_esp32Data.writeTransferHandle,
        //                         writeBuffer, 
        //                         usb_esp32Data.writeLen,
        //                         USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
        //}
        switch(usb_esp32Data.esp32_to_usb_State){
            case ESP32_TO_USB_STATE_INIT:
            {
                // UART受信データあり
                if( recvBufferSize > 0 ){
                    SYS_INT_Disable();
                    memcpy(writeBuffer, recvBuffer, recvBufferSize);
                    writeBufferSize = recvBufferSize;
                    recvBufferSize = 0;
                    SYS_INT_Enable();
                    usb_esp32Data.esp32_to_usb_State = ESP32_TO_USB_STATE_SEND;
                    
                    // UARTから受信したデータをUSB送信する
                    USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &usb_esp32Data.writeTransferHandle,
                        writeBuffer, 
                        writeBufferSize,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                }
                break;
            }
        }
        /*** add ***/
    }
}
</pre>

<br>

#### USB受信定期処理

<br>

USBから受信したデータをUART送信する。

<pre>
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
        /*** add ***/
        // 元の処理をコメントアウト
        //if((usb_esp32Data.readProcessedLen < 8) && (usb_esp32Data.readTransferHandle  == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID))
        //{
        //    USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0,
        //                         &usb_esp32Data.readTransferHandle, 
        //                         readBuffer,
        //                         USB_ESP32_USB_CDC_COM_PORT_SINGLE_READ_BUFFER_SIZE);
        //};
        switch(usb_esp32Data.usb_to_esp32_State){
            case USB_TO_ESP32_STATE_INIT:
            {
                // USBデータ受信処理開始
                memset(readBuffer, 0x00, sizeof(readBuffer));
                USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0,
                                 &usb_esp32Data.readTransferHandle, 
                                 readBuffer,
                                 USB_ESP32_USB_CDC_COM_PORT_SINGLE_READ_BUFFER_SIZE);
                usb_esp32Data.usb_to_esp32_State = USB_TO_ESP32_STATE_WAIT;
                break;
            }
            case USB_TO_ESP32_STATE_SEND:
            {
                // USB からデータを受信したらUART送信する。
                strcpy(sendBuffer, readBuffer);
                sendBufferSize = strlen(readBuffer);
                
                usb_esp32Data.usb_to_esp32_State = USB_TO_ESP32_STATE_WAIT;
                
                sendBufferIdx = 0;
                send();
                break;
            }
            default:
                break;
        }
        /*** add ***/
    }
}
</pre>

<br>

#### UART 送信関数

<br>
1Byteずつデータを送信する。

<pre>
/*** add ***/
static void send(void){
    if( sendBufferIdx >= sendBufferSize){
        usb_esp32Data.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        usb_esp32Data.usb_to_esp32_State = USB_TO_ESP32_STATE_INIT;
    }else{
        DRV_USART_WriteByte( usb_esp32Data.handleUSART0, sendBuffer[sendBufferIdx] );
    }
    return;
}
</pre>

<br>

#### UART送信コールバック関数

<br>
送信完了後呼ばれる。次のデータを送信する。

<pre>
static void callbackSend(const SYS_MODULE_INDEX index){
    sendBufferIdx++;
    send();
    return;
}
</pre>

<br>

#### UART受信コールバック関数

<br>
データを受信したら呼ばれる。データを受信したら受信バッファに保持する。

<pre>
static void callbackRecv(const SYS_MODULE_INDEX index){
    uint8_t bData;

    if (!DRV_USART_ReceiverBufferIsEmpty(usb_esp32Data.handleUSART0)) {
        bData = DRV_USART_ReadByte(usb_esp32Data.handleUSART0);
        
        recvBuffer[readBufferSize] = bData;
        recvBufferSize++;
        if( recvBufferSize >= sizeof(recvBuffer)){
            recvBufferSize = 0;
        }
    }
    return;
}

/*** add ***/
</pre>
