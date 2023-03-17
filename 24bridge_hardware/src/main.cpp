/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 28/12/21

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This the transmitter part of a Serial bridge. This tranmitter receives data on
  the defined serial port and puts that serial data into a LoRa packet which is then transmitted. A
  matching reciever picks up the packet and displays it on the remote Arduinos serial port.

  The purpose of the bridge is to allow the serial output of a device, anothor Arduino or sensor for
  instance, to be remotely monitored, without the need for a long serial cable.

  Serial monitor baud rate should be set at 9600.
*******************************************************************************************************/

#include <SPI.h>                                //the LoRa device is SPI based so load the SPI library                                         
#include <SX128XLT.h>                           //include the appropriate library  

SX128XLT LoRa;                                  //create a library class instance called LoRa

#define TX_EN 26
#define RX_EN 27
#define NSS 5                                  //select pin on LoRa device
#define NRESET 14                                //reset pin on LoRa device
#define RFBUSY 21                                //busy pin on LoRa device 
#define DIO1 4                                  //DIO1 pin on LoRa device, used for sensing RX and TX done
#define LED1 15                                  //indicator LED
#define LORA_DEVICE DEVICE_SX1280               //we need to define the LoRa device we are using
#define TXpower 2                              //LoRa transmit power in dBm

uint8_t TXPacketL;                              //length of transmitted packet
uint8_t RXPacketL;                              //length of received acknowledge
uint16_t PayloadCRC;
const uint16_t MessageID = 257;                             //MessageID identifies each message, 2 bytes at front of packet
uint8_t receivedBytes;                          //count of serial bytes received
uint8_t Attempts;                               //number of times to send packet waiting for acknowledge
uint32_t startuS;                               //used for timeout for serial input
const uint8_t MaxMessageSize = 251;             //max size of array to send with received serial
uint8_t Message[MaxMessageSize];                //array for received serial data

uint8_t RXPayloadL;                             //stores length of payload received
uint16_t RXPayloadCRC;                          //CRC of payload included in received packet

uint32_t Frequency = 2445000000;          //frequency of transmissions
uint32_t Offset = 0;                      //offset frequency for calibration purposes
uint8_t Bandwidth = LORA_BW_0800;         //LoRa bandwidth
uint8_t SpreadingFactor = LORA_SF6;       //LoRa spreading factor
uint8_t CodeRate = LORA_CR_4_5;           //LoRa coding rate
uint16_t NetworkID = 0x1337;              //a unique identifier to go out with packet = 0x3210;

uint32_t SerialTimeoutuS = 50;                       //Timeout in uS before assuming message to send is complete
const uint32_t TimeoutCharacters = 10;          //number of characters at specified baud rate that are a serial timeout
const uint8_t MaxAttempts = 4;                  //Maximum attempts to send message and receive acknowledge
const uint32_t ACKtimeoutmS = 50;               //Acknowledge timeout in mS
const uint32_t TXtimeoutmS = 1000;              //transmit timeout in mS.
const uint32_t RXtimeout = 100 ;              //receive timeout in mS. set to 0 for no timeout
const uint32_t ACKdelay1 = 10;                  //delay in mS before sending acknowledge
const uint32_t ACKdelay2 = 20;                  //delay in mS before sending acknowledge for message already received

#define SerialInput Serial                      //assign serial port for reading data
#define SerialOutput Serial                      //assign serial port for reading data
#define SerialInputBaud 57600                    //baud rate for the serial 

bool RXFlag = false;
bool setMode = false;
bool oldMode = false;

#define DEBUG                                 //enable define to see debug information
#define SENDREADY                               //enable define to see Ready control message at start

#ifdef DEBUG
#define DebugSerial Serial                      //assign serial port for outputing debug data 
#endif

void processMessage(int MessageIDRX)
{
  uint8_t index, endpayload;

  endpayload = RXPacketL - 4;

  for (index = 2; index < endpayload; index++)
  {
    SerialOutput.write(Message[index]);
  }

#ifdef DEBUG
  DebugSerial.print(RXPacketL - 6);              //print number of characters in message
  DebugSerial.print(F(" "));
  DebugSerial.print(MessageIDRX);                  //print message ID
  DebugSerial.print(F(" > "));
  DebugSerial.println();                         //so each message is on new line
#endif
}

void processControlMessage(int MessageIDRX)
{
  if (MessageIDRX == 1)
  {
    #ifdef DEBUG
    DebugSerial.println(F("Ready"));
    #endif
  }
}

void packet_is_Error()
{
  uint16_t IRQStatus;
  IRQStatus = LoRa.readIrqStatus();                  //read the LoRa device IRQ status register
  #ifdef DEBUG
  DebugSerial.print(F("RXPacketL,"));
  DebugSerial.print(RXPacketL);
  DebugSerial.print(F(",IRQStatus,0x"));
  DebugSerial.println(IRQStatus, HEX);
  if (IRQStatus & IRQ_RX_TIMEOUT)                           //check for an RX timeout
  {
    DebugSerial.println(F("RXTimeout"));
  }
  #endif
}

void processPacket() {
    RXPacketL = LoRa.readPacket(Message, MaxMessageSize);

    int MessageIDRX = Message[0] + (Message[1] * 256);    //message number is first two bytes of Message array

    if (RXPacketL == 0)                             //check for a valid packet
    {
#ifdef DEBUG
      DebugSerial.print(MessageIDRX);
      DebugSerial.print(F("E>"));
      packet_is_Error();
#endif
      return;
    }
    
    if (MessageIDRX > 256)                  //new message should be greater than last message
    {
      processMessage(MessageIDRX);
      return;
    }

    if (MessageIDRX <= 256)                            //is it a control message
    {
      processControlMessage(MessageIDRX);
      return;
    }

    return;
}

void IRAM_ATTR recv() {
  detachInterrupt(DIO1);
  RXFlag = true;
}

void IRAM_ATTR send() {    
    detachInterrupt(DIO1);
    LoRa.receiveReliable(Message, MaxMessageSize, NetworkID, 0, NO_WAIT);
    attachInterrupt(DIO1, recv, HIGH);
}

void IRAM_ATTR mode() {
  setMode = digitalRead(0);
}

void loop()
{
  #ifdef DEBUG
  if (setMode != oldMode) {
    oldMode = setMode;
    DebugSerial.print("Mode: ");
    DebugSerial.println(setMode);
  }
  #endif
  if (RXFlag && SerialInput.available() == 0) {
    processPacket();
    LoRa.receiveReliable(Message, MaxMessageSize, NetworkID, 0, NO_WAIT);
    attachInterrupt(DIO1, recv, HIGH);
    RXFlag = false;
  }

  Message[0] = lowByte(MessageID);
  Message[1] = highByte(MessageID);

  receivedBytes = 2;
  startuS = millis();

  while (((uint32_t) (millis() - startuS) < SerialTimeoutuS))
  {
    if (SerialInput.available() > 0)
    {
      if (receivedBytes >= MaxMessageSize)
      {
        break;
      }
      Message[receivedBytes] = SerialInput.read();
      startuS = millis();
      receivedBytes++;
    }
  }

  //the only exits from the serial collection loop above are if there is a timeout, or the maximum
  //message size is reached.

  Attempts = 0;

  do
  {
    if (receivedBytes <= 2) break;
#ifdef DEBUG
    DebugSerial.println(F("> "));
    uint8_t index;
    DebugSerial.print(receivedBytes);
    DebugSerial.print(F(" "));
    DebugSerial.print(MessageID);
    DebugSerial.print(F(" > "));                                   //flag on monitor for transmitting

    for (index = 2; index < receivedBytes; index++)
    {
      DebugSerial.write(Message[index]);
    }
    DebugSerial.println();
#endif
    LoRa.transmitReliable(Message, receivedBytes, NetworkID, TXtimeoutmS, TXpower, NO_WAIT);
    attachInterrupt(DIO1, send, HIGH);
    break;
  }
  while ((RXPacketL == 0) && (Attempts <= MaxAttempts));
}


void setup()
{
  pinMode(LED1, OUTPUT);

#ifdef DEBUG
  DebugSerial.begin(SerialInputBaud);
#endif
  SerialInput.begin(SerialInputBaud);

#ifdef DEBUG
  DebugSerial.println();
  Serial.println(F(__FILE__));
  DebugSerial.println();
#endif

  SPI.begin();

  if (LoRa.begin(NSS, NRESET, RFBUSY, DIO1, RX_EN, TX_EN, LORA_DEVICE))
  {
#ifdef DEBUG
    DebugSerial.println(F("LoRa Device found"));
#endif
  }
  else
  {
    SerialInput.println(F("No LoRa device responding"));
    while (1);
  }

  LoRa.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

#ifdef SENDREADY
  //Send a ready packet so remorte can check its working, send control message 0x0001
  Message[0] = 1;
  Message[1] = 0;
  Message[2] = 'R';
  TXPacketL = LoRa.transmitReliable(Message, 3, NetworkID, TXtimeoutmS, TXpower, WAIT_TX);
  delay(1000);
#endif

  //now calculate timeout in microseconds based on baud rate and number of characters, assuming a 11bit byte

  //SerialTimeoutuS = ((1000000 / SerialInputBaud) * 11) * TimeoutCharacters;

#ifdef DEBUG
  DebugSerial.print(F("SerialTimeoutuS "));
  DebugSerial.println(SerialTimeoutuS);
  DebugSerial.println(F("Clear serial buffer"));
#endif

  while (SerialInput.available() > 0)           //clear serial input
  {
    SerialInput.read();
  }

#ifdef DEBUG
  DebugSerial.println(F("Waiting start of serial input"));
#endif

  LoRa.receiveReliable(Message, MaxMessageSize, NetworkID, 0, NO_WAIT);
  attachInterrupt(DIO1, recv, HIGH);
  attachInterrupt(0, mode, CHANGE);
}
