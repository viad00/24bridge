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
#include <crc8.h>

SX128XLT LoRa;                                  //create a library class instance called LoRa

#define TX_EN 26
#define RX_EN 27
#define NSS 5                                  //select pin on LoRa device
#define NRESET 14                                //reset pin on LoRa device
#define RFBUSY 21                                //busy pin on LoRa device 
#define DIO1 4                                  //DIO1 pin on LoRa device, used for sensing RX and TX done
#define LED1 15                                  //indicator LED
#define LORA_DEVICE DEVICE_SX1280               //we need to define the LoRa device we are using
#define TXpower tx_power                              //LoRa transmit power in dBm

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
uint8_t tx_power = 2;

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

//#define DEBUG                                 //enable define to see debug information
#define SENDREADY                               //enable define to see Ready control message at start

#ifdef DEBUG
#define DebugSerial Serial                      //assign serial port for outputing debug data 
#endif

struct ControlComm {
    uint32_t magic;
    uint8_t request;
    uint8_t answer;
    uint8_t ttl;
    uint8_t mask;
    uint32_t value;
    uint8_t crc;
};

#define MAGIC 0xFACEFEED
#define ES24_HELLO 0
#define ANS 1
#define ES24_FREQ 2
#define ES24_OFFSET 3
#define ES24_BANDWIDTH 4
#define ES24_SF 5
#define ES24_CR 6
#define ES24_NETWORKID 7
#define ES24_SERTIMEOUT 8
#define ES24_TXPOWER 9
#define ES24_RESET 10

void sendMessage(ControlComm s) {
  Message[0] = 2;
  Message[1] = 0;
  rlz_lib::Crc8::Generate((unsigned char*)&s + 4, 4+4, &s.crc);
  memcpy(&Message[2], &s, sizeof(struct ControlComm));
  LoRa.transmitReliable(Message, 2+sizeof(struct ControlComm)-3, NetworkID, TXtimeoutmS, TXpower, WAIT_TX);
  #ifdef DEBUG
  DebugSerial.printf("send %X %hhd %hhd %hhX %hhX %X %hhX\n", s.magic, s.request, s.answer, s.ttl, s.mask, s.value, s.crc);
    for (int i = 0; i < sizeof(struct ControlComm)-3; i++) {
        DebugSerial.printf("\\x%hhX", *((uint8_t*) &s + i));
    }
    DebugSerial.printf("\n");
  #endif
  SerialOutput.write((uint8_t*)&s, sizeof(struct ControlComm));
}

void parseBuff(uint8_t* buff, uint8_t size) {
    unsigned int pattern[] = { MAGIC };
    void* pointer = memmem(buff, size, pattern, sizeof(pattern));
    if (pointer == NULL) return;
    ControlComm value = *((ControlComm*)pointer);
    bool check = false;
    rlz_lib::Crc8::Verify((unsigned char*)&value + 4, 4+4, value.crc, &check);
    #ifdef DEBUG
    DebugSerial.printf("recv %X %hhd %X %hhX %hhd\n", value.magic, value.request, value.value, value.crc, check);
    for (int i = 0; i < sizeof(struct ControlComm); i++) {
        DebugSerial.printf("\\x%hhX", *((uint8_t*) &value + i));
    }
    DebugSerial.printf("\n");
    #endif
    uint8_t run = (value.ttl & value.mask);
    if (check && value.ttl && !run) {
      ControlComm ans = value;
      ans.ttl = ans.ttl >> 1;
      ans.crc = 0;
      sendMessage(ans);
    }
    if (check && value.ttl && run) {
      ControlComm ans;
      switch (value.request)
      {
      case ES24_HELLO:
        ans = { MAGIC, ES24_HELLO, ANS, (uint8_t)(value.ttl >> 1), value.mask, (uint32_t)((value.answer << 16) + (value.ttl << 8) + value.mask), 0};
        sendMessage(ans);
        break;
      case ES24_FREQ:
        Frequency = value.value;
        ans = { MAGIC, ES24_FREQ, ANS, (uint8_t)(value.ttl >> 1), value.mask, Frequency, 0 };
        sendMessage(ans);
        LoRa.setRfFrequency(Frequency, Offset);
        break;
      case ES24_OFFSET:
        Offset = value.value;
        ans = { MAGIC, ES24_OFFSET, ANS, (uint8_t)(value.ttl >> 1), value.mask, Offset, 0 };
        sendMessage(ans);
        LoRa.setRfFrequency(Frequency, Offset);
        break;
      case ES24_BANDWIDTH:
        Bandwidth = (uint8_t) value.value;
        ans = { MAGIC, ES24_BANDWIDTH, ANS, (uint8_t)(value.ttl >> 1), value.mask, Bandwidth, 0 };
        sendMessage(ans);
        LoRa.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
        break;
      case ES24_SF:
        SpreadingFactor = (uint8_t) value.value;
        ans = { MAGIC, ES24_SF, ANS, (uint8_t)(value.ttl >> 1), value.mask, SpreadingFactor, 0 };
        sendMessage(ans);
        LoRa.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
        break;
      case ES24_CR:
        CodeRate = (uint8_t) value.value;
        ans = { MAGIC, ES24_CR, ANS, (uint8_t)(value.ttl >> 1), value.mask, CodeRate, 0 };
        sendMessage(ans);
        LoRa.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
        break;
      case ES24_NETWORKID:
        ans = { MAGIC, ES24_NETWORKID, ANS, (uint8_t)(value.ttl >> 1), value.mask, value.value, 0 };
        sendMessage(ans);
        NetworkID = (uint16_t) value.value;
        break;
      case ES24_SERTIMEOUT:
        ans = { MAGIC, ES24_SERTIMEOUT, ANS, (uint8_t)(value.ttl >> 1), value.mask, value.value, 0 };
        sendMessage(ans);
        SerialTimeoutuS = value.value;
        break;
      case ES24_TXPOWER:
        ans = { MAGIC, ES24_TXPOWER, ANS, (uint8_t)(value.ttl >> 1), value.mask, value.value, 0 };
        sendMessage(ans);
        tx_power = (uint8_t) value.value;
        break;
      case ES24_RESET:
        ans = { MAGIC, ES24_RESET, ANS, (uint8_t)(value.ttl >> 1), value.mask, ES24_RESET, 0 };
        sendMessage(ans);
        delay(5);
        ESP.restart();
        break;
      default:
        break;
      }
    }
}

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
  parseBuff(Message, RXPacketL);
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
    parseBuff(Message, receivedBytes);
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
    //while (1);
    ESP.restart();
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
