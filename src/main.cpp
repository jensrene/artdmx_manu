#include <Arduino.h>
//#include <ArtnetEtherENC.h>
//#include <Artnet.h>
#include <EthernetENC.h>
//ArtnetReceiver artnet;

const uint16_t universe15bit = 0;
const bool debugOut = true;
uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};


#define bytes_to_short(h,l) ( ((h << 8) & 0xff00) | (l & 0x00FF) );

// Some ArtPoll specs of this module.

#define MAX_NUM_PORTS      4
#define SHORT_NAME_LENGTH    18
#define LONG_NAME_LENGTH    64
#define PORT_NAME_LENGTH    32

#define PROTOCOL_VERSION    14    // DMX-Hub protocol version.
#define FIRMWARE_VERSION    0x035A  // DMX-Hub firmware version.
#define OEM_ID          0x1012  // OEM Code
#define STYLE_NODE        0     // Responder is a Node (DMX <-> Ethernet Device)

#define ARTNET_SUBNET 0 // Sub-Net of artnet
#define ARTNET_UNIVERSE 0 // ArtNet-Universe 0 is minimum
#define ARTNET_DMX_ADDRESS 1 // DMX Address starting with 1

#define PORT_TYPE_DMX_OUTPUT  0x80
#define PORT_TYPE_DMX_INPUT   0x40

#define MAX_CHANNELS      512
#define IBG             10    // interbyte gap [us]

#define REFRESH_INTERVAL    4   // [s]


#define BUFFERSIZE 529
#define STATIC 1  // to use a static IP

#if STATIC
// ethernet interface ip address. please change to your own
//for direct link from PC ethernet to ENC board, make ip 192.168.2.2
//on PC, in ipv4 settings, make static IP 192.168.2.1, subnet mask 255.255.255.0 and gwip left empty
// static byte myip[] = { 192, 168, 1, 177 };
// gateway ip address not needed
//static byte gwip[] = { };
#endif

// LED PINS: 
//int red_light_pin = 3;
//int blue_light_pin = 9;
//int green_light_pin = 10;


int red_light_pin = 9;
int green_light_pin = 3;
int blue_light_pin = 10;


// for internal rainbow
unsigned short redstate = 0;
unsigned short greenstate = 0;
unsigned short bluestate = 0;
bool doRed =  false;
bool doGreen= true;  // start with green
bool doBlue = false;
unsigned short manDelay = 0;


//ethernet mac address - //genetated from http://www.miniwebtool.com/mac-address-generator/ using Microchip OUI
//verified at http://sqa.fyicenter.com/Online_Test_Tools/MAC_Address_Format_Validator.php
// static byte mymac[] = {0x01, 0x00, 0x00, 0x45, 0x08, 0x11};
static byte mymac[] = { 0x74, 0x69, 0x69, 0x2D, 0x30, 0x33 };

//CO? byte Ethernet::buffer[BUFFERSIZE]; // tcp/ip send and receive buffer should be 512 dmx channels + 17 bytes for header

const int number_of_channels = MAX_CHANNELS; // 512 for one full DMX universe (THIS TAKES UP LOTS OF SRAM)
const int channel_position = 1; // 1 if you want to read from channel 1
//byte buffer_dmx[number_of_channels+channel_position]; //buffer to store filetered DMX data    ***bypassed this bit to save SRAM***
const int art_net_header_size = 17;
const int max_packet_size = 530;  //should be 512 + 17 bytes
//char ArtNetHead[8] = "Art-Net";  //first byte of an ARTDMX packet contains "Art-Net"
const PROGMEM char ArtNetHeader[9] = "Art-Net\0";  //first byte of an ARTDMX packet contains "Art-Net" with tailing #0
const PROGMEM char artnet_shortName[SHORT_NAME_LENGTH] = "MafunDi LEDPWM\0";
const PROGMEM char artnet_longName[LONG_NAME_LENGTH] = "MafunDi's Art-Net to RGB LED PWM Adaptor\0";

boolean is_opcode_is_dmx = 0;
boolean is_opcode_is_artpoll = 0;
boolean is_poll_is_broadcast = 0;
byte    answerip[4];

bool webOK = false;
bool sendPaket = false;

struct artnet_packet_addr {
  unsigned char  ip[4];
  unsigned short port;
};


struct artnet_poll {
  unsigned char  id[8];
  unsigned short opcode;
  unsigned char  versionH;
  unsigned char  version;
  unsigned char  talkToMe;
  unsigned char  pad;
};

struct artnet_pollreply {
  unsigned char  id[8];
  unsigned short opcode;
  struct artnet_packet_addr addr;
  unsigned char  versionInfoH;
  unsigned char  versionInfo;
  unsigned char  subSwitchH;
  unsigned char  subSwitch;
  unsigned short oem;
  unsigned char  ubeaVersion;
  unsigned char  status;
  unsigned short estaMan;
  char           shortName[SHORT_NAME_LENGTH];
  char           longName[LONG_NAME_LENGTH];
  char           nodeReport[LONG_NAME_LENGTH];
  unsigned char  numPortsH;
  unsigned char  numPorts;
  unsigned char  portTypes[MAX_NUM_PORTS];
  unsigned char  goodInput[MAX_NUM_PORTS];
  unsigned char  goodOutput[MAX_NUM_PORTS];
  unsigned char  swin[MAX_NUM_PORTS];
  unsigned char  swout[MAX_NUM_PORTS];
  unsigned char  swVideo;
  unsigned char  swMacro;
  unsigned char  swRemote;
  unsigned char  spare1;
  unsigned char  spare2;
  unsigned char  spare3;
  unsigned char  style;
  unsigned char  mac[6];
  unsigned char  filler[32];
};

struct artnet_dmx {
  unsigned char  id[8];
  unsigned short opcode;
  unsigned char  versionH;
  unsigned char  version;
  unsigned char  sequence;
  unsigned char  physical;
  unsigned short universe;
  unsigned char  lengthHi;
  unsigned char  length;  
  unsigned char  dataStart;  
};


// callback that deals with received packets
static void artnetPacket(uint16_t port, uint8_t ip[4], uint16_t src_port, const char *data, uint16_t len) {
  //D/Serial.println( F("[artnetPacket] Packet recieved"));       //print to serial for testing
  //D/Serial.println( F("------------------------------"));
  
  //Make sure the packet is an ArtDMX packet
  int check = checkARTDMX(data, len);
  if (check) {
    // It is so process the data to the LEDS
    if (is_opcode_is_dmx) sendDMX(data);
    if (is_opcode_is_artpoll) {
      memcpy(answerip,ip,4);  
      prepArtPollReply();        
    }
  }
}

/*
  Do some checks to see if it's an ArtNet packet. First 17 bytes are the ArtNet header, the rest is DMX values
  Bytes 1 to 8 of an Art-Net Packet contain "Art-Net" so check for "Art-Net" in the first 8 bytes
  Bytes 8 and 9 contain the OpCode - a 16 bit number that tells if its ArtPoll or ArtDMX
  Don't worry about the rest of the bytes until byte 18 on (DMX channel levels)
*/
int checkARTDMX(const char* messagein, int messagelength) {   //messagein prob dont need to use a pointer here since we aren't writing to it
  //D/Serial.println( "checkARTDMX reached");       //print to serial for testing
  char ArtNetHead[9];
  memcpy_P(ArtNetHead,ArtNetHeader,9);

 /* DEBUG//Serial.println(" Debug out: ");
  Serial.print(messagein[8], HEX);
  Serial.print(messagein[9], HEX);
  Serial.print(" Size: ");
  Serial.print(messagelength, DEC);
  Serial.println();
*/

  if (messagelength > 9 && messagelength <= max_packet_size) {
  //  Serial.print("checking Art-Net packet: ");
    //read header
    boolean match_artnet = 1;
    is_opcode_is_artpoll = 0;
    is_opcode_is_dmx = 0;
    for (int i = 0; i < 7; i++)
    {
      if (messagein[i] != ArtNetHead[i])  //tests first byte for  "Art-Net"
      {
        match_artnet = 0; break;    //if not an artnet packet we stop reading
      }
    }

    if (match_artnet == 1) {      
      short Opcode = bytes_to_short(messagein[9], messagein[8]); // convert ArtNet Opcode to nice short variable.
      if (messagelength > art_net_header_size && Opcode == 0x5000) {
        // Normal ArtDMX message
        is_opcode_is_dmx = 1;
        //D/Serial.println(F("ArtDMX recieved"));
      }
      else if (Opcode == 0x2000) {
        is_opcode_is_artpoll = 1;
        //D/Serial.println(sizeof(artnet_poll));
        struct artnet_poll *poll;
        poll = (struct artnet_poll *)  messagein;
        /* DEBUG// for (byte i=0;i<8;i++){
        /   Serial.write(poll->id[i]);
          }
        Serial.println();  
        Serial.print(F("Poll Client Version: "));
        Serial.print(poll->versionH, DEC);
        Serial.print(".");
        Serial.print(poll->version, DEC);
        Serial.println();*/
        if ((poll->talkToMe & 1) == 1) {
          //D/Serial.println(F("Talking back directly to IP"));
          is_poll_is_broadcast = false;
        }
        else { 
          //D/Serial.println(F("Broadcasting, no direct IP communication."));
          is_poll_is_broadcast = true;
        }


      }
      else {
        /*DEBUG//Serial.print("Unsupported Art-Net opcode: 0x");
        Serial.println(Opcode, HEX);*/
      }
    }
    return 1;
  }
  return 0;
}


/*
   function to send artnet poll reply.
*/
void prepArtPollReply()
{
 //D/ Serial.println("ArtPollReply...");
  word i;
  //cleaning buffer
  for (i = 0; i < BUFFERSIZE; i++)
  { //clear eth_buffer to 0
    Ethernet::buffer[i] = 0;
  }

  struct artnet_pollreply *msg;

  msg = (struct artnet_pollreply *) & Ethernet::buffer[0];

  strcpy_P((char*)msg->id,  ArtNetHeader );
  msg->opcode = 0x2100; //ArtPollReply
  msg->addr.ip[0] = ether.myip[0];
  msg->addr.ip[1] = ether.myip[1];
  msg->addr.ip[2] = ether.myip[2];
  msg->addr.ip[3] = ether.myip[3];

  msg->addr.port = 6454;
  msg->versionInfoH = (FIRMWARE_VERSION >> 8) & 0xFF;
  msg->versionInfo = FIRMWARE_VERSION & 0xFF;

  msg->subSwitchH = 0;
  msg->subSwitch = ARTNET_SUBNET & 15;


  msg->oem = OEM_ID;
  msg->ubeaVersion = 0;
  msg->status = 0;
  msg->estaMan = 'M' * 256 + 'D';  // MafunDi
  strcpy_P(msg-> shortName, artnet_shortName);
  strcpy_P(msg-> longName, artnet_longName);
  strcpy_P((char*)msg-> nodeReport, PSTR( "MAF_LEDPWM is ready\0" )); //  text status message

  msg->numPortsH = 0;
  msg->numPorts = 1;

  msg->portTypes[0] = PORT_TYPE_DMX_OUTPUT;

  msg-> goodInput[0] = (1 << 3);





  msg->goodOutput[0] = (1 << 1);
  /*if (artnet_dmxTransmitting == TRUE) {
    msg - > goodOutput[0] |= (1 < < 7);
    }*/ // show if we are transmitting

  byte artnet_inputUniverse1 = ARTNET_UNIVERSE; // default universe start is 0
  byte artnet_outputUniverse1 = ARTNET_UNIVERSE;  // default universe start is 0

  msg->swin[0] = (ARTNET_SUBNET & 15) * 16 | (artnet_inputUniverse1 & 15);
  msg->swout[0] = (ARTNET_SUBNET & 15) * 16 | (artnet_outputUniverse1 & 15);

  msg->style = STYLE_NODE;

  msg->mac[0] = mymac[0];
  msg->mac[1] = mymac[1];
  msg->mac[2] = mymac[2];
  msg->mac[3] = mymac[3];
  msg->mac[4] = mymac[4];
  msg->mac[5] = mymac[5];

  /*
    Serial.print("BUFFER:");
    for (i = 0; i < BUFFERSIZE; i++) {
      Serial.print(" 0x");
      Serial.print(Ethernet::buffer[i], HEX);
    }
    Serial.println(); */ // debug out of paket

  sendPaket = true;
 
  
  //ether.sendUdp(Ethernet::buffer, sizeof(artnet_pollreply), 1234, sendip, 6454);

}

/*
  function to send the dmx data out using DmxSimple library function
  Reads data directly from the packetBuffer and sends straight out
*/
void sendDMX(const char* packetBuffer)
{
  //D/Serial.println(F("sendDMX reached"));       //print to serial for testing
  struct artnet_dmx *dmx_header;
  dmx_header = (struct artnet_dmx *)  packetBuffer;
  short dmx_length = bytes_to_short(dmx_header->length, dmx_header->lengthHi);
  //D/Serial.print(F("DMX packet recieved. Number of Channels transmitted: "));
  //D/Serial.println(dmx_length,DEC);
  if (dmx_header->universe == ARTNET_UNIVERSE) {  // is our universe
    byte red=packetBuffer[ARTNET_DMX_ADDRESS+17];
    byte green=packetBuffer[ARTNET_DMX_ADDRESS+1+17];
    byte blue=packetBuffer[ARTNET_DMX_ADDRESS+2+17];
    
    analogWrite(green_light_pin, green);
    analogWrite(red_light_pin, red);
    analogWrite(blue_light_pin,blue);
  }
/* //DEBUG
  Serial.print("RGB Write: R:");
  Serial.print(red,DEC);
  Serial.print(" - G:");
  Serial.print(green,DEC);
  Serial.print(" - B:");
  Serial.print(blue,DEC);
  Serial.println(" . ");
*/

 /* for (int i = channel_position - 1; i < number_of_channels; i++) //channel position
  {

    //implement LED set here
   

    //buffer_dmx[i]= byte(packetBuffer[i+17]);    //bypassed the dmx buffer altogether to save memory
    //OLD //DmxSimple.write(i,buffer_dmx[i]);
    //OLD //    DmxSimple.write(i, packetBuffer[i + 17]);
    
  }*/
}


void setup() {
  // put your setup code here, to run once:

  if (debugOut) {
    Serial.begin(115200); 
    Serial.println( F("Hello World") );
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
  }


  // start the Ethernet connection:
  
  Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
    }
  }
  
  // print your local IP address:

  

  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  

}

void loop() {
  
}

// put function definitions here:
