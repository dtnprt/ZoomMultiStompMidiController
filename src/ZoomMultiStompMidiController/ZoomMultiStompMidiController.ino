
#if defined(USE_TINYUSB_HOST) || !defined(USE_TINYUSB)
#error "Please use the Menu to select Tools->USB Stack: Adafruit TinyUSB"
#endif

#include "pio_usb.h"
#include "EZ_USB_MIDI_HOST.h"
#include <Wire.h>
#include <SparkFun_Alphanumeric_Display.h> //Click here to get the library: http://librarymanager/All#SparkFun_Qwiic_Alphanumeric_Display by SparkFun
#include "HotButton.h"

#define HOST_PIN_DP   14   // Pin used as D+ for host, D- = D+ + 1

#define PIN_BTN_1 10
#define PIN_BTN_2 11
#define PIN_BTN_3 12
#define PIN_BTN_4 13

#define DISP_MAX_BRIGHTNESS 6
#define MIDI_CHANNEL  1 

#define HEARTBEAT_INTERVAL 10000

#define LONG_PRESS_TIME 500


byte SysEx_Edit_On[6] =           {0xf0,0x52,0x00,0x5f,0x50,0xf7};
byte SysEx_Edit_Off[6] =          {0xf0,0x52,0x00,0x5f,0x51,0xf7};
byte SysEx_Program_Reqeust[6] =   {0xf0,0x52,0x00,0x5f,0x33,0xf7};
byte SysEx_Patch_Reqeust[6] =     {0xf0,0x52,0x00,0x5f,0x29,0xf7};

// Display object
HT16K33 display;

// Buttons
HotButton BTN_1(PIN_BTN_1, true, LOW);
HotButton BTN_2(PIN_BTN_2, true, LOW);
HotButton BTN_3(PIN_BTN_3, true, LOW);
HotButton BTN_4(PIN_BTN_4, true, LOW);

// USB Host object
Adafruit_USBH_Host USBHost;

USING_NAMESPACE_MIDI
USING_NAMESPACE_EZ_USB_MIDI_HOST

RPPICOMIDI_EZ_USB_MIDI_HOST_INSTANCE(usbhMIDI, MidiHostSettingsDefault)

static uint8_t midi_dev_addr = 0;
static uint8_t midi_dev_cable = 0;
static bool core0_booting = true;
static bool core1_booting = true;

static bool stop_reading = false;

unsigned long timer_previous_millis = 0;



int8_t CURRENT_PROGRAM = 0;

byte CURRENT_MODEL = 0x5f;   //0x58 = MS-50G, 0x61 = MS-70CD, 0x5f = MS-60B
byte CURRENT_PATCH[105] =  {0xF0,0x52,0x00,0x5F,0x28,0x02,0x31,0x02,0x00,0x0C,0x0A,0x10,0x01,0x00,0x03,0x50,0x60,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x41,0x00,0x00,0x08,0x42,0x00,0x0C,0x00,0x34,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x31,0x00,0x00,0x04,0x01,0x30,0x58,0x00,0x04,0x20,0x40,0x00,0x11,0x41,0x00,0x06,0x00,0x00,0x00,0x00,0x71,0x01,0x00,0x00,0x0C,0x06,0x58,0x00,0x28,0x50,0x00,0x40,0x06,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x40,0x10,0x0F,0x4F,0x63,0x00,0x74,0x61,0x76,0x65,0x72,0x20,0x20,0x00,0x20,0x00,0xF7};
char CURRENT_NAME[11] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CURRENT_DISPLAY = 4;

uint8_t effectsEnabled[4] = {0, 0, 0, 0};

/* MIDI IN MESSAGE REPORTING */
static void onMidiError(int8_t errCode)
{
    Serial.printf("MIDI Errors: %s %s %s\r\n", (errCode & (1UL << ErrorParse)) ? "Parse":"",
        (errCode & (1UL << ErrorActiveSensingTimeout)) ? "Active Sensing Timeout" : "",
        (errCode & (1UL << WarningSplitSysEx)) ? "Split SysEx":"");
}



static void onControlChange(Channel channel, byte controller, byte value)
{
    //Serial.printf("C%u: CC#%u=%u\r\n", channel, controller, value);
}

static void onProgramChange(Channel channel, byte program)
{
    //Serial.printf("C%u: Prog=%u\r\n", channel, program);
    CURRENT_PROGRAM = program;
    PrintCurrentState();
}


static void onSysEx(byte* message, unsigned size)
{

    
     // DEBUG OUTPUT
    Serial.printf("(i) SysEx received (%d bytes):\r\n", size);
    //for(unsigned i = 0; i < size; i++) Serial.printf("%02x ", message[i]);
    //Serial.printf("\r\n");

    if(size > 5){
      
      Serial.printf("(i) Message ID: %02X\r\n", message[1]);

      if(message[1] == 0x7e){ // Identyfi Response (F0 7E 00 06 02 52 vv 00 00 00 32 2E 31 30 F7)
        CURRENT_MODEL = message[6];
        Serial.printf("(i) model: %02X\r\n", CURRENT_MODEL);
      }
      else if(message[1] == 0x52){ // Patch Response
        //Serial.printf("(i) received patch response\r\n");

        if(message[4] == 0x28){         // F0 52 00 5F 28 ... // Patch Data

          

          memcpy(CURRENT_PATCH, message, size);
          //for(unsigned i = 0; i < size; i++) Serial.printf("%02x ", CURRENT_PATCH[i]);
          Serial.printf("\r\n");
          ParseCurrentPatch();


          display.printf("%d%d%d%d", effectsEnabled[3], effectsEnabled[2], effectsEnabled[1], effectsEnabled[0]);

          //Serial.printf("Effects: %d%d%d%d\r\n", effectsEnabled[3], effectsEnabled[2], effectsEnabled[1], effectsEnabled[0]);

          PrintCurrentState();

          //sendProgramRequest();

      }
      else if(message[4] == 0x31){   // Effect State Change
        Serial.printf("(i) effect state changed...\r\n");
        //F0 52 00 vv 31 ee 00 oo 00 F7
        //vv = 0x58 = MS-50G, 0x61 = MS-70CD, 0x5f = MS-60B
        //ee = effect number (00-02, 03-0n doesn't work)
        //oo = 00: off/ 01: on

        sendPatchRequest(); // ignore state and ask for whole patch
        //sendProgramRequest();

      }
      else if(message[4] == 0x32){   // Effect State Change
          Serial.printf("(i) received heartbeat\r\n");
      }
    }
  
  }
  else
  {
    display.print("Foo");
  }

}


static void onSystemReset()
{
    Serial.printf("SysRst\r\n");
}


static void onMidiInWriteFail(uint8_t devAddr, uint8_t cable, bool fifoOverflow)
{
    if (fifoOverflow)
        Serial.printf("Dev %u cable %u: MIDI IN FIFO overflow\r\n", devAddr, cable);
    else
        Serial.printf("Dev %u cable %u: MIDI IN FIFO error\r\n", devAddr, cable);
}

static void registerMidiInCallbacks()
{
    auto intf = usbhMIDI.getInterfaceFromDeviceAndCable(midi_dev_addr, midi_dev_cable);
    if (intf == nullptr)
        return;
    //intf->setHandleNoteOff(onNoteOff);                      // 0x80
    //intf->setHandleNoteOn(onNoteOn);                        // 0x90
    //intf->setHandleAfterTouchPoly(onPolyphonicAftertouch);  // 0xA0
    intf->setHandleControlChange(onControlChange);          // 0xB0
    intf->setHandleProgramChange(onProgramChange);          // 0xC0
    //intf->setHandleAfterTouchChannel(onAftertouch);         // 0xD0
    //intf->setHandlePitchBend(onPitchBend);                  // 0xE0
    intf->setHandleSystemExclusive(onSysEx);                // 0xF0, 0xF7
    //intf->setHandleTimeCodeQuarterFrame(onSMPTEqf);         // 0xF1
    //intf->setHandleSongPosition(onSongPosition);            // 0xF2
    //intf->setHandleSongSelect(onSongSelect);                // 0xF3
    //intf->setHandleTuneRequest(onTuneRequest);              // 0xF6
    //intf->setHandleClock(onMidiClock);                      // 0xF8
    // 0xF9 as 10ms Tick is not MIDI 1.0 standard but implemented in the Arduino MIDI Library
    //intf->setHandleTick(onMidiTick);                        // 0xF9
    //intf->setHandleStart(onMidiStart);                      // 0xFA
    //intf->setHandleContinue(onMidiContinue);                // 0xFB
    //intf->setHandleStop(onMidiStop);                        // 0xFC
    //intf->setHandleActiveSensing(onActiveSense);            // 0xFE
    intf->setHandleSystemReset(onSystemReset);              // 0xFF
    intf->setHandleError(onMidiError);

    auto dev = usbhMIDI.getDevFromDevAddr(midi_dev_addr);
    if (dev == nullptr)
        return;
    dev->setOnMidiInWriteFail(onMidiInWriteFail);
}

/* CONNECTION MANAGEMENT */
static void onMIDIconnect(uint8_t devAddr, uint8_t nInCables, uint8_t nOutCables)
{
    Serial.printf("MIDI device at address %u has %u IN cables and %u OUT cables\r\n", devAddr, nInCables, nOutCables);
    midi_dev_addr = devAddr;
    registerMidiInCallbacks();
    Refresh();
}

static void onMIDIdisconnect(uint8_t devAddr)
{
    Serial.printf("MIDI device at address %u unplugged\r\n", devAddr);
    midi_dev_addr = 0;
}




static void sendNextProg()
{
  if(++CURRENT_PROGRAM >= 49) CURRENT_PROGRAM = 0;
  sendProgramChange(CURRENT_PROGRAM);
}

static void sendPreviousProg()
{
  if(--CURRENT_PROGRAM < 0) CURRENT_PROGRAM = 49;
  sendProgramChange(CURRENT_PROGRAM);
}

static bool sendProgramChange(uint8_t prog)
{
    auto intf = usbhMIDI.getInterfaceFromDeviceAndCable(midi_dev_addr, midi_dev_cable);
    if (intf == nullptr)
        return false; // not connected
    if(prog >= 49) prog = 49;
    intf->sendProgramChange(prog, MIDI_CHANNEL);
    return true;
}

static bool sendSysEx(unsigned int size, byte* data)
{
    auto intf = usbhMIDI.getInterfaceFromDeviceAndCable(midi_dev_addr, midi_dev_cable);
    if (intf == nullptr)
        return false; // not connected
    intf->sendSysEx(size, data, true);
    //onSysEx(data, size);
    return true;
}

static bool sendEditModeOn()
{
    auto intf = usbhMIDI.getInterfaceFromDeviceAndCable(midi_dev_addr, midi_dev_cable);
    if (intf == nullptr)
        return false; // not connected
    intf->sendSysEx(6, SysEx_Edit_On, true);
    return true;
}

static bool sendPatchRequest()
{   
    sendEditModeOn();
    auto intf = usbhMIDI.getInterfaceFromDeviceAndCable(midi_dev_addr, midi_dev_cable);
    if (intf == nullptr)
        return false; // not connected
    intf->sendSysEx(6, SysEx_Patch_Reqeust, true);
    return true;
}


static bool sendProgramRequest()
{
    sendEditModeOn();
    auto intf = usbhMIDI.getInterfaceFromDeviceAndCable(midi_dev_addr, midi_dev_cable);
    if (intf == nullptr)
        return false; // not connected
     //Request Current Program
      //f0 52 00 5f 33 f7 -> b0 00 00 (\n) b0 20 00 (\n) c0 pp 
      //	pp = patch
    intf->sendSysEx(6, SysEx_Program_Reqeust, true);
    return true;
}

/* APPLICATION STARTS HERE */

// core1's setup
void setup1() {
    while(!Serial);   // wait for native usb
    Serial.println("Core1 setup to run TinyUSB host with pio-usb\r\n");

    // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
    uint32_t cpu_hz = clock_get_hz(clk_sys);
    if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
        delay(2000);   // wait for native usb
        Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
        Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
        while(1) delay(1);
    }

    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = HOST_PIN_DP;
 
 #if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    /* Need to swap PIOs so PIO code from CYW43 PIO SPI driver will fit */
    pio_cfg.pio_rx_num = 0;
    pio_cfg.pio_tx_num = 1;
 #endif /* ARDUINO_RASPBERRY_PI_PICO_W */
 
    USBHost.configure_pio_usb(1, &pio_cfg);
    // run host stack on controller (rhport) 1
    // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
    // host bit-banging processing work done in core1 to free up core0 for other work
    usbhMIDI.begin(&USBHost, 1, onMIDIconnect, onMIDIdisconnect);
    core1_booting = false;
    while(core0_booting) ;
}

// core1's loop
void loop1()
{
    USBHost.task();
}

void setup()
{
  Serial.begin(115200);
  while(!Serial);   // wait for serial port

  Wire.begin(); //Join I2C bus

  pinMode(PIN_BTN_1, INPUT_PULLUP);
  pinMode(PIN_BTN_2, INPUT_PULLUP);
  pinMode(PIN_BTN_3, INPUT_PULLUP);
  pinMode(PIN_BTN_4, INPUT_PULLUP);

  Serial.println("Moin!");

  if (display.begin() == false)
  {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1);
  }
  Serial.println("Display acknowledged.");
  display.setBrightness(DISP_MAX_BRIGHTNESS);
  display.print("Moin");


  core0_booting = false;
  while(core1_booting);
}



void loop() {    
    
    // Handle any incoming data; triggers MIDI IN callbacks
    if(!stop_reading)
      usbhMIDI.readAll();

    HandleButtons();
    Heartbeat();
    
    // Tell the USB Host to send as much pending MIDI OUT data as possible
    usbhMIDI.writeFlushAll();

}

void Refresh()
{
    sendEditModeOn();
    sendPatchRequest();
    usbhMIDI.readAll();
    ParseCurrentPatch();
    sendProgramRequest();
    usbhMIDI.readAll();
}


void Heartbeat(){
  unsigned long current_millis = millis();
  if (!(current_millis - timer_previous_millis >= HEARTBEAT_INTERVAL))
      return;
  timer_previous_millis = current_millis;

 // sendProgramChange(32);
 // sendSysEx(6, SysEx_Edit_On);
  //sendSysEx(6, SysEx_Patch_Reqeust);
  //Refresh();

  
}

void PrintCurrentState(){
  ParseCurrentPatch();

uint8_t c1 = ((CURRENT_PATCH[85] >> 4) & 0x01);
uint8_t c0 = ((CURRENT_PATCH[88] >> 6) & 0x01);

  Serial.printf("(i) Current Patch (%d): %d, %d, %d, %d | %d (%02x), %d (%02x) = %d\r\n", (CURRENT_PROGRAM+1), (CURRENT_PATCH[6] & 1), (CURRENT_PATCH[26] & 1), (CURRENT_PATCH[47] & 1), (CURRENT_PATCH[67] & 1), c1, CURRENT_PATCH[85],  c0, CURRENT_PATCH[88],  CURRENT_DISPLAY);

}

void ParseCurrentPatch(){

    // Current Effect
  uint8_t c1 = ((CURRENT_PATCH[85] >> 4) & 0x01);
  uint8_t c0 = ((CURRENT_PATCH[88] >> 6) & 0x01);

  CURRENT_DISPLAY = (c0 << 1) + (c1 << 0);

  effectsEnabled[0] =  (CURRENT_PATCH[6] & 1);
  effectsEnabled[1] =  (CURRENT_PATCH[26] & 1);
  effectsEnabled[2] =  (CURRENT_PATCH[47] & 1);
  effectsEnabled[3] =  (CURRENT_PATCH[67] & 1);

  CURRENT_NAME[0] = CURRENT_PATCH[91];
  CURRENT_NAME[1] = CURRENT_PATCH[92];
  CURRENT_NAME[2] = CURRENT_PATCH[94];
  CURRENT_NAME[3] = CURRENT_PATCH[95];
  CURRENT_NAME[4] = CURRENT_PATCH[96];
  CURRENT_NAME[5] = CURRENT_PATCH[97];
  CURRENT_NAME[6] = CURRENT_PATCH[98];
  CURRENT_NAME[7] = CURRENT_PATCH[99];
  CURRENT_NAME[8] = CURRENT_PATCH[100];
  CURRENT_NAME[9] = CURRENT_PATCH[102];
  CURRENT_NAME[10] = '\0';

  //Serial.printf("(i) Patch Name: \"%s\" (", CURRENT_NAME);
  //for(unsigned i = 0; i < 10; i++) Serial.printf("%02x ", CURRENT_NAME[i]);
  //Serial.printf(")\r\n");
}

void ToggleEffect(uint8_t num){

    stop_reading = true;

    sendEditModeOn();
    sendPatchRequest();
    usbhMIDI.readAll();

    switch(num){
      case 0:
        CURRENT_PATCH[6] ^= 1;  // EFF 1 On/Off
        CURRENT_PATCH[85] |= (1 << 4);  // c1 = 1
        CURRENT_PATCH[88] |= (1 << 6);  // c0 = 1
        break;
      case 1:
        CURRENT_PATCH[26] ^= 1; // EFF 2 On/Off
        CURRENT_PATCH[85] |= (1 << 4);  // c1 = 1
        CURRENT_PATCH[88] &= ~(1 << 6); // c0 = 0
        break;
      case 2:
        CURRENT_PATCH[47] ^= 1; // EFF 3 On/Off
        CURRENT_PATCH[85] &= ~(1 << 4); // c1 = 0
        CURRENT_PATCH[88] |= (1 << 6);  // c0 = 1
        break;
      case 3:
        CURRENT_PATCH[67] ^= 1; // EFF 4 On/Off
        CURRENT_PATCH[85] &= ~(1 << 4); // c1 = 0
        CURRENT_PATCH[88] &= ~(1 << 6); // c0 = 0

        break;
      default:
        return;
    }
    Serial.printf("(i) Sending patch:\r\n");
    Serial.printf("(i) 06 26 47 67 85 88\r\n");
    Serial.printf("(i) -----------------\r\n");
    Serial.printf("(i) %02x %02x %02x %02x %02x %02x\r\n", CURRENT_PATCH[6], CURRENT_PATCH[26], CURRENT_PATCH[47], CURRENT_PATCH[67], CURRENT_PATCH[85], CURRENT_PATCH[88]);
     Serial.printf("(i) -----------------\r\n");

    sendSysEx(105, CURRENT_PATCH);
    usbhMIDI.writeFlushAll();

    stop_reading = false;
    Refresh();
    Serial.printf("\r\n");

    
}

void HandleButtons()
{

  BTN_1.update();
  BTN_2.update();
  BTN_3.update();
  BTN_4.update();

  if (BTN_1.event(SHORT)){
    //display.print("B1 S");
    ToggleEffect(0);
  }
  if (BTN_1.pressedFor(LONG_PRESS_TIME)){
    Refresh();
  }
  if (BTN_1.isDoubleClick()){
    display.print("B1 D");
    
  }

  if (BTN_2.event(SHORT)){
    //display.print("B2 S");
    ToggleEffect(1);
  }
  if (BTN_2.pressedFor(LONG_PRESS_TIME)){
    //display.print("B1 L");
    sendPreviousProg();
  }
  if (BTN_2.isDoubleClick()){
    display.print("B2 D");
  }

  if (BTN_3.event(SHORT)){
    //display.print("B3 S");
    ToggleEffect(2);
  }
  if (BTN_3.pressedFor(LONG_PRESS_TIME)){
    sendEditModeOn();
    sendPatchRequest();
  }
  if (BTN_3.isDoubleClick()){
    display.print("B3 D");
  }

  if (BTN_4.event(SHORT)){
    //display.print("B4 S");
    ToggleEffect(3);
  }
  if (BTN_4.pressedFor(LONG_PRESS_TIME)){
    //display.print("B4 L");
    sendNextProg();
  }
  if (BTN_4.isDoubleClick()){
    //display.print("B4 D");
  }
}
