/*
   CBUS Module Library - RasberryPi Pico SDK port
   Copyright (c) Kevin Kimber 2023

   Based on work by Duncan Greenwood
   Copyright (C) Duncan Greenwood 2017 (duncan_greenwood@hotmail.com)

   This work is licensed under the:
      Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
   To view a copy of this license, visit:
      http://creativecommons.org/licenses/by-nc-sa/4.0/
   or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.

   License summary:
    You are free to:
      Share, copy and redistribute the material in any medium or format
      Adapt, remix, transform, and build upon the material

    The licensor cannot revoke these freedoms as long as you follow the license terms.

    Attribution : You must give appropriate credit, provide a link to the license,
                  and indicate if changes were made. You may do so in any reasonable manner,
                  but not in any way that suggests the licensor endorses you or your use.

    NonCommercial : You may not use the material for commercial purposes. **(see note below)

    ShareAlike : If you remix, transform, or build upon the material, you must distribute
                 your contributions under the same license as the original.

    No additional restrictions : You may not apply legal terms or technological measures that
                                 legally restrict others from doing anything the license permits.

   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms

    This software is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

*/

// CBUS library header files

#include "CBUSACAN2040.h" // CAN controller and CBUS class
#include "CBUSSwitch.h"   // CBUS FLiM pushbutton switch
#include "CBUSLED.h"      // CBUS LEDs
#include "CBUSConfig.h"   // CBUS module configuration
#include "CBUSParams.h"   // CBUS parameters
#include "cbusdefs.h"     // CBUS constants
#include "CBUSUtil.h"     // Utility macros

#include <cstdio>
#include <pico/stdlib.h>
#include <pico/binary_info.h>

// constants
constexpr uint8_t VER_MAJ = 1;   ///< module code major version
constexpr char VER_MIN = 'a';    ///< module code minor version
constexpr uint8_t VER_BETA = 1;  ///< module code beta sub-version
constexpr uint8_t MODULEID = 99; ///< CBUS module type

// Map CBUS LED's switch to HW
constexpr uint8_t LED_GRN = 21; ///< CBUS Green SLiM LED pin
constexpr uint8_t LED_YLW = 20; ///< CBUS Yellow FLiM LED pin
constexpr uint8_t SWITCH0 = 17; ///< CBUS FLiM push button switch pin

// Map CAN2040 Tx and Rx pins
constexpr uint8_t CAN_RX = 11; ///< CAN2040 Rx pin
constexpr uint8_t CAN_TX = 12; ///< CAN2040 Tx pin

// Map Module IO Pins
constexpr uint8_t INST_BUZZ = 2;      ///< Block Instrument Warning Buzzer (currently not used)
constexpr uint8_t INST_BELL = 3;      ///< Block Instrument Bell (currently not used)
constexpr uint8_t LED_TRAIN_OT_R = 4; ///< Train on Track Remote indication
constexpr uint8_t LED_TRAIN_OT_L = 5; ///< Train on Track Local indication
constexpr uint8_t LED_NORMAL_R = 6;   ///< Line Normal Remote indication
constexpr uint8_t LED_NORMAL_L = 7;   ///< Line Normal Local indication
constexpr uint8_t LED_LINE_CLR_R = 8; ///< Line Clear Remote indication
constexpr uint8_t LED_LINE_CLR_L = 9; ///< Line Clear Local indication

constexpr uint8_t LINE_CLEAR = 14;     ///< Line Clear request switch from local box
constexpr uint8_t NORMAL = 15;         ///< Line Normal switch from local box
constexpr uint8_t TRAIN_ON_TRACK = 16; ///< Train on Track switch from local box
constexpr uint8_t BELL_PUSH = 18;      ///< Attention bell push (to remote box)

constexpr uint8_t WARN_LED = 22; ///< Line Clear request (commucator) locked warning
constexpr uint8_t OCCP_LED = 25; ///< Line 'Occupied LED

// CBUS objects
CBUSConfig module_config; ///< CBUS configuration object

// Construct CBUS Object and assign the module configuration
CBUSACAN2040 CBUS(module_config);

// Block Instrument objects
CBUSLED totremoteLED; ///< Train on Track - remote box indicator
CBUSLED totlocalLED;  ///< Train on Track - local box indicator
CBUSLED nrmremoteLED; ///< Normal - remote box indicator
CBUSLED nrmlocalLED;  ///< Normal - local box indicator
CBUSLED clrremoteLED; ///< Line Clear - remote box indicator
CBUSLED clrlocalLED;  ///< Line Clear - local box indicator

CBUSSwitch lineClearSW;    ///< Line Clear Switch
CBUSSwitch trainOnTrackSW; ///< Train on Track Switch
CBUSSwitch normalSW;       ///< Normal Switch
CBUSSwitch bellPush;       ///< Remote box attention plunger

CBUSLED warnLED; ///< Line Clear request blocked warning indicator
CBUSLED occpLED; ///< Block Occupied indicator

// module name, must be 7 characters, space padded.
module_name_t moduleName = {'B', 'L', 'O', 'C', 'K', ' ', ' '};

/// Event Constants
enum class InEventID
{
   // Incoming event ID's from remote box
   commutatorLock,  ///< Commutator is locked (prevent Line Clear)
   lineClear,       ///< Line Clear
   trainOnTrack,    ///< Train entered block
   blockCleared,    ///< Train left block
   attentionBell,   ///< Attention bell from remote box
   resetLineClear,  ///< Reset from Line Clear state to Normal (abnormal state transition)
   // Incoming event ID's to ACK (or NACK) local changes of state
   lineClearAck,    ///< ACK of our request for Line Clear
   trainOnTrackAck, ///< ACK of our request for Train on Track
   blockClearedAck, ///< ACK of our requst for Block Cleared (Normal)
   lineClearBlocked ///< NACK of our request for Line Clear
};

/// Outgoing events
enum class OutEventID
{
   lineClearBlocked, ///< Line Clear NACK
   lineClearAck,     ///< Line Clear ACK
   trainOnTrackAck,  ///< Train entered block ACK
   blockClearedAck,  ///< Train left block ACK
   attentionBell,    ///< Call attention to remote box
   resetLineClear,   ///< @todo 
   lineClear,        ///< Line Clear request
   trainOnTrack,     ///< Train on Track 
   blockCleared      ///< Train left block
};

constexpr uint8_t MAX_EVENT_ID = 10; ///< Maximum number of incoming event ID's

/// Block Instrument state machine states
enum class BlockState
{
   Normal,       ///< Block is Normal (unoccupied)
   LineClear,    ///< Line Clear authorised
   TrainOnTrack, ///< Train in block
   LCBlocked,    ///< Line Clear request blocked
};

// State machine state records
BlockState remoteBoxState{BlockState::Normal}; ///< Remote Box
BlockState localBoxState{BlockState::Normal};  ///< Local Box

/// Line Clear (commutator) release
bool lineClearReleased{true};

// forward function declarations
void eventhandler(uint8_t index, const CANFrame &msg);
void processModuleSwitchChange(void);

//
/// setup CBUS - runs once at power on from setup()
//

void setupCBUS()
{
   // Declare binary info for Picotool
   bi_decl(bi_program_description("CBUS Pico Block Instrument module"));

   // Notify pin setup for Picotool
   bi_decl(bi_1pin_with_name(LED_GRN, "CBUS Green LED"));
   bi_decl(bi_1pin_with_name(LED_YLW, "CBUS Yellow LED"));
   bi_decl(bi_1pin_with_name(SWITCH0, "CBUS FLiM Switch"));
   bi_decl(bi_1pin_with_name(CAN_TX, "CAN2040 Tx"));
   bi_decl(bi_1pin_with_name(CAN_RX, "CAN2040 Rx"));

   bi_decl(bi_1pin_with_name(WARN_LED, "Warning LED"));

   bi_decl(bi_1pin_with_name(INST_BUZZ, "Block Instrument Warning Buzzer"));
   bi_decl(bi_1pin_with_name(INST_BELL, "Block Instrument Attention Bell"));
   bi_decl(bi_1pin_with_name(LED_TRAIN_OT_R, "Train on Track Remote indication"));
   bi_decl(bi_1pin_with_name(LED_TRAIN_OT_L, "Train on Track Remote indication"));
   bi_decl(bi_1pin_with_name(LED_NORMAL_R, "Line Normal Remote indication"));
   bi_decl(bi_1pin_with_name(LED_NORMAL_L, "Line Normal Local indication"));
   bi_decl(bi_1pin_with_name(LED_LINE_CLR_R, "Line Clear Remote indication"));
   bi_decl(bi_1pin_with_name(LED_LINE_CLR_L, "Line Clear Local indication"));

   bi_decl(bi_1pin_with_name(BELL_PUSH, "Attention Bell push"));

   // set config layout parameters
   module_config.EE_NVS_START = 10;    // Offset start of Node Variables
   module_config.EE_NUM_NVS = 10;      // Number of Node Variables
   module_config.EE_EVENTS_START = 20; // Offset start of Events
   module_config.EE_MAX_EVENTS = 10;   // Maximum number of events
   module_config.EE_NUM_EVS = 1;       // Number of Event Variables per event (the InEventID)
   module_config.EE_BYTES_PER_EVENT = (module_config.EE_NUM_EVS + 4);

   // initialise and load configuration
   module_config.setEEPROMtype(EEPROM_TYPE::EEPROM_USES_FLASH);
   module_config.begin();

   // set module parameters
   CBUSParams params(module_config);
   params.setVersion(VER_MAJ, VER_MIN, VER_BETA);
   params.setModuleId(MODULEID);
   params.setFlags(PF_FLiM | PF_COMBI);

   // assign to CBUS
   CBUS.setParams(params.getParams());
   CBUS.setName(&moduleName);

   // Get the internal CBUS UI objects
   CBUSLED &ledGrn = CBUS.getCBUSGreenLED();
   CBUSLED &ledYlw = CBUS.getCBUSYellowLED();
   CBUSSwitch &sw = CBUS.getCBUSSwitch();

   // set CBUS LED pins
   ledGrn.setPin(LED_GRN);
   ledYlw.setPin(LED_YLW);

   // initialise CBUS switch
   sw.setPin(SWITCH0, false);
   sw.run();

   // module reset - if switch is depressed at startup and module is in SLiM mode
   if (sw.isPressed() && !module_config.getFLiM())
   {
      module_config.resetModule(ledGrn, ledYlw, sw);
   }

   // opportunity to set default NVs after module reset
   if (module_config.isResetFlagSet())
   {
      module_config.clearResetFlag();
   }

   // register our CBUS event handler, to receive event messages of learned events
   CBUS.setEventHandlerCB(eventhandler);

   // set CBUS LEDs to indicate mode
   CBUS.indicateFLiMMode(module_config.getFLiM());

   // configure and start CAN bus and CBUS message processing
   CBUS.setNumBuffers(25, 4);    // more buffers = more memory used, fewer = less
   CBUS.setPins(CAN_TX, CAN_RX); // select pins for CAN tx and rx

   if (!CBUS.begin())
   {
      // Init OK
   }
}

//
/// setup - runs once at power on
//

void setup()
{
   // Setup CBUS Library
   setupCBUS();

   // Setup IO - LED Outputs
   totremoteLED.setPin(LED_TRAIN_OT_R);
   totlocalLED.setPin(LED_TRAIN_OT_L);
   nrmremoteLED.setPin(LED_NORMAL_R);
   nrmlocalLED.setPin(LED_NORMAL_L);
   clrremoteLED.setPin(LED_LINE_CLR_R);
   clrlocalLED.setPin(LED_LINE_CLR_L);

   // Switch Inputs - active LOW with internal Pull-Up
   lineClearSW.setPin(LINE_CLEAR, false);
   trainOnTrackSW.setPin(TRAIN_ON_TRACK, false);
   normalSW.setPin(NORMAL, false);
   bellPush.setPin(BELL_PUSH, false);

   // Indicator LED's
   warnLED.setPin(WARN_LED);
   occpLED.setPin(OCCP_LED);

   // Set default LED states - block NORMAL
   totremoteLED.off();
   totlocalLED.off();
   nrmremoteLED.on();
   nrmlocalLED.on();
   clrremoteLED.off();
   clrlocalLED.off();
   warnLED.off();
   occpLED.off();

   // Map configuration settings to NV's
   uint8_t NVs[module_config.EE_NUM_NVS]{};

   // Block write the NV's from the configuration settings from the SD card
   module_config.writeBytesEEPROM(module_config.EE_NVS_START, NVs, module_config.EE_NUM_NVS);
}

//
/// loop - runs forever
//

void loop()
{
   //
   /// do CBUS message, switch and LED processing
   //

   CBUS.process();

   //
   /// give the switch and LED code some time to run
   //

   warnLED.run();
   occpLED.run();

   totremoteLED.run();
   totlocalLED.run();
   nrmremoteLED.run();
   nrmlocalLED.run();
   clrremoteLED.run();
   clrlocalLED.run();

   lineClearSW.run();
   trainOnTrackSW.run();
   normalSW.run();
   bellPush.run();

   //
   /// Check if switch changed and do any processing for this change.
   //

   processModuleSwitchChange();
}

//
/// Process switch inputs - transmit ACON / ACOF events based on switch states
//
void processModuleSwitchChange()
{
   // Generate request events based on local state machine
   if (BlockState::Normal == localBoxState)
   {
      // From Normal we can only request Line Clear
      if (lineClearSW.stateChanged())
      {
         if (lineClearSW.isPressed())
         {
            // Send line clear request
            CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::blockCleared), false);
            CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::lineClear), true);
         }
      }
   }
   else if (BlockState::LineClear == localBoxState)
   {
      // From Line Clear we can only request Train on Track
      if (trainOnTrackSW.stateChanged())
      {
         if (trainOnTrackSW.isPressed())
         {
            CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::lineClear), false);
            CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::trainOnTrack), true);
         }
      }
   }
   else if (BlockState::TrainOnTrack == localBoxState)
   {
      // From Train on Track we can only request Block Cleared
      if (normalSW.stateChanged())
      {
         if (normalSW.isPressed())
         {
            CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::trainOnTrack), false);
            CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::blockCleared), true);
         }
      }
   }
   else if (BlockState::LCBlocked == localBoxState)
   {
      // hmm think @todo
   }

   // Transmit bell events based on bell push switch state
   if (bellPush.stateChanged())
   {
      CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::attentionBell), bellPush.isPressed());
   }
}

///
/// @brief Process the Remote State machine requests
/// 
/// @param eventID event ID of the incoming event being processed
///
void processRemoteStateMachine(InEventID eventID)
{
   if (BlockState::Normal == remoteBoxState) // Block is NORMAL
   {
      // From Normal, we can only switch to Line Clear
      if (InEventID::lineClear == eventID)
      {
         // Check for commutator released
         if (lineClearReleased)
         {
            // OK to set line clear
            remoteBoxState = BlockState::LineClear;

            // Notify remote box
            CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::blockClearedAck), false); // Normal OFF
            CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::lineClearAck), true);     // Line Clear ON
         }
         else
         {
            // Setting Line Clear is blocked
            remoteBoxState = BlockState::LCBlocked;

            // Notify remote box
            CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::lineClearBlocked), true); // Line Clear Blocked
         }
      }
   }
   else if (BlockState::LineClear == remoteBoxState) // Block is Line Clear
   {
      // From Line Clear, we can only switch to Train on Track
      if (InEventID::trainOnTrack == eventID)
      {
         // Set Train on Track
         remoteBoxState = BlockState::TrainOnTrack;

         // Notify remote box
         CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::lineClearAck), false);   // Line Clear OFF
         CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::trainOnTrackAck), true); // Train on Track ON
      }
   }
   else if (BlockState::TrainOnTrack == remoteBoxState) // Block is Train on Track
   {
      // From Train on Track, we can only switch to Normal
      if (InEventID::blockCleared == eventID)
      {
         // Set Normal
         remoteBoxState = BlockState::Normal;

         // Notify remote box
         CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::trainOnTrackAck), false); // Train in Track OFF
         CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::blockClearedAck), true);  // Normal ON
      }
   }
   else if (BlockState::LCBlocked == remoteBoxState) // Block is Normal with (blocked) request for Line Clear
   {
      // From Line Clear blocked, check for release of lock
      if (lineClearReleased)
      {
         // OK to set line clear
         remoteBoxState = BlockState::LineClear;

         // Notify remote box
         CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::lineClearBlocked), false); // Line Clear Blocked OFF
         CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::blockClearedAck), false); // Normal OFF
         CBUS.sendMyEvent(static_cast<uint8_t>(OutEventID::lineClearAck), true);  // Line Clear ON
      }
   }
}

//
/// user-defined event processing function
/// called from the CBUS library when a learned event is received
/// it receives the event table index and the CAN frame
//

void eventhandler(uint8_t index, const CANFrame &msg)
{
   // Get OpCode of event
   uint8_t opCode = msg.data[0];

   // Check for Long or Short Accessory events
   if ((opCode == OPC_ACON) || (opCode == OPC_ACOF) || (opCode == OPC_ASON) || (opCode == OPC_ASOF))
   {
      // read the value of the (single) event variable (EV) associated with this learned event, this is the eventID
      uint8_t ID = module_config.getEventEVval(index, 1);

      // Validate before processing
      if (ID >= MAX_EVENT_ID)
      {
         return;
      }

      // Convert to type safe enum
      InEventID eventID = static_cast<InEventID>(ID);

      switch (eventID)
      {
      case InEventID::commutatorLock:
         
         // Lock or release Line Clear commutator
         lineClearReleased = opCode == OPC_ACON ? false : true;

         // Process state machine on both ACON and ACOF
         processRemoteStateMachine(eventID);
         break;
      case InEventID::lineClear:
      case InEventID::trainOnTrack:
      case InEventID::blockCleared:
         // Only process state machine notification on rising edge (event going ON)
         if (opCode == OPC_ACON)
         {
            processRemoteStateMachine(eventID);
         }
         break;
      case InEventID::resetLineClear:
         /// @todo reset to Line Clear
         break;
      case InEventID::attentionBell:
         /// @todo Set Bell output
         break;
      case InEventID::lineClearAck:
         // ACK of our Line Clear request
         localBoxState = BlockState::LineClear;
         break;
      case InEventID::trainOnTrackAck:
         // ACK of our Train on Track request
         localBoxState = BlockState::TrainOnTrack;
         break;
      case InEventID::blockClearedAck:
         // ACK of our Line Normal request
         localBoxState = BlockState::Normal;
         break;
      case InEventID::lineClearBlocked:
         // NACK of our Line Clear request
         localBoxState = BlockState::LCBlocked;
         break;
      }
   }

   // Update block status LEDs from the remote box
   switch (remoteBoxState)
   {
   case BlockState::Normal: // Normal (unoccupied) state
      totremoteLED.off();
      nrmremoteLED.on();
      clrremoteLED.off();
      warnLED.off();
      occpLED.off();
      break;
   case BlockState::LineClear: // Line Clear authorised
      totremoteLED.off();
      nrmremoteLED.off();
      clrremoteLED.on();
      warnLED.off();
      occpLED.on();
      break;
   case BlockState::TrainOnTrack: // Train is on Track
      totremoteLED.on();
      nrmremoteLED.off();
      clrremoteLED.off();
      warnLED.off();
      occpLED.on();
      break;
   case BlockState::LCBlocked: // Line Clear authorisation blocked (commutator locked)
      totremoteLED.off();
      nrmremoteLED.on();
      clrremoteLED.blink();
      warnLED.on();
      occpLED.off();
      break;
   };

   // Update block status LEDs for the local box
   switch (localBoxState)
   {
   case BlockState::Normal: // Normal (unoccupied) state
      totlocalLED.off();
      nrmlocalLED.on();
      clrlocalLED.off();
      break;
   case BlockState::LineClear: // Line Clear authorised
      totlocalLED.off();
      nrmlocalLED.off();
      clrlocalLED.on();
      break;
   case BlockState::TrainOnTrack: // Train is on Track
      totlocalLED.on();
      nrmlocalLED.off();
      clrlocalLED.off();
      break;
   case BlockState::LCBlocked: // Line Clear authorisation blocked (commutator locked)
      totlocalLED.off();
      nrmlocalLED.on();
      clrlocalLED.blink();
      break;
   };
}

// MODULE MAIN ENTRY

extern "C" int main(int, char **)
{
   // Init stdio lib (only really required if UART logging etc.)
   stdio_init_all();

#if LIB_PICO_STDIO_SEMIHOSTING
   // Setup CRLF options
   stdio_set_translate_crlf(&stdio_semihosting, false);

   printf("CANLocking : Initializing\n");
#endif

   // Initialize
   setup();

   // Run periodic processing - forever
   while (1)
   {
      loop();
   }
}
