//*************************************************************************
//*
//*                 Ocean Modules Offshore V2
//*
//*                 utilities
//*
//*                 Ocean Modules 2010
//*
//*
//*  Development environment: IAR Embedded Workbench IDE 5.1.0.417.7663
//*									IAR Assembler for ARM      5.50.0.51878
//*									IAR C/C++ Compiler for ARM 5.50.0.51878
//*									IAR Elf Linker for ARM     5.50.0.51878
//*									IAR Build Utility          5.1.0.417.7663
//*
//*  Filename:      $Workfile: utilities.c $
//*  Revision:      $Revision: 37 $
//*
//*  Prepared by:   R&D Christer Johansson, 
//*
//*  Description:   Common utility functions
//*
//*************************************************************************


//*************************************************************************
//*
//*              REVISION HISTORY
//*
//*  Date:       Name:      Description:
//*
//*  2010-08-02  CJN        Started/Created
//*
//*************************************************************************
/*
 * $History: utilities.c $
 * 
 * *****************  Version 37  *****************
 * User: Mats Ahlstrand Date: 17-08-14   Time: 16:06
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This is the R054 version which will be shipped to the SEATERRA L3000
 * ROV. Now the Altimeter cruising with an ON-OFF control algorithm is
 * working. The SW is dependent on the R029 version of the ROV Analyser
 * application. The parameters are set in the Altimeter Data tab page in
 * ROV Analyser. The altimeter cruising algorithm int the ROV is only
 * enabled when R029 version of ROV Analyser is running on the system
 * computer
 * 
 * *****************  Version 36  *****************
 * User: Mats Ahlstrand Date: 17-07-07   Time: 14:23
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This is the versionof the altimeter steering algorithm extended with
 * hysteresis and ramping of the thrust
 * 
 * *****************  Version 29  *****************
 * User: Mats Ahlstrand Date: 15-11-24   Time: 15:11
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This is P054eta version . This handles altimeter with 1 byte interrupt
 * length for the altimeter serial buffer.... In this version camera 3 is
 * moved to option board 5 also
 * 
 * *****************  Version 28  *****************
 * User: Mats Ahlstrand Date: 15-11-12   Time: 10:31
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This version is based on L3000 P047 Mögster with Altimeter cruising
 * support as in R048 for Baltic Offshore. 
 * 
 * *****************  Version 14  *****************
 * User: Mats Ahlstrand Date: 15-02-07   Time: 20:56
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Updated with new voltage steering table in order to reduce current use
 * in Mogsters L3000. This is done in RovSpecifics.c
 * 
 * *****************  Version 10  *****************
 * User: Christer Johansson Date: 14-08-25   Time: 15:31
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Debugprint allowed changed from 
 * #ifdef DEBUG_TXT_SHOW 
 * --> 
 * #if defined(DEBUG_TXT_SHOW) || defined(CJN_MAIN_TESTS)
 * for better support when running CJN_MAIN_TESTS
 * 
 * *****************  Version 9  *****************
 * User: Christer Johansson Date: 14-04-03   Time: 12:47
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Removed support for NDE_VEHICLE since the ROV does no longer exist.
 * 
 * *****************  Version 8  *****************
 * User: Christer Johansson Date: 14-03-25   Time: 14:28
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for IMU data over UDP, applicationMsg
 * 
 * *****************  Version 7  *****************
 * User: Christer Johansson Date: 14-03-06   Time: 10:30
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for EXTOUT port mapping registers in PowerBoard.
 * 
 * *****************  Version 6  *****************
 * User: Christer Johansson Date: 14-03-04   Time: 14:11
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for LED, Gripper and 24V feed 4-7 from PowerBoard
 * 
 * *****************  Version 5  *****************
 * User: Christer Johansson Date: 13-12-17   Time: 10:00
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * ROYAL NAVY no longer special    - it is std M500
 * JD CONTRACTOR no longer special - it is std M500
 * 
 * *****************  Version 4  *****************
 * User: Christer Johansson Date: 13-11-18   Time: 16:16
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Asart using BCDToUnsignedChar() again
 * 
 * *****************  Version 3  *****************
 * User: Christer Johansson Date: 13-10-03   Time: 11:25
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Stop program execution if asked for it in call to
 * CheckIfThisBoardSupportsThisFwFunction()
 * 
 * *****************  Version 2  *****************
 * User: Christer Johansson Date: 13-09-17   Time: 10:49
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Changed text in GetRovType()
 * 
 * *****************  Version 1  *****************
 * User: Christer Johansson Date: 13-09-06   Time: 14:00
 * Created in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * 
 * *****************  Version 20  *****************
 * User: Christer Johansson Date: 13-08-23   Time: 13:04
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added control of if text to overlay is allowed or not.
 * 
 * *****************  Version 19  *****************
 * User: Christer Johansson Date: 13-04-23   Time: 13:04
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for STD_FENIX
 * 
 * *****************  Version 18  *****************
 * User: Christer Johansson Date: 13-04-22   Time: 16:27
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for MAX_TEXT_LENGHT to see that text is not too long.
 * 
 * *****************  Version 17  *****************
 * User: Christer Johansson Date: 13-04-22   Time: 15:44
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added for support for JD_CONTRACTOR_FENIX
 * 
 * *****************  Version 16  *****************
 * User: Christer Johansson Date: 13-03-22   Time: 12:46
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for ROYAL_NAVY_FENIX
 * 
 * *****************  Version 15  *****************
 * User: Christer Johansson Date: 13-03-07   Time: 8:33
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Replace STD_FENIX to CHINA_UNIV_FENIX
 * 
 * *****************  Version 14  *****************
 * User: Christer Johansson Date: 13-03-01   Time: 13:14
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for TESTJOYSTICKCOMMAND
 * 
 * *****************  Version 13  *****************
 * User: Christer Johansson Date: 13-02-26   Time: 9:00
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for showing "LEA-TEST"
 * 
 * *****************  Version 12  *****************
 * User: Christer Johansson Date: 13-02-20   Time: 14:21
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Removed unused debug info.
 * 
 * *****************  Version 11  *****************
 * User: Christer Johansson Date: 13-02-12   Time: 11:55
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for CHINA_UNIV_FENIX
 * 
 * *****************  Version 10  *****************
 * User: Christer Johansson Date: 13-02-04   Time: 15:16
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Changed FENIX to M500
 * Changed SESAM to L3000
 * Added support for NDE
 * 
 * *****************  Version 9  *****************
 * User: Christer Johansson Date: 13-01-25   Time: 13:43
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for FENIX.
 * Removed OLD_BACKPLANE defines since we do it in another way.
 * 
 * *****************  Version 8  *****************
 * User: Christer Johansson Date: 12-11-16   Time: 12:03
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Changed PrintRovTypeOnDebugPort() to use GetRovType() for debug
 * printout.
 * 
 * *****************  Version 7  *****************
 * User: Christer Johansson Date: 12-11-13   Time: 11:08
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for SeaTerra.
 * 
 * *****************  Version 6  *****************
 * User: Christer Johansson Date: 12-10-05   Time: 13:56
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for FPGA IO Register Map according to doc I019-301,
 * version A from 2012-02-03.
 * Added support for new backplane with EXTOUT 0-3 / 4-7 swopped.
 * 
 * *****************  Version 5  *****************
 * User: Christer Johansson Date: 12-08-21   Time: 17:18
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for showing System sw rev in ROV Analyser new tab.
 * 
 * *****************  Version 4  *****************
 * User: Christer Johansson Date: 12-08-20   Time: 9:55
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Correcter NMEA checksum calculation so it does no longer include the $
 * character at start of string.
 * 
 * *****************  Version 3  *****************
 * User: Christer Johansson Date: 12-08-15   Time: 10:59
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for stringlength in call to WriteTextToRovAnalyser()
 * 
 * *****************  Version 2  *****************
 * User: Christer Johansson Date: 12-08-08   Time: 15:41
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for UDP_CONTROL_MSG_TYPE.
 * Added support for CTRL_BAD_TIME_SETTING.
 * 
 * *****************  Version 1  *****************
 * User: Christer Johansson Date: 12-02-27   Time: 14:30
 * Created in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * 
 * *****************  Version 26  *****************
 * User: Christer Johansson Date: 12-02-21   Time: 9:40
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Commented unused udpMessageToTransmit.
 * 
 * *****************  Version 25  *****************
 * User: Christer Johansson Date: 12-02-17   Time: 14:46
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Use udpMessageDelay with xSerialUDPTxTaskQueue in case queue is busy.
 * 
 * *****************  Version 24  *****************
 * User: Christer Johansson Date: 12-02-16   Time: 11:15
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Use #ifdef DEBUG_INCLUDED around dbgprintf()
 * 
 * *****************  Version 23  *****************
 * User: Christer Johansson Date: 12-02-10   Time: 15:58
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support to show temperature on ControlBoard and optionBoards as
 * 16 bit signed (2-compement).
 * 
 * *****************  Version 22  *****************
 * User: Christer Johansson Date: 12-02-09   Time: 12:11
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added SendTextToSmallPcuAndOverlay() function.
 * 
 * *****************  Version 21  *****************
 * User: Christer Johansson Date: 12-02-08   Time: 16:03
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for sending message to ROC Analyser fronm ROV.
 * 
 * *****************  Version 20  *****************
 * User: Christer Johansson Date: 12-02-07   Time: 14:45
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for setting time on the boards in the system.
 * 
 * *****************  Version 19  *****************
 * User: Christer Johansson Date: 12-01-15   Time: 12:41
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Commented unused functions to save flash space.
 * 
 * *****************  Version 18  *****************
 * User: Christer Johansson Date: 11-12-12   Time: 13:55
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Changed text handling to overlay to show correct text.
 * Increased lowest temperature warning from 45 to 50 degrees.
 * Increased time for temperature checking so text will blink on PCU and
 * show continuesly on overlay when 70 degrees is reached.
 * 
 * *****************  Version 17  *****************
 * User: Christer Johansson Date: 11-12-09   Time: 11:56
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Start using ErrorReportingWithDebugText() so we do not get spammed by
 * error messages...
 * 
 * *****************  Version 16  *****************
 * User: Christer Johansson Date: 11-11-08   Time: 13:17
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Moved WriteTextImPcuDisplay() and WriteTextToOverlay() to utilies.c
 * 
 * *****************  Version 15  *****************
 * User: Christer Johansson Date: 11-10-31   Time: 15:38
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/SystemProgram/OM_APP
 * Added addCheckSumNMEAtype()
 * 
 * *****************  Version 14  *****************
 * User: Christer Johansson Date: 11-10-27   Time: 11:06
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/SystemProgram/OM_APP
 * Clean up code
 * 
 * *****************  Version 13  *****************
 * User: Christer Johansson Date: 11-10-27   Time: 10:49
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/SystemProgram/OM_APP
 * Added support for different hw/fw in OptionBoard.
 * Start using bit definitions in reg 3 in EXTOUTX
 * 
 * *****************  Version 12  *****************
 * User: Christer Johansson Date: 11-10-24   Time: 16:28
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/SystemProgram/OM_APP
 * Fixed Windows NewLine problem: Changed "\n\r" to "\r\n" to make
 * debugtext look nicer in Windows.
 * 
 * *****************  Version 11  *****************
 * User: Christer Johansson Date: 11-10-24   Time: 14:00
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/SystemProgram/OM_APP
 * Changed name of efunctionToCheckType to eFunctionToCheckType.
 * Changed name of efunctionToCheckSeverityType to
 * eFunctionToCheckSeverityType.
 * 
 * *****************  Version 10  *****************
 * User: Christer Johansson Date: 11-10-24   Time: 13:31
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/SystemProgram/OM_APP
 * Added CheckIfThisBoardSupportsThisFwFunction() to check if FPGA has
 * support for a function or not.
 * 
 * *****************  Version 9  *****************
 * User: Christer Johansson Date: 11-07-08   Time: 11:26
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added support for IsEthernetUpAndRunningFlag and function
 * IsEthernetUpAndRunning() to see when ethernet is up and running.
 * 
 * *****************  Version 8  *****************
 * User: Christer Johansson Date: 11-06-27   Time: 14:47
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Removed support for InternalVariables.rovType.
 * 
 * *****************  Version 7  *****************
 * User: Christer Johansson Date: 11-03-21   Time: 11:02
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added support for IMU and depth sensors.
 * Added support to get all data from IMU and depth sensor to
 * ROVControlTask in one message.
 * Added state machine in ExternalIOHandler for better handling.
 * Let vTaskIncrementTick() send message to ExternalIOTask triggering
 * sensor reading every 50ms.
 * etc.
 * 
 * *****************  Version 6  *****************
 * User: Christer Johansson Date: 11-02-18   Time: 12:23
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added some new functions etc. for more easy handling oc UART queues
 * etc.
 * 
 * *****************  Version 5  *****************
 * User: Christer Johansson Date: 10-12-14   Time: 11:51
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Tidy the code up a bit.
 * 
 * *****************  Version 4  *****************
 * User: Christer Johansson Date: 10-12-14   Time: 10:50
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Clean up the code a bit.
 * 
 * *****************  Version 3  *****************
 * User: Christer Johansson Date: 10-08-24   Time: 13:24
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Clean up code
 * 
 * *****************  Version 2  *****************
 * User: Christer Johansson Date: 10-08-18   Time: 15:23
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Updated software, removed hw support for RTC, Handler functions etc.
 * 
 * *****************  Version 1  *****************
 * User: CJN      Date: 10-08-05   Time: 15:21
 * Created in $/Ocean Modules
 * 
 */

//*************************************************************************
//* Disable the keyword functionallity
//* $NoKeywords: $
//*************************************************************************


//*************************************************************************
//*
//*        INCLUDE FILES
//*
//*************************************************************************

// Standard includes.
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>

// This file's own include
//#include "constants.h" // includes in debugprintf.h
#include "defines.h"
#include "constants.h"
#include "utilities.h"
#include "RTC.h"
#include "globals.h"
#include "wdt.h"
#include "CANhandler.h"

#ifdef  DEBUG_INCLUDED     // Debugging allowed?
// Make it possible to use dbgprintf() etc to debug port
#include "debugprintf.h"
#endif //DEBUG_INCLUDED    // Debugging allowed?


//*************************************************************************
//*
//*        GLOBALS
//*  used in CheckIfThisBoardSupportsThisFwFunction()
//*
//*************************************************************************

// FPGA WRITE REGISTERS
UBYTE FpgaWriteRegExtOut0PwmHi = 0x00;
UBYTE FpgaWriteRegExtOut0PwmLo = 0x00;
UBYTE FpgaWriteRegExtOut1PwmHi = 0x00;
UBYTE FpgaWriteRegExtOut1PwmLo = 0x00;
UBYTE FpgaWriteRegExtOut2PwmHi = 0x00;
UBYTE FpgaWriteRegExtOut2PwmLo = 0x00;
UBYTE FpgaWriteRegExtOut3PwmHi = 0x00;
UBYTE FpgaWriteRegExtOut3PwmLo = 0x00;

UBYTE FpgaWriteRegExtOut4PwmHi = 0x00;
UBYTE FpgaWriteRegExtOut4PwmLo = 0x00;
UBYTE FpgaWriteRegExtOut5PwmHi = 0x00;
UBYTE FpgaWriteRegExtOut5PwmLo = 0x00;
UBYTE FpgaWriteRegExtOut6PwmHi = 0x00;
UBYTE FpgaWriteRegExtOut6PwmLo = 0x00;
UBYTE FpgaWriteRegExtOut7PwmHi = 0x00;
UBYTE FpgaWriteRegExtOut7PwmLo = 0x00;

// FPGA READ REGISTERS
UBYTE FpgaReadRegExtOut0Volt      = 0x00;
//UBYTE FpgaReadRegExtOut0PwmStatus = 0x00;
UBYTE FpgaReadRegExtOut1Volt      = 0x00;
//UBYTE FpgaReadRegExtOut1PwmStatus = 0x00;
UBYTE FpgaReadRegExtOut2Volt      = 0x00; 
//UBYTE FpgaReadRegExtOut2PwmStatus = 0x00;
UBYTE FpgaReadRegExtOut3Volt      = 0x00;
//UBYTE FpgaReadRegExtOut3PwmStatus = 0x00;

UBYTE FpgaReadRegExtOut4Volt      = 0x00;
//UBYTE FpgaReadRegExtOut4PwmStatus = 0x00; 
UBYTE FpgaReadRegExtOut5Volt      = 0x00;
//UBYTE FpgaReadRegExtOut5PwmStatus = 0x00;
UBYTE FpgaReadRegExtOut6Volt      = 0x00;
//UBYTE FpgaReadRegExtOut6PwmStatus = 0x00;
UBYTE FpgaReadRegExtOut7Volt      = 0x00;
//UBYTE FpgaReadRegExtOut7PwmStatus = 0x00;

// FPGA REGISTER BIT DFFINITIONS

// EXTOUT0-7, REGISTER 3, bit definitions;
UINT16 FpgaWriteExtOutReg3OutPutControl    = 0x00;
UINT16 FpgaWriteExtOutReg3PolarityControl  = 0x00;
UINT16 FpgaWriteExtOutReg3PwmEnable1       = 0x00;
UINT16 FpgaWriteExtOutReg3PwmEnable2       = 0x00;
UINT16 FpgaWriteExtOutReg3PwmMode          = 0x00;
UINT16 FpgaWriteExtOutReg3SingelCont       = 0x00;
UINT16 FpgaWriteExtOutReg3HiLevelTimeBase1 = 0x00;
UINT16 FpgaWriteExtOutReg3HiLevelTimeBase2 = 0x00;
UINT16 FpgaWriteExtOutReg3HiLevelTimeBase3 = 0x00;
UINT16 FpgaWriteExtOutReg3LoLevelTimeBase1 = 0x00;
UINT16 FpgaWriteExtOutReg3LoLevelTimeBase2 = 0x00;
UINT16 FpgaWriteExtOutReg3LoLevelTimeBase3 = 0x00;



//*************************************************************************
//*
//*        FUNCTION PROTOTYPES
//*
//*************************************************************************
//UINT16  HexToInt16( UBYTE *stringpointer, UBYTE numberofBytes );
//UBYTE   TwoHexToByte( UBYTE *stringpointer );
//UBYTE   OneHexToByte( UBYTE *stringpointer );
//void    sprintUint64( UBYTE *strptr, UINT64 uint64Value ); // converts UINT64 decimal value to string.
//void    sprintSint64( UBYTE *strptr, SINT64 sint64Value ); // converts SINT64 decimal value to string.
//void    MyStringCopy( UBYTE *toPointer, UBYTE *fromPointer, UINT16 sizeOfFromField, UINT16 sizeOfToField, UBYTE padChar );
//UINT16  CountDoubleHash( UBYTE *fromPointer, UINT16 numberOfCharsInString );
//UBYTE   MyCompareString( UBYTE *firstPointer, UBYTE *secondPointer );
//UINT64  GetUintValueFromInputString( UBYTE *stringPointer, UINT16 sizeOfFromField );
//SINT64  GetSintValueFromInputString( UBYTE *stringPointer, UINT16 sizeOfFromField );
//SBYTE   Uint64ToStrWithComma( UBYTE *pRtrStr, SBYTE sizeOfStr, UBYTE padChar, UINT64 uintToConv );
//SBYTE   Sint64ToStrWithComma( UBYTE *pRtrStr, SBYTE sizeOfStr, UBYTE padChar, SINT64 sintToConv );
//SBYTE   Uint64ToStrWithoutComma( UBYTE *pRtrStr, SBYTE sizeOfStr, UBYTE padChar, UINT64 uintToConv );
//static  UBYTE unsignedCharToBCD( UBYTE charToConvert );
//static  UBYTE BCDToUnsignedChar( UBYTE charToConvert );
//void    LoadFpgaVariablesDependingOnHwFwVersion( void );
//bool    CheckIfThisBoardSupportsThisFwFunction( eFunctionToCheckType functionToCheck, eFunctionToCheckSeverityType severity );
//void    addCheckSumNMEAtype( UBYTE *stringPointer, UBYTE maxNumber );


//************************************************************************************
//
//                  HexToInt16
//
//   Convert 1-4 bytes in a string to hexadecimal value
//   Example "1D6A" will returnr 0x1D6A
//
//   Input:          *UBYTE            Pointer to string
//                   UBYTE             Number of bytes (1-4) in string to convert
//
//   Output:         UINT16             value
//
//   Sets:           -
//
//************************************************************************************
UINT16 HexToInt16( UBYTE *stringpointer, UBYTE numberofBytes )
{
UBYTE  tempByte1 = 0;
UINT16 tempInt16 = 0;

   if( numberofBytes > 3 )
   {
      // 2 complete hex bytes (XXXX)
      tempInt16 = TwoHexToByte( stringpointer );
      stringpointer++;
      stringpointer++;
      tempByte1 = TwoHexToByte( stringpointer );
   }
   else if( numberofBytes > 2 )
   {
      // 1,5 complete hex bytes (XXX)
      tempInt16 = OneHexToByte(stringpointer);
      stringpointer++;
      tempByte1 = TwoHexToByte( stringpointer );
   }
   else if( numberofBytes > 1 )
   {
      // 1 complete hex bytes (XX)
      tempByte1 = TwoHexToByte( stringpointer );
   }
   else
   {
      // Just 1 digit (X)
      tempByte1 = OneHexToByte(stringpointer);
   }
   return((tempInt16<<8) + tempByte1);
}


//************************************************************************************
//
//                  TwoHexToByte
//
//   Convert 2 bytes in a string to hexadecimal value
//   Example "6A" will returnr 0x6A
//
//   Input:          *static UBYTE     Pointer to string
//
//   Output:         UBYTE             1-byte value
//
//   Sets:           -
//
//************************************************************************************
UBYTE TwoHexToByte( UBYTE *stringpointer )
{
UBYTE tempByte = 0;
UBYTE i;

   for( i=0 ; i<2 ; i++ )
   {
      tempByte += OneHexToByte(stringpointer);

      if( i == 0 )
      {
         tempByte = tempByte<<4;
      }
      stringpointer++;
   }
   return( tempByte );
}



//************************************************************************************
//
//                  OneHexToByte
//
//   Convert 1 bytes in a string to hexadecimal value
//   Example "A" will return 0x0A
//
//   Input:          *static UBYTE     Pointer to string
//
//   Output:         UBYTE             1-byte value
//
//   Sets:           -
//
//************************************************************************************
UBYTE OneHexToByte( UBYTE *stringpointer )
{
UBYTE tempByte = 0;

   if( *stringpointer <= 0x39 )
   {
      tempByte = (*stringpointer - 0x30);
   }
   else if( *stringpointer >= 0x61 )
   {
      tempByte = (*stringpointer - 0x57);
   }
   else
   {
      tempByte = (*stringpointer - 0x37);
   }
   return( tempByte );
}





//// *************************************************************************
//// *
//// *                 sprintUint64
//// *  This function converts UINT64 value to decimal string.
//// *
//// *
//// *  Input:         UBYTE *  strptr
//// *                 UINT64
//// *
//// *  Output:        -
//// *
//// *  Sets:          string according to strptr
//// *
//// *************************************************************************
//void sprintUint64( UBYTE *strptr, UINT64 uint64Value )
//{
//UINT32   temp32Value1;
//UINT32   temp32Value2;
//UINT64   temp64Value;
//
//   if( uint64Value >= 1000000000 )
//   {
//      temp32Value1 = uint64Value / 1000000000;
//      temp64Value  = temp32Value1;  // how many billions?
//      temp64Value *= 1000000000;    // make it real number
//      temp64Value = uint64Value - temp64Value; // remaining 9 digits below 1 billion
//      temp32Value2 = (UINT32)(temp64Value);
//      sprintf( strptr, "%d%09d", temp32Value1, temp32Value2 );
//   }
//   else
//   {
//      temp32Value1 = (UINT32)(uint64Value);
//      sprintf( strptr, "%d", temp32Value1 );
//   }
//}



//// *************************************************************************
//// *
//// *                 sprintSint64()
//// *  This function converts SINT64 value to decimal string.
//// *
//// *
//// *  Input:         UBYTE *strptr
//// *                 SINT64
//// *
//// *  Output:        -
//// *
//// *  Sets:          string according to strptr
//// *
//// *************************************************************************
//void sprintSint64( UBYTE *strptr, SINT64 sint64Value )
//{
//SINT64   temp64Value;
//
//   if( sint64Value < 0 )
//   {
//      temp64Value = (-sint64Value);
//      sprintUint64( strptr+1, (UINT64)temp64Value );
//      strptr[0] = '-';
//   }
//   else
//   {
//      sprintUint64( strptr, (UINT64)sint64Value );
//   }
//}




////************************************************************************************
////
////                  MyStringCopy
////
////   Copy string from one string (like interpretFieldInString) to another
////   string (like ptrSTstoreTransactionStruct->controlData) right-adjusted.
////   String will be padded uing padChar.
////
////   Input:          *UBYTE            Pointer to string to copy TO
////                   *UBYTE            Pointer to string to copy FROM
////                   UINT16            Number of data to get from FROM string
////                   UINT16            Size of field where to write data
////                   UBYTE             Chararcter for padding, normally '0' or ' '.
////
////   Output:         -
////
////   Sets:           output string
////
////************************************************************************************
//void MyStringCopy( UBYTE *toPointer, UBYTE *fromPointer, UINT16 sizeOfFromField, UINT16 sizeOfToField, UBYTE padChar )
//{
//UINT16   i;
//UINT16   numberOfDataCopied;
//UINT16   numberOfSquareCharsToCopy;
//UBYTE *  internalPointer;
//
//   // start by padding
//   internalPointer = toPointer;
//   for( i=0 ; i<sizeOfToField ; i++ )
//   {
//      *internalPointer = padChar;
//      internalPointer++;
//   }
//
//   // check for ## in string
//   internalPointer = fromPointer;
//   numberOfSquareCharsToCopy = 0;
//   for( i=0 ; i<sizeOfFromField ; i++ )
//   {
//      if(*internalPointer == '#' )
//      {
//         numberOfSquareCharsToCopy++;
//         i++;                 // skip next '#'
//         internalPointer++;   // skip next '#'
//      }
//      internalPointer++;
//   }
//
//
//   // Time to copy...
//   // Compensate for one extra '#' for every '#'
//   internalPointer = toPointer + sizeOfToField - sizeOfFromField + numberOfSquareCharsToCopy;
//   numberOfDataCopied = 0;
//   for( i=0 ; i<sizeOfFromField ; i++ )
//   {
//      *internalPointer = *fromPointer++;
//      if(*internalPointer == '#' )
//      {
//         fromPointer++;    // skip next '#'
//         i++;              // skip next '#'
//      }
//      internalPointer++;
//      numberOfDataCopied++;
//      if( numberOfDataCopied >= sizeOfToField )
//      {
//         break;   // Don't copy more than buffer size!
//      }
//   }
//}








////************************************************************************************
////
////                  CountDoubleHash
////
////   Count "##" in string
////
////   Input:          *UBYTE            Pointer to string.
////                   UINT16            Number of bytes in string
////
////   Output:         UINT16            Number of "##" that should be replaced by "#"
////
////   Sets:           -
////
////************************************************************************************
//UINT16 CountDoubleHash( UBYTE *fromPointer, UINT16 numberOfCharsInString )
//{
//UINT16 i;
//UINT16 numDoubleHash;
//
//
//   numDoubleHash = 0;
//   for( i=0 ; i<numberOfCharsInString-1 ; i++ )
//   {
//      if( *fromPointer++ == '#' )
//      {
//         if( *fromPointer == '#' )
//         {
//            // "##" found!
//            numDoubleHash++;     // Double '#' found!
//            fromPointer++;       // Skip the next '#' when looking.
//            i++;                 // Keep track of how many data has been checked
//         }
//      }
//   }
//   return( numDoubleHash );
//}





////************************************************************************************
////
////                  MyCompareString
////
////   Compare one string to another, but first string does not have end-of-string '\0'
////
////   Input:          *static UBYTE     Pointer to string without end-of-string '\0'
////                   *static UBYTE     Pointer to string with end-of-string '\0'
////
////   Output:         TRUE if equal
////                   FALSE if not equal
////
////   Sets:           -
////
////************************************************************************************
//UBYTE MyCompareString( UBYTE *firstPointer, UBYTE *secondPointer )
//{
//UINT16   len;
//UINT16   i;
//UBYTE    equal = TRUE;
//
//
//   len = strlen( secondPointer );
//   for( i=0 ; i<len ; i++ )
//   {
//      if( firstPointer[i] != secondPointer[i] )
//      {
//         equal = FALSE;
//      }
//   }
//   return( equal );
//}



//************************************************************************************
//
//                  GetUintValueFromInputString()
//
//   Get value from from interpretFieldInString.
//   Removes any decimal comma sign ',' before converting.
//   Ends string if finding '#'.
//   Max allowed number is dec.     1 000 000 000 000 000 000.
//   (with 2 decimals max number is    10 000 000 000 000 000,00)
//
//   Input:          *static UBYTE     Pointer to string to copy FROM
//                   UINT16            Number of data to get from FROM string
//
//   Output:         UINT64            Value of string (no decimal)
//
//   Sets:           -
//
//************************************************************************************
#define  INTERNAL_STR_LEN 20
UINT64 GetUintValueFromInputString( UBYTE *stringPointer, UINT16 sizeOfFromField )
{
UBYTE    i;
UBYTE    internalString[INTERNAL_STR_LEN];
UBYTE    internalStringLen;
UBYTE *  internalPointer;
UINT32   tempUint32;
UINT64   tempUint64;

   if(sizeOfFromField >= INTERNAL_STR_LEN)
   {
#ifdef  DEBUG_INCLUDED     // Debugging allowed?
      dbgprintf( "\r\n\n!!! TOO SHORT STRING (%.02x Hex) internalString in GetValueFromInputString() !!!\r\n",sizeOfFromField );
#endif //DEBUG_INCLUDED    // Debugging allowed?
   }

   // copy string without the ',' character
   internalPointer = &internalString[0];
   for( i=0 ; i<sizeOfFromField ; i++ )
   {
      if(*stringPointer == '#' )
      {
         break;
      }
//      else if(*stringPointer == ' ' && internalPointer > &internalString[0] )
//      {
//         // We just found space character after we have found numbers.
//         // This means data is left-adjusted
//         break;
//      }
      else if(*stringPointer != ',' )
      {
         *internalPointer++ = *stringPointer;
      }
      stringPointer++;
   }
   *internalPointer = '\0';
   internalStringLen = strlen(internalString);

   if( internalStringLen > 9 )
   {
      tempUint32 = atoi(&internalString[internalStringLen-9]);
      internalString[internalStringLen-9] = '\0';
      tempUint64 = atoi(internalString);
      tempUint64 *= 1000000000;
      tempUint64 += tempUint32;
   }
   else
   {
      tempUint64 = atoi(internalString);
   }

   // Time to return the value...
   return(tempUint64);
}





//************************************************************************************
//
//                  GetSintValueFromInputString()
//
//   Get value from from interpretFieldInString.
//   Removes any decimal comma sign ',' before converting.
//   Ends string if finding '#'.
//   Max allowed number is dec.     1 000 000 000 000 000 000.
//   (with 2 decimals max number is    10 000 000 000 000 000,00)
//
//   Input:          *static UBYTE     String to copy FROM
//                   UINT16            Number of data to get from FROM string
//
//   Output:         SINT64            Value of string (no decimal)
//
//   Sets:           -
//
//************************************************************************************
#define  INTERNAL_STR_LEN 20
SINT64 GetSintValueFromInputString( UBYTE *stringPointer, UINT16 sizeOfFromField )
{
UBYTE    i;
UBYTE    internalString[INTERNAL_STR_LEN];
UBYTE    internalStringLen;
UBYTE    negativeValue;
UBYTE *  internalPointer;
SINT32   tempSint32;
SINT64   tempSint64;

   internalPointer = &internalString[0];
   negativeValue = FALSE;
   if(sizeOfFromField >= INTERNAL_STR_LEN)
   {
#ifdef  DEBUG_INCLUDED     // Debugging allowed?
      dbgprintf( "\r\n\n!!! TOO SHORT STRING (%.02x Hex) internalString in GetValueFromInputString() !!!\r\n",sizeOfFromField );
#endif //DEBUG_INCLUDED    // Debugging allowed?
   }

   // copy string without the ',' and '-' characters
   for( i=0 ; i<sizeOfFromField ; i++ )
   {
      if(*stringPointer == '#' )
      {
         break;
      }
//      else if(*stringPointer == ' ' && internalPointer > &internalString[0] )
//      {
//         // We just found space character after we have found numbers.
//         // This means data is left-adjusted
//         break;
//      }
      else if(*stringPointer != ',' )
      {
         if(*stringPointer == '-' )
         {
            negativeValue = TRUE;
         }
         else
         {
            *internalPointer++ = *stringPointer;
         }
      }
      stringPointer++;
   }
   *internalPointer = '\0';
   internalStringLen = strlen(internalString);

   // SINT32 = 0x0000 00000 - 0x7FFF FFFF plus sign.
   // -2147483647 to +2147483647

   // SINT32 = 0x0000 00000 - 0x7FFF FFFF plus sign.
   // -2147483647 to +2147483647

   if( internalStringLen > 9 )
   {
      tempSint32 = atoi(&internalString[internalStringLen-9]);
      internalString[internalStringLen-9] = '\0';
      tempSint64 = atoi(internalString);
      tempSint64 *= 1000000000;
      tempSint64 += tempSint32;
   }
   else
   {
      tempSint64 = atoi(internalString);
   }

   // Time to return the value...
   if( negativeValue == TRUE )
   {
      return(-tempSint64);
   }
   return(tempSint64);
}




//// *************************************************************************
//// *
//// *           Uint64ToStrWithComma()
//// *
//// *  This function takes an UINT64 and returns the value in a string with two
//// *  digits after the comma.
//// *  The string will be right adjusted with padCars inserted.
//// *
//// *  Input:
//// *     UBYTE *pRtrStr                      -- Contains the original string to be stored.
//// *     UBYTE sizeOfStr                     -- Size of the returnstring without the '\0'.
//// *     UBYTE padChar                       -- Character to use for padding.
//// *     UINT64 uintToConv                   -- The UINT64 to convert.
//// *
//// *  Output:
//// *     SBYTE
//// *        OK                         -- No problem encountered.
//// *        FAIL                       -- Failed to save data.
//// *
//// *
//// *************************************************************************
//// 64 bit => 18446744073709551615 20 ascii + '\0'
//// *************************************************************************
//SBYTE Uint64ToStrWithComma( UBYTE *pRtrStr, SBYTE sizeOfStr, UBYTE padChar, UINT64 uintToConv )
//{
//   UBYTE string[21];
//   SBYTE convStrLen;
//
//   // the string has to be large enough to contain 0,00
//   if ( sizeOfStr < 4 )
//   {
//      return ( FAIL );
//   }
//
//   memset( pRtrStr, padChar, sizeOfStr );
//   pRtrStr[sizeOfStr] = '\0';
//   // convert the UINT64 to a string
//   sprintUint64( string, uintToConv );
//
//   // copy the return string from sprintUint64 to the return pointer received in the funciton call
//   // make room for the ',' at the same time.
//   convStrLen = strlen(string);
//   // decrese convStrLen with one to make it index correctly in an array that starts with 0
//   convStrLen -= 1;
//   sizeOfStr -= 1;
//
//   // if the numer is only cents
//   if ( convStrLen < 2 )      // less then two since we subtracted on earlier.
//   {
//      if ( convStrLen < 1 )      // If the return string only consists of one char.
//      {
//         pRtrStr[sizeOfStr] = string[convStrLen];
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = '0';
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = ',';
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = '0';
//         return ( OK );
//      }
//      else                      // If the return string consists of two chars.
//      {
//         pRtrStr[sizeOfStr] = string[convStrLen];
//         sizeOfStr--;
//         convStrLen--;
//         pRtrStr[sizeOfStr] = string[convStrLen];
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = ',';
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = '0';
//         return ( OK );
//      }
//   }
//   else
//   {
//      // copy the string from left to right and insert ',' at the correct place.
//      pRtrStr[sizeOfStr] = string[convStrLen];
//      convStrLen -= 1;
//      sizeOfStr -= 1;
//      pRtrStr[sizeOfStr] = string[convStrLen];
//      sizeOfStr -= 1;
//      pRtrStr[sizeOfStr] = ',';
//
//      while ( ( convStrLen > 0 ) && ( sizeOfStr > 0 ) )
//      {
//         convStrLen -= 1;
//         sizeOfStr -= 1;
//         pRtrStr[sizeOfStr] = string[convStrLen];
//      }
//      return ( OK );
//   }
//}



//// *************************************************************************
//// *
//// *           Sint64ToStrWithComma()
//// *
//// *  This function takes an SINT64 and returns the value in a string with two
//// *  digits after the comma character.
//// *  The string will be right adjusted with padCars inserted.
//// *
//// *  Input:
//// *     UBYTE *pRtrStr                      -- Contain the original string to be stored.
//// *     UBYTE sizeOfStr                     -- Size of the returnstring without the '\0'.
//// *     UBYTE padChar                       -- Character to use for padding.
//// *     SINT64 sintToConv                   -- The SINT64 to convert.
//// *
//// *  Output:
//// *     SBYTE
//// *        OK                         -- No problem encountered.
//// *        FAIL                       -- Failed to save data.
//// *
//// *
//// *************************************************************************
//// 64 bit => 18446744073709551615 20 ascii + '\0'
//// *************************************************************************
//SBYTE Sint64ToStrWithComma( UBYTE *pRtrStr, SBYTE sizeOfStr, UBYTE padChar, SINT64 sintToConv )
//{
//   UBYTE string[21];
//   SBYTE convStrLen;
//
//   if( sintToConv < 0 )
//   {
//      // the string has to be large enough to contain -0,01
//      if ( sizeOfStr < 5 )
//      {
//         return ( FAIL );
//      }
//   }
//   else
//   {
//      // the string has to be large enough to contain 0,00
//      if ( sizeOfStr < 4 )
//      {
//         return ( FAIL );
//      }
//   }
//
//   memset( pRtrStr, padChar, sizeOfStr );
//   pRtrStr[sizeOfStr] = '\0';
//   sprintSint64( string, sintToConv );   // convert the SINT64 to a string
//
//   // copy the return string from sprintUint64 to the return pointer received in the funciton call
//   // make room for the ',' at the same time.
//   convStrLen = strlen(string);
//   // decrese convStrLen with one to make it index correctly in an array that starts with 0
//   convStrLen -= 1;
//   sizeOfStr -= 1;
//
//   // if the number is only cents
//   if ( convStrLen < 2 )      // less then two since we subtracted on earlier.
//   {
//      if ( convStrLen < 1 )      // If the return string only consists of one char.
//      {
//         pRtrStr[sizeOfStr] = string[convStrLen];
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = '0';
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = ',';
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = '0';
//      }
//      else                      // If the return string consists of two chars.
//      {
//         pRtrStr[sizeOfStr] = string[convStrLen];
//         sizeOfStr--;
//         convStrLen--;
//         pRtrStr[sizeOfStr] = string[convStrLen];
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = ',';
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = '0';
//      }
//      if( sintToConv < 0 )
//      {
//         sizeOfStr--;
//         pRtrStr[sizeOfStr] = '-';
//      }
//      return ( OK );
//   }
//   else
//   {
//      // copy the string from left to right and insert ',' at the correct place.
//      pRtrStr[sizeOfStr] = string[convStrLen];
//      convStrLen -= 1;
//      sizeOfStr -= 1;
//      pRtrStr[sizeOfStr] = string[convStrLen];
//      sizeOfStr -= 1;
//      pRtrStr[sizeOfStr] = ',';
//
//      while ( ( convStrLen > 0 ) && ( sizeOfStr > 0 ) )
//      {
//         convStrLen -= 1;
//         sizeOfStr -= 1;
//         pRtrStr[sizeOfStr] = string[convStrLen];
//      }
//      return ( OK );
//   }
//}





//// *************************************************************************
//// *
//// *           Uint64ToStrWithoutComma()
//// *
//// *  This function takes an UINT64 and returns the value in a string.
//// *  The string will be right adjusted with padCars inserted to the correct len.
//// *
//// *  Input:
//// *     UBYTE *pRtrStr                      -- Pointer to string where to write result.
//// *     UBYTE sizeOfStr                     -- Size of the created string without any '\0'.
//// *     UBYTE padChar                       -- Character to use for padding.
//// *     UINT64 uintToConv                   -- The UINT64 to convert.
//// *
//// *  Output:
//// *     SBYTE
//// *        OK                         -- No problem encountered.
//// *        FAIL                       -- failed to save data.
//// *
//// *
//// *************************************************************************
//// 64 bit => 18446744073709551615 20 ascii + '\0'
//// *************************************************************************
//SBYTE Uint64ToStrWithoutComma( UBYTE *pRtrStr, SBYTE sizeOfStr, UBYTE padChar, UINT64 uintToConv )
//{
//   UBYTE string[21];
//   SBYTE convStrLen;
//
//   memset( pRtrStr, padChar, sizeOfStr );
//   pRtrStr[sizeOfStr] = '\0';
//   // convert the UINT64 to a string
//   sprintUint64( string, uintToConv );
//
//   // copy the return string from sprintUint64 to the return pointer received in the funciton call
//   // make room for the ',' at the same time.
//   convStrLen = strlen(string);
//
//   // copy the string from left to right.
//
//   while ( ( convStrLen > 0 ) && ( sizeOfStr > 0 ) )
//   {
//      convStrLen -= 1;
//      sizeOfStr -= 1;
//      pRtrStr[sizeOfStr] = string[convStrLen];
//   }
//   return ( OK );
//}




// *************************************************************************
// *
// *           MyFindString()
// *
// *  Locates an array of bytes in another array of bytes (like strcmp)
// *
// *
// *  Input:
// *     UBYTE[] *string1                  pointer to array of bytes to search
// *     UBYTE[] *string2                  pointer to array of bytes as "searchstring"
// *     UINT16   numberOfBytesInString1   length of string1
// *     UINT16   numberOfBytesInString2   length of string2
// *
// *  Output:
// *     UINT16                            position 1, 2, 3... where string was found, 
// *                                                0          if not found.
// *
// *************************************************************************
UINT16 MyFindString( UBYTE *string1, UBYTE *string2, UINT16 numberOfBytesInString1, UINT16 numberOfBytesInString2 )
{
SINT16 i;
SINT16 j;

   for( i=0 ; i<=(numberOfBytesInString1-numberOfBytesInString2) ; i++ )
   {
      for( j=0 ; j<numberOfBytesInString2 ; j++ )
      {
         if( string1[i+j] != string2[j] ) 
         {
            break;   // Not correct - try next position in main string (next i)
         }
      }
      if( j == numberOfBytesInString2 )
      {
         return( i+1 ); // comparasion correct!
      }
   }
   return( 0 );   // not found!
}



#ifdef  DEBUG_INCLUDED     // Debugging allowed?
//-----------------------------------------------------------------------------
/// Send rovType text to debug port
//-----------------------------------------------------------------------------
void PrintRovTypeOnDebugPort( void )
{
UBYTE whatRovTypeString[20];

   GetRovType( whatRovTypeString );   // get RovType
   dbgprintf( "ROV TYPE: %s", whatRovTypeString );
}
#endif //DEBUG_INCLUDED


//-----------------------------------------------------------------------------
/// Copy rovType text to memory location
//  NOTE &THAT WE MUST NOT USE TOO LONG STRINGS!
//  MAX STRING LENGTH IS 20 BYTES FOR NOW
//-----------------------------------------------------------------------------
UBYTE GetRovType( UBYTE *whereToWriteIt )
{
#define MAX_TEXT_LENGHT 20
UBYTE stringLength = 0;   
UBYTE stringAddLength = 0;

   #ifdef TEST300VONOFF
      stringLength = 8+1; // including end-of-string char
      memcpy( whereToWriteIt, "LEA-TEST", stringLength );
   #elif defined TESTJOYSTICKCOMMAND
      stringLength = 8+1; // including end-of-string char
      memcpy( whereToWriteIt, "JOY-TEST", stringLength );
   #elif defined TEST_IMU_BAD_DATA
      stringLength = 8+1; // including end-of-string char
      memcpy( whereToWriteIt, "IMU-TEST", stringLength );
      
   // ---------------------------------------------------------
      
   #elif defined STD_L3000
      stringLength = 6+1; // including end-of-string char
      memcpy( whereToWriteIt, "CF:STD", stringLength );
   #elif defined GU_VEHICLE
      stringLength = 5+1; // including end-of-string char
      memcpy( whereToWriteIt, "CF:GU", stringLength );
   #elif defined SEMATEK_VEHICLE
      stringLength = 10+1;
      memcpy( whereToWriteIt, "CF:Sematek", stringLength );
   #elif defined SEATERRA_VEHICLE
      stringLength = 11+1;
      memcpy( whereToWriteIt, "CF:SeaTerra", stringLength );
   #elif defined CHINA_UNIV_FENIX
      stringLength = 13+1;
      memcpy( whereToWriteIt, "CF:ChinaUniv.", stringLength );
   //   #elif defined ROYAL_NAVY_FENIX
   //      stringLength = 16+1;
   //      memcpy( whereToWriteIt, "CF:GB Royal Navy", stringLength );
   //   #elif defined JD_CONTRACTOR_FENIX
   //      stringLength = 16+1;
   //      memcpy( whereToWriteIt, "CF:JD-Contractor", stringLength );
   #elif defined STD_FENIX
      stringLength = 6+1;
      memcpy( whereToWriteIt, "CF:Std", stringLength );
   #else
      stringLength = 10+1;
      memcpy( whereToWriteIt, "CF:Unknown", stringLength );
   #endif

   if( stringLength > MAX_TEXT_LENGHT )
   {
      stringLength = 16+1;
      memcpy( whereToWriteIt, "TOO LONG STRING!", stringLength );
      return( stringLength );  // including end-of-string char
   }
   whereToWriteIt += (stringLength-1);
   #ifdef ROV_V8_FENIX
      stringAddLength = 5; // Add for M500
      if( (stringAddLength + stringLength) > MAX_TEXT_LENGHT )
      {
         return( stringLength );  // TOO LONG TO SUPPORT  THIS!
      }
      memcpy( whereToWriteIt, " M500", stringAddLength+1 ); // (incl. End-of-string)
   #else
      stringAddLength = 6; // Add for L3000
      if( (stringAddLength + stringLength) > MAX_TEXT_LENGHT )
      {
         return( stringLength );  // TOO LONG TO SUPPORT  THIS!
      }
      memcpy( whereToWriteIt, " L3000", stringAddLength+1 ); // (incl. End-of-string)
   #endif

   stringLength += stringAddLength; // Add for " (M500)" or " (L3000)"
   return( stringLength );  // including end-of-string char
}




//************************************************************************************
//
//                  IsEthernetUpAndRunning()
//
//   Keeps track of weither ethenet cable is connected and ethernet is running
//
//   Input:         -
//
//   Output:        bool   IsEthernetUpAndRunningFlag
//
//   Sets:          -
//
//************************************************************************************
bool IsEthernetUpAndRunning( void )
{
   return( IsEthernetUpAndRunningFlag );
}

//// *************************************************************************
//// *
//// *           unsignedCharToBCD()
//// *
//// *  Converts a char to BCD format.
//// *
//// *
//// *  Input:
//// *     UBYTE charToConvert          -- char to convert.
//// *
//// *  Output:
//// *     UBYTE                        -- char converted to BCD format
//// *
//// *************************************************************************
//static UBYTE unsignedCharToBCD( UBYTE charToConvert )
//{
//   UBYTE upperNibble;
//   UBYTE lowerNibble;
//
//   upperNibble = charToConvert / 10;
//   lowerNibble = charToConvert % 10;
//
//   return ( ( upperNibble << 4 ) | lowerNibble );
//}


// *************************************************************************
// *
// *           BCDToUnsignedChar()
// *
// *  Converts a char in BCD format to integer format.
// *
// *
// *  Input:
// *     UBYTE charToConvert          -- Char in BCD format.
// *
// *  Output:
// *     UBYTE                        -- Char converted to integer format
// *
// *************************************************************************
UBYTE BCDToUnsignedChar( UBYTE charToConvert )
{
   UBYTE upperNibble;
   UBYTE lowerNibble;

   upperNibble = charToConvert >> 4;
   lowerNibble = charToConvert & 0x0F;

   return ( ( upperNibble * 10 ) + lowerNibble );
}



//// *************************************************************************
//// *
//// * TestByteOrder()
//// * Tests byte alignment to determine Endian Format of local host.
//// *
//// * returns:     The ENDIAN platform identifier.
//// *************************************************************************
//int TestByteOrder()
//{
//   short int word = 0x0001;
//   char *byte = (char *) &word;
//   return(byte[0] ? LITTLE_ENDIAN : BIG_ENDIAN);
//}


//// *************************************************************************
//// * FloatFromBytes
//// * Converts bytes to Float.
//// *
//// * parameters:  pBytes : received buffer containing pointer to 4 bytes
//// *
//// * returns:     a float value.
//// *************************************************************************
//float FloatFromBytes(const unsigned char* pBytes)
//{
//	float f = 0;
//	if(TestByteOrder() != BIG_ENDIAN) {
//	   ((BYTE*)(&f))[0] = pBytes[3];
//	   ((BYTE*)(&f))[1] = pBytes[2];
//	   ((BYTE*)(&f))[2] = pBytes[1];
//	   ((BYTE*)(&f))[3] = pBytes[0];
//	}else{
//	   ((BYTE*)(&f))[0] = pBytes[0];
//	   ((BYTE*)(&f))[1] = pBytes[1];
//	   ((BYTE*)(&f))[2] = pBytes[2];
//	   ((BYTE*)(&f))[3] = pBytes[3];
//	}
//	
//	return f; 
//}



// ******************************************************************************************************************
// *
// *           LoadFpgaVariablesDependingOnHwFwVersion()
// *
// *   Loads up the globals for used FPGA functions (depends on hw and fw version) 
// *       Note that there are:
// *         X card types
// *         Y hw versions
// *         Z fw versions
// *       ...and sometimes a function is changed so we then need to define different behaviour
// *
// *
// *  Input:
// *     -
// *
// *  Output:
// *     -
// *
// ******************************************************************************************************************
void LoadFpgaVariablesDependingOnHwFwVersion( void )
{
   
   // Start by writing default values to all globals...
   // All variables arer set to 0x00 at start-up.
   
   // Support for FPGA_WRITE_REG_EXTOUT0_SETUP (0 to 7) differs depending on hw/fw ver!
   if(  (CardType == OPTION_BOARD_CARD_TYPE && HardwareRevision >= 2) ||
             (CardType == POWER_BOARD_CARD_TYPE)   )
   {
      // New HW/FW version 2011-10-27 on OptionBoard
      // New board type PowerBoard 2014-03-01 also uses same EXTOUT as OptionBoard new version

      // -------------------------------------            
      // ------ FPGA WRITE REGISTERS ---------
      // -------------------------------------            
      // 
      // EXTOUT PWM.
      FpgaWriteRegExtOut0PwmHi = FPGA_WRITE_REG_EXTOUT0_VOLT+1;   // Earlier FPGA_WRITE_REG_EXTOUT0_PWM_HI     (depends on fw rev)
      FpgaWriteRegExtOut0PwmLo = FPGA_WRITE_REG_EXTOUT0_VOLT+2;   // Earlier FPGA_WRITE_REG_EXTOUT0_PWM_LO     (depends on fw rev)
      FpgaWriteRegExtOut1PwmHi = FPGA_WRITE_REG_EXTOUT1_VOLT+1;   // ...
      FpgaWriteRegExtOut1PwmLo = FPGA_WRITE_REG_EXTOUT1_VOLT+2;
      FpgaWriteRegExtOut2PwmHi = FPGA_WRITE_REG_EXTOUT2_VOLT+1;
      FpgaWriteRegExtOut2PwmLo = FPGA_WRITE_REG_EXTOUT2_VOLT+2;
      FpgaWriteRegExtOut3PwmHi = FPGA_WRITE_REG_EXTOUT3_VOLT+1;
      FpgaWriteRegExtOut3PwmLo = FPGA_WRITE_REG_EXTOUT3_VOLT+2;
      
      FpgaWriteRegExtOut4PwmHi = FPGA_WRITE_REG_EXTOUT4_VOLT+1;
      FpgaWriteRegExtOut4PwmLo = FPGA_WRITE_REG_EXTOUT4_VOLT+2;
      FpgaWriteRegExtOut5PwmHi = FPGA_WRITE_REG_EXTOUT5_VOLT+1;
      FpgaWriteRegExtOut5PwmLo = FPGA_WRITE_REG_EXTOUT5_VOLT+2;
      FpgaWriteRegExtOut6PwmHi = FPGA_WRITE_REG_EXTOUT6_VOLT+1;
      FpgaWriteRegExtOut6PwmLo = FPGA_WRITE_REG_EXTOUT6_VOLT+2;
      FpgaWriteRegExtOut7PwmHi = FPGA_WRITE_REG_EXTOUT7_VOLT+1;
      FpgaWriteRegExtOut7PwmLo = FPGA_WRITE_REG_EXTOUT7_VOLT+2;

      // EXTOUT REGISTER 3 [FPGA_WRITE_REG_EXTOUTX_SETUP] bit definitions
      FpgaWriteExtOutReg3OutPutControl     = 0x0001;   // Bit  0 - OutPutControl (affects PWM as well in new fw)
      FpgaWriteExtOutReg3PolarityControl   = 0x0002;   // Bit  1 - PolarityControl (affects PWM as well in new fw)
      FpgaWriteExtOutReg3PwmEnable1        = 0x0004;   // Bit  2 - PwmEnable1 (same as old fw)
      FpgaWriteExtOutReg3PwmEnable2        = 0x0008;   // Bit  3 - PwmEnable1 (same as old fw)
      FpgaWriteExtOutReg3PwmMode           = 0x0010;   // Bit  4 - PwmEnable1 (same as old fw)
      FpgaWriteExtOutReg3SingelCont        = 0x0020;   // Bit  5 - SingelShot/Continuous
      FpgaWriteExtOutReg3HiLevelTimeBase1  = 0x0040;   // Bit  6 - HiLevelTimeBase
      FpgaWriteExtOutReg3HiLevelTimeBase2  = 0x0080;   // Bit  7 - HiLevelTimeBase
      FpgaWriteExtOutReg3HiLevelTimeBase3  = 0x0100;   // Bit  8 - HiLevelTimeBase
      FpgaWriteExtOutReg3LoLevelTimeBase1  = 0x0200;   // Bit  9 - LoLevelTimeBase
      FpgaWriteExtOutReg3LoLevelTimeBase2  = 0x0400;   // Bit 10 - LoLevelTimeBase
      FpgaWriteExtOutReg3LoLevelTimeBase3  = 0x0800;   // Bit 11 - LoLevelTimeBase

      
      // ------------------------------------            
      // ------ FPGA READ REGISTERS ---------
      // ------------------------------------            
      //
      // --- EXTOUT VOLTAGE 0-3 ---
      FpgaReadRegExtOut0Volt     = 0x40;  // Earlier FPGA_READ_REG_EXTOUT0_VOLT            (depends on fw rev)
      //FPGA_READ_REG_           ( 0x41 ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x42 ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x43 ) // SEE DEFINITION IN CONSTANTS.H
      FpgaReadRegExtOut1Volt     = 0x44;  // Earlier FPGA_READ_REG_EXTOUT1_VOLT
      //FPGA_READ_REG_           ( 0x45 ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x46 ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x47 ) // SEE DEFINITION IN CONSTANTS.H
      FpgaReadRegExtOut2Volt     = 0x48;  // Earlier FPGA_READ_REG_EXTOUT2_VOLT
      //FPGA_READ_REG_           ( 0x49 ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x4A ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x4B ) // SEE DEFINITION IN CONSTANTS.H
      FpgaReadRegExtOut3Volt     = 0x4C;  // Earlier FPGA_READ_REG_EXTOUT3_VOLT
      //FPGA_READ_REG_           ( 0x4D ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x4E ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x4F ) // SEE DEFINITION IN CONSTANTS.H

               
      // --- EXTOUT VOLTAGE 4-7 ---
      FpgaReadRegExtOut4Volt     = 0x80;  // Earlier FPGA_READ_REG_EXTOUT4_VOLT
      //FPGA_READ_REG_           ( 0x81 ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x82 ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x83 ) // SEE DEFINITION IN CONSTANTS.H
      FpgaReadRegExtOut5Volt     = 0x84;  // Earlier FPGA_READ_REG_EXTOUT5_VOLT
      //FPGA_READ_REG_           ( 0x85 ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x86 ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x87 ) // SEE DEFINITION IN CONSTANTS.H
      FpgaReadRegExtOut6Volt     = 0x88;  // Earlier FPGA_READ_REG_EXTOUT6_VOLT
      //FPGA_READ_REG_           ( 0x89 ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x8A ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x8B ) // SEE DEFINITION IN CONSTANTS.H
      FpgaReadRegExtOut7Volt     = 0x8C;  // Earlier FPGA_READ_REG_EXTOUT7_VOLT
      //FPGA_READ_REG_           ( 0x8D ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x8E ) // SEE DEFINITION IN CONSTANTS.H
      //FPGA_READ_REG_           ( 0x8F ) // SEE DEFINITION IN CONSTANTS.H
   } // end of "if( CardType == OPTION_BOARD_CARD_TYPE...
   else if( CardType == OPTION_BOARD_CARD_TYPE &&
       HardwareRevision < 2 )
   {
      // OptionBoard, HardwareRevision <= 1
      // OLD HARDWARE and FIRMWARE version on OPTIONBOARD
      // Very few out there, maybe only Sematek has this version

      // -------------------------------------            
      // ------ FPGA WRITE REGISTERS ---------
      // -------------------------------------            
      // 
      // EXTOUT PWM.
      FpgaWriteRegExtOut0PwmLo = FPGA_WRITE_REG_EXTOUT0_VOLT+1;   // Earlier FPGA_WRITE_REG_EXTOUT0_PWM_LO
      FpgaWriteRegExtOut0PwmHi = FPGA_WRITE_REG_EXTOUT0_VOLT+2;   // Earlier FPGA_WRITE_REG_EXTOUT0_PWM_HI
      FpgaWriteRegExtOut1PwmLo = FPGA_WRITE_REG_EXTOUT1_VOLT+1;   // ...
      FpgaWriteRegExtOut1PwmHi = FPGA_WRITE_REG_EXTOUT1_VOLT+2;
      FpgaWriteRegExtOut2PwmLo = FPGA_WRITE_REG_EXTOUT2_VOLT+1;
      FpgaWriteRegExtOut2PwmHi = FPGA_WRITE_REG_EXTOUT2_VOLT+2;
      FpgaWriteRegExtOut3PwmLo = FPGA_WRITE_REG_EXTOUT3_VOLT+1;
      FpgaWriteRegExtOut3PwmHi = FPGA_WRITE_REG_EXTOUT3_VOLT+2;
      
      //   FpgaWriteRegExtOut4PwmLo = 0x00; // Not supported
      //   FpgaWriteRegExtOut4PwmHi = 0x00; // Not supported
      //   FpgaWriteRegExtOut5PwmLo = 0x00; // Not supported
      //   FpgaWriteRegExtOut5PwmHi = 0x00; // Not supported
      //   FpgaWriteRegExtOut6PwmLo = 0x00; // Not supported
      //   FpgaWriteRegExtOut6PwmHi = 0x00; // Not supported
      //   FpgaWriteRegExtOut7PwmLo = 0x00; // Not supported
      //   FpgaWriteRegExtOut7PwmHi = 0x00; // Not supported
      

      // REGISTER 3 [FPGA_WRITE_REG_EXTOUTX_SETUP] bit definitions
      FpgaWriteExtOutReg3OutPutControl     = 0x0001;   // Bit 0 - OutPutControl (Does NOT affect PWM in old fw)
      FpgaWriteExtOutReg3PolarityControl   = 0x0002;   // Bit 1 - PolarityControl (Does NOT affect PWM in old fw)
      FpgaWriteExtOutReg3PwmEnable1        = 0x0004;   // Bit 2 - PwmEnable1 (same as old fw)
      FpgaWriteExtOutReg3PwmEnable2        = 0x0008;   // Bit 3 - PwmEnable1 (same as old fw)
      FpgaWriteExtOutReg3PwmMode           = 0x0010;   // Bit 4 - PwmEnable1 (same as old fw)
      FpgaWriteExtOutReg3SingelCont        = 0x0000;  // No support for SingelShot/Continuous
      FpgaWriteExtOutReg3LoLevelTimeBase1  = 0x0020;   // Bit  5 - LoLevelTimeBase
      FpgaWriteExtOutReg3LoLevelTimeBase2  = 0x0040;   // Bit  6 - LoLevelTimeBase
      FpgaWriteExtOutReg3LoLevelTimeBase3  = 0x0080;   // Bit  7 - LoLevelTimeBase
      FpgaWriteExtOutReg3HiLevelTimeBase1  = 0x0100;   // Bit  8 - HiLevelTimeBase
      FpgaWriteExtOutReg3HiLevelTimeBase2  = 0x0200;   // Bit  9 - HiLevelTimeBase
      FpgaWriteExtOutReg3HiLevelTimeBase3  = 0x0400;   // Bit 10 - HiLevelTimeBase
      

      // ------------------------------------            
      // ------ FPGA READ REGISTERS ---------
      // ------------------------------------            
      //   EXTOUT VOLTAGE
      //   FpgaReadRegExtOut0Volt      = 0x00; // Not supported
      //   FpgaReadRegExtOut1Volt      = 0x00; // Not supported
      //   FpgaReadRegExtOut2Volt      = 0x00; // Not supported
      //   FpgaReadRegExtOut3Volt      = 0x00; // Not supported
      //   FpgaReadRegExtOut4Volt      = 0x00; // Not supported
      //   FpgaReadRegExtOut5Volt      = 0x00; // Not supported
      //   FpgaReadRegExtOut6Volt      = 0x00; // Not supported
      //   FpgaReadRegExtOut7Volt      = 0x00; // Not supported
      //   
      //   PWM.
      //   FpgaReadRegExtOut0PwmStatus = 0x00; // Not supported
      //   FpgaReadRegExtOut1PwmStatus = 0x00; // Not supported
      //   FpgaReadRegExtOut2PwmStatus = 0x00; // Not supported
      //   FpgaReadRegExtOut3PwmStatus = 0x00; // Not supported
      //   FpgaReadRegExtOut4PwmStatus = 0x00; // Not supported
      //   FpgaReadRegExtOut5PwmStatus = 0x00; // Not supported
      //   FpgaReadRegExtOut6PwmStatus = 0x00; // Not supported
      //   FpgaReadRegExtOut7PwmStatus = 0x00; // Not supported
   }
}


// ******************************************************************************************************************
// *
// *           CheckIfThisBoardSupportsThisFwFunction()
// *
// *  Check if this board supports the function or not.
// *       Note that there are:
// *         X card types
// *         Y hw versions
// *         Z fw versions
// *       ...and sometimes a function is changed so we then need to define different behaviour
// *
// *
// *  Input:
// *     UINT16                        functionToCheck              The FPGA function to check
// *     eFunctionToCheckSeverityType  terminateIfNotSupported      FATAL   = Stop execusion of the program here.
// *                                                                WARNING = Bad enough to create warning message.
// *                                                                NONE    = No warning
// *
// *  Output:
// *     bool                                TRUE  if the board supports this function
// *                                         FALSE if no support
// *
// ******************************************************************************************************************
bool CheckIfThisBoardSupportsThisFwFunction( eFunctionToCheckType functionToCheck, eFunctionToCheckSeverityType severity )
{
eCommandsAdc  AdcCommand;

   switch( functionToCheck )
   {
      case EXTOUT4_7:
         // See doc. FSP-30-10002-01 "EXTOUT"
         if( CardType == OPTION_BOARD_CARD_TYPE )
         {
            if( HardwareRevision > 1 )
            {
               return ( TRUE );
               // Support for FPGA_WRITE_REG_EXTOUT4_VOLT and FPGA_WRITE_REG_EXTOUT4_SETUP (4 to 7) exists for all fw ver!
               //if( FirmwareRevision > XXX )
               //{
               //}
            }
         }
         else if( CardType == POWER_BOARD_CARD_TYPE )
         {
            // Available in all POWER_BOARD_CARD_TYPE
            return ( TRUE ); 
         }
         // No support for EXTOUT4 to 7!
         break;                


      case WRITE_REG_EXTOUT0_7_SETUP_VOLT_PWM:
        // See doc. FSP-30-10002-01 "EXTOUT"
         if( CardType == OPTION_BOARD_CARD_TYPE )
         {
            // Available in all OPTION_BOARD_CARD_TYPE
            // also see "case EXTOUT4_7" since old hw onlty supports EXTOUT0 to 3
            //
            // Support for FPGA_WRITE_REG_EXTOUT0_SETUP differs depending on fw ver!
            // Support for FPGA_WRITE_REG_EXTOUT0_VOLT  differs depending on fw ver!
            // See LoadFpgaVariablesDependingOnHwFwVersion() for setting up globals
            //
            return ( TRUE ); 
         }
         else if( CardType == POWER_BOARD_CARD_TYPE )
         {
            // Available in all POWER_BOARD_CARD_TYPE
            return ( TRUE ); 
         }
         
         // No support for EXTOUT0 to 7 in other boards!
         break;                


      default:
         ErrorReporting( FPGASupportNonExistingFunctionToCheck );
         #if defined(DEBUG_TXT_SHOW) || defined(CJN_MAIN_TESTS)
            dbgprintf( "Fatal ERROR: Non supported functionToCheck calling\r\n CheckIfThisBoardSupportsThisFwFunction()!!" );
         #endif //DEBUG_TXT_SHOW      
         return ( FALSE );
   } // end of switch()
   
   
   if( severity == WARNING )
   {
      // This is bad enough to create warning message.
      ErrorReporting( FPGASupportNonSupportingFunctionWarning );
      #if defined(DEBUG_TXT_SHOW) || defined(CJN_MAIN_TESTS)
         dbgprintf( "Trying to use not supported FPGA function!!" );
      #endif //DEBUG_TXT_SHOW
   }
   else if( severity == FATAL )
   {
      // This is bad enough to stop execution
      ErrorReporting( FPGASupportNonSupportingFunctionFatal );
      #if defined(DEBUG_TXT_SHOW) || defined(CJN_MAIN_TESTS)
         dbgprintf( "Trying to use not supported FPGA function!!!" );
      #endif //DEBUG_TXT_SHOW
         
      // STOP PROGRAM EXECUTION!   
      AdcCommand = StopProgramExecutionCommand;
      xQueueSendToFront( xAdcQueue, &AdcCommand, portNO_DELAY );  // send a shutdown message to the ADC task.
      // Do not set the global flag for under voltage detection so we don't restart again!
      // UnderVoltageDetectionFlag = TRUE;
   }
   
   return ( FALSE );
}


// ******************************************************************************************************************
// *
// *           addCheckSumNMEAtype()
// *
// *  Calculate checksum to the NMEA string and add it last in the string.
// *  "The checksum field consists of a "*" and two hex digits representing the
// *   exclusive OR of all characters between, but not including, the "$" and "*"."
// *
// *  Input:
// *     UINT16     UBYTE *stringPointer
// *
// *  Output:
// *     -
// *
// ******************************************************************************************************************
void addCheckSumNMEAtype( UBYTE *stringPointer, UBYTE maxNumber )
{
UBYTE checksum = 0;   
UBYTE *localStringPointer = stringPointer;
UBYTE counter = 0;
   
   while( *localStringPointer != '\x00' && counter<maxNumber )
   {
      if( *localStringPointer == '\x24' ) // If '$' start over!
      {
			checksum = 0;
      }
      else
      {
         checksum = checksum ^ *localStringPointer;
      }
      localStringPointer++;
      counter++;
   }
   sprintf( localStringPointer, "*%.2X", checksum );
}





//************************************************************************************
//
//                  SendTextToSmallPcuOverlayAndRovAnalyser()
//
//   Present text on PCU, overlay and ROV analyser program
//
//   Input:         UBYTE     *stringPointerUpperLine  
//                  UBYTE     *stringPointerLowLine 
//                            Max length of strings = MAX_UDP_TX_MESSAGE_DATA_SIZE-3
//
//   Output:        
//
//   Sets:          PCU text, Overlay text ROV Analyser text
//
//************************************************************************************
void SendTextToSmallPcuOverlayAndRovAnalyser( UBYTE *stringPointerUpperLine, UBYTE *stringPointerLowLine )
{
   WriteTextToPcuDisplay( stringPointerUpperLine, stringPointerLowLine );
   WriteTextToOverlay( stringPointerUpperLine, stringPointerLowLine );
   WriteTextToRovAnalyser( stringPointerUpperLine, 9 );
   WriteTextToRovAnalyser( stringPointerLowLine, 9 );
}




//************************************************************************************
//
//                  SendTextToSmallPcuAndOverlay()
//
//   Present text on PCU, overlay and ROV analyser program
//
//   Input:         UBYTE     *stringPointerUpperLine  
//                  UBYTE     *stringPointerLowLine 
//                            Total length of both strings = MAX_UDP_TX_MESSAGE_DATA_SIZE-3
//
//   Output:        
//
//   Sets:          PCU text, Overlay text ROV Analyser text
//
//************************************************************************************
void SendTextToSmallPcuAndOverlay( UBYTE *stringPointerUpperLine, UBYTE *stringPointerLowLine )
{
   WriteTextToPcuDisplay( stringPointerUpperLine, stringPointerLowLine );
   WriteTextToOverlay( stringPointerUpperLine, stringPointerLowLine );
}



//************************************************************************************
//
//                  WriteTextToRovAnalyser()
//
//   Present text on Windows program
//
//   Input:         UBYTE     *stringPointer
//                  UINT16    length of string
//                            Total length = MAX_UDP_TX_MESSAGE_DATA_SIZE-3
//
//   Output:        
//
//   Sets:          ROV Analyser text
//
//************************************************************************************
void WriteTextToRovAnalyser( UBYTE *stringPointer, UINT16 length )
{
UDPMsgStructType  udpMessageToTransmit;

   // -------------------------------------               
   // Send error message to Windows program
   // -------------------------------------               
   // No Msg before ethernet is up and running!
   if( IsEthernetUpAndRunning() == TRUE )
   {
      // Send message to Windows program at surface PC.
      udpMessageToTransmit.cCommand = SendTextMsg;
      udpMessageToTransmit.length   = length+1; // CardAddress=1 + length
      udpMessageToTransmit.data[0]  = CardAddress;
      
      memcpy( &udpMessageToTransmit.data[1], stringPointer, MAX_UDP_TX_MESSAGE_DATA_SIZE-1 );

      if( xQueueSendToBack( xSerialUDPTxTaskQueue, &udpMessageToTransmit, udpMessageDelay ) != pdPASS )
      {
         // Set the flag indicating problem.
         ErrorReporting( SerialUDPTxSendTextMsgCommand ); // Log error code
      }
   }
}




//************************************************************************************
//
//                  SendTextToRovAnalyserUsingPtr()
//
//   Present text on Windows program
//
//   Input:         UBYTE     *stringPointer
//                            Total length = MAX_UDP_TX_MESSAGE_DATA_SIZE-3
//
//   Output:        
//
//   Sets:          ROV Analyser text
//
//************************************************************************************
void SendTextToRovAnalyserUsingPtr( UBYTE *stringPointer )
{
//UDPMsgStructTypePtr  udpMessageToTransmit;

   // -------------------------------------               
   // Send error message to Windows program
   // -------------------------------------               
   // No Msg before ethernet is up and running!
   if( IsEthernetUpAndRunning() == TRUE )
   {
      // NOTE THAT WE MUST MAKE TYPECAST FROM UDPMsgStructType to UDPMsgStructTypePtr
#warning "SendTextToRovAnalyserUsingPtr() must make typecast when using it"      
      
      //      // Send message to Windows program at surface PC.
      //      udpMessageToTransmit.cCommand      = SendTextPtrMsg;
      //      udpMessageToTransmit.length        = 1+strlen(stringPointer); // CardAddress + string
      //      udpMessageToTransmit.cardAddress   = CardAddress;
      //      udpMessageToTransmit.stringPointer = stringPointer;
      //         
      //      if( xQueueSendToBack( xSerialUDPTxTaskQueue, &udpMessageToTransmit, portNO_DELAY ) != pdPASS )
      //      {
      //         // Set the flag indicating problem.
      //         ErrorReporting( SerialUDPTxSendAliveMsgCommand ); // Log error code
      //      }
   }
}


//************************************************************************************
//
//                  WriteTextImPcuDisplay()
//
//   Present text on PCU
//
//   Input:         UBYTE     *stringPointerUpperLine
//                  UBYTE     *stringPointerLowLine 
//   Output:        
//
//   Sets:          PCU text
//
//************************************************************************************
void WriteTextToPcuDisplay( UBYTE *stringPointerUpperLine, UBYTE *stringPointerLowLine )
{
CanTxdIntMsgStructType thisCanMsg;
UBYTE                  *displayTextPointer;

      
   if( *stringPointerUpperLine != '\0' )
   {
      displayTextPointer    = stringPointerUpperLine;
      thisCanMsg.cCommand   = DataHandlerCANTxdCommand;
      thisCanMsg.canMsg.RID = MSG_UPPER_ROW_TXT; // ID (see UP0001-407)
      thisCanMsg.canMsg.RDL = 8;                 // length (number of bytes)
      thisCanMsg.canMsg.RMH = *displayTextPointer<<24     | *(displayTextPointer+1)<<16 | *(displayTextPointer+2)<<8 | *(displayTextPointer+3); 
      thisCanMsg.canMsg.RML = *(displayTextPointer+4)<<24 | *(displayTextPointer+5)<<16 | *(displayTextPointer+6)<<8 | *(displayTextPointer+7); 
      SendCanMsgViaDataHandler(thisCanMsg, FALSE);    // Let DataHandler send this CAN message to interrupt routine via xCANTxdQueue
   }
   
   
   if( *stringPointerLowLine != '\0' )
   {
      displayTextPointer    = stringPointerLowLine;
      thisCanMsg.cCommand   = DataHandlerCANTxdCommand;
      thisCanMsg.canMsg.RID = MSG_LOWER_ROW_TXT; // ID (see UP0001-407)
      thisCanMsg.canMsg.RDL = 8;                 // length (number of bytes)
      thisCanMsg.canMsg.RMH = *displayTextPointer<<24     | *(displayTextPointer+1)<<16 | *(displayTextPointer+2)<<8 | *(displayTextPointer+3); 
      thisCanMsg.canMsg.RML = *(displayTextPointer+4)<<24 | *(displayTextPointer+5)<<16 | *(displayTextPointer+6)<<8 | *(displayTextPointer+7); 
      SendCanMsgViaDataHandler(thisCanMsg, FALSE);    // Let DataHandler send this CAN message to interrupt routine via xCANTxdQueue
   }
}



//************************************************************************************
//
//                  WriteTextToOverlay()
//
//   Present text on Overlay
//
//   Input:         UBYTE     *stringPointerUpperLine
//                  UBYTE     *stringPointerLowLine 
//   Output:        
//
//   Sets:          Overlay text  (shows text for a while)
//
//************************************************************************************
void WriteTextToOverlay( UBYTE *stringPointerUpperLine, UBYTE *stringPointerLowLine )
{
CanTxdIntMsgStructType thisCanMsg;
UBYTE                  *displayTextPointer;

   if( TextToOverlayAllowed == TRUE )
   {
      thisCanMsg.cCommand   = DataHandlerCANTxdCommand;
      thisCanMsg.canMsg.RID = MSG_TXT_TO_OVERLAY;  // ID (see UP0001-407)
      thisCanMsg.canMsg.RDL = 2;                   // length (number of bytes)
      thisCanMsg.canMsg.RMH = 0x40404040;          // shall be "@@", so we fill all 8 data with '@' (0x40)...
      thisCanMsg.canMsg.RML = 0x40404040;
      SendCanMsgViaDataHandler(thisCanMsg, FALSE); // Let DataHandler send this CAN message to interrupt routine via xCANTxdQueue
   
      
      displayTextPointer    = stringPointerUpperLine;
      thisCanMsg.cCommand   = DataHandlerCANTxdCommand;
      thisCanMsg.canMsg.RID = MSG_TXT_TO_OVERLAY; // ID (see UP0001-407)
      thisCanMsg.canMsg.RDL = 8;                  // length (number of bytes)
      if( *stringPointerUpperLine != '\0' )
      {
         //thisCanMsg.canMsg.RMH = *(displayTextPointer+3) | *(displayTextPointer+2)<<8 | *(displayTextPointer+1)<<16 | *displayTextPointer<<24; 
         //thisCanMsg.canMsg.RML = *(displayTextPointer+7) | *(displayTextPointer+6)<<8 | *(displayTextPointer+5)<<16 | *(displayTextPointer+4)<<24; 
         
         thisCanMsg.canMsg.RML = *(displayTextPointer+0) | *(displayTextPointer+1)<<8 | *(displayTextPointer+2)<<16 | *(displayTextPointer+3)<<24; 
         thisCanMsg.canMsg.RMH = *(displayTextPointer+4) | *(displayTextPointer+5)<<8 | *(displayTextPointer+6)<<16 | *(displayTextPointer+7)<<24; 
      }
      else
      {
         thisCanMsg.canMsg.RMH = 0x20202020; // write space if no data. 
         thisCanMsg.canMsg.RML = 0x20202020; 
      }
      SendCanMsgViaDataHandler(thisCanMsg, FALSE);    // Let DataHandler send this CAN message to interrupt routine via xCANTxdQueue
      
      
   
   
      displayTextPointer    = stringPointerLowLine;
      thisCanMsg.cCommand   = DataHandlerCANTxdCommand;
      thisCanMsg.canMsg.RID = MSG_TXT_TO_OVERLAY; // ID (see UP0001-407)
      thisCanMsg.canMsg.RDL = 8;                  // length (number of bytes)
      if( *stringPointerLowLine != '\0' )
      {
         //thisCanMsg.canMsg.RMH = *(displayTextPointer+3) | *(displayTextPointer+2)<<8 | *(displayTextPointer+1)<<16 | *displayTextPointer<<24; 
         //thisCanMsg.canMsg.RML = *(displayTextPointer+7) | *(displayTextPointer+6)<<8 | *(displayTextPointer+5)<<16 | *(displayTextPointer+4)<<24; 
   
         thisCanMsg.canMsg.RML = *(displayTextPointer+0) | *(displayTextPointer+1)<<8 | *(displayTextPointer+2)<<16 | *(displayTextPointer+3)<<24; 
         thisCanMsg.canMsg.RMH = *(displayTextPointer+4) | *(displayTextPointer+5)<<8 | *(displayTextPointer+6)<<16 | *(displayTextPointer+7)<<24; 
      }
      else
      {
         thisCanMsg.canMsg.RMH = 0x20202020; // write space if no data. 
         thisCanMsg.canMsg.RML = 0x20202020; 
      }
      SendCanMsgViaDataHandler(thisCanMsg, FALSE);    // Let DataHandler send this CAN message to interrupt routine via xCANTxdQueue
   }
}


//************************************************************************************
//
//                  SetTimeFromStr
//
//   This function sets time from a supplied string
//   The string should be on the format yy-mm-dd hh:mm:ss
//
//   Input:          -
//
//   Output:        TRUE if OK / FALSE if BAD failed
//
//   Sets:          -
//
//************************************************************************************
bool SetTimeFromStr( UBYTE *timeStr )
{
SoftwareRTCClockStructType clockStruct;
int                        tmp[6];

   sscanf(timeStr, "%d-%d-%d %d:%d:%d", &tmp[0], &tmp[1], &tmp[2], &tmp[3], &tmp[4], &tmp[5] );

   clockStruct.year   = (UBYTE)tmp[0];
   clockStruct.month  = (UBYTE)tmp[1];
   clockStruct.date   = (UBYTE)tmp[2];
   clockStruct.hour   = (UBYTE)tmp[3];
   clockStruct.minute = (UBYTE)tmp[4];
   clockStruct.second = (UBYTE)tmp[5];
   clockStruct.RTCUpdateTick = xTaskGetTickCount();


//#ifdef DEBUG_INCLUDED
//   dbgprintf( "testing timestamp: Y=%d, M=%d, D=%d, h=%d, m=%d, s=%d\r\n", clockStruct.year,
//                                                                           clockStruct.month,
//                                                                           clockStruct.date,
//                                                                           clockStruct.hour,
//                                                                           clockStruct.minute,
//                                                                           clockStruct.second );
//#endif //DEBUG_INCLUDED

   
   
   
   if ( timestampSanityCheck( &clockStruct ) != OK )
   {
#ifdef DEBUG_INCLUDED
      dbgprintf( "Invalid timestamp: Y=%d, M=%d, D=%d, h=%d, m=%d, s=%d\r\n", clockStruct.year,
                                                                              clockStruct.month,
                                                                              clockStruct.date,
                                                                              clockStruct.hour,
                                                                              clockStruct.minute,
                                                                              clockStruct.second );
#endif //DEBUG_INCLUDED
      return( FALSE);
   }
   else
   {
      if ( SetTime(&clockStruct) != OK )
      {
         #ifdef DEBUG_INCLUDED
         dbgprintf( "Error in SetTime() function using timestamp: Y=%d, M=%d, D=%d, h=%d, m=%d, s=%d\r\n", clockStruct.year,
                                                                                                           clockStruct.month,
                                                                                                           clockStruct.date,
                                                                                                           clockStruct.hour,
                                                                                                           clockStruct.minute,
                                                                                                           clockStruct.second );
         #endif //DEBUG_INCLUDED
         return( FALSE);
      }
   }
   return( TRUE );
}


//************************************************************************************
//
//                  Convert12BitSignedTo16BitSigned
//
//   This function converts 12 bit signed value in UINT16 to 16 bit signed.
//
//   Input:         UINT16    Signed12bBitValue
//
//   Output:        SINT16    Signed16BitValue
//
//   Sets:          -
//
//************************************************************************************
SINT16 Convert12BitSignedTo16BitSigned( UINT16 Signed12bBitValue )
{
SINT16 LocalSigned16BitValue;

   // Check MSB to see if it is negative
   if((Signed12bBitValue & 0x0800) == 0x0800 )
   {
      // Negative!
      // 2-complement number should have high bits set as well.
      LocalSigned16BitValue = Signed12bBitValue | 0xF000;
   }
   else
   {
      // Positive!
      // 2-complement number should have high bits zero.
      LocalSigned16BitValue = Signed12bBitValue & 0x0FFF;
   }
   return( LocalSigned16BitValue );
}


//************************************************************************************
//
//                  ConvertDepthToMeters
//
//   This function converts data read from depthSensor as current 4-20mA to meters
//
//    D = Depth
//    I = measured current
//    Vad = Voltage mesured in A/D.
//
//         I-4
//    D = ----- x 3000 [m]
//         16
//
//
//          Vad
//    I = --------- x 20.48 [mA]  when using 16-bit value
//         2^16 -1
//
//          Vad
//    I = --------- x 20.48 [mA]  when using 24-bit value
//         2^24 -1
//
//   Input:          UINT32   24 bit A/D value from depthsensor via FPGA 
//
//   Output:         float    Depth in meters
//
//   Sets:           -
//
//************************************************************************************
float ConvertDepthToMeters( UINT32 depthValue )
{
float depth_f;   

   depth_f = depthValue * 1.22070321e-6;  // This is the current I [ma] (4-20) using 24-bit value

#ifdef DEPTH_MEAS_500M   
   depth_f = (depth_f-4)*31.25;           // This depth in meters (0-500)
#elif defined(DEPTH_MEAS_3000M)   
   depth_f = (depth_f-4)*187.5;           // This depth in meters (0-3000)
#else
   #warning">>>> THIS IS NOT SUPPORTED DEPTH MEASUREMENT !!!!!!!!!!!! <<<<<"
#endif    
   
   return( depth_f );
}


//************************************************************************************
//
//                  ReadCpCalibrationValueFromEEPROM
//
//   Read 16 bits of EEPROM data for DP Calibration @0xE0+E1 
//   (FPGA_READ_REG_EEPROM_00 + FPGA_READ_REG_EEPROM_01)
//                   
//
//   Input:          -
//
//   Output:         SINT16 stored CalibrationValue
//
//   Sets:           -
//
//************************************************************************************
SINT16 ReadCpCalibrationValueFromEEPROM( void )
{
extern UINT16 ReadEEPROM( UBYTE whichdataAddress );
UINT16  UINT16CompensationValue;
UINT16  UINT16temporary;

   UINT16CompensationValue = ReadEEPROM( 0x01 );         // Hi byte
   UINT16CompensationValue = UINT16CompensationValue << 8;
   UINT16temporary = ReadEEPROM( 0x00 );                 // Lo byte
   UINT16CompensationValue += (UINT16temporary & 0xFF);
   
   return( (SINT16)UINT16CompensationValue );
}



//************************************************************************************
//
//                  ReadAnalogueCpProbeValue
//
//   Read 24-bit value from Analogue CP Probe connected to EXTIN2 using calibration
//                   
//
//   Input:          bool useCalibration     TRUE  = Use calibration value
//                                           FALSE = Do NOT use calibration value
//
//   Output:         SINT32 stored CalibrationValue
//
//   Sets:           -
//
//************************************************************************************
SINT32 ReadAnalogueCpProbeValue( bool useCalibration )
{
extern UINT32 FpgaRead24bits( UINT16 fpgaAddress, bool enableInt );
UINT32  UINT32temporary;
SINT32  SINT32temporary;
SINT64  SINT64temporary;

   UINT32temporary = FpgaRead24bits( FPGA_READ_REG_EXTIN2_HI_VALUE | GlobalThisCardsFpgaBaseAddress, TRUE ); 
   SINT64temporary = UINT32temporary+CpCalibrationValue;
   SINT64temporary = 6138*SINT64temporary;         // 100 TIMES TOO HIGH VALUE
   SINT64temporary = SINT64temporary/1677722;      // 1000 TIMES TOO HIGH VALUE [mV] (2^24 = 16777216)
   SINT64temporary -= 30690;                       // 30.69V -> [mV]
   SINT32temporary = (SINT32)SINT64temporary;      // max +/- 30690 mV.
   return( SINT32temporary );
}

#ifdef ALTIMETER_USED
//************************************************************************************
//
//                  AltitudeFromUart
//
//   Read the next Altitude value from the altimeter RS232 serial data
//   
//                   
//
//   Input:          Uart buffer nullterminated
//
//   Output:         Altitude in mm, zero indicates an error
//
//   Sets:           -
//
//************************************************************************************
UINT16 AltitudeFromUart( UBYTE* uartBuff )
{
UBYTE altitudeString[7];
char *uartRemainder;
//dbgprintf("uartbuff = %s \n", uartBuff);
//   int i = 0;
//   while (uartBuff[i] != '\0')
//   {
//     if (uartBuff[i] == '\r' && uartBuff[i+1] == '\n')
//            dbgprintf("Heureka\n");
//     i++;
//   }
   uartRemainder = strchr(uartBuff,'\n');
//    dbgprintf("Remainder = %s\n", uartRemainder);   
   if (uartRemainder)
   {
      altitudeString[0] =  uartRemainder[1];
      altitudeString[1] =  uartRemainder[2];
      altitudeString[2] =  uartRemainder[3];
      altitudeString[3] =  uartRemainder[5];
      altitudeString[4] =  uartRemainder[6];
      altitudeString[5] =  uartRemainder[7];
      altitudeString[6] = '\0';      
//     strncpy(altitudeString , uartRemainder + 1, 3);
//     strncpy(altitudeString + 3, uartRemainder + 5, 3);
//     altitudeString[6] = '\0';   
//     dbgprintf("AltFromUart = %s\n", altitudeString);
//     dbgprintf("Remainder = %s\n", uartRemainder);
   }
   else
     altitudeString[0] = '\0';
   
   return( atoi(altitudeString) );
}

//
//bool isAltitudeToHigh(UINT16 altimeterValue)
//{
//  static UBYTE altNoEchoes = 0; 
//
//  // Check if Altimeter is too high above the seabed  
//  if (altimeterValue == ALTIMETER_NO_ECHO)
//  {
//    altNoEchoes++;
//    
//    if (altNoEchoes >= MAX_NO_ECHOES)
//    {
//      LeaveAltimeterCruising();
//      altNoEchoes = 0;
//    }
//    
//    return TRUE; 
//  }
//
//  return FALSE;
//} 


SINT32 CalcAverage(SINT32* altimeterValues, UBYTE noValidValues)
{
  UBYTE i = 0;
  UINT32 theSum = 0;
  
  if (noValidValues) 
  {
    do
    {    
      theSum+= altimeterValues[i];
    }  while (++i < noValidValues);

//    dbgprintf("Kilroy was here 5, theSum = %d ", theSum);
    return theSum/noValidValues;       
  }
  else
  {
    return 0;
  }
}



eAltimeterDataStatus AltimeterData(bool bReset, bool bNewValue, SINT32 newAltitude, SINT32* averageAltitude )
{
//  static UBYTE altValidThreshold = 20; // In % compare the new value with the latest valid and reject if new < last*(1-threshold/100) or new > last*(1+threshold/100)
//  static UBYTE altHoldLastValid = 2; // If = 0 no thresholding is done, other values determines how many time consecutive new values are considered invalid if not within the threshold limits
  static UBYTE noHoldedValues = 0;
  static SINT32 altimeterValues[ALTIMETER_BUFFER_SIZE];
  static UBYTE bufferIndex = 0;
  static UBYTE noValidValues = 0; // # valid values
  static UBYTE altNoEchoes = 0; 
  float lastValid = 0;

  if (bReset)
  {
//      dbgprintf("Kilroy Here 1,AltimeterData reset");
      // reset counters
      bufferIndex = 0;
      noValidValues = 0;
      noHoldedValues = 0;
      altNoEchoes = 0;
      
      return VALID;
  }
  
#if defined(CJN_MAIN_TESTS)
  
  // Check consistency, changing of this parameter could make things inconsistent
  if (InternalVariables.altBufferSize > ALTIMETER_BUFFER_SIZE) InternalVariables.altBufferSize = 1;
  
  if (bufferIndex >= InternalVariables.altBufferSize)
  // BufferSize has changed (only done in the debug menu
  {
//      dbgprintf("Kilroy Here 2,AltimeterData bufferIndex problem");
    // reset counters
      bufferIndex = 0;
      noValidValues = 0;
      noHoldedValues = 0;
  }
  
#endif
  
  if (bNewValue)
  {
    // First check if the new data is invalid
    if (newAltitude == 0)  
    {
      // Check if to many consecutive invalid values so the altimeter should be considered lost
      if (IsAltimeterLost(TRUE, TRUE)) // Count up the lost data semaphore
      {
//        dbgprintf("Kilroy Here 3,Altimeter lost");
        // reset counters
        bufferIndex = 0;
        noValidValues = 0;
        noHoldedValues = 0;
      } 
    }
    // Check if Altimeter is too high above the seabed  
    else if (newAltitude >= ALTIMETER_NO_ECHO)
    {
//         dbgprintf("Kilroy Here 4, No echo");
     
      altNoEchoes++;
      altNoEchoes %= (MAX_NO_ECHOES + 1);
      IsAltimeterLost(TRUE, FALSE);      
    }    
    else
    {
      //
      // We have a valid new altitude value here, depending on the gating 
      // store the new value or the previously stored value in the buffer
      //
      /////////////////////////////////////////////////////////////////////
      
      // Check if gating should be done
      if ((noValidValues > 0) && (InternalVariables.altHoldLastValid > noHoldedValues))
      {
        if (bufferIndex > 0)
          lastValid = (float) altimeterValues[bufferIndex-1];
        else
          lastValid = (float) altimeterValues[InternalVariables.altBufferSize-1];
 
        if (((float) newAltitude < lastValid*(1.-((float)InternalVariables.altValidThreshold)/100.)) || ((float) newAltitude > lastValid*(1.+((float)InternalVariables.altValidThreshold)/100. )))
        // new altitude is outside the gate, use last valid altitude measurement
        {
          altimeterValues[bufferIndex] = (SINT32) lastValid;  // use last Valid altimeter measurement
          noHoldedValues++;
        }
        else
        // within the gate, use the new altitude
        {
          altimeterValues[bufferIndex] = newAltitude;
          noHoldedValues = 0;
        }
      }
      else
      // first altitude received or already to many holds...
      {
        altimeterValues[bufferIndex] = newAltitude;
        noHoldedValues = 0;
      }  
       
      // Update indicies
      bufferIndex++, bufferIndex = bufferIndex % InternalVariables.altBufferSize;
      noValidValues++;
      if (noValidValues > InternalVariables.altBufferSize) noValidValues = InternalVariables.altBufferSize;
      
      // Count down semaphores
      if (altNoEchoes >0) altNoEchoes--;
      IsAltimeterLost(TRUE, FALSE);
    } // valid Value
  } // New value
  
  // return the average altitude as out parameter
  *averageAltitude = CalcAverage(altimeterValues, noValidValues);
  
 // dbgprintf("Kilroy was here 2, Altitude = %d, # Values = %d\n", *averageAltitude, noValidValues);

  //
  // Chose suitable return value
  ////////////////////////////////////////////////
  if (IsAltimeterLost(FALSE, TRUE))
  {
// NB! MASSE turn on dbgprintf if debugtxt_show in future
//    dbgprintf( " AltimeterData = TO_MANY_INVALID \r\n");
    return TO_MANY_INVALID;  
  }
  if (altNoEchoes == MAX_NO_ECHOES)
  {
//    dbgprintf( " AltimeterData = TO_MANY_ECHOES \r\n");
    return TO_MANY_ECHOES; 
  }
  
  if (noValidValues != InternalVariables.altBufferSize) 
  {
//    dbgprintf( " AltimeterData = NOT_READY \r\n");
    return NOT_READY; // return NOT_READY if not the altitude buffer is filled
  }
  
  return  VALID;     //Everything is fine here
}


bool IsAltimeterLost(bool bWrite, bool bIsEmptyOrInvalid)
{
  static UBYTE altNoLost = 0;
  
  if (bWrite)
  {
    if (bIsEmptyOrInvalid)
    {
      altNoLost++;
      if (altNoLost > MAX_NO_ALT_VOID_DATA) altNoLost = MAX_NO_ALT_VOID_DATA; 
    }
    else
    {
      if (altNoLost > 0) altNoLost--;
    }   
  }
  
  // return , read mode starts here directly
  if (altNoLost == MAX_NO_ALT_VOID_DATA) return TRUE;
  
  // Altimeter still considered to be alive
  return FALSE;
}

bool EnterAltimeterCruising( void )
{
  SINT32 averageAltitude;

  // First check if altimeter cruising function is enabled and thrusters is on and we get valid data from altimeter
  if (AltCruisingEnable && (Thrust_On) && (AltimeterData( FALSE, FALSE, 0, &averageAltitude ) == VALID) )
  {
    // Then check if in Horizon mode
    InternalVariables.altimeterCruising = ON;
#ifdef SIM   
//    InternalVariables.altitude_Wanted = averageAltitude - SimDeltaDepth;
    AltitudeWanted = averageAltitude - SimDeltaDepth;
  //  SimStartAltitudeWanted = InternalVariables.altitude_Wanted;
#else
//    InternalVariables.altitude_Wanted = averageAltitude;   
    AltitudeWanted = averageAltitude;   
#endif
//    Depth_Started = Depth_Latest;
    
//    // Intítialize the control system for switching to altimeter instead of the depth sensor to get a smooth transition
//    for( UBYTE i=0 ; i<NUMBER_OF_OLD_DEPTH_SAMPLES ; i++ )
//    {
//       Depth_OldReading[i] = ((float)averageAltitude)/1000.;
//    }
//    Depth_Current = ((float)averageAltitude)/1000.;
//    DeltaDepth = 0.;
//    DeltaDepthToAdd = 0.;

#if defined(DEBUG_TXT_SHOW) || defined(CJN_MAIN_TESTS)     // Debugging allowed?
    if( AllowDbgPrintoutFromCJNTests == TRUE )
    {
       //   dbgprintf( " ALTITUDE CRUISING ON...\r\n" );
        DebugPrintAltitudeCruisingMode(TRUE);
    }
#endif //DEBUG_TXT_SHOW    // Debugging allowed?    

    return TRUE;
  }
  else
  {
    InternalVariables.altimeterCruising = OFF;
    return FALSE;
  }
}

bool LeaveAltimeterCruising( void )
{
  if ( InternalVariables.altimeterCruising != OFF )
  { 
    InternalVariables.altimeterCruising = OFF;
    
//    // Intítialize the control system for switching to depth sensor instead of the altimeter to get a smooth transition
//    for( UBYTE i=0 ; i<NUMBER_OF_OLD_DEPTH_SAMPLES ; i++ )
//    {
//       Depth_OldReading[i] = Depth_Latest;
//    }
//    Depth_Current = Depth_Latest;
//    DeltaDepth = 0.;
//    DeltaDepthToAdd = 0.;

#if defined(DEBUG_TXT_SHOW) || defined(CJN_MAIN_TESTS)     // Debugging allowed?
    if( AllowDbgPrintoutFromCJNTests == TRUE )
    {
       //   dbgprintf( " ALTITUDE CRUISING OFF...\r\n" );
        DebugPrintAltitudeCruisingMode(FALSE);
    }
#endif //DEBUG_TXT_SHOW    // Debugging allowed?      
    
    return TRUE;
  }
  else
  {
    return FALSE;  // Already OFF cruising
  }
}

void DebugPrintCurrentAltitude(UINT16 CurrAlt)
{
  if (continuousDebugPrintOn==2)
  {
    dbgprintf( " Current Altitude = %d mm \r\n", CurrAlt);
    if (continuousDebugPrintOn == 2) continuousDebugPrintOn = 0;   // NB! this is done globally, i.e. other One shots is turned off even if they have not been fired
  }
}

void DebugPrintAltitudeCruisingMode(bool isOn)
{
  if (continuousDebugPrintOn == ON)
  {
    if (isOn)
    {
      dbgprintf( " ALTITUDE CRUISING ON... \r\n" );
//      dbgprintf( " Wanted Altitude = %d mm \r\n", InternalVariables.altitude_Wanted);
      dbgprintf( " Wanted Altitude = %d mm \r\n", AltitudeWanted);
    }
    else
      dbgprintf( " ALTITUDE CRUISING OFF...\r\n" );
  }
}

void DebugPrintAverageAltitude()
{
  if (continuousDebugPrintOn == ON)
  {
    dbgprintf( "Current Depth: %d mm.\r\n", (SINT32)(Depth_Current*1000) );
//    dbgprintf( "Wanted  Depth: %d mm.\r\n\n", (SINT32)(Depth_Wanted*1000 ) );

    dbgprintf( " Average Altitude = %d mm \r\n",AltitudeMeasAverageCalc );
    dbgprintf( " Wanted Altitude = %d mm \r\n", AltitudeWanted);  
  }
}



////-------------------------------------------------------------------------------------
////
////                  CalcAltJoystickPosition
////
////   This function regulates the altitude with the OnOff control algorithm
////
////   Input:          ?
////
////   Output:         ?
////
////   Sets:           ?
////
////-------------------------------------------------------------------------------------
//void CalcAltJoystickPosition(Vector_3 *AltJoystick_Position, Vector_3 Joystick_Position)
//{
//   static bool HYST_ON = false;
//   static int RampCounter = 0; // from 0 upto RampLimit, i.e. when counterRamplimit is given full thrust
//   int RampLimit = 20;  // Counter is counted upto 20 on 20*50 ms i.e. 1 sec
//   static float LastFullZPos = 0.;
//   static int debugCntr = 0;  // MASSE
//   
//   if (debugCntr++ % 10 == 0) 
//     dbgprintf( " CalcAltJoystickPosition Entered \r\n");
//   
//   for( int i = 0; i < 3; i++ ) 
//   {
//      (*AltJoystick_Position)[i] = Joystick_Position[i];
//   }
//         
//   if (InternalVariables.altimeterCruising == ON)
//   {
//     if (debugCntr++ % 10 == 0) 
//        dbgprintf( " CalcAltJoystickPosition cruising ON  \r\n");
//
//     if (fabs((*AltJoystick_Position)[2]) > 0.05)
//     {
//        if (debugCntr % 10 == 0) 
//          dbgprintf( " CalcAltJoystickPosition new wanted altitude \r\n");
//#ifdef SIM
//        InternalVariables.altitude_Wanted = AltitudeMeasAverageCalc - SimDeltaDepth;
//#else
//        InternalVariables.altitude_Wanted = AltitudeMeasAverageCalc;
//#endif   
//        return;
//     }
//     
//     float DeadBandUpLevel = InternalVariables.altitude_Wanted + InternalVariables.altOnOffDeadBand*100.;  // AltOnOff deadband is in dm and needs to be converted to mm
//     if (DeadBandUpLevel > InternalVariables.maxAltimeterRange*100.) DeadBandUpLevel = InternalVariables.maxAltimeterRange*100.;
//
//     float DeadBandLowLevel = InternalVariables.altitude_Wanted;
//     if (DeadBandLowLevel < InternalVariables.minAltimeterRange*100.) DeadBandLowLevel = InternalVariables.minAltimeterRange*100.;
//     
//     float HystSize = 0.3; // between 0 - 0.5      
//
//     float HystUpLevel = DeadBandUpLevel - InternalVariables.altOnOffDeadBand*100.*HystSize;
//     if (HystUpLevel < InternalVariables.minAltimeterRange*100.) HystUpLevel = InternalVariables.minAltimeterRange*100.;    
//     
//     float HystLowLevel = DeadBandLowLevel + InternalVariables.altOnOffDeadBand*100.*HystSize;
//     if (HystLowLevel > InternalVariables.maxAltimeterRange*100.) HystLowLevel = InternalVariables.maxAltimeterRange*100.;    
//   
//#ifdef SIM
//     if (AltitudeMeasAverageCalc - SimDeltaDepth > DeadBandUpLevel)
//#else
//     if (AltitudeMeasAverageCalc > DeadBandUpLevel)
//#endif   
//     // Turn On steering Down
//     {
//   if (debugCntr++ % 10 == 0) 
//     dbgprintf( " Steering Down  \r\n");
//        // Z
//        (*AltJoystick_Position)[2] += InternalVariables.altOnOffZFactor/100.;
//        if ((*AltJoystick_Position)[2] > 1.) (*AltJoystick_Position)[2] = 1.;    
//        HYST_ON = true;
//        
//        // Up Ramping
//        LastFullZPos = (*AltJoystick_Position)[2];
//        if (RampCounter < RampLimit) 
//        {
//          RampCounter++;
//          (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//        }        
//     }
//#ifdef SIM
//     else if (AltitudeMeasAverageCalc - SimDeltaDepth > HystUpLevel)
//#else
//     else if (AltitudeMeasAverageCalc > HystUpLevel)
//#endif   
//     {
//        if (HYST_ON)
//        {
//          // Z
//          (*AltJoystick_Position)[2] += InternalVariables.altOnOffZFactor/100.;
//          if ((*AltJoystick_Position)[2] > 1.) (*AltJoystick_Position)[2] = 1.;    
//         
//          // Up Ramping
//          LastFullZPos = (*AltJoystick_Position)[2];
//          if (RampCounter < RampLimit) 
//          {
//            RampCounter++;
//            (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//          }       
//        }
//        else
//        {
//          // Down Ramping
//          if (RampCounter > 0) 
//          {
//            RampCounter--;
//            (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*LastFullZPos;
//          }              
//        }
//     }
//#ifdef SIM
//     else if (AltitudeMeasAverageCalc - SimDeltaDepth > HystLowLevel)
//#else
//     else if (AltitudeMeasAverageCalc > HystLowLevel)
//#endif   
//     {
//        HYST_ON = false;
//
//        // Down Ramping
//        if (RampCounter > 0) 
//        {
//          RampCounter--;
//         (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*LastFullZPos;
//        }        
//     }
//#ifdef SIM
//     else if (AltitudeMeasAverageCalc - SimDeltaDepth < DeadBandLowLevel)
//#else
//     else if (AltitudeMeasAverageCalc < DeadBandLowLevel)
//#endif   
//     // Turn On steering Up 
//     {
//   if (debugCntr++ % 10 == 0) 
//     dbgprintf( " Steering Up  \r\n");
//       // Z
//        (*AltJoystick_Position)[2] -= InternalVariables.altOnOffZFactor/100.;
//        if ((*AltJoystick_Position)[2] < -1.) (*AltJoystick_Position)[2] = -1.;
//        HYST_ON = true;
//        
//        // Up Ramping
//        LastFullZPos = (*AltJoystick_Position)[2];
//        if (RampCounter < RampLimit) 
//        {
//          RampCounter++;
//         (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//        }               
//     }
//#ifdef SIM
//     else if (AltitudeMeasAverageCalc - SimDeltaDepth < HystLowLevel)
//#else
//     else if (AltitudeMeasAverageCalc < HystLowLevel)
//#endif   
//     {
//       if (HYST_ON)
//       {
//         // Z
//         (*AltJoystick_Position)[2] -= InternalVariables.altOnOffZFactor/100.;
//         if ((*AltJoystick_Position)[2] < -1.) (*AltJoystick_Position)[2] = -1.;
//        
//         // Up Ramping
//         LastFullZPos = (*AltJoystick_Position)[2];
//         if (RampCounter < RampLimit) 
//         {
//           RampCounter++;
//           (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//         }  
//       }
//       else
//       {
//         // Down Ramping
//         if (RampCounter > 0) 
//         {
//           RampCounter--;
//           (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*LastFullZPos;
//         }              
//       }
//     }
//   }
//   else
//   {
//     // reset ramping
//     RampCounter = 0;
//     LastFullZPos = 0.;
//   }   
//}


   
//   
////-------------------------------------------------------------------------------------
////
////                  CalcAltJoystickPosition
////
////   This function regulates the altitude with the OnOff control algorithm
////
////   Input:          ?
////
////   Output:         ?
////
////   Sets:           ?
////
////-------------------------------------------------------------------------------------
//void CalcAltJoystickPosition(Vector_3 *AltJoystick_Position, Vector_3 Joystick_Position)
//{
//   static bool HYST_ON = false;
//   static int RampCounter = 0; // from 0 upto RampLimit, i.e. when counterRamplimit is given full thrust
//   int RampLimit = 20;  // Counter is counted upto 20 on 20*50 ms i.e. 1 sec
//   static float LastFullZPos = 0.;
//   static int debugCntr = 0;  // MASSE
//   
//   if (debugCntr++ % 10 == 0) 
//     dbgprintf( " CalcAltJoystickPosition Entered \r\n");
//   
//   for( int i = 0; i < 3; i++ ) 
//   {
//      (*AltJoystick_Position)[i] = Joystick_Position[i];
//   }
// 
//   if (InternalVariables.altimeterCruising == ON)
//   {
//     if (debugCntr % 10 == 0) 
//        dbgprintf( " CalcAltJoystickPosition cruising ON  \r\n");
//
//     if (fabs((*AltJoystick_Position)[2]) > 0.05)
//     {
//#ifdef SIM
////        InternalVariables.altitude_Wanted = AltitudeMeasAverageCalc - SimDeltaDepth;
//        AltitudeWanted = AltitudeMeasAverageCalc - SimDeltaDepth;
//#else
////        InternalVariables.altitude_Wanted = AltitudeMeasAverageCalc;
//        AltitudeWanted = AltitudeMeasAverageCalc;
//#endif   
//        if (debugCntr % 10 == 0) 
//        {
////          dbgprintf( " CalcAltJoystickPosition new wanted altitude %d \r\n", InternalVariables.altitude_Wanted );
//          dbgprintf( " CalcAltJoystickPosition new wanted altitude %d \r\n", AltitudeWanted );
//        }
//
//        return;
//     }
//     
////     float DeadBandUpLevel = InternalVariables.altitude_Wanted + InternalVariables.altOnOffDeadBand*100.;  // AltOnOff deadband is in dm and needs to be converted to mm
//     float DeadBandUpLevel = AltitudeWanted + InternalVariables.altOnOffDeadBand*100.;  // AltOnOff deadband is in dm and needs to be converted to mm
//     if (DeadBandUpLevel > InternalVariables.maxAltimeterRange*100.) DeadBandUpLevel = InternalVariables.maxAltimeterRange*100.;
//
////     float DeadBandLowLevel = InternalVariables.altitude_Wanted;
//     float DeadBandLowLevel = AltitudeWanted;
//     if (DeadBandLowLevel < InternalVariables.minAltimeterRange*100.) DeadBandLowLevel = InternalVariables.minAltimeterRange*100.;
//     
//     float HystSize = 0.3; // between 0 - 0.5      
//
//     float HystUpLevel = DeadBandUpLevel - InternalVariables.altOnOffDeadBand*100.*HystSize;
//     if (HystUpLevel < InternalVariables.minAltimeterRange*100.) HystUpLevel = InternalVariables.minAltimeterRange*100.;    
//     
//     float HystLowLevel = DeadBandLowLevel + InternalVariables.altOnOffDeadBand*100.*HystSize;
//     if (HystLowLevel > InternalVariables.maxAltimeterRange*100.) HystLowLevel = InternalVariables.maxAltimeterRange*100.;    
//   
//#ifdef SIM
//     if (AltitudeMeasAverageCalc - SimDeltaDepth > DeadBandUpLevel)
//#else
//     if (AltitudeMeasAverageCalc > DeadBandUpLevel)
//#endif   
//     // Turn On steering Down
//     {
//   if (debugCntr % 10 == 0) 
//     dbgprintf( " Steering Down  \r\n");
//        // Z
//        (*AltJoystick_Position)[2] += InternalVariables.altOnOffZFactor/100.;
//        if ((*AltJoystick_Position)[2] > 1.) (*AltJoystick_Position)[2] = 1.;    
//        HYST_ON = true;
//        
//        // Up Ramping
//        LastFullZPos = (*AltJoystick_Position)[2];
//        if (RampCounter < RampLimit) 
//        {
//          RampCounter++;
//          (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//        }        
//     }
//#ifdef SIM
//     else if (AltitudeMeasAverageCalc - SimDeltaDepth > HystUpLevel)
//#else
//     else if (AltitudeMeasAverageCalc > HystUpLevel)
//#endif   
//     {
//        if (HYST_ON)
//        {
//          // Z
//          (*AltJoystick_Position)[2] += InternalVariables.altOnOffZFactor/100.;
//          if ((*AltJoystick_Position)[2] > 1.) (*AltJoystick_Position)[2] = 1.;    
//         
//          // Up Ramping
//          LastFullZPos = (*AltJoystick_Position)[2];
//          if (RampCounter < RampLimit) 
//          {
//            RampCounter++;
//            (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//          }       
//        }
//        else
//        {
//          // Down Ramping
//          if (RampCounter > 0) 
//          {
//            RampCounter--;
//            (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*LastFullZPos;
//          }              
//        }
//     }
//#ifdef SIM
//     else if (AltitudeMeasAverageCalc - SimDeltaDepth > HystLowLevel)
//#else
//     else if (AltitudeMeasAverageCalc > HystLowLevel)
//#endif   
//     {
//        HYST_ON = false;
//
//        // Down Ramping
//        if (RampCounter > 0) 
//        {
//          RampCounter--;
//         (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*LastFullZPos;
//        }        
//     }
//#ifdef SIM
//     else if (AltitudeMeasAverageCalc - SimDeltaDepth < DeadBandLowLevel)
//#else
//     else if (AltitudeMeasAverageCalc < DeadBandLowLevel)
//#endif   
//     // Turn On steering Up 
//     {
//   if (debugCntr % 10 == 0) 
//     dbgprintf( " Steering Up  \r\n");
//       // Z
//        (*AltJoystick_Position)[2] -= InternalVariables.altOnOffZFactor/100.;
//        if ((*AltJoystick_Position)[2] < -1.) (*AltJoystick_Position)[2] = -1.;
//        HYST_ON = true;
//        
//        // Up Ramping
//        LastFullZPos = (*AltJoystick_Position)[2];
//        if (RampCounter < RampLimit) 
//        {
//          RampCounter++;
//         (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//        }               
//     }
//#ifdef SIM
//     else if (AltitudeMeasAverageCalc - SimDeltaDepth < HystLowLevel)
//#else
//     else if (AltitudeMeasAverageCalc < HystLowLevel)
//#endif   
//     {
//       if (HYST_ON)
//       {
//         // Z
//         (*AltJoystick_Position)[2] -= InternalVariables.altOnOffZFactor/100.;
//         if ((*AltJoystick_Position)[2] < -1.) (*AltJoystick_Position)[2] = -1.;
//        
//         // Up Ramping
//         LastFullZPos = (*AltJoystick_Position)[2];
//         if (RampCounter < RampLimit) 
//         {
//           RampCounter++;
//           (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//         }  
//       }
//       else
//       {
//         // Down Ramping
//         if (RampCounter > 0) 
//         {
//           RampCounter--;
//           (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*LastFullZPos;
//         }              
//       }
//     }
//   }
//   else
//   {
//     // reset ramping
//     RampCounter = 0;
//     LastFullZPos = 0.;
//   }   
//}

 
   
//-------------------------------------------------------------------------------------
//
//                  CalcAltJoystickPosition
//
//   This function regulates the altitude with the OnOff control algorithm
//
//   Input:          ?
//
//   Output:         ?
//
//   Sets:           ?
//
//-------------------------------------------------------------------------------------
void CalcAltJoystickPosition(Vector_3 *AltJoystick_Position, Vector_3 Joystick_Position)
{
   static bool HYST_ON = false;
//   static int RampCounter = 0; // from 0 upto RampLimit, i.e. when counterRamplimit is given full thrust
//   int RampLimit = 20;  // Counter is counted upto 20 on 20*50 ms i.e. 1 sec
//   static float LastFullZPos = 0.;
//   static int debugCntr = 0;  // MASSE
   
//   if (debugCntr++ % 10 == 0) 
//     dbgprintf( " CalcAltJoystickPosition Entered \r\n");
   
   for( int i = 0; i < 3; i++ ) 
   {
      (*AltJoystick_Position)[i] = Joystick_Position[i];
   }
 
   if (InternalVariables.altimeterCruising == ON)
   {
//     if (debugCntr % 10 == 0) 
//        dbgprintf( " CalcAltJoystickPosition cruising ON  \r\n");

     if (fabs((*AltJoystick_Position)[2]) > 0.05)
     {
#ifdef SIM
//        InternalVariables.altitude_Wanted = AltitudeMeasAverageCalc - SimDeltaDepth;
        AltitudeWanted = AltitudeMeasAverageCalc - SimDeltaDepth;
#else
//        InternalVariables.altitude_Wanted = AltitudeMeasAverageCalc;
        AltitudeWanted = AltitudeMeasAverageCalc;
#endif   
//        if (debugCntr % 10 == 0) 
//        {
////          dbgprintf( " CalcAltJoystickPosition new wanted altitude %d \r\n", InternalVariables.altitude_Wanted );
//          dbgprintf( " CalcAltJoystickPosition new wanted altitude %d \r\n", AltitudeWanted );
//        }

        return;
     }
     
//     float DeadBandUpLevel = InternalVariables.altitude_Wanted + InternalVariables.altOnOffDeadBand*100.;  // AltOnOff deadband is in dm and needs to be converted to mm
     float DeadBandUpLevel = AltitudeWanted + InternalVariables.altOnOffDeadBand*100.;  // AltOnOff deadband is in dm and needs to be converted to mm
     if (DeadBandUpLevel > InternalVariables.maxAltimeterRange*100.) DeadBandUpLevel = InternalVariables.maxAltimeterRange*100.;

//     float DeadBandLowLevel = InternalVariables.altitude_Wanted;
     float DeadBandLowLevel = AltitudeWanted;
     if (DeadBandLowLevel < InternalVariables.minAltimeterRange*100.) DeadBandLowLevel = InternalVariables.minAltimeterRange*100.;
     
     float HystSize = 0.3; // between 0 - 0.5      

     float HystUpLevel = DeadBandUpLevel - InternalVariables.altOnOffDeadBand*100.*HystSize;
     if (HystUpLevel < InternalVariables.minAltimeterRange*100.) HystUpLevel = InternalVariables.minAltimeterRange*100.;    
     
     float HystLowLevel = DeadBandLowLevel + InternalVariables.altOnOffDeadBand*100.*HystSize;
     if (HystLowLevel > InternalVariables.maxAltimeterRange*100.) HystLowLevel = InternalVariables.maxAltimeterRange*100.;    
   
#ifdef SIM
     if (AltitudeMeasAverageCalc - SimDeltaDepth > DeadBandUpLevel)
#else
     if (AltitudeMeasAverageCalc > DeadBandUpLevel)
#endif   
     // Turn On steering Down
     {
//   if (debugCntr % 10 == 0) 
//     dbgprintf( " Steering Down  \r\n");
        // Z
        (*AltJoystick_Position)[2] += InternalVariables.altOnOffZFactor/100.;
        if ((*AltJoystick_Position)[2] > 1.) (*AltJoystick_Position)[2] = 1.;    
        HYST_ON = true;
        
//        // Up Ramping
//        LastFullZPos = (*AltJoystick_Position)[2];
//        if (RampCounter < RampLimit) 
//        {
//          RampCounter++;
//          (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//        }        
     }
#ifdef SIM
     else if (AltitudeMeasAverageCalc - SimDeltaDepth > HystUpLevel)
#else
     else if (AltitudeMeasAverageCalc > HystUpLevel)
#endif   
     {
        if (HYST_ON)
        {
          // Z
          (*AltJoystick_Position)[2] += InternalVariables.altOnOffZFactor/100.;
          if ((*AltJoystick_Position)[2] > 1.) (*AltJoystick_Position)[2] = 1.;    
         
//          // Up Ramping
//          LastFullZPos = (*AltJoystick_Position)[2];
//          if (RampCounter < RampLimit) 
//          {
//            RampCounter++;
//            (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//          }       
        }
        else
        {
//          // Down Ramping
//          if (RampCounter > 0) 
//          {
//            RampCounter--;
//            (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*LastFullZPos;
//          }       
          (*AltJoystick_Position)[2] = 0.;
        }
     }
#ifdef SIM
     else if (AltitudeMeasAverageCalc - SimDeltaDepth > HystLowLevel)
#else
     else if (AltitudeMeasAverageCalc > HystLowLevel)
#endif   
     {
        HYST_ON = false;

//        // Down Ramping
//        if (RampCounter > 0) 
//        {
//          RampCounter--;
//         (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*LastFullZPos;
//        }       
        (*AltJoystick_Position)[2] = 0.;
     }
#ifdef SIM
     else if (AltitudeMeasAverageCalc - SimDeltaDepth < DeadBandLowLevel)
#else
     else if (AltitudeMeasAverageCalc < DeadBandLowLevel)
#endif   
     // Turn On steering Up 
     {
//   if (debugCntr % 10 == 0) 
//     dbgprintf( " Steering Up  \r\n");
       // Z
        (*AltJoystick_Position)[2] -= InternalVariables.altOnOffZFactor/100.;
        if ((*AltJoystick_Position)[2] < -1.) (*AltJoystick_Position)[2] = -1.;
        HYST_ON = true;
        
//        // Up Ramping
//        LastFullZPos = (*AltJoystick_Position)[2];
//        if (RampCounter < RampLimit) 
//        {
//          RampCounter++;
//         (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//        }               
     }
#ifdef SIM
     else if (AltitudeMeasAverageCalc - SimDeltaDepth < HystLowLevel)
#else
     else if (AltitudeMeasAverageCalc < HystLowLevel)
#endif   
     {
       if (HYST_ON)
       {
         // Z
         (*AltJoystick_Position)[2] -= InternalVariables.altOnOffZFactor/100.;
         if ((*AltJoystick_Position)[2] < -1.) (*AltJoystick_Position)[2] = -1.;
        
//         // Up Ramping
//         LastFullZPos = (*AltJoystick_Position)[2];
//         if (RampCounter < RampLimit) 
//         {
//           RampCounter++;
//           (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*(*AltJoystick_Position)[2];
//         }  
       }
       else
       {
//         // Down Ramping
//         if (RampCounter > 0) 
//         {
//           RampCounter--;
//           (*AltJoystick_Position)[2] = (((float)RampCounter)/((float)RampLimit))*LastFullZPos;
//         }  
         
         (*AltJoystick_Position)[2] = 0.;
       }
     }
   }
//   else
//   {
//     // reset ramping
//     RampCounter = 0;
//     LastFullZPos = 0.;
//   }   
}



//************************************************************************************
//
//                  FloatToS16
//
//   convert a 32bit float into 16 bit signed integer and divides into 2 bytes MSB&LSB
//                   
//
//   Input:          float F
//
//   Output:         UBYTE *MSB *LSB
//
//   Sets:           -
//
//************************************************************************************
void FloatToS16( float F, UBYTE *MSB, UBYTE *LSB )
{
  F = F >= 0 ? (F+0.5) : (F-0.5);
  SINT16 myS16 = (SINT16) F;
  UINT16 myU16 = (UINT16) myS16;
  *MSB = myU16 >> 8;
  *LSB = myU16 & 0x0F;
}



#endif // ALTIMETER_USED