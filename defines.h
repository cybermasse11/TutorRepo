
//************************************************************************************
//
//                  Ocean Modules Offshore V2
//
//                  Ocean Modules 2010
//*
//*                 defines
//*
//*
//*  Development environment: IAR Embedded Workbench IDE 5.1.0.417.7663
//*									IAR Assembler for ARM      5.50.0.51878
//*									IAR C/C++ Compiler for ARM 5.50.0.51878
//*									IAR Elf Linker for ARM     5.50.0.51878
//*									IAR Build Utility          5.1.0.417.7663
//*
//*  Filename:      $Workfile: defines.h $
//*  Revision:      $Revision: 102 $
//*
//*  Prepared by:   R&D Christer Johansson, 
//*
//*  Description:   Definitions of how to build Offshore V2 sw
//*                 Example: Tests included or nor
//*                          RS232 default baudrate
//*                          etc.
//*
//*************************************************************************


//*************************************************************************
//*
//*              REVISION HISTORY
//*
//*  Date:       Name:      Description:
//*
//*  2010-06-14  CJN    Started/Created
//*
//*************************************************************************
/*
 * $History: defines.h $
 * 
 * *****************  Version 102  *****************
 * User: Mats Ahlstrand Date: 17-08-14   Time: 16:06
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * This is the R054 version which will be shipped to the SEATERRA L3000
 * ROV. Now the Altimeter cruising with an ON-OFF control algorithm is
 * working. The SW is dependent on the R029 version of the ROV Analyser
 * application. The parameters are set in the Altimeter Data tab page in
 * ROV Analyser. The altimeter cruising algorithm int the ROV is only
 * enabled when R029 version of ROV Analyser is running on the system
 * computer
 * 
 * *****************  Version 101  *****************
 * User: Mats Ahlstrand Date: 17-07-07   Time: 14:23
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * This is the versionof the altimeter steering algorithm extended with
 * hysteresis and ramping of the thrust
 * 
 * *****************  Version 94  *****************
 * User: Mats Ahlstrand Date: 16-05-23   Time: 16:39
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * This is the P054mu version. The systemprogram supports the internal
 * variables maxAltimeterRange, minAltimeterRange,
 * workDepthBelowWarnLevel,w orkDepthAboveWarnLevel.  can messages and
 * routines for sending the info to the overlay has been done, i.e.
 * MSG_OVERLAY_ALT_LIMITS & MSG_OVERLAY_DEPTHWARN_LIMITS. and
 * SendOverlayAltLimits and SendOverlayDepthWarnLimits. a new parameter in
 * defines is added, i.e. ifdef DEPTH_WARNING
 * 
 * *****************  Version 93  *****************
 * User: Mats Ahlstrand Date: 16-02-12   Time: 9:57
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * In externaliohandler.c 2 new functions is designed:
 * UpdateAltitudeMessage & ValidateAltitudeMessage
 * 
 * *****************  Version 92  *****************
 * User: Mats Ahlstrand Date: 16-02-08   Time: 15:45
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * This is the 1st version for Blom and also for a SEATERRA L3000 machine
 * on service. The steering voltages is reduced between -4,6 to 4,7 V  for
 * both L3000 and M500.
 * 
 * *****************  Version 91  *****************
 * User: Mats Ahlstrand Date: 16-01-27   Time: 14:12
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * In this version the problem with lost messages for the altimeter is
 * solved and "cleaned up". Also a problem with false messages when the
 * thrusters is turned on is  fixed. This version is rudimentary tested
 * and more tests is needed
 * 
 * *****************  Version 90  *****************
 * User: Mats Ahlstrand Date: 16-01-20   Time: 10:57
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * This version is a working version where  the problem of lost messages
 * from the altimeter is solved
 * 
 * *****************  Version 89  *****************
 * User: Mats Ahlstrand Date: 15-11-24   Time: 15:11
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * This is P054eta version . This handles altimeter with 1 byte interrupt
 * length for the altimeter serial buffer.... In this version camera 3 is
 * moved to option board 5 also
 * 
 * *****************  Version 88  *****************
 * User: Mats Ahlstrand Date: 15-11-12   Time: 10:31
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * This version is based on L3000 P047 Mögster with Altimeter cruising
 * support as in R048 for Baltic Offshore. 
 * 
 * *****************  Version 66  *****************
 * User: Mats Ahlstrand Date: 15-02-07   Time: 20:56
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Updated with new voltage steering table in order to reduce current use
 * in Mogsters L3000. This is done in RovSpecifics.c
 * 
 * *****************  Version 44  *****************
 * User: Christer Johansson Date: 14-09-09   Time: 11:18
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw ver to P040Beta
 * 
 * *****************  Version 43  *****************
 * User: Christer Johansson Date: 14-08-25   Time: 13:58
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Added support for M500_SLOW_DOWN_TO_SIMULATE_L3000 to make M500 slower
 * to simulate L3000
 * 
 * *****************  Version 42  *****************
 * User: Christer Johansson Date: 14-08-20   Time: 8:43
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw ver to P038_Mogster2
 * 
 * *****************  Version 41  *****************
 * User: Christer Johansson Date: 14-08-20   Time: 8:28
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw ver to P038_Mogster1
 * 
 * *****************  Version 40  *****************
 * User: Christer Johansson Date: 14-08-19   Time: 10:55
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw rev to "P037 Mogster"
 * 
 * *****************  Version 39  *****************
 * User: Christer Johansson Date: 14-08-11   Time: 14:09
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Try some more tests before going to Mögster
 * Stepped to P037
 * 
 * *****************  Version 38  *****************
 * User: Christer Johansson Date: 14-08-10   Time: 14:14
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw to P037Epsilon
 * 
 * *****************  Version 37  *****************
 * User: Christer Johansson Date: 14-08-10   Time: 12:14
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Added support for external temp sensor in EXTIN1
 * 
 * *****************  Version 36  *****************
 * User: Christer Johansson Date: 14-08-10   Time: 9:11
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped __swRev to P037Gamma
 * 
 * *****************  Version 35  *****************
 * User: Christer Johansson Date: 14-08-09   Time: 11:20
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Added support for new digital report from ProtectionBoard with
 * restart-flag etc.
 * 
 * *****************  Version 34  *****************
 * User: Christer Johansson Date: 14-06-13   Time: 14:01
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw ver to P037Beta
 * 
 * *****************  Version 33  *****************
 * User: Christer Johansson Date: 14-06-11   Time: 14:01
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Set ROV_V8_FENIX as std
 * 
 * *****************  Version 32  *****************
 * User: Christer Johansson Date: 14-06-11   Time: 10:50
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw ver to P037Alfa
 * 
 * *****************  Version 31  *****************
 * User: Christer Johansson Date: 14-06-05   Time: 10:28
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw ver to P036
 * 
 * *****************  Version 30  *****************
 * User: Christer Johansson Date: 14-06-05   Time: 9:45
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sv ver to P036Zeta
 * 
 * *****************  Version 29  *****************
 * User: Christer Johansson Date: 14-06-03   Time: 19:20
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw rev to P036Epsilon
 * 
 * *****************  Version 28  *****************
 * User: Christer Johansson Date: 14-06-03   Time: 18:07
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw ver to P036Delta
 * 
 * *****************  Version 27  *****************
 * User: Christer Johansson Date: 14-06-03   Time: 18:03
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw ver to P036Gamma
 * 
 * *****************  Version 26  *****************
 * User: Christer Johansson Date: 14-06-02   Time: 15:19
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw ver to P036Beta
 * Compiles with CJN_MAIN_TESTS for more test possibilities.
 * 
 * *****************  Version 25  *****************
 * User: Christer Johansson Date: 14-05-15   Time: 16:29
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * #define ROV_V8_SESAM för Mögster
 * 
 * *****************  Version 24  *****************
 * User: Christer Johansson Date: 14-05-09   Time: 16:22
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Added InternalVariables.SwModuleImuDataAsUdp for program modules
 * activation by encrypted code.
 * 
 * *****************  Version 23  *****************
 * User: Christer Johansson Date: 14-04-20   Time: 15:07
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped to P034 as sw version
 * 
 * *****************  Version 22  *****************
 * User: Christer Johansson Date: 14-04-08   Time: 15:39
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Preliminary version
 * 
 * *****************  Version 21  *****************
 * User: Christer Johansson Date: 14-04-08   Time: 15:29
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Timwe for R032
 * 
 * *****************  Version 20  *****************
 * User: Christer Johansson Date: 14-04-03   Time: 12:49
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw version to P032
 * Removed support for NDE_VEHICLE since the ROV does no longer exist.
 * Moved #define TRAFOSENSOR_USED to its own line for new L3000 supports
 * 
 * *****************  Version 19  *****************
 * User: Christer Johansson Date: 14-03-25   Time: 16:52
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Added support to chose UDP message type for ROVDataMsg.
 * 
 * *****************  Version 18  *****************
 * User: Christer Johansson Date: 14-03-25   Time: 14:28
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Added support for IMU data over UDP, applicationMsg
 * 
 * *****************  Version 17  *****************
 * User: Christer Johansson Date: 14-03-06   Time: 11:02
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * ROV_V8_FENIX
 * 
 * *****************  Version 16  *****************
 * User: Christer Johansson Date: 14-03-04   Time: 14:12
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Added support for LED, Gripper and 24V feed 4-7 from PowerBoard
 * 
 * *****************  Version 15  *****************
 * User: Christer Johansson Date: 14-03-03   Time: 15:07
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw ver to P031
 * 
 * *****************  Version 14  *****************
 * User: Christer Johansson Date: 14-03-03   Time: 11:12
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw rev to R030
 * 
 * *****************  Version 13  *****************
 * User: Christer Johansson Date: 14-02-20   Time: 13:13
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw rev
 * 
 * *****************  Version 12  *****************
 * User: Christer Johansson Date: 14-01-27   Time: 14:35
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * sw ver P030
 * 
 * *****************  Version 11  *****************
 * User: Christer Johansson Date: 14-01-21   Time: 13:56
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw version to R030 Beta
 * 
 * *****************  Version 10  *****************
 * User: Christer Johansson Date: 14-01-10   Time: 13:51
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped __swRev to R029
 * 
 * *****************  Version 9  *****************
 * User: Christer Johansson Date: 13-12-17   Time: 16:16
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped __swRev to R028
 * 
 * *****************  Version 8  *****************
 * User: Christer Johansson Date: 13-12-17   Time: 10:00
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * ROYAL NAVY no longer special    - it is std M500
 * JD CONTRACTOR no longer special - it is std M500
 * 
 * *****************  Version 7  *****************
 * User: Christer Johansson Date: 13-12-05   Time: 11:02
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Checking of all before december delivery
 * Stepped rev to P028
 * 
 * *****************  Version 6  *****************
 * User: Christer Johansson Date: 13-11-29   Time: 15:12
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Added support for resetting thruster log time from ROV Analyser:
 * UDP_FRAME_SEND_THRUSTERLOGTIMERESET
 * CTRL_THRUSTERLOGTIMERESET_DONE
 * 
 * *****************  Version 5  *****************
 * User: Christer Johansson Date: 13-11-20   Time: 15:30
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * P028Alfa buit for LIMITED_TESTS
 * 
 * *****************  Version 4  *****************
 * User: Christer Johansson Date: 13-11-04   Time: 15:27
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Added support for ROV Analyser asking for VCC and Temperature from
 * CAN-type of boards.
 * 
 * *****************  Version 3  *****************
 * User: Christer Johansson Date: 13-11-01   Time: 15:53
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped __swRev to P028Alfa
 * 
 * *****************  Version 2  *****************
 * User: Christer Johansson Date: 13-10-30   Time: 8:50
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * Stepped sw rev to P027 to make some pre-job on next release
 * 
 * *****************  Version 1  *****************
 * User: Christer Johansson Date: 13-09-06   Time: 14:04
 * Created in $/OM/ROV Software/Platform/SystemPgm/OM_APP/include
 * 
 * *****************  Version 67  *****************
 * User: Christer Johansson Date: 13-09-06   Time: 8:59
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Time to move R&D to new location on server....
 * 
 * *****************  Version 66  *****************
 * User: Christer Johansson Date: 13-08-21   Time: 16:09
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Stepped rev to P026beta
 * 
 * *****************  Version 65  *****************
 * User: Christer Johansson Date: 13-06-14   Time: 13:24
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Stepped swrev to P025
 * 
 * *****************  Version 64  *****************
 * User: Christer Johansson Date: 13-06-04   Time: 14:10
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Fixed REPORT DIGITAL STATUS from Protectionbioards and Filterboard.
 * 
 * *****************  Version 63  *****************
 * User: Christer Johansson Date: 13-06-04   Time: 13:32
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Build code for std Fenix (M500 Basic + M500 Plus)
 * 
 * *****************  Version 62  *****************
 * User: Christer Johansson Date: 13-06-03   Time: 16:34
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Stop using CAMERA_SWITCH_USED
 * 
 * *****************  Version 61  *****************
 * User: Christer Johansson Date: 13-05-30   Time: 14:30
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Built for Limited test + M500 (RN)
 * 
 * *****************  Version 60  *****************
 * User: Christer Johansson Date: 13-05-28   Time: 11:30
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Stepped __swRev to P024alfa
 * 
 * *****************  Version 59  *****************
 * User: Christer Johansson Date: 13-05-23   Time: 9:17
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Stepped Software rev to "R023"
 * Built for Royal Navy M500 with the new restricted power use when using
 * long tether.
 * 
 * *****************  Version 58  *****************
 * User: Christer Johansson Date: 13-05-16   Time: 15:55
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Sw rev "P023" +  LIMITED_TESTS compilation
 * 
 * *****************  Version 57  *****************
 * User: Joakim Östlund Date: 13-05-15   Time: 14:21
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Added functionality in ROV-control for voltage limiting.
 * For 400m tether + 50m deck cable, 50% limiting of max sum voltage to
 * thruster control is a first try. Result in small pool:
 * Min system voltage: 208V
 * Found at 53% reduction of control voltage sum, equals sum 21,2V
 * 
 * At no limit: Min system voltage 180V
 * 
 * *****************  Version 56  *****************
 * User: Christer Johansson Date: 13-05-15   Time: 9:33
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP/include
 * Added support for thrustPowerFactorActive and thrustPowerFactor
 * in InternalVariables
 * 
 * 
 * *****************  Version 1  *****************
 * User: CJN      Date: 09-02-05   Time: 15:23
 * Created
 *
 */

//*************************************************************************
//* Disable the keyword functionality
//* $NoKeywords: $
//*************************************************************************
// #define configCHECK_FOR_STACK_OVERFLOW 2  // CJN may be checking of stack overflow in FreeRTOSConfig.h 
// see http://www.freertos.org/index.html?http://www.freertos.org/Stacks-and-stack-overflow-checking.html


//************************************************************************************
//
//       REVISIONS.
//
//************************************************************************************
#define __swRev          "P065_GX5"    // Software rev. of this program (P=preliminary/R=Release)



//************************************************************************************
//
//       MCU TYPE USED
//
//************************************************************************************
// Note that you also need to change [Project] [Options] [Target]
//************************************************************************************
//#define     USE_AT91SAM7X256        // Using Atmel AT91SAM7X256 MPU (influences storage of parameters etc.)
#define     USE_AT91SAM7X512      // Using Atmel AT91SAM7X512 MPU (influences storage of parameters etc.)
//#define     USE_AT91SAM7XC256     // Using Atmel AT91SAM7XC256 MPU (influences storage of parameters etc.)
//#define     USE_AT91SAM7XC512     // Using Atmel AT91SAM7XC512 MPU (influences storage of parameters etc.)

//************************************************************************************
//
//       IMU TYPE USED
//
//************************************************************************************
//#define GX1
#define GX5

//************************************************************************************
//
//       Other definitions like ROV type etc.
//
//************************************************************************************
//#define     ROV_V8_FENIX            // Some files like ROVControl may be same file as for other ROVs
#define ROV_V8_SESAM          // Some files like ROVControl may be same file as for other ROVs
#define ALTIMETER_USED //The Vehicle supports altimeter cruising¨
//#define DEPTH_WARNING // The vehicle has a depth below and a depth above limit defined which triggers a warning if above or below the warning levels used in JFN_VEHICLE

//************************************************************************************
//
//       F E N I X     M 5 0 0
//
//************************************************************************************
#ifdef ROV_V8_FENIX
#undef ROV_V8_SESAM

// Define standard configuration for FENIX (can be undefined later)
#define STD_FENIX // M500 Basic or M500 Plus is supported

// ----- Select ONE of these versions - if none, Fenix Basic or Fenix Plus is supported ------
//#define     CHINA_UNIV_FENIX      // First Fenix sw for Chinese university
   // ROYAL NAVY no longer special    - it is std M500
   // JD CONTRACTOR no longer special - it is std M500
 // ----- Select ONE of these versions ------

 // ----- Choose if standard configuration is used or not ------
 // define customer as well. Will take care of small changes.
 // Note that CHINA_UNIV_FENIX and ROYAL_NAVY_FENIX do NOT support standard configuration!
#if defined(CHINA_UNIV_FENIX)
  #undef STD_FENIX
#endif

 // Other definitions for M500
 #define DEPTH_MEAS_500M            // Depth sensor is rated different to 3000 meter sensor


//************************************************************************************
//
//      S E S A M     L 3 0 0 0
//
//************************************************************************************
#elif defined(ROV_V8_SESAM)
#undef ROV_V8_FENIX

 // ----- Select ONE of these versions ------
 #define       STD_L3000            // STANDARD L3000 sw
 //#define     GU_VEHICLE           // GU (Göteborgs Universitet) special sw
 //#define     SEMATEK_VEHICLE      // Sematek special sw
 //#define     SEATERRA_VEHICLE     // SeaTerra V8 Offshore
 // --> THIS ROV IS DEAD!!!!  #define     NDE_VEHICLE          // NDE Offshore - THIS ROV IS DEAD !!!!!!!!
 // ----- Select ONE of these versions ------

 #define DEPTH_MEAS_3000M
 #define TRAFOSENSOR_USED           // Transformer's compensator's oil level sensor is used 
                                    // (introduced in 200272 SeaTerra V8 Offshore)

#endif // ROV_V8_FENIX/ROV_V8_SESAM


//************************************************************************************
//
//       Faking board number.
//       Note that if none is defined here, program will use 
//       InternalVariables.optionBoardFakeBoardNo if needed
//
//************************************************************************************
//#define     FAKE_BOARD_NO  0xA3     // Fake address for surface OptionBoard.
//#define     FAKE_BOARD_NO  0xA1     // Fake address for OptionBoard inside ROV.
//#define     FAKE_BOARD_NO  0x00     // Fake address for ControlBoard inside ROV.



//************************************************************************************
//
//       Special features for specific customers
//
//************************************************************************************
#ifdef GU_VEHICLE
 //#define  PAN_TILT_SIDUS           // Sidus Pan and Tilt support
 #define  PAN_TILT_KONGSBERG         // Kongsberg Pan and Tilt support

#elif defined(SEMATEK_VEHICLE)
 //#define  LEAKSENSOR1_PROBLEM      // Problem med läckagesensor #1

#elif defined(SEATERRA_VEHICLE)
#define  TRAFOSENSOR_USED           // Add it here as well just to be sure :)

#endif //GU_VEHICLE


//************************************************************************************
//
//       PAN-TILT settings
//
//************************************************************************************
// NOW ADD definition of PAN_TILT as well.
#ifdef   PAN_TILT_SIDUS
#define  PAN_TILT                   // Pan and Tilt support
#endif //PAN_TILT_SIDUS

#ifdef   PAN_TILT_KONGSBERG
#define  PAN_TILT                   // Pan and Tilt support
#endif //PAN_TILT_KONGSBERG




//************************************************************************************
//
//       DEBUG
//
//************************************************************************************
   
//#define     SIM                        // Simulate driving the ROV, for the moment only in z direction. Also ALTIMETER_USED is needed for the moment.
   
#define     DEBUG_INCLUDED             // Debug printout possibility shall be included
// NOTE configUSE_TRACE_FACILITY set to 1 in file FreeRTOSConfig.h
// NOTE configCHECK_FOR_STACK_OVERFLOW can be used by setting to 0 in FreeRTOS.h
#ifdef      DEBUG_INCLUDED
#define     DEBUG_BAUDRATE                AT91C_BAUDRATE_115200   // Debug port's baudrate

// Show more debugging text if DEBUG_TXT_SHOW is defined
//#define     DEBUG_TXT_SHOW
//#define     DO_SHOW_ADC_PROBLEM   // ADC needs attention!!!

// L3000 SIMULATOR  ----  Make M500 behave slower to simulate L3000
//#define     M500_SLOW_DOWN_TO_SIMULATE_L3000

//************************************************************************************
//
//       TESTS (only if DEBUG_INCLUDED is defined)
//
//************************************************************************************
#define     TESTING                    // Uncomment if any tests are to be run (also affected by DEBUG_INCLUDED above)
#ifdef      TESTING                    // Any tests at all?

#define     LIMITED_TESTS              // LIMITED - Uncomment if MAIN tests (limited tests) are being performed
//#define   CJN_MAIN_TESTS             // CJN     - Uncomment if CJN tests are being performed
//#define   JOS_MAIN_TESTS             // JOS     - Uncomment if JOS tests are being performed

// *******  SPECIAL TESTS  ************
//#define   MFS_TEST
//#define   TEST300VONOFF                   // LEA TESTS FOR 300V MOTORS
//#define   TESTJOYSTICKCOMMAND             // CJN TESTS FOR DETECTIG UNEXPECTED JOYSTICK COMMANDS
//#define   TEST_IMU_BAD_DATA               // CJN TESTS FOR CHECKING STRANGE/BAD IMU DATA
//#define   EXTIN1_TEMPERATURE_PROBES_USAGE // CJN - CHECKING TEMPERATURE ON PROTECTIONBOARDS (MÖGSTER)

// Common defines
#if defined(LIMITED_TESTS) || defined(CJN_MAIN_TESTS) || defined(JOS_MAIN_TESTS)
#define     COMMON_TESTS
#endif

#endif      // TESTING                 // Any tests at all?
#endif      //DEBUG_INCLUDED

//-----------------------------------------------------------
