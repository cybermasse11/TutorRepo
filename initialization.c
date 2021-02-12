//*************************************************************************
//*
//*                 Ocean Modules Offshore V2
//*
//*                 initialization
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
//*  Filename:      $Workfile: initialization.c $
//*  Revision:      $Revision: 54 $
//*
//*  Prepared by:   R&D Christer Johansson, 
//*
//*  Description:   init
//*
//*************************************************************************


//*************************************************************************
//*
//*              REVISION HISTORY
//*
//*  Date:       Name:      Description:
//*
//*  2010-08-02  CJN    Started/Created
//*  2010-08-03  CJN    Added InitEmacEthAddr and fields to InternalVariables
//*
//*************************************************************************
/*
 * $History: initialization.c $
 * 
 * *****************  Version 54  *****************
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
 * *****************  Version 53  *****************
 * User: Mats Ahlstrand Date: 17-07-07   Time: 14:23
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This is the versionof the altimeter steering algorithm extended with
 * hysteresis and ramping of the thrust
 * 
 * *****************  Version 46  *****************
 * User: Mats Ahlstrand Date: 16-05-23   Time: 16:39
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This is the P054mu version. The systemprogram supports the internal
 * variables maxAltimeterRange, minAltimeterRange,
 * workDepthBelowWarnLevel,w orkDepthAboveWarnLevel.  can messages and
 * routines for sending the info to the overlay has been done, i.e.
 * MSG_OVERLAY_ALT_LIMITS & MSG_OVERLAY_DEPTHWARN_LIMITS. and
 * SendOverlayAltLimits and SendOverlayDepthWarnLimits. a new parameter in
 * defines is added, i.e. ifdef DEPTH_WARNING
 * 
 * *****************  Version 45  *****************
 * User: Mats Ahlstrand Date: 16-02-12   Time: 9:57
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * In externaliohandler.c 2 new functions is designed:
 * UpdateAltitudeMessage & ValidateAltitudeMessage
 * 
 * *****************  Version 44  *****************
 * User: Mats Ahlstrand Date: 16-01-20   Time: 10:57
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This version is a working version where  the problem of lost messages
 * from the altimeter is solved
 * 
 * *****************  Version 43  *****************
 * User: Mats Ahlstrand Date: 15-11-24   Time: 15:11
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This is P054eta version . This handles altimeter with 1 byte interrupt
 * length for the altimeter serial buffer.... In this version camera 3 is
 * moved to option board 5 also
 * 
 * *****************  Version 42  *****************
 * User: Mats Ahlstrand Date: 15-11-12   Time: 10:31
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This version is based on L3000 P047 Mögster with Altimeter cruising
 * support as in R048 for Baltic Offshore. 
 * 
 * *****************  Version 28  *****************
 * User: Mats Ahlstrand Date: 15-02-07   Time: 20:56
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Updated with new voltage steering table in order to reduce current use
 * in Mogsters L3000. This is done in RovSpecifics.c
 * 
 * *****************  Version 23  *****************
 * User: Christer Johansson Date: 14-09-09   Time: 11:11
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for serial number in Internal Variables of ControlBoard.
 * 
 * *****************  Version 22  *****************
 * User: Christer Johansson Date: 14-08-10   Time: 14:14
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added new InternalVariables.ThrusterSwap_26_and_48 
 * Added support to show and change ThrusterSwap_26_and_48
 * 
 * *****************  Version 21  *****************
 * User: Christer Johansson Date: 14-06-12   Time: 16:24
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for keeping track of ProtectionBoard_X_Sw
 * 
 * *****************  Version 20  *****************
 * User: Christer Johansson Date: 14-06-11   Time: 10:51
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added InternalVariables.ThrusterNewPBConnection for new thruster power
 * connection in PowerBoards
 * 
 * *****************  Version 19  *****************
 * User: Christer Johansson Date: 14-06-05   Time: 13:29
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for CardTemperatures[CardTempValueNum] for average temp
 * calculation.
 * 
 * *****************  Version 18  *****************
 * User: Christer Johansson Date: 14-06-03   Time: 17:33
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for InternalVariables.AllowSwUpdate
 * 
 * *****************  Version 17  *****************
 * User: Christer Johansson Date: 14-05-09   Time: 16:22
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added InternalVariables.SwModuleImuDataAsUdp for program modules
 * activation by encrypted code.
 * 
 * *****************  Version 16  *****************
 * User: Christer Johansson Date: 14-03-25   Time: 16:52
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support to chose UDP message type for ROVDataMsg.
 * 
 * *****************  Version 15  *****************
 * User: Christer Johansson Date: 14-03-17   Time: 15:14
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for OptionBoard 0x06 test- and default settings
 * 
 * *****************  Version 14  *****************
 * User: Christer Johansson Date: 14-03-14   Time: 15:33
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for putting serial port values used in production test
 * into internal variables.
 * 
 * *****************  Version 13  *****************
 * User: Christer Johansson Date: 14-03-11   Time: 15:07
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * GripperForce now uses 0-100% as incoming and outgoing value as well as
 * InternalVariables stored value.
 * 
 * *****************  Version 12  *****************
 * User: Christer Johansson Date: 14-03-06   Time: 8:55
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for Gripper force control
 * 
 * *****************  Version 11  *****************
 * User: Christer Johansson Date: 14-03-04   Time: 14:11
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for LED, Gripper and 24V feed 4-7 from PowerBoard
 * 
 * *****************  Version 10  *****************
 * User: Christer Johansson Date: 14-02-14   Time: 11:49
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added new default setup for OptionBoard 0x01 and PowerBoard 0x04
 * 
 * *****************  Version 9  *****************
 * User: Christer Johansson Date: 14-02-04   Time: 13:50
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Increased variable size InternalVariables.depthCompensationValue from
 * SINT16 to SINT32 to support depth corrections > 500,00 Meters.
 * 
 * *****************  Version 8  *****************
 * User: Christer Johansson Date: 14-01-27   Time: 13:52
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for temperature in thruster start-up data string.
 * Added T300VStatusTable[i].ThrusterTemperature
 * 
 * *****************  Version 7  *****************
 * User: Christer Johansson Date: 14-01-10   Time: 13:48
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Changed name of InternalVariables.thrustPowerLimitPercentage to
 * InternalVariables.thrustPowerFactor
 * Changed name of InternalVariables.thrustPowerLimitActive to
 * InternalVariables.thrustPowerFactorActive
 * 
 * *****************  Version 6  *****************
 * User: Christer Johansson Date: 14-01-08   Time: 14:08
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Use different default settings depending on CardAddress and type of
 * board
 * 
 * *****************  Version 5  *****************
 * User: Christer Johansson Date: 13-12-17   Time: 10:00
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * ROYAL NAVY no longer special    - it is std M500
 * JD CONTRACTOR no longer special - it is std M500
 * 
 * *****************  Version 4  *****************
 * User: Christer Johansson Date: 13-11-18   Time: 16:14
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added ThusterHardwareVersion in e300VStatusTableType.
 * Added ThusterType in e300VStatusTableType.
 * Added ThusterSoftwareType in e300VStatusTableType.
 * Added ThusterSoftwareNo in e300VStatusTableType.
 * Added ThusterMinuteLogTime in e300VStatusTableType.
 * Added ThusterMinuteTotalTime in e300VStatusTableType.
 * 
 * 
 * *****************  Version 3  *****************
 * User: Christer Johansson Date: 13-11-04   Time: 15:27
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for ROV Analyser asking for VCC and Temperature from
 * CAN-type of boards.
 * 
 * *****************  Version 2  *****************
 * User: Christer Johansson Date: 13-09-26   Time: 9:37
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for InternalVariables.depthDesiredValueFactor
 * 
 * *****************  Version 1  *****************
 * User: Christer Johansson Date: 13-09-06   Time: 14:00
 * Created in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * 
 * *****************  Version 25  *****************
 * User: Christer Johansson Date: 13-08-27   Time: 12:35
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support to save depth compensation value
 * 
 * *****************  Version 24  *****************
 * User: Christer Johansson Date: 13-08-26   Time: 13:54
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Handle depth compensation value to show overlay and NMEA data as the
 * same value.
 * 
 * *****************  Version 23  *****************
 * User: Christer Johansson Date: 13-08-23   Time: 13:04
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added control of if text to overlay is allowed or not.
 * 
 * *****************  Version 22  *****************
 * User: Christer Johansson Date: 13-06-14   Time: 13:20
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for InternalVariables.whatBoardStackAdrRunsRovCtrlTask
 * Added support for InternalVariables.altitudeMeasuringFpgaADport
 * Added support for InternalVariables.altitudeMeasAverageCalcInFPGA
 * 
 * *****************  Version 21  *****************
 * User: Christer Johansson Date: 13-06-03   Time: 16:31
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for InternalVariables.videoSwitchConfiguration
 * 
 * *****************  Version 20  *****************
 * User: Christer Johansson Date: 13-05-30   Time: 14:29
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added reverseModeByAUX2Allowed for supporting reverse mode via AUX2.
 * 
 * *****************  Version 19  *****************
 * User: Christer Johansson Date: 13-05-28   Time: 11:19
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for ReverseMode
 * 
 * *****************  Version 18  *****************
 * User: Christer Johansson Date: 13-05-16   Time: 15:50
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for hdCameraPort where Bowtech is 0x51 and Kongsberg
 * 0x55.
 * 
 * *****************  Version 17  *****************
 * User: Christer Johansson Date: 13-05-15   Time: 9:33
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for thrustPowerFactorActive and thrustPowerFactor
 * in InternalVariables
 * 
 * *****************  Version 16  *****************
 * User: Christer Johansson Date: 13-05-13   Time: 16:30
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for camera1Type, camera3Type and hdCameraType in setup. 
 * 
 * *****************  Version 15  *****************
 * User: Christer Johansson Date: 13-03-01   Time: 13:14
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for TESTJOYSTICKCOMMAND
 * 
 * *****************  Version 14  *****************
 * User: Christer Johansson Date: 13-02-20   Time: 14:20
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for camera select switch inside ROV.
 * 
 * *****************  Version 13  *****************
 * User: Christer Johansson Date: 13-02-18   Time: 16:53
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for via setup change thruster direction.
 * Used to experiment with better thruster force/ROV speed.
 * 
 * *****************  Version 12  *****************
 * User: Christer Johansson Date: 12-12-20   Time: 15:06
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * BREAK detection added.
 * 
 * *****************  Version 11  *****************
 * User: Christer Johansson Date: 12-11-26   Time: 9:49
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * alwaysd initialize AutoFocusTriggering.
 * alwaysd initialize AutoFocusDelay.
 * 
 * *****************  Version 10  *****************
 * User: Christer Johansson Date: 12-10-12   Time: 14:35
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for Rotator speed as InternalVariables.GUmagasineSpeed
 * 
 * *****************  Version 9  *****************
 * User: Christer Johansson Date: 12-10-10   Time: 15:58
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for Transformer oil level detector
 * 
 * *****************  Version 8  *****************
 * User: Christer Johansson Date: 12-09-04   Time: 12:51
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Splitted size of comm buffer depending of type of board.
 * Size of comm buffer increased in OptionBoard for better support for
 * high speed serial ports.
 * 
 * *****************  Version 7  *****************
 * User: Christer Johansson Date: 12-08-22   Time: 15:45
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for showing CAN boards hw, fw and sw versions.
 * 
 * *****************  Version 6  *****************
 * User: Christer Johansson Date: 12-06-01   Time: 11:28
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for pan and tilt limits in setup.
 * 
 * *****************  Version 5  *****************
 * User: Christer Johansson Date: 12-05-31   Time: 10:38
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * split panTiltTiltSpeed for Sidus and Kongsberg units.
 * split panTiltPanSpeed for Sidus and Kongsberg units.
 * 
 * *****************  Version 4  *****************
 * User: Christer Johansson Date: 12-03-20   Time: 12:40
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for InternalVariables.canOverEthernetBoardStackAddress1.
 * and InternalVariables.canOverEthernetBoardStackAddress2.
 * 
 * *****************  Version 3  *****************
 * User: Christer Johansson Date: 12-03-09   Time: 15:05
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * STD_L3000 also uses 2 cameras using Focus, Zoom and autofocus.
 * 
 * *****************  Version 2  *****************
 * User: Christer Johansson Date: 12-02-28   Time: 15:10
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Corrected T300VStatusTable (Show correct protection board number)
 * 
 * *****************  Version 1  *****************
 * User: Christer Johansson Date: 12-02-27   Time: 14:30
 * Created in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * 
 * *****************  Version 59  *****************
 * User: Christer Johansson Date: 12-02-27   Time: 11:46
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * InternalVariables no longer keeps:
 * MACAddress[6]
 * emacIPAddr[4]
 * emacNetMask[4]
 * emacGatewayAddr[4]
 * dnsServerAddr1[4]
 * dnsServerAddr2[4]
 * 
 * *****************  Version 58  *****************
 * User: Christer Johansson Date: 12-02-21   Time: 9:39
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Stop using InitEmacEthAddr[]
 * 
 * *****************  Version 57  *****************
 * User: Christer Johansson Date: 12-01-17   Time: 15:40
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Do not use UARTs before they are initialized.
 * 
 * *****************  Version 56  *****************
 * User: Christer Johansson Date: 12-01-11   Time: 13:49
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Tidy up the code regarding commands to tasks etc:
 * commandType definitions to ADC task repaced by eCommandsAdc.
 * 
 * *****************  Version 55  *****************
 * User: Christer Johansson Date: 12-01-09   Time: 14:58
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Changed serial-over-ethernet from sending package with destination UART
 * address to  sending package with transmitting UART address.
 * 
 * *****************  Version 54  *****************
 * User: Christer Johansson Date: 11-12-23   Time: 8:24
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Corrected initialization of ProtectionBoardNumber
 * 
 * *****************  Version 53  *****************
 * User: Christer Johansson Date: 11-12-07   Time: 14:33
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support foir ErrorReportingWithDebugText()
 * with maximum number of times to show each error text.
 * 
 * *****************  Version 52  *****************
 * User: Christer Johansson Date: 11-11-27   Time: 15:26
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Corrected A/D values in tabel T300VStatusTable.
 * Use MSG_300V_ALLTHRUSTERS_ON 3dr time we try to restart thrusters.
 * Get what300VoutputsAreOrderedToBeOn from ProtectionBoard status
 * Get motorStatusJustChangedTimer0 from ProtectionBoard status
 * Get what300VoutputsAreOn0 from ProtectionBoard status
 * 
 * *****************  Version 51  *****************
 * User: Christer Johansson Date: 11-11-26   Time: 17:02
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added functionality to Restart Thrusters
 * 
 * *****************  Version 50  *****************
 * User: Christer Johansson Date: 11-11-26   Time: 16:13
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Stop state machine turning thrusters on if ROVCRequestThrusterOff comes
 * in.
 * 
 * *****************  Version 49  *****************
 * User: Christer Johansson Date: 11-11-26   Time: 15:56
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added InitializeGlobals() in initialization.c
 * 
 * *****************  Version 48  *****************
 * User: Christer Johansson Date: 11-11-11   Time: 11:40
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Changed power down detection and behaviour
 * 
 * *****************  Version 47  *****************
 * User: Christer Johansson Date: 11-11-10   Time: 8:31
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for redirecting debugport data to another board via
 * serial-over-ethernet.
 * 
 * *****************  Version 46  *****************
 * User: Christer Johansson Date: 11-10-21   Time: 16:25
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/SystemProgram/OM_APP
 * Make it possible to send debuginformation to serial-over-ethernet
 * 
 * *****************  Version 45  *****************
 * User: Christer Johansson Date: 11-10-20   Time: 14:44
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/SystemProgram/OM_APP
 * Added support for InternalVariables.NMEAPortOverEthernetAddress
 * Changed name from MoveInternalVariablesFromFlashToRAM to
 * CopyInternalVariablesFromFlashToRAM.
 * Changed name from MoveInternalVariablesFromRAMToFlash to
 * CopyInternalVariablesFromRAMToFlash.
 * 
 * *****************  Version 44  *****************
 * User: Christer Johansson Date: 11-10-11   Time: 14:26
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/SystemProgram/OM_APP
 * Added support for Manipulator in GU and in Sematek code.
 * 
 * *****************  Version 43  *****************
 * User: Christer Johansson Date: 11-09-15   Time: 10:43
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added support for variable aliveMsgToPcuBoardStackAddress for Alive msg
 * to PCU
 * 
 * *****************  Version 42  *****************
 * User: Christer Johansson Date: 11-06-27   Time: 14:09
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * InternalVariables.rovType is no longer used (but still there).
 * 
 * *****************  Version 41  *****************
 * User: Christer Johansson Date: 11-06-16   Time: 14:42
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * renamed depthMeasuringPort to depthMeasuringFpgaADport.
 * GetDepthFromFPGA() now reads 24 bits value (earlier it was 16)
 * 
 * *****************  Version 40  *****************
 * User: Christer Johansson Date: 11-06-16   Time: 12:55
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added support for InternalVariables.port2OverEthernetAddress
 * 
 * *****************  Version 39  *****************
 * User: Christer Johansson Date: 11-06-16   Time: 7:24
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added support for setup of 8 serial ports in InternalVariables.
 * 
 * *****************  Version 38  *****************
 * User: Christer Johansson Date: 11-06-14   Time: 17:40
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added support for enabel/disabe CAN-over-Ethernet
 * 
 * *****************  Version 37  *****************
 * User: Christer Johansson Date: 11-06-08   Time: 14:39
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * typedef struct always named XxxxxxStructType.
 * 
 * *****************  Version 36  *****************
 * User: Lars-erik Ahnell Date: 11-05-24   Time: 11:51
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * ANIS 2011-05-15
 * 
 * *****************  Version 35  *****************
 * User: Lars-erik Ahnell Date: 11-05-02   Time: 14:48
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * ANIS 2011-05-02: Added code for dataflash and OMFileSystem
 * 
 * *****************  Version 34  *****************
 * User: Christer Johansson Date: 11-04-29   Time: 11:22
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added rovType,GUpumpMotorMaxVoltand GUskidMotorVolt to
 * InternalVariables.
 * 
 * *****************  Version 33  *****************
 * User: Christer Johansson Date: 11-04-26   Time: 10:38
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * ANIS - Ethernet
 * 
 * *****************  Version 32  *****************
 * User: Christer Johansson Date: 11-04-26   Time: 8:34
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Get back to before ANIS version is checked in
 * 
 * *****************  Version 30  *****************
 * User: Christer Johansson Date: 11-04-19   Time: 9:32
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Change to use UART RX and TX queues as indexed list.
 * xQueueHandle UartRxQueues[32];
 * xQueueHandle UartTxQueues[32];
 * 
 * *****************  Version 29  *****************
 * User: Christer Johansson Date: 11-04-14   Time: 19:08
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added whatBoardIsRunningRovCtrlTask in internalvariables to control
 * which board in the stack that runs ROVControlTask
 * 
 * *****************  Version 28  *****************
 * User: Christer Johansson Date: 11-04-13   Time: 17:09
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added internalVariables support for being able to fake bordaddress.
 * Added all board/stack addresses for functions in GU ROV.
 * 
 * *****************  Version 27  *****************
 * User: Christer Johansson Date: 11-03-23   Time: 15:49
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added check of queue width for some queues
 * 
 * *****************  Version 26  *****************
 * User: Christer Johansson Date: 11-03-22   Time: 13:41
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Fixed size of message queues etc.
 * 
 * *****************  Version 25  *****************
 * User: Christer Johansson Date: 11-02-10   Time: 13:52
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Changed parameter whatBoardIsRunningStack0-3 to
 * whatBoardIsRunningThisStack in InternalVariables.
 * 
 * *****************  Version 24  *****************
 * User: Christer Johansson Date: 11-02-09   Time: 17:52
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Only run ROV Control Task on one board in the system.
 * 
 * *****************  Version 23  *****************
 * User: Christer Johansson Date: 11-02-09   Time: 14:13
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Keep track of what board this is, it's stack pos and pos inside the
 * stack.
 * Keep track of what queues to start, tasks to start etc. via global
 * varibales ThisBoardRunsThisStack and ThisBoardRunsRovCtrlTask.
 * 
 * *****************  Version 22  *****************
 * User: Christer Johansson Date: 11-02-08   Time: 16:59
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added support for flash page setup of
 * whatBoardIsRunningRovCtrlTask,imuPort,imuSettings,panTiltPort,panTiltSe
 * ttings and depthMeasuringPort.
 * 
 * *****************  Version 21  *****************
 * User: Christer Johansson Date: 11-02-02   Time: 11:50
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added queues for the 8 UARTS in FPGA
 * 
 * *****************  Version 20  *****************
 * User: Christer Johansson Date: 11-02-01   Time: 13:53
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Defined all FPGA registers.
 * Start using new FPGA registers according to new spec.
 * 
 * *****************  Version 19  *****************
 * User: Christer Johansson Date: 10-12-28   Time: 10:43
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added support for card ID when sending overtemperatureerror msg
 * 
 * *****************  Version 18  *****************
 * User: Christer Johansson Date: 10-12-14   Time: 11:51
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Tidy the code up a bit.
 * 
 * *****************  Version 17  *****************
 * User: Christer Johansson Date: 10-11-30   Time: 12:00
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added some more logic to ROV Control task
 * 
 * *****************  Version 16  *****************
 * User: Christer Johansson Date: 10-11-26   Time: 8:11
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added files from V8 Sii för ROV control into this Offshore project.
 * 
 * *****************  Version 15  *****************
 * User: Christer Johansson Date: 10-11-19   Time: 11:36
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * CAN transmit via interrupt and message queue implemented.
 * 
 * *****************  Version 14  *****************
 * User: Christer Johansson Date: 10-10-28   Time: 16:36
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Make it possible to send alive+powerOk CAN messages from service task
 * to keep old type small PCU not indication LINK error nor POWER error.
 * 
 * *****************  Version 13  *****************
 * User: Christer Johansson Date: 10-10-26   Time: 7:32
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Fungerar med halva uppdateringarna. - message queues updated.
 * 
 * *****************  Version 12  *****************
 * User: Christer Johansson Date: 10-10-25   Time: 9:23
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Time is running, soft clock can be read.
 * 
 * *****************  Version 11  *****************
 * User: Christer Johansson Date: 10-10-22   Time: 15:43
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * All tasks running without error.
 * 
 * *****************  Version 10  *****************
 * User: Christer Johansson Date: 10-10-20   Time: 10:55
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * ADC measurement and under/over voltage detection working.
 * 
 * *****************  Version 9  *****************
 * User: Christer Johansson Date: 10-10-19   Time: 17:18
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Partly added functionality to measure all 4 voltages
 * Does not work correctly yet!
 * 
 * *****************  Version 8  *****************
 * User: Christer Johansson Date: 10-10-08   Time: 9:00
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * COM1 and COM2 code is removed.
 * CAN message comes via interrupt routine to DataHandler via message
 * queue.
 * Interrupt routine also sends alert meassage to service task.
 * Service task sends alert message to DataHandler.
 * DataHandler gets CAN message and prints it on screen via debug port.
 * 
 * *****************  Version 7  *****************
 * User: Christer Johansson Date: 10-09-03   Time: 15:39
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Rensat i filer och adderat CAN som inte funkar ännu.
 * 
 * *****************  Version 6  *****************
 * User: Christer Johansson Date: 10-08-24   Time: 13:24
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Clean up code
 * 
 * *****************  Version 5  *****************
 * User: Christer Johansson Date: 10-08-20   Time: 16:27
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Tidy the code up a bit more.
 * 
 * *****************  Version 4  *****************
 * User: Christer Johansson Date: 10-08-20   Time: 10:52
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Renamed handler to DataHandler
 * 
 * *****************  Version 3  *****************
 * User: Christer Johansson Date: 10-08-20   Time: 9:46
 * Updated in $/OM/ROV Software/ROV V8 Offshore v2/KommunikationskortYta/OM_APP
 * Added ExtIOHandlerTask
 * 
 */

//*************************************************************************
//* Disable the keyword functionallity
//* $NoKeywords: $
//*************************************************************************


//*************************************************************************
//*
//*        FUNCTION PROTOTYPES
//*
//*************************************************************************
//void prvSetupHardware( void );
//void initAdcMeasurement( void );
//void prvSetupQueues( void );
//#ifdef  TESTING
//void writeVariablesToFlash( void );
//void WriteDefaultScriptToFlash( UBYTE *downloadedReceiptInterpreterScriptStruct );
//#endif
//void GetVarDataFromFlash( void );
//void putDefaultValuesIntoIntVar( void );
//UBYTE* GetPointerToReceiptInterpreterScriptInFlash( void );
//void InitializeGlobals( void );




//*************************************************************************
//*
//*        INCLUDE FILES
//*
//*************************************************************************
#include <string.h>
//#include "defines.h" // inluded i header file
#include "constants.h"
#include "globals.h"       // Here it only acts as "extern"...
#include "debugprintf.h"
//#include "serial.h"
#include "DataHandler.h"
#include "flash.h"
#include "PIOhandler.h"
#include "initialization.h"
#include "DataFlash.h"

//*************************************************************************
//
//        EXTERNALS
//
//*************************************************************************
// Interrupt entry point written in the assembler file ADCISR.s79.
extern void vADCISREntry( void );


//*************************************************************************
//*
//*        GLOBAL VARIABLES
//*
//*************************************************************************
/*
         char  XxxxxxBuf[100];	// Storage buffer to...
volatile int   Xxxx;	            // Xxxx..
         char  LocalXxxxx;       // Local Xxx....
*/

//*************************************************************************
//*
//*        MODULE VARIABLES
//*
//*************************************************************************
/*
static   char  XxxxBuf[100];
static   WORD  Xxxx;
*/

//************************************************************************************
//
//                  prvSetupHardware
//
//   This function Initializes the hardware
//
//   Input:         -
//
//   Output:        -
//
//   Sets:          harware reg       CPU registers etc.
//
//************************************************************************************
void prvSetupHardware( void )
{

   // configurate pioa PA15 as an input for chip detect SD card 0, 1 and 2 signal
   AT91F_PIO_CfgInput( AT91C_BASE_PIOA, AT91C_PIO_PA15 | AT91C_PIO_PA14 | AT91C_PIO_PA12 );

	/* When using the JTAG debugger the hardware is not always initialised to
	the correct default state.  This line just ensures that this does not
	cause all interrupts to be masked at the start. */
	AT91C_BASE_AIC->AIC_EOICR = 0;
	
	/* Most setup is performed by the low level init function called from the
	startup asm file.

	Configure the PIO Lines corresponding to LED1 to LED4 to be outputs as
	well as the UART Tx line. */

	/* Enable the peripheral clock. */
	AT91F_PMC_EnablePeriphClock( AT91C_BASE_PMC, 1 << AT91C_ID_PIOA );
	AT91F_PMC_EnablePeriphClock( AT91C_BASE_PMC, 1 << AT91C_ID_PIOB ) ;


   // configure SPI0 pins for SPI operation
   // disable PIO from controlling MOSI, MISO, SCK and CS
   AT91C_BASE_PIOA->PIO_PDR = SPI0_MISO | SPI0_MOSI | SPI0_SPCK | SPI0_NPCS;
   // set pin-functions in PIO Controller
   AT91C_BASE_PIOA->PIO_ASR = SPI0_MISO | SPI0_MOSI | SPI0_SPCK | SPI0_NPCS;
        
        
   // clear the under voltage detection flag.
   UnderVoltageDetectionFlag = FALSE;

   // clear the queue error detection flag.
   QueueInitilizeErrorDetected = FALSE;

   // clear the flash error detection flag.
   FlashInitilizeErrorDetected = FALSE;

   // clear the first run detection flag.
   FlashFirstTimeInitilizeErrorDetected = FALSE;

   // This is a flag that only is used by the RTC task.
   // The value of it affects other tasks so it must be set before the tasks start up.
   RtcTimeValid = FALSE;

   // clear the ethernet running flag.
   EthernetRunningFlag = FALSE;  // Reset ethernet flag
   
   AllTasksStarted = FALSE;

   // AllTasksUpAndRunning should be OK (0) when all tasks are upp and running correctly
   // During startup we'll clear each bit whenever a task has reported that it's up and running.
   AllTasksUpAndRunning = ( AVAILABLE_TASKS_SOFT_WDT_CHECK_MASK | AVAILABLE_INTERRUPTS_SOFT_WDT_CHECK_MASK );


#ifdef  DEBUG_INCLUDED     // Debugging allowed?
   StorageTaskRunningFlagDisable = 0;
   TwiTaskRunningFlagDisable = 0;
   HidTaskRunningFlagDisable = 0;
   HandlerTaskRunningFlagDisable = 0;
//   ServiceTaskRunningFlagDisable = 0;
   UsbTaskRunningFlagDisable = 0;
   ServerWebTaskRunningFlagDisable = 0;
#endif //DEBUG_INCLUDED    // Debugging allowed?

}


//************************************************************************************
//
//                  initAdcMeasurement
//
//   Initialte the adc hardware to continiously run conversions.
//   AD0 = +24V (A or B feeding MCU)   measured 1.126V.
//   AD1 = +1.5V.                      measured 1.503V.
//   AD2 = +15V.                       measured 1.370V.
//   AD3 = +3.3 and -3.3V.             measured 1.971V.
//
//   Input:         -
//
//   Output:        -
//
//   Sets:          harware reg       CPU registers etc.
//
//************************************************************************************
void initAdcMeasurement( void )
{
   //
   // Set up and enable the ADC check of the Vin value
   //

   // *****************************************************************************
   //              Analog to Digital Converter
   // *****************************************************************************
   // Analog-to-Digital Converter (ADC) User Interface
   // ------------------------------------------------
   // Offset Register                     Name     Access      Reset value
   // 0x00   Control Register             ADC_CR   Write-only  –
   // 0x04   Mode Register	               ADC_MR   Read-write  0x00000000
   // 0x08   Reserved – – –
   // 0x0C   Reserved – – –
   // 0x10   Channel Enable Register      ADC_CHER Write-only  –
   // 0x14   Channel Disable Register     ADC_CHDR Write-only  –
   // 0x18   Channel Status Register      ADC_CHSR Read-only   0x00000000
   // 0x1C   Status Register              ADC_SR   Read-only   0x000C0000
   // 0x20   Last Converted Data Register ADC_LCDR Read-only   0x00000000
   // 0x24   Interrupt Enable Register    ADC_IER  Write-only  –
   // 0x28   Interrupt Disable Register   ADC_IDR  Write-only  –
   // 0x2C   Interrupt Mask Register      ADC_IMR  Read-only   0x00000000
   // 0x30   Channel Data Register 0      ADC_CDR0 Read-only   0x00000000
   // 0x34   Channel Data Register 1      ADC_CDR1 Read-only   0x00000000
   // ... ... ... ... ...
   // 0x4C   Channel Data Register 7      ADC_CDR7 Read-only   0x00000000
   // 0x50 - 0xFC Reserved   
   // *****************************************************************************
   //
   // Sets the ADCMR register.
   // HW trigger enabled
   // 10-bit resolution
   // normal mode
   // The AT91F_ADC_CfgTimings is not used since I want to maximize the time
   // for the conversion to get the most accurate result.
   AT91F_ADC_CfgModeReg( AT91C_BASE_ADC,                             // pointer to a ADC controller - write to the Mode Register
                            0 |                                      // Start with 0x0000
                            AT91C_ADC_TRGEN_EN |                     // Hardware trigger selected by TRGSEL field is enabled.
                            AT91C_ADC_TRGSEL_TIOA0 |                 // (ADC) Selected TRGSEL = TIAO0-2=0) 000=> TIOA Ouput of the Timer Counter Channel 0
                         ( (ADC_PRESCAL<<8) & AT91C_ADC_PRESCAL) |   // (ADC) Prescaler rate selection: 1MHz
                         ( (ADC_STARTUP<<16) & AT91C_ADC_STARTUP) |  // (ADC) Startup Time: maximal time
                         ( (ADC_SHTIM<<24) & AT91C_ADC_SHTIM ));     // (ADC) Sample & Hold Time: Maximal sample and hold time


#ifdef  DEBUG_INCLUDED     // Debugging allowed?
   // Disable all channels if we are resstarting without power down
   AT91F_ADC_DisableChannel( AT91C_BASE_ADC, AT91C_ADC_CH0 );   // Write to the CHDR register CH0
   AT91F_ADC_DisableChannel( AT91C_BASE_ADC, AT91C_ADC_CH1 );   // Write to the CHDR register CH1
   AT91F_ADC_DisableChannel( AT91C_BASE_ADC, AT91C_ADC_CH2 );   // Write to the CHDR register CH2
   AT91F_ADC_DisableChannel( AT91C_BASE_ADC, AT91C_ADC_CH3 );   // Write to the CHCR register CH3
   AT91F_ADC_DisableChannel( AT91C_BASE_ADC, AT91C_ADC_CH4 );   // Write to the CHDR register CH4
   AT91F_ADC_DisableChannel( AT91C_BASE_ADC, AT91C_ADC_CH5 );   // Write to the CHDR register CH5
   AT91F_ADC_DisableChannel( AT91C_BASE_ADC, AT91C_ADC_CH6 );   // Write to the CHDR register CH6
   AT91F_ADC_DisableChannel( AT91C_BASE_ADC, AT91C_ADC_CH7 );   // Write to the CHCR register CH7
#endif //DEBUG_INCLUDED    // Debugging allowed?

   
   // sets the ADC_CHER register which also means assignment of PIO to ADC multiplexing is automatically done to ADC.
   AT91F_ADC_EnableChannel( AT91C_BASE_ADC, AT91C_ADC_CH4 );   // Write to the CHER register CH4
   AT91F_ADC_EnableChannel( AT91C_BASE_ADC, AT91C_ADC_CH5 );   // Write to the CHER register CH5
   AT91F_ADC_EnableChannel( AT91C_BASE_ADC, AT91C_ADC_CH6 );   // Write to the CHER register CH6
   AT91F_ADC_EnableChannel( AT91C_BASE_ADC, AT91C_ADC_CH7 );   // Write to the CHER register CH7

   // Read a few register to "clear" the interrupt
   // I don't know which one needs to be read or why but it works.
   AT91F_ADC_GetStatus( AT91C_BASE_ADC );             // Return ADC Interrupt Status         ADC_SR
   AT91F_ADC_GetLastConvertedData( AT91C_BASE_ADC );  // Return the Last Converted Data      ADC_LCDR
   AT91F_ADC_GetConvertedDataCH7( AT91C_BASE_ADC );   // Return the Channel 3 Converted Data ADC_CDR3  (maybe not needed)
   AT91F_ADC_GetConvertedDataCH5( AT91C_BASE_ADC );   // Return the Channel 2 Converted Data ADC_CDR2  (maybe not needed)
   AT91F_ADC_GetConvertedDataCH5( AT91C_BASE_ADC );   // Return the Channel 1 Converted Data ADC_CDR1  (maybe not needed)
   AT91F_ADC_GetConvertedDataCH4( AT91C_BASE_ADC );   // Return the Channel 0 Converted Data ADC_CDR0   This is a MUST to make interrupt work!

   // Init the ADC interrupt in the AIC
   // AT91F_AIC_ConfigureIt Interrupt Handler Initialization
   AT91F_AIC_ConfigureIt( AT91C_BASE_AIC,                      // \arg pointer to the AIC registers
                          AT91C_ID_ADC,                        // \arg interrupt number to initialize
                          ADC_INTERRUPT_LEVEL,                 // \arg priority to give to the interrupt
                          AT91C_AIC_SRCTYPE_INT_POSITIVE_EDGE, // \arg activation and sense of activation
                          ( void (*)( void ) ) vADCISREntry ); // \arg address of the interrupt handler
   
   AT91F_AIC_EnableIt( AT91C_BASE_AIC, AT91C_ID_ADC );

   // Enable the end of conversion interrupt for AD4
   AT91F_ADC_EnableIt( AT91C_BASE_ADC, AT91C_ADC_EOC4 );


   // sätter upp TIOA0 så att jag kan se att jag genererar en clk puls...PB23 00800000
   //   AT91C_BASE_PIOB->PIO_PDR   = 0x00800000;  // perifial output enabled
   //   AT91C_BASE_PIOB->PIO_ASR   = 0x00800000;  // peripheral A
   //   AT91C_BASE_PIOB->PIO_PPUER = 0x00800000;  // Pull-up Enable Register
   //   AT91C_BASE_PIOB->PIO_MDDR = 0x00800000;  // multidrive Register

   AT91F_TC0_CfgPMC();  // Enable Peripheral clock in PMC for TC0

  
   
   
   
   // *****************************************************************************
   //              Timer Counter Channel 0 triggering A/D conversion start
   // *****************************************************************************
    // Disable TC clock
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;   // TC Channel Control Register  --> (TC) Counter Clock Disable Command
    // Disable interrupts
    AT91C_BASE_TC0->TC_IDR = 0xFFFFFFFF;        // TC Channel Control Register  --> Interrupt Disable Register
    // Clear status register
    AT91C_BASE_TC0->TC_SR;                      // TC Channel Control Register  --> Status Register
    // Set mode                                 // TC Channel Control Register  --> Channel Mode Register (Capture Mode / Waveform Mode)
    AT91C_BASE_TC0->TC_CMR = ( AT91C_TC_CLKS_TIMER_DIV1_CLOCK |
                               AT91C_TC_BURST_NONE |
                               AT91C_TC_EEVTEDG_NONE |
                               AT91C_TC_EEVT_TIOB |
                               AT91C_TC_WAVESEL_UP_AUTO |
                               AT91C_TC_WAVE |
                               AT91C_TC_ACPA_NONE |
                               AT91C_TC_ACPC_TOGGLE |
                               AT91C_TC_AEEVT_NONE |
                               AT91C_TC_ASWTRG_NONE |
                               AT91C_TC_BCPB_NONE |
                               AT91C_TC_BCPC_NONE |
                               AT91C_TC_BEEVT_NONE |
                               AT91C_TC_BSWTRG_NONE );


   // TC Block Mode Register
   AT91C_BASE_TCB->TCB_BMR = ( AT91C_TCB_TC0XC0S_NONE |
                               AT91C_TCB_TC1XC1S_NONE |
                               AT91C_TCB_TC2XC2S_NONE );

   // Set compare level
   // 48M/2 = 24M
   // 24M/1000 = 24000 ticks / ms
   // Since each tick only togles TIOA the value in RC must be half that.
   // AT91C_BASE_TC0->TC_RC = 12000;
   AT91C_BASE_TC0->TC_RC = AT91C_MASTER_CLOCK/2/NUMBER_OF_VIN_UNDERVOLTAGE_LEVEL_CHECKS_S/2;
   
   
   // Starts the timer clock.
   AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;

}


//************************************************************************************
//
//                  prvSetupQueues
//
//   This function creates the external queues
//   Note that InitializeFPGAandGetGlobals() must be run prior to this function!
//
//   Input:         -
//
//   Output:        -
//
//   Sets:          harware reg       CPU registers etc.
//
//************************************************************************************
void prvSetupQueues( void )
{
int i;

//   xTWICommandQueue  = xQueueCreate( TWI_QUEUE_COMMAND_LEN, (unsigned portBASE_TYPE) sizeof( TWIMsgStruct ) );
//   if( ( xTWICommandQueue == INVALID_QUEUE ) )
//   {
//      // Some problem with the queue...
//      QueueInitilizeErrorDetected = TRUE;
//#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
//         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xTWICommandQueue in vTWITask()!!!!!!!!!!!!\r\n" );
//#     endif //DEBUG_INCLUDED    // Debugging allowed?
//   }


   xRTCCommandQueue  = xQueueCreate( RTC_QUEUE_COMMAND_LEN, (unsigned portBASE_TYPE) sizeof( RTCMsgStructType ) );
   if( ( xRTCCommandQueue == INVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xRTCCommandQueue in vTWITask()!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }


   // Create the queue used to give commands to Service task
   //xServiceCommandQueue  = xQueueCreate( SERVICE_QUEUE_COMMAND_LEN, (unsigned portBASE_TYPE) sizeof(UBYTE) );
   xServiceCommandQueue  = xQueueCreate( SERVICE_QUEUE_COMMAND_LEN, (unsigned portBASE_TYPE) sizeof( eServiceCommandsToServiceTask ));
   if( ( xServiceCommandQueue == serINVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xServiceCommandQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }


   // Create the queue used to receive CAN messages from interrupt routine to DataHandler task
   xCANRxdQueue = xQueueCreate( CAN_COM_QUEUE_RXD_LEN, (unsigned portBASE_TYPE) sizeof(CanRxdIntMsgStructType) );
   if( ( xCANRxdQueue == serINVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xCANRxdQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }

   
   // Create the queue used to send CAN messages to interrupt routine.
   xCANTxdQueue = xQueueCreate( CAN_COM_QUEUE_TXD_LEN, (unsigned portBASE_TYPE) sizeof(CanTxdIntMsgStructType) );
   if( ( xCANTxdQueue == serINVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xCANTxdQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }


   
   // Create the queue used to give commands to DataHandler task
   // Check queue width
   if( NUMBER_OF_BYTES_IN_DQATAHANDLERCOMMANDMSG_MESSAGE < sizeof(CanTxdIntMsgStructType) )
   {
      // ERROR!!!!!!!
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR - TO NARROW QUEUE: xDataHandlerCommandQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }
   
//   xDataHandlerCommandQueue  = xQueueCreate( DATA_HANDLER_COMMAND_QUEUE_LEN, (unsigned portBASE_TYPE) sizeof(UBYTE) );
   xDataHandlerCommandQueue  = xQueueCreate( DATA_HANDLER_COMMAND_QUEUE_LEN, (unsigned portBASE_TYPE) sizeof(DataHandlerCommandMsgStructType) );
   if( ( xDataHandlerCommandQueue == INVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xDataHandlerCommandQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }

   
   // OptionBoard type of long buffer as default
   ComQueRxLen = COM_QUEUE_RX_LEN_OB;
   ComQueTxLen = COM_QUEUE_TX_LEN_OB;
   
   
   // Only start queues and task handling ROV Contrl if this is the board that is supposed to run it
   if( ThisBoardRunsRovCtrlTask == TRUE )
   {
      // Check queue width
      if( NUMBER_OF_BYTES_IN_ROVCTRLMSG_MESSAGE < sizeof(canMessageStructType) ||
          NUMBER_OF_BYTES_IN_ROVCTRLMSG_MESSAGE < NUMBER_OF_STABCOMPLETE_BYTES )
      {
         // ERROR!!!!!!!
#        ifdef  DEBUG_INCLUDED     // Debugging allowed?
            dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR - TO NARROW QUEUE: xROVControlCommandQueue!!!!!!!!!!!!\r\n" );
#        endif //DEBUG_INCLUDED    // Debugging allowed?
      }
      
      // Create the queue used to give commands to ROV Control task
      xROVControlCommandQueue  = xQueueCreate( ROV_CONTROL_COMMAND_QUEUE_LEN, (unsigned portBASE_TYPE) sizeof(ROVControlMsgStruct) );
      if( ( xROVControlCommandQueue == INVALID_QUEUE ) )
      {
         // Some problem with the queue...
         QueueInitilizeErrorDetected = TRUE;
#        ifdef  DEBUG_INCLUDED     // Debugging allowed?
            dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xROVControlCommandQueue!!!!!!!!!!!!\r\n" );
#        endif //DEBUG_INCLUDED    // Debugging allowed?
      }
      
      // ControlBoard type of short buffer
      ComQueRxLen = COM_QUEUE_RX_LEN_CB;
      ComQueTxLen = COM_QUEUE_TX_LEN_CB;
   }

   
   
   
   
   // Create the queue used to give commands to External IO Handler task
   // Check queue width
   if( NUMBER_OF_BYTES_IN_EXTIOMSG_MESSAGE < sizeof(canMessageStructType) ||
       NUMBER_OF_BYTES_IN_EXTIOMSG_MESSAGE < sizeof(MotorDemandsStruct) )
   {
      // ERROR!!!!!!!
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR - TO NARROW QUEUE: xROVControlCommandQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }
   
   xExtIOHandlerCommandQueue  = xQueueCreate( EXT_IO_HANDLER_COMMAND_QUEUE_LEN, (unsigned portBASE_TYPE) sizeof(ExtIOMsgStructType) );
   if( ( xExtIOHandlerCommandQueue == INVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xExtIOHandlerCommandQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }

   
   
//   xCom2RXDQueue  = xQueueCreate( COM2_COM_QUEUE_RXD_LEN, (unsigned portBASE_TYPE) sizeof(signed portCHAR) );
//   if( ( xCom2RXDQueue == serINVALID_QUEUE ) )
//   {
//      // Some problem with the queue...
//      QueueInitilizeErrorDetected = TRUE;
//#ifdef  DEBUG_INCLUDED     // Debugging allowed?
//      dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xCom2RXDQueue!!!!!!!!!!!!\r\n" );
//#endif //DEBUG_INCLUDED    // Debugging allowed?
//   }
//
//
//   xCom2TXDQueue = xQueueCreate( COM2_COM_QUEUE_TXD_LEN + 1, (unsigned portBASE_TYPE) sizeof(signed portCHAR) );
//   if( ( xCom2TXDQueue == serINVALID_QUEUE ) )
//   {
//      // Some problem with the queue...
//      QueueInitilizeErrorDetected = TRUE;
//#ifdef  DEBUG_INCLUDED     // Debugging allowed?
//      dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xCom2TXDQueue!!!!!!!!!!!!\r\n" );
//#endif //DEBUG_INCLUDED    // Debugging allowed?
//   }
//
//
//   xCom1RXDQueue  = xQueueCreate( COM1_COM_QUEUE_RXD_LEN, (unsigned portBASE_TYPE) sizeof(signed portCHAR) );
//   if( ( xCom1RXDQueue == serINVALID_QUEUE ) )
//   {
//      // Some problem with the queue...
//      QueueInitilizeErrorDetected = TRUE;
//#ifdef  DEBUG_INCLUDED     // Debugging allowed?
//      dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xCom1RXDQueue!!!!!!!!!!!!\r\n" );
//#endif //DEBUG_INCLUDED    // Debugging allowed?
//   }


   xStorageTaskCommandQueue  = xQueueCreate( STORAGE_TASK_QUEUE_MSG_LEN, (unsigned portBASE_TYPE) sizeof( StorageTaskMsgStructType ) );
   if( ( xStorageTaskCommandQueue == INVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xStorageTaskCommandQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }
   

   xWDTCommandQueue  = xQueueCreate( WDT_QUEUE_COMMAND_LEN, (unsigned portBASE_TYPE) sizeof( WDTMsgStructType ) );
   if( ( xWDTCommandQueue == INVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xTWICommandQueue in vWDTTask()!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }

   
//   xStorageTaskCommandQueue  = xQueueCreate( STORAGE_TASK_QUEUE_MSG_LEN, (unsigned portBASE_TYPE) sizeof( StorageTaskMsgStructType ) );
//   if( ( xStorageTaskCommandQueue == INVALID_QUEUE ) )
//   {
//      // Some problem with the queue...
//      QueueInitilizeErrorDetected = TRUE;
//#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
//         dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xStorageTaskCommandQueue!!!!!!!!!!!!\r\n" );
//#     endif //DEBUG_INCLUDED    // Debugging allowed?
//   }


//   xStorageTaskAckQueue  = xQueueCreate( STORAGE_TASK_ACK_QUEUE_LEN, (unsigned portBASE_TYPE) sizeof(signed portCHAR) );
//   if( ( xStorageTaskAckQueue == INVALID_QUEUE ) )
//   {
//      // Some problem with the queue...
//      QueueInitilizeErrorDetected = TRUE;
//#ifdef  DEBUG_INCLUDED     // Debugging allowed?
////      dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xStorageTaskAckQueue!!!!!!!!!!!!\r\n" );
//#endif //DEBUG_INCLUDED    // Debugging allowed?
//   }

   xAdcQueue  = xQueueCreate( ADC_TASK_QUEUE_LEN, (unsigned portBASE_TYPE) sizeof(eCommandsAdc) );
   if( ( xAdcQueue == INVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
//      dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xStorageTaskAckQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }

   xHidQueue  = xQueueCreate( HID_TASK_QUEUE_LEN, (unsigned portBASE_TYPE) sizeof( HIDMsgStructType ) );
   if( ( xHidQueue == INVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
//      dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xStorageTaskAckQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }

   xStoredErrorsQueue  = xQueueCreate( STORED_ERRORS_QUEUE_LEN, (unsigned portBASE_TYPE) sizeof(eErrorCodes) );
   if( ( xStoredErrorsQueue == INVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
//      dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xStoredErrorsQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }


   // Server Web task queue for prototype software
   xSerialUDPTxTaskQueue  = xQueueCreate( SERIAL_UDP_TX_QUEUE_LEN, (unsigned portBASE_TYPE) sizeof( UDPMsgStructType ) );
   if( ( xSerialUDPTxTaskQueue == INVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
#     ifdef  DEBUG_INCLUDED     // Debugging allowed?
//      dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating xStorageTaskCommandQueue!!!!!!!!!!!!\r\n" );
#     endif //DEBUG_INCLUDED    // Debugging allowed?
   }

   
#ifdef  DEBUG_INCLUDED     // Debugging allowed?
   xDebugPortDataQueue  = xQueueCreate( DEBUG_PORT_QUEUE_LEN, (unsigned portBASE_TYPE) sizeof(UBYTE) );
   if( ( xDebugPortDataQueue == INVALID_QUEUE ) )
   {
      // Some problem with the queue...
      QueueInitilizeErrorDetected = TRUE;
   }
#endif //DEBUG_INCLUDED    // Debugging allowed?

   

   
   //---------------------------
   // FPGA UART 0-32 MSG QUEUES
   //---------------------------
   
   
   //------------------------------------------------
   // FPGA UART 0-7 MSG QUEUES IF BOARD 0 IS PRESENT
   //------------------------------------------------
   if( Board0Present == TRUE )
   {
      for( i=0 ; i<=7 ; i++ )
      {
         // FPGA UART0 MSG QUEUES
         if( ThisBoardRunsRovCtrlTask == TRUE && (i%4 > 1) ) continue;
             
         // Use different size of buffer for ControlBoard compared to OptionBoard!
         UartRxQueues[i] = xQueueCreate( ComQueRxLen, (unsigned portBASE_TYPE) sizeof(UBYTE) );
         if( ( UartRxQueues[i] == serINVALID_QUEUE ) )
         {
            // Some problem with the queue...
            QueueInitilizeErrorDetected = TRUE;
#           ifdef  DEBUG_INCLUDED     // Debugging allowed?
               dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating UartRxQueues[%d]!!!!!!!!!!!!\r\n", i );
#           endif //DEBUG_INCLUDED    // Debugging allowed?
         }
      
         // Use different size of buffer for ControlBoard compared to OptionBoard!
         UartTxQueues[i] = xQueueCreate( ComQueTxLen, (unsigned portBASE_TYPE) sizeof(UBYTE) );
         if( ( UartTxQueues[i] == serINVALID_QUEUE ) )
         {
            // Some problem with the queue...
            QueueInitilizeErrorDetected = TRUE;
#           ifdef  DEBUG_INCLUDED     // Debugging allowed?
               dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating UartTxQueues[%d]!!!!!!!!!!!!\r\n", i );
#           endif //DEBUG_INCLUDED    // Debugging allowed?
         }
      }
   } 
           
           
           
           
   //-------------------------------------------------
   // FPGA UART 8-15 MSG QUEUES IF BOARD 1 IS PRESENT
   //-------------------------------------------------
   if( Board1Present == TRUE )
   {
      // FPGA UART8 MSG QUEUES
      for( i=8 ; i<=15 ; i++ )
      {
         // FPGA UART0 MSG QUEUES
         if( ThisBoardRunsRovCtrlTask == TRUE && (i%4 > 1) ) continue;
         
         // Use different size of buffer for ControlBoard compared to OptionBoard!
         UartRxQueues[i] = xQueueCreate( ComQueRxLen, (unsigned portBASE_TYPE) sizeof(UBYTE) );
         if( ( UartRxQueues[i] == serINVALID_QUEUE ) )
         {
            // Some problem with the queue...
            QueueInitilizeErrorDetected = TRUE;
#           ifdef  DEBUG_INCLUDED     // Debugging allowed?
               dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating UartRxQueues[%d]!!!!!!!!!!!!\r\n", i );
#           endif //DEBUG_INCLUDED    // Debugging allowed?
         }
      
         // Use different size of buffer for ControlBoard compared to OptionBoard!
         UartTxQueues[i] = xQueueCreate( ComQueTxLen, (unsigned portBASE_TYPE) sizeof(UBYTE) );
         if( ( UartTxQueues[i] == serINVALID_QUEUE ) )
         {
            // Some problem with the queue...
            QueueInitilizeErrorDetected = TRUE;
#           ifdef  DEBUG_INCLUDED     // Debugging allowed?
               dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating UartTxQueues[%d]!!!!!!!!!!!!\r\n", i );
#           endif //DEBUG_INCLUDED    // Debugging allowed?
         }
      }
   }
   
   
   
   //-------------------------------------------------
   // FPGA UART 16-23 MSG QUEUES IF BOARD 2 IS PRESENT
   //-------------------------------------------------
   if( Board2Present == TRUE )
   {
      // FPGA UART16 MSG QUEUES
      for( i=16 ; i<=23 ; i++ )
      {
         // FPGA UART0 MSG QUEUES
         if( ThisBoardRunsRovCtrlTask == TRUE && (i%4 > 1) ) continue;
         
         // Use different size of buffer for ControlBoard compared to OptionBoard!
         UartRxQueues[i] = xQueueCreate( ComQueRxLen, (unsigned portBASE_TYPE) sizeof(UBYTE) );
         if( ( UartRxQueues[i] == serINVALID_QUEUE ) )
         {
            // Some problem with the queue...
            QueueInitilizeErrorDetected = TRUE;
#           ifdef  DEBUG_INCLUDED     // Debugging allowed?
               dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating UartRxQueues[%d]!!!!!!!!!!!!\r\n", i );
#           endif //DEBUG_INCLUDED    // Debugging allowed?
         }
      
         // Use different size of buffer for ControlBoard compared to OptionBoard!
         UartTxQueues[i] = xQueueCreate( ComQueTxLen, (unsigned portBASE_TYPE) sizeof(UBYTE) );
         if( ( UartTxQueues[i] == serINVALID_QUEUE ) )
         {
            // Some problem with the queue...
            QueueInitilizeErrorDetected = TRUE;
#           ifdef  DEBUG_INCLUDED     // Debugging allowed?
               dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating UartTxQueues[%d]!!!!!!!!!!!!\r\n", i );
#           endif //DEBUG_INCLUDED    // Debugging allowed?
         }
      }
   }

   
   
   
   //--------------------------------------------------
   // FPGA UART 24-31 MSG QUEUES IF BOARD 3 IS PRESENT
   //--------------------------------------------------
   if( Board3Present == TRUE )
   {
      // FPGA UART24 MSG QUEUES
      for( i=24 ; i<=31 ; i++ )
      {
         // FPGA UART0 MSG QUEUES
         if( ThisBoardRunsRovCtrlTask == TRUE && (i%4 > 1) ) continue;
         
         // Use different size of buffer for ControlBoard compared to OptionBoard!
         UartRxQueues[i] = xQueueCreate( ComQueRxLen, (unsigned portBASE_TYPE) sizeof(UBYTE) );
         if( ( UartRxQueues[i] == serINVALID_QUEUE ) )
         {
            // Some problem with the queue...
            QueueInitilizeErrorDetected = TRUE;
#           ifdef  DEBUG_INCLUDED     // Debugging allowed?
               dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating UartRxQueues[%d]!!!!!!!!!!!!\r\n", i );
#           endif //DEBUG_INCLUDED    // Debugging allowed?
         }
      
         // Use different size of buffer for ControlBoard compared to OptionBoard!
         UartTxQueues[i] = xQueueCreate( ComQueTxLen, (unsigned portBASE_TYPE) sizeof(UBYTE) );
         if( ( UartTxQueues[i] == serINVALID_QUEUE ) )
         {
            // Some problem with the queue...
            QueueInitilizeErrorDetected = TRUE;
#           ifdef  DEBUG_INCLUDED     // Debugging allowed?
               dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR when creating UartTxQueues[%d]!!!!!!!!!!!!\r\n", i );
#           endif //DEBUG_INCLUDED    // Debugging allowed?
         }
      }
   }   
}


//// For testing use a fixed MAC address
//#ifdef  TESTING
//#warning "Use REAL EmacEthAddr"
//const unsigned char InitEmacEthAddr[] = {0, 0xbd, 0x33, 0x06, 0x68, 0x22};
//#endif //TESTING



//#ifdef  TESTING
//************************************************************************************
//
//                  writeVariablesToFlash
//
//   This function writes RAM varables to flash if CRC checksum is wrong
//
//   Input:         -
//
//   Output:        -
//
//   Sets:          harware reg       CPU registers etc.
//
//************************************************************************************
void writeVariablesToFlash( void )
{

   // Using the structure since some functions require that...
   InternalVariablesStructType *varInFlash, *VarInRam;
   UBYTE *tmp;//, *tmp1, *tmp2;
   varInFlash = ( InternalVariablesStructType * ) STATIC_VAR_FLASH_ADDRESS;
   VarInRam = (InternalVariablesStructType *) &InternalVariables;
   tmp = (UBYTE *) &InternalVariables;

   //init the flash
   FLA_Initialize();

   // put in some constants if it isn't correct.
   // I'm using the InternalVariables since it should be empty still.
   memset( tmp, 0, sizeof(InternalVariables) );
   //memcpy( tmp, InitEmacEthAddr, sizeof(InitEmacEthAddr) );

   // create the crc field
   calculateInternalVariablesCRC(VarInRam);

   if ( ( checkInternalVariablesCRC( varInFlash ) != OK ) |
        ( compareInternalVariablesCRC( varInFlash, VarInRam ) != OK ) )
   {
      // Write the RAM data to flash
      if ( FLA_Write( STATIC_VAR_FLASH_ADDRESS, VarInRam, AT91C_IFLASH_PAGE_SIZE ) != OK )
      {
         // send notice
#ifdef  DEBUG_INCLUDED
          dbgprintf( "\r\n\n!!!!!!!!!!!!!W: FLA_Write failed in writeVariablesToFlash!!!!!!!!!!!!\r\n" );
#endif //DEBUG_INCLUDED    // Debugging allowed?
      }
      else
      {
         // Check if the CRC field in the flash mem is correct.
         if ( checkInternalVariablesCRC( varInFlash ) != OK )
         {
            // send notice
#ifdef  DEBUG_INCLUDED
             dbgprintf( "\r\n\n!!!!!!!!!!!!!W: checkInternalVariablesCRC failed in writeVariablesToFlash!!!!!!!!!!!!\r\n" );
#endif //DEBUG_INCLUDED    // Debugging allowed?
         }
      }
   }
}
//#endif //TESTING





//************************************************************************************
//
//                  GetVarDataFromFlash
//
//   This function extracts the internal variables from flash.
//
//   Input:         -
//
//   Output:        -
//
//   Sets:          -
//
//************************************************************************************
void GetVarDataFromFlash( void )
{
   InternalVariablesStructType *varInFlash;
   InternalVariablesStructType *varInFlashBackup;
   UINT32 i;

   varInFlash = ( InternalVariablesStructType * ) INTERNAL_VARIABLES_FLASH_ADDRESS;
   varInFlashBackup = ( InternalVariablesStructType * ) INTERNAL_VARIABLES_BACKUP_FLASH_ADDRESS;
   //init the flash
   FLA_Initialize();

   
//#ifdef  TESTING
//#warning "We should not write parameters to the flash in the finished program."
   writeVariablesToFlash();
//#endif


   
   // if this is the first time the Offshore V2 has been run.
   // The first run variable in both pages must be wrong for us to write default values there.
   if ( ( varInFlash->firstRun != INTERNAL_VAR_FIRST_RUN ) &&
        ( varInFlashBackup->firstRun != INTERNAL_VAR_FIRST_RUN ) )
   {
#ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!E: First time variables are written to flash in GetVarDataFromFlash!!!!!!!!!!!!\r\n" );
#endif //DEBUG_INCLUDED    // Debugging allowed?

      putDefaultValuesIntoIntVar();

      if( CopyInternalVariablesFromRAMToFlash( &InternalVariables ) != OK )
      {
         FlashInitilizeErrorDetected = TRUE;
#ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!E: GetVarDataFromFlash failed to write the internal variables to flash!!!!!!!!!!!!\r\n" );
#endif //DEBUG_INCLUDED    // Debugging allowed?

      }
      FlashFirstTimeInitilizeErrorDetected = TRUE;
      // Here we light RED LED as well as GREEN LED which is on by default.
      // Programming unit can here detect both LED's on end restart unit.
      forceBothLedsOn();
      
      // Format the dataflash memory
      DataFlash_Format();
#ifdef  DEBUG_INCLUDED     // Debugging allowed?
      dbgprintf( "\r\n\n!!!!!!!!!!!!!E: DataFlash is formated!!!!!!!!!!!!\r\n" );
#endif //DEBUG_INCLUDED    // Debugging allowed?

      
      // Give time to test equipment to sense and drop voltage before error written to log in memory
      // wait approximately two sec for power to stabilize
      for ( i=0; i<12000000; i++ );

   }
   else
   {
      if( CopyInternalVariablesFromFlashToRAM( &InternalVariables ) != OK )
      {
         FlashInitilizeErrorDetected = TRUE;
#ifdef  DEBUG_INCLUDED     // Debugging allowed?
         dbgprintf( "\r\n\n!!!!!!!!!!!!!E: GetVarDataFromFlash failed to get the internal variables!!!!!!!!!!!!\r\n" );
#endif //DEBUG_INCLUDED    // Debugging allowed?
      }
      
      if( InternalVariables.canOverEthernetBoardStackAddress1 == 0x00 &&
          InternalVariables.canOverEthernetBoardStackAddress2 == 0x00 )
      {
         // Error!  These should not be allowed - use default!
         InternalVariables.canOverEthernetBoardStackAddress1 = CAN_OVER_ETHERNET_ADDRESS1_DEFAULT_VALUE;
         InternalVariables.canOverEthernetBoardStackAddress2 = CAN_OVER_ETHERNET_ADDRESS2_DEFAULT_VALUE;
         //CopyInternalVariablesFromRAMToFlash( &InternalVariables ); // save the internal variables.
      }
   }
   
   // Get GripperForceFactor
   GripperForceFactor = InternalVariables.gripperForceFactor;
   if( GripperForceFactor > GRIPPER_FORCE_MAX_FACTOR )
   {
      GripperForceFactor = GRIPPER_FORCE_MAX_FACTOR;
   }
   if( GripperForceFactor < GRIPPER_FORCE_MIN_FACTOR )
   {
      GripperForceFactor = GRIPPER_FORCE_MIN_FACTOR;
   }   
  
   // A global variable for altimeterCruising should be introduced, meanwhile set internalvariable to off during startup.
   InternalVariables.altimeterCruising = OFF;
}


//************************************************************************************
//
//                  putDefaultValuesIntoIntVar
//
//   This function puts the default values into the RAM based InternalVariables
//   and calculates the crc.
//
//   Input:         -
//
//   Output:        -
//
//   Sets:          -
//
//************************************************************************************
void putDefaultValuesIntoIntVar( void )
{
   // make sure that there is absolutely nothing in the InternalVariables
   memset( &InternalVariables, 0x00, sizeof(InternalVariablesStructType) );

   //   InternalVariables.numberOfLoggedErrorCodes = NUMER_OF_LOGGED_ERROR_CODES_DEFAULT_VALUE;

   InternalVariables.firstRun = FIRST_RUN_DEFAULT_VALUE;

   InternalVariables.communicationCanOverEthernetAllowed = CAN_OVER_ETHERNET_ALLOWED_DEFAULT_VALUE;
   
   //   InternalVariables.MACAddress[0] = EMAC_MAC_ADDR0_DEFAULT_VALUE;
   //   InternalVariables.MACAddress[1] = EMAC_MAC_ADDR1_DEFAULT_VALUE;
   //   InternalVariables.MACAddress[2] = EMAC_MAC_ADDR2_DEFAULT_VALUE;
   //   InternalVariables.MACAddress[3] = EMAC_MAC_ADDR3_DEFAULT_VALUE;
   //   InternalVariables.MACAddress[4] = EMAC_MAC_ADDR4_DEFAULT_VALUE;
   //   InternalVariables.MACAddress[5] = EMAC_MAC_ADDR5_DEFAULT_VALUE;   // NOT USED!!! depends on board address.
      
   //   InternalVariables.emacIPAddr[0] = EMAC_IPADDR0_DEFAULT_VALUE;
   //   InternalVariables.emacIPAddr[1] = EMAC_IPADDR1_DEFAULT_VALUE;
   //   InternalVariables.emacIPAddr[2] = EMAC_IPADDR2_DEFAULT_VALUE;
   //   InternalVariables.emacIPAddr[3] = EMAC_IPADDR3_DEFAULT_VALUE;
   
   //   InternalVariables.emacNetMask[0] = EMAC_NETMASK0_DEFAULT_VALUE;
   //   InternalVariables.emacNetMask[1] = EMAC_NETMASK1_DEFAULT_VALUE;
   //   InternalVariables.emacNetMask[2] = EMAC_NETMASK2_DEFAULT_VALUE;
   //   InternalVariables.emacNetMask[3] = EMAC_NETMASK3_DEFAULT_VALUE;
   
   //   InternalVariables.emacGatewayAddr[0] = EMAC_GATEWAYADDR0_DEFAULT_VALUE;
   //   InternalVariables.emacGatewayAddr[1] = EMAC_GATEWAYADDR1_DEFAULT_VALUE;
   //   InternalVariables.emacGatewayAddr[2] = EMAC_GATEWAYADDR2_DEFAULT_VALUE;
   //   InternalVariables.emacGatewayAddr[3] = EMAC_GATEWAYADDR3_DEFAULT_VALUE;

   InternalVariables.whatBoardIsRunningThisStack         = WHAT_BOARD_POS_RUNS_THIS_STACK_DEFAULT_VALUE;
   InternalVariables.whatBoardStackAdrRunsRovCtrlTask    = WHAT_BOARDSTACK_RUNS_ROVCTRLTASK;
   InternalVariables.imuPort                             = IMU_PORT_DEFAULT_VALUE;
   InternalVariables.imuSettings                         = IMU_SETTINGS_DEFAULT_VALUE;
   InternalVariables.panTiltPort                         = PANTILT_PORT_DEFAULT_VALUE;
   InternalVariables.panTiltSettings                     = PANTILT_SETTINGS_DEFAULT_VALUE;
   InternalVariables.panTiltBoardStackAddress            = PANTILT_BOARDSTACK_ADDRESS_DEFAULT_VALUE;

#ifdef   PAN_TILT_SIDUS
   InternalVariables.panTiltPanSpeed                     = PANTILT_PANSPEED_SIDUS_DEFAULT_VALUE;
   InternalVariables.panTiltTiltSpeed                    = PANTILT_TILTSPEED_SIDUS_DEFAULT_VALUE;
#endif //PAN_TILT_SIDUS

#ifdef   PAN_TILT_KONGSBERG
   InternalVariables.panTiltPanSpeed                     = PANTILT_PANSPEED_KONGSBERG_DEFAULT_VALUE;
   InternalVariables.panTiltTiltSpeed                    = PANTILT_TILTSPEED_KONGSBERG_DEFAULT_VALUE;

   InternalVariables.panTiltPanMaxRight                  = PANTILT_MAXRIGHT_KONGSBERG_DEFAULT_VALUE;
   InternalVariables.panTiltPanMaxLeft                   = PANTILT_MAXLEFT_KONGSBERG_DEFAULT_VALUE;
   InternalVariables.panTiltTiltMaxDown                  = PANTILT_MAXDOWN_KONGSBERG_DEFAULT_VALUE;
   InternalVariables.panTiltTiltMaxUp                    = PANTILT_MAXUP_KONGSBERG_DEFAULT_VALUE;
#endif //PAN_TILT_KONGSBERG
   
   InternalVariables.depthMeasuringFpgaADport            = DEPTHMEAS_AD_PORT_DEFAULT_VALUE;
   InternalVariables.depthMeasAverageCalcInFPGA          = DEPTH_MEAS_AVERAGE_CALC_IN_FPGA_DEFAULT_VALUE;

   InternalVariables.altitudeMeasuringFpgaADport         = ALTITUDEMEAS_AD_PORT_DEFAULT_VALUE;
   
   InternalVariables.bowtechLEDBoardStackAddress         = BOWTECH_LED_BOARDSTACK_ADDRESS_DEFAULT_VALUE;
   InternalVariables.camera1FocusZoomIrisBoardStackAddress = CAM1_FOCUSZOOM_BOARDSTACK_ADDRESS_DEFAULT_VALUE;
   InternalVariables.camera3FocusZoomIrisBoardStackAddress = CAM3_FOCUSZOOM_BOARDSTACK_ADDRESS_DEFAULT_VALUE;
   InternalVariables.camera24VFeedBoardStackAddress      = CAM24V_BOARDSTACK_ADDRESS_DEFAULT_VALUE;
   InternalVariables.lasers24VFeedBoardStackAddress      = LASER24V_BOARDSTACK_ADDRESS_DEFAULT_VALUE;
   InternalVariables.servoRotateBoardStackAddress        = SERVO_ROTATION_BOARDSTACK_ADDRESS_DEFAULT_VALUE;
   
   InternalVariables.optionBoardFakeBoardNo              = FAKE_OPTIONBOARD_DEFAULT_VALUE;
      
   InternalVariables.altitudeMeasAverageCalcInFPGA       = ALTITUDE_MEAS_AVERAGE_CALC_IN_FPGA_DEFAULT_VALUE;
   
   InternalVariables.GUpumpMotorMaxVolt                  = GU_PUMP_MOTOR_MAX_VOLTAGE_DEFAULT_VALUE;
   InternalVariables.GUskidMotorVolt                     = GU_SKID_MOTOR_VOLTAGE_DEFAULT_VALUE;
   InternalVariables.GUmagasineSpeed                     = GU_SKID_ROTATOR_SPEED_DEFAULT_VALUE;

   InternalVariables.port1TxdOverEthAddress              = PORT_TXD_OVER_ETHERNET_DEFAULT_VALUE;
   InternalVariables.port1OverEthernetSettings           = PORT_OVER_ETHERNET_SETTINGS_DEFAULT_VALUE;
   InternalVariables.port1RxdOverEthAddress              = PORT_RXD_OVER_ETHERNET_ADDRESS_DEFAULT_VALUE;
   InternalVariables.port2TxdOverEthAddress              = PORT_TXD_OVER_ETHERNET_DEFAULT_VALUE;
   InternalVariables.port2OverEthernetSettings           = PORT_OVER_ETHERNET_SETTINGS_DEFAULT_VALUE;
   InternalVariables.port2RxdOverEthAddress              = PORT_RXD_OVER_ETHERNET_ADDRESS_DEFAULT_VALUE;
   InternalVariables.port3TxdOverEthAddress              = PORT_TXD_OVER_ETHERNET_DEFAULT_VALUE;
   InternalVariables.port3OverEthernetSettings           = PORT_OVER_ETHERNET_SETTINGS_DEFAULT_VALUE;
   InternalVariables.port3RxdOverEthAddress              = PORT_RXD_OVER_ETHERNET_ADDRESS_DEFAULT_VALUE;
   InternalVariables.port4TxdOverEthAddress              = PORT_TXD_OVER_ETHERNET_DEFAULT_VALUE;
   InternalVariables.port4OverEthernetSettings           = PORT_OVER_ETHERNET_SETTINGS_DEFAULT_VALUE;
   InternalVariables.port4RxdOverEthAddress              = PORT_RXD_OVER_ETHERNET_ADDRESS_DEFAULT_VALUE;
   InternalVariables.port5TxdOverEthAddress              = PORT_TXD_OVER_ETHERNET_DEFAULT_VALUE;
   InternalVariables.port5OverEthernetSettings           = PORT_OVER_ETHERNET_SETTINGS_DEFAULT_VALUE;
   InternalVariables.port5RxdOverEthAddress              = PORT_RXD_OVER_ETHERNET_ADDRESS_DEFAULT_VALUE;
   InternalVariables.port6TxdOverEthAddress              = PORT_TXD_OVER_ETHERNET_DEFAULT_VALUE;
   InternalVariables.port6OverEthernetSettings           = PORT_OVER_ETHERNET_SETTINGS_DEFAULT_VALUE;
   InternalVariables.port6RxdOverEthAddress              = PORT_RXD_OVER_ETHERNET_ADDRESS_DEFAULT_VALUE;
   InternalVariables.port7TxdOverEthAddress              = PORT_TXD_OVER_ETHERNET_DEFAULT_VALUE;
   InternalVariables.port7OverEthernetSettings           = PORT_OVER_ETHERNET_SETTINGS_DEFAULT_VALUE;
   InternalVariables.port7RxdOverEthAddress              = PORT_RXD_OVER_ETHERNET_ADDRESS_DEFAULT_VALUE;
   InternalVariables.port8TxdOverEthAddress              = PORT_TXD_OVER_ETHERNET_DEFAULT_VALUE;
   InternalVariables.port8OverEthernetSettings           = PORT_OVER_ETHERNET_SETTINGS_DEFAULT_VALUE;
   InternalVariables.port8RxdOverEthAddress              = PORT_RXD_OVER_ETHERNET_ADDRESS_DEFAULT_VALUE;

   InternalVariables.aliveMsgToPcuBoardStackAddress      = SEND_ALIVE_MSG_TO_PCU_BOARDSTACK_ADDRESS_DEFAULT_VALUE;
   InternalVariables.gripperBoardStackAddress            = GRIPPER_CONTROL_BOARDSTACK_ADDRESS_DEFAULT_VALUE;
   InternalVariables.NMEAPortOverEthernetAddress         = NMEA_PORT_OVER_ETHERNET_ADDRESS_DEFAULT_VALUE;
   InternalVariables.debugPortOverEthernetAllowed        = DEBUG_PORT_OVER_ETHERNET_ALLOWED_DEFAULT_VALUE;
      
   InternalVariables.canOverEthernetBoardStackAddress1   = CAN_OVER_ETHERNET_ADDRESS1_DEFAULT_VALUE;
   InternalVariables.canOverEthernetBoardStackAddress2   = CAN_OVER_ETHERNET_ADDRESS2_DEFAULT_VALUE;

   InternalVariables.trafoCompLevelBoardStackAddress     = TRAFO_COMP_LEVEL_BOARDSTACK_ADDRESS_DEFAULT_VALUE;

   InternalVariables.thrusterTurnedAround                = THRUSTER_TURNED_AROUND_DEFAULT_VALUE;
   InternalVariables.cameraSelectBoardStackAddress       = CAMERA_SELECT_BOARDSTACK_ADDRESS_DEFAULT_VALUE;

   InternalVariables.camera1Type                         = CAMERA1_TYPE_DEFAULT_VALUE;
   InternalVariables.camera3Type                         = CAMERA3_TYPE_DEFAULT_VALUE;
   InternalVariables.hdCameraPort                        = HD_CAMERA_PORT_DEFAULT_VALUE;

   InternalVariables.thrustPowerFactor                   = THRUST_POWER_LIMIT_PERCENTAGE_DEFAULT_VALUE;
   InternalVariables.thrustPowerFactorActive             = THRUST_POWER_LIMIT_ACTIVE_DEFAULT_VALUE;
   
   InternalVariables.reverseModeByAUX2Allowed            = REVERSE_MODE_BY_AUX2_DEFAULT_VALUE;
   InternalVariables.videoSwitchConfiguration            = VIDEO_SWITCH_CONFIG_DEFAULT_VALUE;
   
   InternalVariables.unusedINT16                         = NOT_USED_INT16_DEFAULT_VALUE;

   InternalVariables.depthDesiredValueFactor             = DEPTH_DESIRED_VALUE_DEFAULT_FACTOR;

   InternalVariables.depthCompensationValue              = DEPTH_COMP_DEFAULT_VALUE;

   InternalVariables.gripperForceFactor                  = GRIPPER_FORCE_DEFAULT_FACTOR;

   InternalVariables.ROVDataMsgType                      = ROV_DATA_MSG_TYPE_DEFAULT_VALUE;

   InternalVariables.SwModuleImuDataAsUdp                = SW_MODULE_IMU_DATA_AS_UDP_DEFAULT_VALUE;
   InternalVariables.SwModule2NotUsedYet                 = SW_MODULE_2_UNUSED_DEFAULT_VALUE;
   InternalVariables.SwModule3NotUsedYet                 = SW_MODULE_3_UNUSED_DEFAULT_VALUE;

   InternalVariables.AllowSwUpdate                       = ALLOW_SW_UPDATE_DEFAULT_VALUE;
   InternalVariables.ThrusterNewPBConnection             = THRUSTER_1_5_CONNECTION_DEFAULT_VALUE;
   InternalVariables.ThrusterSwap_26_and_48              = THRUSTER_SWAP_26_48_DEFAULT_VALUE;

   InternalVariables.ROVSerialNumberYearPart             = SERNO_YEAR_DEFAULT_VALUE;
   strcpy( &InternalVariables.ROVSerialNumberTypePart[0],  SERNO_TYPE_DEFAULT_VALUE );          
   InternalVariables.ROVSerialNumber                     = SERNO_NUMBER_DEFAULT_VALUE;

   InternalVariables.AnalogueCpProbeBoardStackAdr        = ANALOGUE_CP_PROBE_BOARD_STACK_ADR;
   InternalVariables.AnalogueCpProbeCalc                 = ANALOGUE_CP_PROBE_CALC_DEFAULT;
   InternalVariables.AnalogueCpProbePreAmpGain           = ANALOGUE_CP_PROBE_PREAMP_GAIN;

   
   // Special settings depending on CardAddress and type of board
   if( CardAddress == 0x05 )
   {
      // OptionBoard in ROV.
      if( CardType == OPTION_BOARD_CARD_TYPE )
      {
         // Double-check that this is correct type of board!
         InternalVariables.port1TxdOverEthAddress = PORT1_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port2TxdOverEthAddress = PORT2_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port3TxdOverEthAddress = PORT3_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port4TxdOverEthAddress = PORT4_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port5TxdOverEthAddress = PORT5_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port6TxdOverEthAddress = PORT6_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port7TxdOverEthAddress = PORT7_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port8TxdOverEthAddress = PORT8_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
      
         InternalVariables.port1RxdOverEthAddress = PORT1_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port2RxdOverEthAddress = PORT2_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port3RxdOverEthAddress = PORT3_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port4RxdOverEthAddress = PORT4_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port5RxdOverEthAddress = PORT5_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port6RxdOverEthAddress = PORT6_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port7RxdOverEthAddress = PORT7_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port8RxdOverEthAddress = PORT8_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;

         InternalVariables.whatBoardIsRunningThisStack = WHAT_BOARD_POS_RUNS_THIS_STACK_DEFAULT_VALUE_OPTBRD_0x05;
      }
   }
   else if( CardAddress == 0x0F )
   {
      // OptionBoard in SURFACE UNIT.
      if( CardType == OPTION_BOARD_CARD_TYPE )
      {
         // Double-check that this is correct type of board!
         InternalVariables.port1TxdOverEthAddress = PORT1_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port2TxdOverEthAddress = PORT2_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port3TxdOverEthAddress = PORT3_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port4TxdOverEthAddress = PORT4_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port5TxdOverEthAddress = PORT5_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port6TxdOverEthAddress = PORT6_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port7TxdOverEthAddress = PORT7_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port8TxdOverEthAddress = PORT8_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;

         InternalVariables.port1RxdOverEthAddress = PORT1_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port2RxdOverEthAddress = PORT2_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port3RxdOverEthAddress = PORT3_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port4RxdOverEthAddress = PORT4_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port5RxdOverEthAddress = PORT5_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port6RxdOverEthAddress = PORT6_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port7RxdOverEthAddress = PORT7_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port8RxdOverEthAddress = PORT8_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;

         InternalVariables.whatBoardIsRunningThisStack = WHAT_BOARD_POS_RUNS_THIS_STACK_DEFAULT_VALUE_OPTBRD_0x0F;
      }
   }
   else if( CardAddress == 0x06 )
   {
      // OptionBoard in ROV.
      if( CardType == OPTION_BOARD_CARD_TYPE )
      {
         // Double-check that this is correct type of board!
         // Double-check that this is correct type of board!
         InternalVariables.port1TxdOverEthAddress = PORT1_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port2TxdOverEthAddress = PORT2_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port3TxdOverEthAddress = PORT3_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port4TxdOverEthAddress = PORT4_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port5TxdOverEthAddress = PORT5_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port6TxdOverEthAddress = PORT6_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port7TxdOverEthAddress = PORT7_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port8TxdOverEthAddress = PORT8_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
      
         InternalVariables.port1RxdOverEthAddress = PORT1_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port2RxdOverEthAddress = PORT2_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port3RxdOverEthAddress = PORT3_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port4RxdOverEthAddress = PORT4_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port5RxdOverEthAddress = PORT5_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port6RxdOverEthAddress = PORT6_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port7RxdOverEthAddress = PORT7_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port8RxdOverEthAddress = PORT8_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;

         InternalVariables.whatBoardIsRunningThisStack = WHAT_BOARD_POS_RUNS_THIS_STACK_DEFAULT_VALUE_OPTBRD_0x06;
      }
   }
   else if( CardAddress == 0x01 )
   {
      // OptionBoard in ROV.
      if( CardType == OPTION_BOARD_CARD_TYPE )
      {
         InternalVariables.port1TxdOverEthAddress = PORT1_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port2TxdOverEthAddress = PORT2_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port3TxdOverEthAddress = PORT3_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port4TxdOverEthAddress = PORT4_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port5TxdOverEthAddress = PORT5_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port6TxdOverEthAddress = PORT6_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port7TxdOverEthAddress = PORT7_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port8TxdOverEthAddress = PORT8_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
      
         InternalVariables.port1RxdOverEthAddress = PORT1_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port2RxdOverEthAddress = PORT2_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port3RxdOverEthAddress = PORT3_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port4RxdOverEthAddress = PORT4_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port5RxdOverEthAddress = PORT5_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port6RxdOverEthAddress = PORT6_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port7RxdOverEthAddress = PORT7_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;
         InternalVariables.port8RxdOverEthAddress = PORT8_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x01;

         InternalVariables.whatBoardIsRunningThisStack = WHAT_BOARD_POS_RUNS_THIS_STACK_DEFAULT_VALUE_OPTBRD_0x01;
      }
   }
   else if( CardAddress == 0x04 )
   {
      // PowerBoard in ROV.
      if( CardType == POWER_BOARD_CARD_TYPE )
      {
         InternalVariables.whatBoardIsRunningThisStack = WHAT_BOARD_POS_RUNS_THIS_STACK_DEFAULT_VALUE_POWERBRD_0x04;
         InternalVariables.bowtechLEDBoardStackAddress = BOWTECH_LED_BOARDSTACK_ADDRESS_DEFAULT_VALUE_POWERBRD_0x04;
         InternalVariables.gripperBoardStackAddress    = GRIPPER_CONTROL_BOARDSTACK_ADDRESS_DEFAULT_VALUE_POWERBRD_0x04;
      }
   }
   else if( CardAddress == 0x00 )
   {
      // ControlBoard in ROV.
      if( CardType == CONTROL_BOARD_CARD_TYPE )
      {
         // Double-check that this is correct type of board!
      }
   }
   
   InternalVariables.altimeterCruising                   = OFF;
   InternalVariables.altimeterPort                       = ALTIMETER_PORT_DEFAULT_VALUE;   
   InternalVariables.altimeterSettings                   = ALTIMETER_SETTINGS_DEFAULT_VALUE;
   InternalVariables.altitude_Wanted                     = 0;
   InternalVariables.altValidThreshold                   = ALTIMETER_VALIDTHRESHOLD_DEFAULT_VALUE;
   InternalVariables.altHoldLastValid                    = ALTIMETER_HOLDLASTVALID_DEFAULT_VALUE;
   InternalVariables.altBufferSize                       = ALTIMETER_BUFFERSIZE_DEFAULT_VALUE;
   InternalVariables.maxAltimeterRange                   = ALTIMETER_RANGE_MAX_DEFAULT_VALUE;      
   InternalVariables.minAltimeterRange                   = ALTIMETER_RANGE_MIN_DEFAULT_VALUE;                           
   InternalVariables.workDepthBelowWarnLevel             = WORKDEPTH_WARNING_BELOW_DEFAULT_VALUE;
   InternalVariables.workDepthAboveWarnLevel             = WORKDEPTH_WARNING_ABOVE_DEFAULT_VALUE;     
   InternalVariables.altOnOffDeadBand                    = ALTIMETER_ONOFF_DEADBAND_DEFAULT_VALUE;      
   InternalVariables.altOnOffZFactor                     = ALTIMETER_ONOFF_Z_FACTOR_DEFAULT_VALUE;                           
   
   // create the crc field
   calculateInternalVariablesCRC(&InternalVariables);
}





//************************************************************************************
//
//                  putTestValuesIntoIntVar
//
//   This function puts the values used in production test into the RAM based 
//   InternalVariables and calculates the crc.
//
//   Input:         -
//
//   Output:        -
//
//   Sets:          -
//
//************************************************************************************
void putTestValuesIntoIntVar( void )
{


   // Special settings depending on CardAddress and type of board
   if( CardType == OPTION_BOARD_CARD_TYPE )
   {
      if( CardAddress == 0x05 )
      {
         // OptionBoard in ROV.
         InternalVariables.port1TxdOverEthAddress = PORT1_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port2TxdOverEthAddress = PORT2_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port3TxdOverEthAddress = PORT3_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port4TxdOverEthAddress = PORT4_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port5TxdOverEthAddress = PORT5_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port6TxdOverEthAddress = PORT6_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port7TxdOverEthAddress = PORT7_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;
         InternalVariables.port8TxdOverEthAddress = PORT8_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x05;

         InternalVariables.port1OverEthernetSettings = 3;  // 9600
         InternalVariables.port2OverEthernetSettings = 3;  // 9600
         InternalVariables.port3OverEthernetSettings = 3;  // 9600
         InternalVariables.port4OverEthernetSettings = 3;  // 9600
         InternalVariables.port5OverEthernetSettings = 3;  // 9600
         InternalVariables.port6OverEthernetSettings = 3;  // 9600
         InternalVariables.port7OverEthernetSettings = 3;  // 9600
         InternalVariables.port8OverEthernetSettings = 3;  // 9600
         
         InternalVariables.port1RxdOverEthAddress = 0x08;
         InternalVariables.port2RxdOverEthAddress = 0x0A;
         InternalVariables.port3RxdOverEthAddress = 0xF1;
         InternalVariables.port4RxdOverEthAddress = 0xF2;
         InternalVariables.port5RxdOverEthAddress = 0xF3;
         InternalVariables.port6RxdOverEthAddress = 0x0C;
         InternalVariables.port7RxdOverEthAddress = 0x0D;
         InternalVariables.port8RxdOverEthAddress = 0x0E;
      }
      else if( CardAddress == 0x0F )
      {
         // OptionBoard in SURFACE UNIT.
         InternalVariables.port1TxdOverEthAddress = PORT1_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port2TxdOverEthAddress = PORT2_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port3TxdOverEthAddress = PORT3_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port4TxdOverEthAddress = PORT4_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port5TxdOverEthAddress = PORT5_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port6TxdOverEthAddress = PORT6_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port7TxdOverEthAddress = PORT7_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port8TxdOverEthAddress = PORT8_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;

         InternalVariables.port1OverEthernetSettings = 2;  // 4800
         InternalVariables.port2OverEthernetSettings = 3;  // 9600
         InternalVariables.port3OverEthernetSettings = 3;  // 9600
         InternalVariables.port4OverEthernetSettings = 3;  // 9600
         InternalVariables.port5OverEthernetSettings = 0;  // not used
         InternalVariables.port6OverEthernetSettings = 0;  // not used
         InternalVariables.port7OverEthernetSettings = 0;  // not used
         InternalVariables.port8OverEthernetSettings = 0;  // not used
        
         InternalVariables.port1RxdOverEthAddress = PORT1_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port2RxdOverEthAddress = PORT2_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port3RxdOverEthAddress = PORT3_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port4RxdOverEthAddress = PORT4_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port5RxdOverEthAddress = PORT5_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port6RxdOverEthAddress = PORT6_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port7RxdOverEthAddress = PORT7_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
         InternalVariables.port8RxdOverEthAddress = PORT8_RXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x0F;
      }
      else if( CardAddress == 0x06 )
      {
         // OptionBoard 0x06 in ROV. Used in Plus-version of L3000
         InternalVariables.port1TxdOverEthAddress = PORT1_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port2TxdOverEthAddress = PORT2_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port3TxdOverEthAddress = PORT3_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port4TxdOverEthAddress = PORT4_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port5TxdOverEthAddress = PORT5_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port6TxdOverEthAddress = PORT6_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port7TxdOverEthAddress = PORT7_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;
         InternalVariables.port8TxdOverEthAddress = PORT8_TXD_OVER_ETHERNET_DEFAULT_VALUE_OPTBRD_0x06;

         InternalVariables.port1OverEthernetSettings = 3;  // 9600
         InternalVariables.port2OverEthernetSettings = 3;  // 9600
         InternalVariables.port3OverEthernetSettings = 3;  // 9600
         InternalVariables.port4OverEthernetSettings = 3;  // 9600
         InternalVariables.port5OverEthernetSettings = 3;  // 9600
         InternalVariables.port6OverEthernetSettings = 3;  // 9600
         InternalVariables.port7OverEthernetSettings = 3;  // 9600
         InternalVariables.port8OverEthernetSettings = 3;  // 9600
         
         InternalVariables.port1RxdOverEthAddress = 0x38;
         InternalVariables.port2RxdOverEthAddress = 0x39;
         InternalVariables.port3RxdOverEthAddress = 0x3A;
         InternalVariables.port4RxdOverEthAddress = 0x3C;
         InternalVariables.port5RxdOverEthAddress = 0x3D;
         InternalVariables.port6RxdOverEthAddress = 0x3E;
         InternalVariables.port7RxdOverEthAddress = 0x48;
         InternalVariables.port8RxdOverEthAddress = 0x49;
      }
      else if( CardAddress == 0x01 )
      {
         // OptionBoard 0x01 in ROV.
      }
   }
   else if( CardType == CONTROL_BOARD_CARD_TYPE )
   {
      if( CardAddress == 0x00 )
      {
      }
   }
   else if( CardType == POWER_BOARD_CARD_TYPE )
   {
      if( CardAddress == 0x04 )
      {
         // PowerBoard in ROV.
      }
   }
   
   // create the crc field
   calculateInternalVariablesCRC(&InternalVariables);
}




//************************************************************************************
//
//                  InitializeGlobals
//
//   This function initializes some globals
//
//   Input:         -
//
//   Output:        -
//
//   Sets:          global variables
//
//************************************************************************************
void InitializeGlobals( void )
{
UBYTE i;   

   UartsInitialized = FALSE;           // UARTS are NOT initialized yet!!
   RovControlTimer = 50;               // initlialize one global used in taskswitch code.
   IsEthernetUpAndRunningFlag = FALSE; // initlialize global flag.


   // reset all T300VStatusTable...
   for( i=0 ; i<NUMBER_OF_300V_OUTPUTS ; i++ )
   {
      T300VStatusTable[i].ThrusterNumber         = i+1;
      T300VStatusTable[i].FPGA15VOn              = FALSE;
      T300VStatusTable[i].ProtectionBoardNumber  = 0;
      if( Thruster300VControlProtBoardUsedTable[i] == (eCANBoardTypes)PROTECTIONBOARD_1 )
      {
         T300VStatusTable[i].ProtectionBoardNumber  = 1;
      }
      T300VStatusTable[i].ThrusterExpectedStatus = FALSE;
      T300VStatusTable[i].ThrusterFbStatus       = FALSE;
      T300VStatusTable[i].ThrusterVoltage        = 0;
      T300VStatusTable[i].ThrusterCurrent        = 0;
   }

   
// IMU_Set = Set_None;           // Initialize IMU to normal measurements, nothing going on.
// IMU_Set_Next = Set_Normal;    // Initialize IMU to normal measurements, nothing going on.

   RcSupportState          = StateStartup;          // init. global variable
   RequestChangeState      = NoStateChangeRequest;  // init. global variable
   ThrusterState           = StateThrusterFinished; // init. global variable
   ThrusterOldState        = StateThrusterFinished; // init. global variable
   SkidAndPumpState        = SkidAndPumpStartupState;// init. global variable
   timeToGetImuData        = FALSE;                 // init. global variable
   RequestThrusters300Von  = FALSE;                 // init. global variable
   LeakSensor0Value        = LEAK_SENSOR_LIMIT+1;   // init. global variable
   LeakSensor1Value        = LEAK_SENSOR_LIMIT+1;   // init. global variable
   ThrusterOnExtIoGlobal   = FALSE;                 // init. global variable
   indexInImuSendBuffer    = 0;                     // index pointer to ImuSendBuffer
   indexInImuRxBuffer      = 0;                     // index pointer to ImuReceiveBuffer
   checksumIMU             = 0;                     // 16bit checksum
   indexInPanoTiltRxBuffer = 0;                     // index pointer to PanoTiltSendBuffer
   numDataToSendToPanoTilt = 0;                     // Number of bytes to sen to PanoTiltSendBuffer
   indexInPanoTiltSendBuffer = 0;                   // index pointer to PanoTiltRxBuffer
   extIoRovControlMode     = ROV_CONTROL_DECKMODE;  // init. global variable
   RestartThrusterWhenStateMachineIsReady = FALSE;  // thuster problem found!
   RestartThrusterCounter  = 0;                     // thuster restart counter
   CameraSelected          = 1;                     // default camera
   AutoFocusTriggering     = FALSE;                 // init. global variable - no countdown initiated.
   AutoFocusDelay          = 0;                     // init. global variable - no countdown initiated.
   ReceivedBreakDetectedPort = -1;                  // Reset BREAK detection flag
   ReverseModeUsed         = FALSE;                 // init. global variable - not using reverse mode
   TextToOverlayAllowed    = TRUE;                  // init. global variable - allow text to overlay.
   DepthCompensationValue  = 0;                     // init. global variable
   HeadingCalibrateValue   = 0;                     // init. global variable
                
   NoFailedMessage = 0;
   NoSuccessMessage = 0;
   NoTimesNormalState = 0;
   NoPolls = 0;
#ifdef ALTIMETER_USED
   AltitudeMeasAverageCalc = INVALID_ALTITUDE;      // Init the altitude to the invalid value
   AltOutputOn = OFF;
   AltOutputHz = 1;
   AltCruisingEnable = OFF;
   AltitudeWanted = 0;
#endif //ALTIMETER_USED
   
#ifdef SIM
   SimDeltaDepth = 0.;
#endif   
   ReplyFromDcDcBrdVccTempToRovAnalyser   = FALSE;  // Should this board forward Vcc+Temp info from DC/DC board to ROV Analyser?
   ReplyFromProtcBrdVccTempToRovAnalyser  = FALSE;  // Should this board forward Vcc+Temp info from DC/DC board to ROV Analyser?
   ReplyFromFilterBrdVccTempToRovAnalyser = FALSE;  // Should this board forward Vcc+Temp info from DC/DC board to ROV Analyser?
   ReplyFromInterfaceVccTempToRovAnalyser = FALSE;  // Should this board forward Vcc+Temp info from DC/DC board to ROV Analyser?

   
   // keeps track of how many times this error message has been shown in debug window
   for( i=0 ; i<NUMBER_OF_ERROR_CODE_COUNTERS ; i++ )
   {
      ErrorCodeCounterStruct.ErrorCodeType[i] = ErrOk; // error message type
      ErrorCodeCounterStruct.ErrorCodeCounter[i] = 0;  // how many times this error message has been shown
   }   

   for( i = 0 ; i < NUMBER_OF_300V_OUTPUTS ; i++ )
   {
      T300VStatusTable[i].ThrusterType = 0;
      T300VStatusTable[i].ThrusterHardwareVersion = 0;
      T300VStatusTable[i].ThrusterSoftwareType = 'B';   // Bad
      T300VStatusTable[i].ThrusterSoftwareNo = 0;
      T300VStatusTable[i].ThrusterMinuteLogTime = 0;
      T300VStatusTable[i].ThrusterMinuteTotalTime = 0;
      T300VStatusTable[i].ThrusterTemperature = 0;
   }

   for( CardTempValueNum = 0 ; CardTempValueNum < NumCardTempValues ; CardTempValueNum++ )
   {
      CardTemperatures[CardTempValueNum] = 40; // 20 degrees
   }
   GlobalCardTemperature = 40; // 20 degrees
   
   ProtectionBoard_0_Sw = 4;  // R004 as default
   ProtectionBoard_1_Sw = 4;  // R004 as default
   
#ifdef TESTJOYSTICKCOMMAND  // CJN TESTS FOR DETECTIG UNEXPECTED JOYSTICK COMMANDS
   TimeToCheckJoystickCommand = FALSE;
#endif

#ifdef TEST_IMU_BAD_DATA          // CJN TESTS FOR CHECKING STRANGE/BAD IMU DATA
   TimeToCheckIMUData = FALSE;
#endif

}