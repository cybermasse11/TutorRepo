//************************************************************************************
//
//                  Ocean Modules Offshore V2
//
//                  main
//
//                  Ocean Modules 2010
//
//
//   Development environment: IAR Embedded Workbench IDE 5.1.0.417.7663
//                  IAR Assembler for ARM      5.50.0.51878
//                  IAR C/C++ Compiler for ARM 5.50.0.51878
//                  IAR Elf Linker for ARM     5.50.0.51878
//                            IAR Build Utility          5.1.0.417.7663
//
//*  Filename:      $Workfile: main.c $
//*  Revision:      $Revision: 19 $
//
//   Prepared by:    R&D Christer Johansson, 
//
//   Description:    main program for Offshore V2
//
//************************************************************************************
//	FreeRTOS.org V4.1.3 - copyright (C) 2003-2006 Richard Barry.
//
//	This file is part of the FreeRTOS.org distribution.
//
//	FreeRTOS.org is free software; you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation; either version 2 of the License, or
//	(at your option) any later version.
//
//	FreeRTOS.org is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	You should have received a copy of the GNU General Public License
//	along with FreeRTOS.org; if not, write to the Free Software
//	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//	A special exception to the GPL can be applied should you wish to distribute
//	a combined work that includes FreeRTOS.org, without being obliged to provide
//	the source code for any proprietary components.  See the licensing section
//	of http://www.FreeRTOS.org for full details of how and when the exception
//	can be applied.
//
//	***************************************************************************
//	See http://www.FreeRTOS.org for documentation, latest information, license
//	and contact details.  Please ensure to read the configuration and relevant
//	port sections of the online documentation.
//	***************************************************************************
//
//
// * Creates all the application tasks, then starts the scheduler.
// *
// * A task is also created called "uIP".  This executes the uIP stack and small
// * WEB server sample.  All the other tasks are from the set of standard
// * demo tasks.  The WEB documentation provides more details of the standard
// * demo application tasks.
// *
// * Main.c also creates a task called "Check".  This only executes every three
// * seconds but has the highest priority so is guaranteed to get processor time.
// * Its main function is to check the status of all the other demo application
// * tasks.  LED mainCHECK_LED is toggled every three seconds by the check task
// * should no error conditions be detected in any of the standard demo tasks.
// * The toggle rate increasing to 500ms indicates that at least one error has
// * been detected.
//************************************************************************************


//************************************************************************************
//
//              REVISION HISTORY
//
// Date:       Name:      Description:
//
// 2010-08-02  CJN        Started/Continued from Richard Berrys example code as above.
//
//************************************************************************************
/*
 * $History: main.c $
 * 
 * *****************  Version 19  *****************
 * User: Mats Ahlstrand Date: 17-07-07   Time: 14:23
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This is the versionof the altimeter steering algorithm extended with
 * hysteresis and ramping of the thrust
 * 
 * *****************  Version 12  *****************
 * User: Mats Ahlstrand Date: 15-11-12   Time: 10:31
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * This version is based on L3000 P047 Mögster with Altimeter cruising
 * support as in R048 for Baltic Offshore. 
 * 
 * *****************  Version 4  *****************
 * User: Christer Johansson Date: 14-03-11   Time: 15:07
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * GripperForce now uses 0-100% as incoming and outgoing value as well as
 * InternalVariables stored value.
 * 
 * *****************  Version 3  *****************
 * User: Christer Johansson Date: 14-02-25   Time: 9:29
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * Added support for accessing bootloader SoftwareRevision from system sw.
 * We start using bootloader sw rev as string.
 * 
 * *****************  Version 2  *****************
 * User: Christer Johansson Date: 14-02-25   Time: 8:33
 * Updated in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * ANIS: Bootloader revision support added.
 * 
 * *****************  Version 1  *****************
 * User: Christer Johansson Date: 13-09-06   Time: 14:00
 * Created in $/OM/ROV Software/Platform/SystemPgm/OM_APP
 * 
 * *****************  Version 4  *****************
 * User: Christer Johansson Date: 13-09-06   Time: 8:59
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Time to move R&D to new location on server....
 * 
 * *****************  Version 3  *****************
 * User: Christer Johansson Date: 13-04-18   Time: 15:03
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Added support for Limited Debug Menu
 * 
 * *****************  Version 2  *****************
 * User: Christer Johansson Date: 12-09-10   Time: 8:58
 * Updated in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * Divide priority for UDP TX and RX tasks to be able to optimize
 * receiving of UDP packages.
 * 
 * *****************  Version 1  *****************
 * User: Christer Johansson Date: 12-02-27   Time: 14:30
 * Created in $/OM/ROV Software/Platform/SystemProgram/OM_APP
 * 
 * *****************  Version 1  *****************
 * User: CJN      Date: 09-02-05   Time: 15:21
 * Created
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
//int main( void );


//************************************************************************************
//
//        INCLUDE FILES
//
//************************************************************************************
// Standard includes.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Scheduler includes.
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "constants.h"
#define DEFINE                      // This makes GLOBALS.h act different!
#include "globals.h"                // Here it really adds the wanted globals

// This file's own include
#include "defines.h"
#include "main.h"
#include "initialization.h"
#include "RTC.h"
//#include <stdbool.h>       // needed for WDT.h - included in globals.h
#include "WDT.h"
#include "ADC.h"
#include "HID.h"
#include "storageTask.h"
#include "flash.h"

#ifdef  DEBUG_INCLUDED     // Debugging allowed?
// Make it possible to use dbgprintf() etc to debug port
#include "debugprintf.h"
#endif //DEBUG_INCLUDED    // Debugging allowed?

// Application includes.
//#include "BasicWeb.h"
//#include "serial.h"
#include "PIOhandler.h"
#include "fatSd.h"
#include "service.h"
//#include "lib_twi.h"
#include "DataHandler.h"
#include "ROVControl.h"
#include "ExternalIOHandler.h"
#include "ConfigFile.h"
#include "ServerWebTask.h"
#include "DataFlash.h"

#ifdef LIMITED_TESTS		            // Only include if this is defined in defines.h
#include "limitedtest.h"
#endif //LIMITED_TESTS

#ifdef CJN_MAIN_TESTS		         // Only include if this is defined in defines.h
#include "cjnmaintest.h"
#endif // CJN_MAIN_TESTS		      // Only include if this is defined in defines.h

//#ifdef STORAGE_TASK_TESTS		   // Only include if this is defined in defines.h
//#include "storagetasktest.h"
//#endif // EMUL_ESCPOS_TESTS		   // Only include if this is defined in defines.h

// Linker script defines
extern char __ICFEDIT_intvec_start__;
extern char __ICFEDIT_region_ROM_start__;
extern char __ICFEDIT_region_ROM_end__;
extern char __ICFEDIT_region_RAM_start__;
extern char __ICFEDIT_region_RAM_end__;


//************************************************************************************
//
//                  main
//
//   This function initiates queues and hardware, starts all the other tasks and then starts the scheduler.
//
//   Input:         -
//
//   Output:        -
//
//   Sets:          harware reg       Calls all hw setup to be able to run
//
//************************************************************************************
int main( void )
{
   UINT32 i; 
   char *pVectSram;
   char *pVectFlash;
   int intVectSize;

   // get the data from the linker script
   intVectSize=(int)& __ICFEDIT_region_ROM_start__ - (int)& __ICFEDIT_intvec_start__;
   pVectSram = ( char *) (int)& __ICFEDIT_region_RAM_start__ - intVectSize;
   pVectFlash = ( char *) (int)& __ICFEDIT_intvec_start__;

   pBootloaderSoftwareRevision = ( char *) BOOTLOADER_REVISION_ADDRESS;
   
   // Check if there's a bootloader revision at BOOTLOADER_REVISION_ADDRESS
   // If not, the revision should be set to R003.
   // Format for the revision string is Rxxx'\0' where xxx is the revision number.
   strcpy( bootloaderSwRevision, "R003" );   // default value
   bootloaderSoftwareRevision = 3;           // default value
   
   if((*pBootloaderSoftwareRevision =='R') ||
      (*pBootloaderSoftwareRevision =='P'))
   {
      if ( (*(pBootloaderSoftwareRevision+1) >= '0') && (*(pBootloaderSoftwareRevision+1) <= '9') )
      {
         if ( (*(pBootloaderSoftwareRevision+2) >= '0') && (*(pBootloaderSoftwareRevision+2) <= '9') )
         {
            if ( (*(pBootloaderSoftwareRevision+3) >= '0') && (*(pBootloaderSoftwareRevision+3) <= '9') )
            {
               if ( *(pBootloaderSoftwareRevision+4) == '\0' )
               {
                  bootloaderSoftwareRevision = ( *(pBootloaderSoftwareRevision+1) - 0X30 ) *100;
                  bootloaderSoftwareRevision += ( *(pBootloaderSoftwareRevision+2) - 0X30 ) *10;
                  bootloaderSoftwareRevision += ( *(pBootloaderSoftwareRevision+3) - 0X30 );
                  
                  strcpy( bootloaderSwRevision, pBootloaderSoftwareRevision);
               }
            }
         }
      }
   }
   
   // initialize some global variables to their default values.
   InitializeGlobals();
   
   for (i=0; i<intVectSize; i++)
   {
      *pVectSram = *pVectFlash;
      pVectFlash++;
      pVectSram++;
   }
   
   AT91C_BASE_MC->MC_RCR = AT91C_MC_RCB;

   ONE_AND_A_HALFSEC_DELAY;

  
   // Configure the processor.
   prvSetupHardware();

   // MOVE THIS TO AFTER WE INITIALIZE FPGA
   // create all the FREERTOS queues
   // prvSetupQueues();


#ifdef  DEBUG_INCLUDED     // Debugging allowed?
   // Make it possible to use dbgprintf() etc to debug port
   dbgSetupHardware();
#endif //DEBUG_INCLUDED    // Debugging allowed?




   crcInit();
   
   // Make sure that the size of the structure containing the internal variables is
   // exactly one page in flash.
   // If not, halt the processor.
   // make sure crc-ccitt is aligned last in the structure.
   if ( ( (UBYTE *)&InternalVariables.crcCcitt - (UBYTE *)&InternalVariables ) != ( AT91C_IFLASH_PAGE_SIZE - 4 ) )
   {
#ifdef  DEBUG_INCLUDED
      dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR, the crc value in InternalVariablesStructType is not aligned correctly. crc-ccitt is aligned at %d (252)!!!!!!!\r\n", ( (UBYTE *)&InternalVariables.crcCcitt - (UBYTE *)&InternalVariables ) );
      dbgprintf( "sizeof InternalVariablesStructType is %d!!!!!!!\r\n", sizeof(InternalVariables) );
#endif //DEBUG_INCLUDED

      // turn on the red light when everytning has been saved.
      forceRedLedOn();
      // halt the processor
      while(1);
   }

  
   
#ifdef  DEBUG_INCLUDED
   dbgprintf("\r\n=========================================================\r\n");
   dbgprintf("AT91SAM7X is starting up in main()\r\n");
   dbgprintf( "Booloader revision %s.\r\n", bootloaderSwRevision );
#endif //DEBUG_INCLUDED
   
   
   // initialize parallel ports directly attached to MCU IO-pins
   xInternalIOPortInit();

   // initialize the dataflash
   DataFlash_Init();

   
   // get the internal variables from internal Flash page.
   GetVarDataFromFlash();  // Note! Uses LEDs initialized in xInternalIOPortInit()

   // Initialize the FPGAs and get board id, board stack, board type etc.
   InitializeFPGAandGetGlobals();

   // MOVE QUEUE CREATION TO AFTER WE INITIALIZE FPGA
   // THEN WE HAVE CONTROL OVER WHAT TO RUN.
   // create all the FREERTOS queues
   prvSetupQueues();

   
   // Initialize the WDT task (must be done before using the ErrorReporting function)
   // This has been moved here so the error reporting mutex is initialized before being used
   WDTTaskCreate( WDT_TASK_PRIORITY );

   // Time to read the config file from dataflash
   ConfigFile_StartupRead();     // Read config file
   UpdateConfigFileRead( true );       // Mark config file as read
  
   // Start the task that handles external IO ports via FPGA functionality.
   ExtIOHandlerTask();

   // Start the task that handles the Data Handler functionality.
   DataHandlerTask();

   // initialize the ADC task
   ADCTaskCreate( ADC_TASK_PRIORITY );

   // initialize the TWI task
//   TWITaskCreate( TWI_TASK_PRIORITY );

   // initialize the TWC task
   RTCTaskCreate( RTC_TASK_PRIORITY );

   // initialize the HID task
   HIDTaskCreate( HID_TASK_PRIORITY );

   
//   // If the config data could not be read, there is no use in starting TCP/IP or storage task
//   if (ConfigFileData != NULL)
//   {
//#ifdef OMFS_TEST
//      // COMMENT: No lwIP and no StorageTask during OMFS testing
//#else
//      // Start the task that handles the TCP/IP and WEB server functionality.
//      lwIPTaskCreate(mainWEBSERVER_PRIORITY);

      // Start the task that handles UDP communication.
      UdpRxTxTaskCreate(UDP_TX_TASK_PRIORITY, UDP_RX_TASK_PRIORITY);
      
//#endif
//   }

//#warning "Storage task is not started!"   
	// Start the storage task.
   StorageTaskCreate( STORAGE_TASK_PRIORITY );
   // DataFlashTaskCreate( MAIN_CJN_TEST_TASK_PRIORITY );
	// Start the application tasks.
   Service();  // Start the service task
   
   // Only start queues and task handling ROV Contrl if this is the board that is supposed to run it
   if( ThisBoardRunsRovCtrlTask == TRUE )
   {
      // Start the task that handles the ROV Control functionality.
      ROVControlTask();
   }   

#ifdef LIMITED_TESTS		            // Only include if this is defined in defines.h
   LimitedTests();                  // Start the Limited Tests task
#endif //LIMITED_TESTS		         // Only include if this is defined in defines.h
   
#ifdef CJN_MAIN_TESTS		         // Only include if this is defined in defines.h
   CjnMainTests();                  // Start the Cjn Main Tests task
#endif // CJN_MAIN_TESTS		      // Only include if this is defined in defines.h

#ifdef JOS_MAIN_TESTS		         // Only include if this is defined in defines.h
   JosMainTests();                  // Start the Jos Main Tests task
#endif // JOS_MAIN_TESTS		      // Only include if this is defined in defines.h

   
   // The adc is started before all tasks are started just to make sure it's up and running for the wdt task.
   // We've waited this long just to make sure that the power is stable.
   initAdcMeasurement();

   
	// Now all the tasks have been started - start the scheduler.
	vTaskStartScheduler();
   
#ifdef  DEBUG_INCLUDED     // Debugging allowed?
   dbgprintf( "\r\n\n!!!!!!!!!!!!!ERROR SCHEDULER FAILED. ENDING HERE!!!!!!!!!!!!\r\n" );
#endif //DEBUG_INCLUDED    // Debugging allowed?
   
	// Should never reach here because the tasks should now be executing!
	return 0;
}

