/******************************************************************************\
** 	            Real-Time Acoustic Observing System
**
** RAOS Top Version 3.x Controls
**
** -  Controls the surface buoy electronics.
** -  Interface via Acoustic Modem to RAOS Bottom System
** -  Compiles Bottom detections and system info for upload.
** -  Connects to RUDICS Basestation through NAL Iridium Modem A3LA-RG
**    >  Data included in upload file includes system vitals and controllable
**       system variables for bottom side recording.
**    >  Variables can be updated by sending a command string from RUDICS
**       Basestation defined inside the parathesis of the follwoing string:
**
**       $$$RAOS(xxxxx)***
**
**    >  These variables are controlled by sending the characters listed
**       below followed by up to 5 numbers (including a '.' for min sys volt)
**       **Note: No "0" zero digit holders necessary
**
**       I:    Detection Interval period to inquiry WISPR. Defined in minutes
**       A:    AModem Call interval. rounded up depending on Detection Interval
**       D:    Number of detections to return per detection call.
**             > These will be save to the .dtx file on bottom and appended to
**               the system info before being uploaded to the Top System.
**       S:    Maximum number of Startups
**       v:    Minimum System Voltage (Absolute Minimum 10.5volts for AModem)
**       N:    The Number of Detections for RAOSBottom to induce an AModem Call
**       G:    WISPR Gain
**       C:    DutyCycle giving in integer to be represented as percentage (0 ->
100)
**             >  A DETINT of 60 minutes would produce ADCounter==70;
**             >  The first half of the ADCounter (35 counts) the WISPR would be
left off
**             >  The second half of that 60 minute Detection Interval WISPR
would be on
**
**    >  Implement start modes for WISPR.
**       -  WISPR Requests more parameters at start such as recording mode.
**
**
**	August 2015  Alex Turpin RAOSVersion 3.4
**
** -  Implement (near) Real-Time Detections by Calling to land after # of
detections
**       (parameter 'N') is surpassed by the number of returned detecions during
one
**       Detection interval cycle (Parameter 'I').
**    >  This should also mean quicker updating of parameters to the Bottom
system
**       including GPS time. RAOSBottom leaves WISPR off and AModem ON during
RAOSTop
**       Iridium call, waiting for Top to call Bottom, timing out after certain
wait period
**
**
** -  Setup MINVOLT program cutoff
**    >  Different levels of power savings
**       1. No WISPR (RAOSBottom Only)
**       2. No AModem
**          >  Minimum working system voltage: 10.5V
**       3. No Iridium (RAOSTop only)
**       4. Low Power Hibernation
**          >  Less than 10.5 for Bottom
**          >  Less than 8? for Top
**
** -  Send Most recent file first followed by oldest unsent file
**    >  This would gives us an idea of what is happening right now. Plus, we
need to
**       see what detection are being returned if those detections are invoking
this call.
**
**
** September 2015 Alex Turpin RAOSVersion 3.5
**
** - Bug Fixes:
**
** January 2016 Alex Turpin RAOSVersion 4.0
**
**    Implementation of MPC Common Files Version 3.0
**    Move from numered log file to activity.log
**    Implementation of BT Module
**
**	May 2016 Alex Turpin RAOSVersion 4.1
**
**		Removal of TUPORT.h & .c
**		Improving Bluetooth & Powerlogging options
**		Multi-File AModem Uploads.
**
*******************************************************************************
\*****************************************************************************/

#include <assert.h>
#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <locale.h>
#include <math.h>
#include <setjmp.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <ADS.h>
#include <MPC_Global.h>
#include <Settings.h> // For VEEPROM settings definitions
#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <fcntl.h>    // PicoDOS POSIX-like File Access Definitions
#include <stat.h>     // PicoDOS POSIX-like File Status Definitions
#include <termios.h>  // PicoDOS POSIX-like Terminal I/O Definitions
#include <unistd.h>   // PicoDOS POSIX-like UNIX Function Definitions
#ifdef BLUETOOTH
#include <BT.h>
#endif
#include <AModem.h>
#include <PLATFORM.h>

#include <GPSIRID.h>

#define TICKS_TO_MS(x) ((ushort)(x / (CFX_CLOCKS_PER_SEC / 1000)))

SystemParameters MPC;

#ifdef IRIDIUM
extern IridiumParameters IRID;
#endif
#ifdef ACOUSTICMODEM
extern AMODEMParameters AMDM;
#endif

/// WDT Watch Dog Timer definition
short CustomSYPCR = WDT419s | HaltMonEnable | BusMonEnable | BMT32;
#define CUSTOM_SYPCR CustomSYPCR // Enable watch dog  HM 3/6/2014

// LOCAL FUNCTIONS
void InitializeRAOS(ulong *);
void Incoming_Data();
void Console(char);
void Sleep_AModem();
int WriteFile(ulong TotalSeconds);
void SleepUntilWoken();
void Vitals(short);
void StreamLoop(short);
static void IRQ5_ISR(void);
static void IRQ3_ISR(void);
static void IRQ4_ISR(void);
IEV_C_PROTO(ExtFinishPulseRuptHandler);

// LOCAL VARIABLES
bool SystemOn;
ushort SysTimeInt;
short LOGTOCON = 1; // Log activity to a logfile. flogf can be used.
bool DataAlarm;
bool SelfInduced;
int AModemPhase;

ushort SysTimeInt;
int ActiveAModemCounter;

// RAOS4.0 Variables Added
char *WriteBuffer;
bool BT_LP;
long SystemFreeSpace;
static char uploadfile[] = "c:00000000.dat";

/******************************************************************************\
**	Main
\******************************************************************************/
void main() {

  ulong TimeOn;
  char *string = {""};
  // int ret;
  ulong PwrOnTime = 0, PwrOffTime = 0;
  bool init_flag = false;
  int count = 0;
  char filenum[] = "00000000";

  // Force AModem Off at Power up
  PIOClear(AMODEMPWR);

  // Setup System Functions
  InitializeRAOS(&PwrOnTime);
  init_flag = true; // Log File Created flag

  // Program Main While Loop
  while (SystemOn) {
    while (!BT_LP) {

      // Post Iridium File Transfer.
      if (DataAlarm && !SelfInduced) {
        AModemSend("ACK");
        AModemPhase = 5;

        count = ADCounter + 4;
        while (DataAlarm && ADCounter < count) {
          Sleep_AModem();

          Incoming_Data();

          if (ADCounter == (count - 1))
            AModemResend();
        }
      }

      if (init_flag == false) {
        Free_Disk_Space();
        Reset_ADCounter();
        DataTimer = 0;
        MPC.FILENUM++;
        sprintf(filenum, "%08ld", MPC.FILENUM);
        VEEStoreStr(FILENUM_NAME, filenum);
        DataAlarm = false; // Reassert AModem Alarm
        SelfInduced = false;
      }

      // sprintf(&uploadfile[2], "%08ld.dat", MPC.FILENUM);

      // Check Detection and Data Transfer Intervals, and WISPR Duty Cycle
      Check_Timers(Return_ADSTIME());

      AModemPhase = 1;

      while (!DataAlarm && AModemPhase < 6) {
        Sleep_AModem();

        Incoming_Data();

        if (System_Timer() == 2) {
          SelfInduced = true;
          DataAlarm = true;
        }
      }

      if (SystemOn) {
        flogf("\n%s|Data Transfer Interval!", Time(NULL));
        RTCDelayMicroSeconds(
            5000000L); // Wait to make sure Bottom gets last DONE sent
      }

      // Get time in seconds since start of program/end of Iridium call
      Time(&PwrOffTime);
      TimeOn = PwrOffTime - PwrOnTime;

      flogf("\n\t||sprintf");
      sprintf(&uploadfile[2], "%08ld.dat", MPC.FILENUM);
      flogf(": %s", uploadfile);
      cdrain();
      coflush();

      // Write Upload File
      WriteFile(TimeOn);
      DOS_Com("del", MPC.FILENUM, "PWR", NULL);

      OpenTUPort_AModem(false);
      if (!SystemOn)
        break;

      // Start A-D power logging
      Setup_ADS(true, MPC.FILENUM + 1);

      Time(&PwrOnTime);

      if (SystemOn)
        IRIDGPS();

      // AModem_Init(); removed V4.1 (integrated into OpenTUPort_AModem();)
      OpenTUPort_AModem(true);
      AModemPhase = 4;

      init_flag = false;
    }
#ifdef BLUETOOTH
    if (BT_LP)
      ret = Bluetooth_LowPower();
    if (ret == 1)
      BT_LP = false;
    else if (ret == 2)
      BIOSReset();
    else if (ret == 3)
      SystemOn = false;
#endif
  }

  TURelease();

  // Low Power Hibernation Mode
  SleepUntilWoken();

} //____ Main() ____//

/*********************************************************************************\
** InitializeRAOS
\*********************************************************************************/
void InitializeRAOS(ulong *PwrOn) {

  ulong time = NULL;
  bool check = false;

  // Define unused pins here
  uchar mirrorpins[] = {1,  15, 16, 17, 18, 19, 24, 25, 26,
                        28, 31, 32, 35, 36, 37, 42, 0};

  PIOMirrorList(mirrorpins);
  // Get the system settings running
  SetupHardware();

  // Initialize and open TPU UART
  TUInit(calloc, free);
  RTCDelayMicroSeconds(100000L);

#ifdef BLUETOOTH
  Bluetooth_Power(true);
#endif
  // Count Down Timer to Start Program or go to PICO dos/Settings
  PreRun();

  GetSettings();

  // Identify IDs
  flogf("\nProject ID: %s   Platform ID: %s   Start ups: %d", MPC.PROJID,
        MPC.PLTFRMID, MPC.STARTUPS);

  // Get Power On Time
  Time(&time);
  *PwrOn = time;

#ifdef POWERLOGGING
  // Turn on A to D Logging for Power Consumption
  Setup_ADS(true, MPC.FILENUM);
#endif
  PinClear(21);
#ifdef BLUETOOTH
  if (BT_LP)
    Bluetooth_LowPower();

  BT_LP = false;
  Bluetooth_Power(false);
#endif

  WriteBuffer = (char *)malloc(256);

  // Parse Startup Pameters from System.prm file
  // ParseStartupParams();//only parse new params

  if (MPC.STARTUPS == 0)
    Make_Directory("SNT");

  // Get Free Disk Space
  SystemFreeSpace = Free_Disk_Space(); // Does finding the free space of a large
                                       // CF card cause program to crash? or
                                       // Hang?

#ifdef POWERLOGGING
  flogf("\n\t|Check Startup Voltage: %5.2f", Voltage_Now());
#endif

  // Store Number of Bootups
  MPC.STARTUPS++;
  VEEStoreShort(STARTUPS_NAME, MPC.STARTUPS);

  DataAlarm = false;
  SelfInduced = false;
  SystemOn = true;
  TickleSWSR();

  Reset_ADCounter();
  BT_LP = false;
  // AModem_Init();
  // Turn on AModem
  OpenTUPort_AModem(true);

} //____ InitializeAUH() ____//

/****************************************************************************\
** void Incoming_Data()
\****************************************************************************/
void Incoming_Data(void) {
  bool incoming = true;
  int amodemreturn = -1;
  int count = 0;

  while (incoming) {

    AD_Check();

    // Get anything in AModem Queue
    if (tgetq(AModemPort)) {
      amodemreturn = AModem_Data();

      if (amodemreturn == 2 && AModemPhase == 3) {
        StreamLoop(amodemreturn);

      } else if (amodemreturn == 2 && AModemPhase == 6) {
        AD_Check();
        RTCDelayMicroSeconds(500000L);
        DataAlarm = false;
        Delay_AD_Log(3);
        TURxFlush(AModemPort);
        TUTxFlush(AModemPort);

      }

      else if (amodemreturn == -4 &&
               AModemPhase == 4) { // only comes here when serial data received.
                                   // Go to AModemStream.
        AModemSend("RESEND");
        StreamLoop(2);
      } else if (amodemreturn == -6 && AModemPhase == 6) {
        count++; // if AModem_Data() continues to return -6
        if (count > 4) {
          SelfInduced = false;
          DataAlarm = true;
        }
      }
#ifdef DEBUG
      else
        flogf("\n\t|AModemReturn: %d\n\t|AModemPhase: %d", amodemreturn,
              AModemPhase);
#endif
      incoming = true;
    }

    else if (cgetq()) {
      Console(cgetc());
      incoming = true;
    }
#ifdef BLUETOOTH
    // Bluetooth Check if on
    else if (BT_On) {
      if (tgetq(BTPort)) {
        flogf("\nBT_Q: %d", tgetq(BTPort));
        Bluetooth_Interface();
        incoming = true;
      } else {
        flogf("nothing in BTPORT");
        incoming = false;
      }
    }
#endif

    else
      incoming = false;
  }

} //_____ Incoming_SeaGlider_Data() _____//

/************************************************************************************************************************\
** void Console      Testing Opportunity for Debugging
\************************************************************************************************************************/
void Console(char in) {
#ifdef DEBUG
  flogf("Incoming Char: %c\n", in);
  putflush();
  CIOdrain();
#endif
  RTCDelayMicroSeconds(2000L);

  switch (in) {
  case '1':
    SystemOn = false;
    DataAlarm = DataAlarm ? false : true;
    flogf("\nSystem Off...");
    break;

  case 'A':
    DataAlarm = DataAlarm ? false : true;
    SelfInduced = true;
    break;
  /*
     case 'b':
     TUTxPrintf(AModem, "~");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "\%");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "L");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "Q");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "U");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "W");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "M");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, ".");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "C");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "O");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "M");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "D");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "6");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "1");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "0");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "1");
     break;

     case 'B':

     TUTxPrintf(AModem, "~");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "\%");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "L");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "Q");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "U");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "W");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "M");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, ".");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "C");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "O");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "M");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "D");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "6");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "1");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "0");RTCDelayMicroSeconds(10000);
     TUTxPrintf(AModem, "4");
     break;
   */

  case 'H':
  case 'h':
    AModem_SetPower(true);
    break;

  case 'L':
  case 'l':
    AModem_SetPower(false);
    break;
  }

  return;
}

/******************************************************************************\
**	SleepUntilWoken		Finish up
**
** 1-st release 9/14/99
** 2nd release 6/24/2002 by HM -Changed to use ADS8344/45
** 3rd release 2014 AT
\******************************************************************************/
void Sleep_AModem(void) {

  RTCDelayMicroSeconds(5000L);
  IEVInsertAsmFunct(IRQ3_ISR, level3InterruptAutovector); // AModem Interrupt
  IEVInsertAsmFunct(IRQ3_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ4_ISR, level4InterruptAutovector); // Console Interrupt
  IEVInsertAsmFunct(IRQ4_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ5_ISR,
                    level5InterruptAutovector); // BT (PAM Port) Intterupt
  IEVInsertAsmFunct(IRQ5_ISR, spuriousInterrupt);

  SCITxWaitCompletion();
  EIAForceOff(true);
  CFEnable(false);

  TickleSWSR(); // another reprieve

  PinBus(IRQ3RXX); // AModem Interrupt
  while ((PinTestIsItBus(IRQ3RXX)) == 0)
    PinBus(IRQ3RXX);
  PinBus(IRQ5);
  while ((PinTestIsItBus(IRQ5)) == 0)
    PinBus(IRQ5);
  PinBus(IRQ4RXD); // Console Interrupt
  while ((PinTestIsItBus(IRQ4RXD)) == 0)
    PinBus(IRQ4RXD);

  while (PinRead(IRQ5) && PinRead(IRQ3RXX) && PinRead(IRQ4RXD) && !data)
    LPStopCSE(FastStop);

  EIAForceOff(false); // turn on the RS232 driver
  CFEnable(true);     // turn on the CompactFlash card
  // CTMRun(true);

  PIORead(IRQ3RXX);
  while ((PinTestIsItBus(IRQ3RXX)) != 0)
    PIORead(IRQ3RXX);
  PIORead(IRQ5);
  while ((PinTestIsItBus(IRQ5)) != 0)
    PIORead(IRQ5);
  PIORead(IRQ4RXD);
  while ((PinTestIsItBus(IRQ4RXD)) != 0)
    PIORead(IRQ4RXD);

  RTCDelayMicroSeconds(5000L);

} //____ Sleep_AModem() ____//

/*************************************************************************\
**  static void Irq5ISR(void)
\*************************************************************************/
static void IRQ5_ISR(void) {
  PinIO(IRQ5);
  RTE();
} //____ Irq3ISR ____//

/*************************************************************************\
**  static void Irq3ISR(void)
\*************************************************************************/
static void IRQ3_ISR(void) {
  PinIO(IRQ3RXX);
  RTE();
} //____ Irq3ISR ____//

/*************************************************************************\
**  static void IRQ4_ISR(void)
\*************************************************************************/
static void IRQ4_ISR(void) {
  PinIO(IRQ4RXD);
  RTE();
} //____ Irq4ISR() ____//

/******************************************************************************\
**	ExtFinishPulseRuptHandler		IRQ5 logging stop request
interrupt
**
\******************************************************************************/
IEV_C_FUNCT(ExtFinishPulseRuptHandler) {
#pragma unused(ievstack) // implied (IEVStack *ievstack:__a0) parameter

  PinIO(IRQ5);
  SystemOn = false; // Hmmmm? Interrupt button?? or Pam?

  PinRead(IRQ5);

} //____ ExtFinishPulseRuptHandler() ____//

/******************************************************************************\
**	WriteFile      //The Engineering file to be compiled every #minutes
** 1)Detections: Average/Median
** 2)Sample detections
** 3)Power data
\******************************************************************************/
int WriteFile(ulong TotalSeconds) {

  long BlkLength = 1024;
  // char* string;
  float voltage = 0.0;
  int filehandle;
  char botfname[] = "c:00000000.bot";
  int byteswritten = 0;
  float floater;
  long filenumber;

  // string= (char*)calloc(8,1);

  flogf("\n%s|WriteFile(%s)", Time(NULL), uploadfile);
  cdrain();
  coflush();

  filehandle = open(uploadfile, O_WRONLY | O_CREAT | O_TRUNC);
  RTCDelayMicroSeconds(25000L);
  if (filehandle <= 0) {
    flogf("\nERROR  |WriteFile() %s open error: errno: %d", uploadfile, errno);
    if (errno != 0)
      return -1;
  }
#ifdef DEBUG
  else
    flogf("\n\t|WriteFile() %s opened", uploadfile);
#endif
  GetFileName(true, false, &filenumber, "DAT");
  RTCDelayMicroSeconds(25000L);
  memset(WriteBuffer, 0, 256 * sizeof(WriteBuffer[0]));
  sprintf(
      WriteBuffer,
      "===%s Program %3.2f===\nWrite Time:%s\naa:bb.cccc North ddd:ee.ffff "
      "West\nFile Number:%ld\n"
      "Files to be send: %ld\nCF2 Free Space: %ld\nTotal Time:%lu Secs\n"
      "Starts:%d of %d\nDetection Interval:%3dMins\nData Interval:%4d Mins\0",
      MPC.PROGNAME, PROG_VERSION, Time(NULL), MPC.FILENUM, filenumber,
      SystemFreeSpace, TotalSeconds, MPC.STARTUPS, MPC.STARTMAX, MPC.DETINT,
      MPC.DATAXINT);
#ifdef DEBUG
  flogf("\n%s", WriteBuffer);
  cdrain();
  coflush();
#endif
  byteswritten = write(filehandle, WriteBuffer, strlen(WriteBuffer));
#ifdef DEBUG
  flogf("\n\tBytes Written: %d", byteswritten);
#endif
  floater = Power_Monitor(TotalSeconds, filehandle, MPC.BATLOG, MPC.BATCAP);

  // Adjust and record Battery Capacity
  if (MPC.BATLOG) {
    if (floater != 0.00) {
      sprintf(MPC.BATCAP, "%7.2f", floater);
      VEEStoreStr(BATTERYCAPACITY_NAME, MPC.BATCAP);
    }
  }

  sprintf(&botfname[2], "%08ld.bot", MPC.FILENUM);

  Append_Files(filehandle, botfname, true);

  RTCDelayMicroSeconds(25000L);

  if (close(filehandle) != 0) {
    RTCDelayMicroSeconds(10000L);
    flogf("\nERROR  |WriteFile() %s error closing: %d", uploadfile, errno);
  }
#ifdef DEBUG
  else
    flogf("\n\t|WriteFile() %s Closed", uploadfile);
#endif
  RTCDelayMicroSeconds(25000L);

  return 0;

} //____ WriteFile() ____//

/******************************************************************************\
**	SleepUntilWoken		Sleep until IRQ4 is interrupted
**
** 1-st release 9/14/99
** 2nd release 6/24/2002 by HM -Changed to use ADS8344/45
\******************************************************************************/
void SleepUntilWoken(void) {

  ciflush(); // flush any junk
  flogf("\n%s|SleepUntilWoken()", Time(NULL));
  flogf("\nLow-power sleep mode until keyboard input is received...");

  // Install the interrupt handlers that will break us out by "break signal"
  // from RS232 COM input.
  IEVInsertAsmFunct(IRQ4_ISR, level4InterruptAutovector);
  IEVInsertAsmFunct(IRQ4_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ5_ISR, level5InterruptAutovector);
  IEVInsertAsmFunct(IRQ5_ISR, spuriousInterrupt);

  PITSet51msPeriod(PITOff); // disable timer (drops power)
  CTMRun(false);            // turn off CTM6 module
  SCITxWaitCompletion();    // let any pending UART data finish
  EIAForceOff(true);        // turn off the RS232 driver
  QSMStop();                // shut down the QSM
  CFEnable(false);          // turn off the CompactFlash card

  PinBus(IRQ4RXD); // make it an interrupt pin
  PinBus(IRQ5);

  TickleSWSR();      // another reprieve
  TMGSetSpeed(1600); // Changed July 2015
  while (PinTestIsItBus(IRQ4RXD) && PinTestIsItBus(IRQ5)) {
    //*HM050613 added to reduce current when Silicon System CF card is used
    //*(ushort *)0xffffe00c=0xF000; //force CF card into Card-1 active mode

    LPStopCSE(FullStop); // we will be here until interrupted
    TickleSWSR();        // by break
  }

  CSSetSysAccessSpeeds(nsFlashStd, nsRAMStd, nsCFStd, WTMODE);
  TMGSetSpeed(SYSCLK);

  // CONCLUDE
  PinIO(IRQ4RXD);
  PinIO(IRQ5);

  EIAForceOff(false); // turn on the RS232 driver
  QSMRun();           // bring back the QSM
  CFEnable(true);     // turn on the CompactFlash card
  PIORead(IRQ4RXD);   // disable interrupt by IRQ4
  ciflush();          // discard any garbage characters
  flogf("\n%s|Aquisition ended!", Time(NULL));
  putflush(); // tell 'em we're back
  //                      BIOSResetToPicoDOS();
  //}
} //____ SleepUntilWoken() ____//

/******************************************************************************\
** Vitals
\******************************************************************************/
void Vitals(short status) {

  if ((status = Check_Vitals())) {
    switch (status) {
    case 1:
      SystemOn = false;
      BT_LP = false;
      DataAlarm = true;
      break;
    case 2:
      SystemOn = false;
      BT_LP = true;
      DataAlarm = true;
      break;
    case 3:
      SystemOn = false;
      BT_LP = true;
      DataAlarm = true;
      break;
    }
  }
} //____ Vitals() ____//

/*******************************************************************************\
** void StreamLoop()
\*******************************************************************************/
void StreamLoop(short amodemreturn) {
  char filename[] = "c:00000000.bot";
  int count = 0;
  int amodemfile;

  sprintf(&filename[2], "%08ld.bot", MPC.FILENUM);

  flogf("\n%s|StreamLoop(%s)", Time(NULL), filename);

  amodemfile = open(filename, O_RDWR | O_CREAT | O_TRUNC);

  if (amodemfile <= 0) {
    flogf("\nERROR  |%s open error: errno: %d", filename, errno);
    if (errno != 0)
      return;
  }
#ifdef DEBUG
  else
    flogf("\n\t|%s Opened", filename);
  cdrain();
  coflush();
#endif

  // Stream Data Loop
  while (amodemreturn != 1 && count < 7 && SystemOn) {

    Sleep_AModem();

    if (AD_Check()) {
      count++;
      flogf("count");
    }

    if (tgetq(AModemPort)) {
      amodemreturn = AModemStream(amodemfile);
      if (amodemreturn >= 0)
        count = 0;
      if (amodemreturn == 1)
        DataAlarm = true;
      else if (amodemreturn == -2) {
        count++;
#ifdef DEBUG
        flogf("count");
#endif
      } else if (amodemreturn == -3) {
        flogf("\n\t|Resend Request Exceeded");
        break;
      }
    }

    if (cgetq())
      Console(cgetc());
  }
#ifdef DEBUG
  if (!SystemOn) {
    flogf("\n\t|SYSTEM OFF! Exit!");
    flogf("\n%s|Out of AModemStream Loop, count: %d, amodemreturn: %d",
          Time(NULL), count, amodemreturn);
  } else
    flogf("\n\t|StreamLoop() exit: amodemreturn: %d", amodemreturn);
  cdrain();
  coflush();
#endif
  if (close(amodemfile) < 0)
    flogf("\nERROR  |%s close errno: %d", filename, errno);
  else
    flogf("\n\t|%s closed", filename);

  AModemPhase = 4;
#ifdef DEBUG
  flogf("count: %d", count);
#endif
} //____ StreamLoop() ____//

/******************************************************************************\
** InduceHibernation
\******************************************************************************/
void InduceHibernation() {

  SystemOn = false;
  DataAlarm = DataAlarm ? false : true;
}
