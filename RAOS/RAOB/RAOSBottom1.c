/******************************************************************************\
** 	            Real-Time Acoustic Observing System
**
** RAOS Top Version 4.x Controls
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
**       C:    DutyCycle ? Percentage or decimal? 0.5 vs 50% of ADCounter for
each DETINT
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
** -  Implement Battery Capacity Calculator
**    >  Subtract power calculated by Power Monitor Plus defined error
(POWERERROR) of
**       some perecentage (1.05 = 5%)
**    >  Using 2 packs of 8 (parallel) by 10 (series) D Cell Alkalines we
achieve 15v and roughly
**       288 AH. This pans out to be about 13MJ. (288AH * 12.5 (avg conservative
voltage) *3600seconds per hour).)
**
**
**	May 2016 Alex Turpin RAOSVersion 4.1
**
** - Cutting out TUPort.c & .h files
** - Implementing smart learning into AModem Error
**
*******************************************************************************
\*****************************************************************************/
#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions

#include <assert.h>
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

#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <fcntl.h>    // PicoDOS POSIX-like File Access Definitions
#include <stat.h>     // PicoDOS POSIX-like File Status Definitions
#include <termios.h>  // PicoDOS POSIX-like Terminal I/O Definitions
#include <unistd.h>   // PicoDOS POSIX-like UNIX Function Definitions

#include <ADS.h>
#include <AModem.h>
#include <MPC_Global.h>
#include <PLATFORM.h>
#include <Settings.h> // For VEEPROM settings definitions
#include <WISPR.h>
#ifdef BLUETOOTH
#include <BT.h>
#endif

// WDT Watch Dog Timer definition
short CustomSYPCR = WDT419s | HaltMonEnable | BusMonEnable | BMT32;
#define CUSTOM_SYPCR CustomSYPCR // Enable watch dog  HM 3/6/2014

SystemParameters MPC;

#ifdef WISPR
extern WISPRParameters WISP;
extern bool WISPR_On;
#endif
#ifdef ACOUSTICMODEM
extern AMODEMParameters AMDM;
#endif

// FUNCTIONS
void SetupHardware(void);
void PreRun(void);
void InitializeRAOS(ulong *);
void Incoming_Data();
void Incoming_Data_AModem();
void Console(char);
void Sleep();
void Sleep_AModem();
int WriteFile(ulong TotalSeconds);
void SleepUntilWoken();
static void IRQ2_ISR(void);
static void IRQ3_ISR(void);
static void IRQ4_ISR(void);
static void IRQ5_ISR(void);
IEV_C_PROTO(ExtFinishPulseRuptHandler);

void Vitals();

// LOCAL VARIABLES
bool SystemOn = false;
bool DataAlarm;
short LOGTOCON = 1;
int AModemPhase;
int ActiveAModemCounter;

// newly added variables for Raos4.0
bool BT_LP;
long SystemFreeSpace;
static char uploadfile[] = "c:00000000.dat";

/******************************************************************************\
**	Main
\******************************************************************************/
void main() {

  ulong PwrOn, PwrOff, TimeOn, AmodemTime = 0;
  short count = 0;
  char *string = {""};
  char *paramfilename = {"raospara.prm"};
  bool init_flag = false;
  short vitals = 0;
  long counter = 0;
  char filenum[9] = "00000000";

  PIOClear(AMODEMPWR);

#ifdef BT_TEST

  // Initialize and open TPU UART
  Initflog("activity.log", true);
  TUInit(calloc, free);
  Bluetooth_Test();
  // Low Power Hibernation Mode
  return;
#endif

  // RAOB Initialization
  InitializeRAOS(&PwrOn);
  init_flag = true;

  // CHECK VITALS BEFORE AMODEM LOOP

  // Main Program Loop. Only interrupted by Console Input or Vitals to set in
  // Long Term Hibernation.
  while (SystemOn) {

    // While not set for Low_Power BLUETOOTH Mode.
    while (!BT_LP) {

      // If This isn't the first loop through non-BT_LP Mode
      if (init_flag == false) {
        // Check parameters in file: "SYSTEM.CFG"
        // ParseStartupParams(); //This takes place only when new parameters
        // have been received.

        Reset_ADCounter();
        DataTimer = 0;
        MPC.FILENUM++;
        sprintf(filenum, "%08ld", MPC.FILENUM);
        VEEStoreStr(FILENUM_NAME, filenum);
        create_dtx_file(MPC.FILENUM);
      }

      // Check Detection and Data Transfer Intervals, and WISPR Duty Cycle
      Check_Timers(Return_ADSTIME());

      // Turn on Wispr for recording
      if (WISP.DUTYCYCL == 100)
        WISPRPower(true);

      // WISPR Recording, Calling Detections, Waiting for AMXFER Timer to go off
      while (!DataAlarm) {

        Sleep();

        // Check the system timers
        if (System_Timer() == 2)
          DataAlarm = true;

        // Make sure system is still good to run
        Vitals();

        // Grabs data from TUPort Devices or logs AD_Data
        Incoming_Data();
      }

      if (WISP.DUTYCYCL == 100 || WISPR_On)
        WISPRSafeShutdown();

      // Turn off WISPR Port
      OpenTUPort_WISPR(false);

      // Get SystemTimeOn, Finish Logging Power, Update System of Average
      // Voltage, Start Power Logging
      Time(&PwrOff);
      TimeOn = PwrOff - PwrOn;
      PwrOn = PwrOff;

      // Add last Acoustic Modem Time to total Call Interval Cycle
      TimeOn += AmodemTime;

      // Name the uploadfile
      sprintf(&uploadfile[2], "%08ld.dat", MPC.FILENUM);

      // Gather Free Space from CF2
      SystemFreeSpace = Free_Disk_Space();

      // Write Engineering File for this power cycle to be uploaded to the Top
      // Acoustic Modem.
      WriteFile(TimeOn);
      DOS_Com("del", MPC.FILENUM, "PWR", NULL);
      // If console exit '1' exit here and go to LP Hibernation
      if (!SystemOn || BT_LP)
        break;

      // Start Power Monitoring
      Setup_ADS(true, MPC.FILENUM + 1);

      // AModem_Init(); Removed for 4.1

      OpenTUPort_AModem(true);
      RTCDelayMicroSeconds(100000L);
      AModemSend("$ACK**");
      AModemPhase = 1;
      ActiveAModemCounter = 0;

      // count is just used for debugging process of waiting for AModem to
      // respond. Might be too quick.
      while (DataAlarm) {

        Sleep_AModem();

        Incoming_Data_AModem();

        if (!SystemOn || BT_LP)
          break;
#ifdef DEBUG
        flogf("\n\t|ActiveAModemCounter: %d", ActiveAModemCounter);
#endif
        if (ActiveAModemCounter == 0)
          continue;
        else if (ActiveAModemCounter >= 20) {
          DataAlarm = false;
          break;
        } else if ((ActiveAModemCounter % 2) == 0)
          AModemResend();
      }

      // TURxFlush(AModemPort);
      // TUTxFlush(AModemPort);

      // Turn off AModem TUPORT
      OpenTUPort_AModem(false);
      //      AModem_Clear(); Removed for 4.1

      // Get Time for Duration of AModem Power
      Time(&AmodemTime);
      AmodemTime -= PwrOff;

      // Turn on WISPR  after uplaod.
      OpenTUPort_WISPR(true);

      // Reset Parameters and Alarms
      DataAlarm = false;

      // Reset_Timer();
      init_flag = false;
#ifdef DEBUG
      flogf("\n%s|Restarting Main() Loop", Time(NULL));
#endif
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

  SleepUntilWoken();

} //____ Main() ____//

/*********************************************************************************\
** InitializeRAOS
\*********************************************************************************/
void InitializeRAOS(ulong *PwrOn) {

  ulong time = NULL;
  bool check = false;

  // Define unused pins here
  uchar mirrorpins[] = {1,  15, 16, 17, 18, 19, 21, 24, 25,
                        26, 28, 31, 32, 35, 36, 37, 42, 0};

  PIOMirrorList(mirrorpins);

  // Get the system settings running
  SetupHardware();

  // Initialize and open TPU UART
  TUInit(calloc, free);

#ifdef BLUETOOTH
  Bluetooth_Power(true);
#endif

  // Count Down Timer to Start Program or go to PICO dos/Settings
  PreRun();

  // Get all Platform Settings
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

  // Get Free Disk Space
  SystemFreeSpace = Free_Disk_Space(); // Does finding the free space of a large
                                       // CF card cause program to crash? or
                                       // Hang?

  // Assert AModem Off.
  PIOClear(AMODEMPWR);

  InitializeWISPR();

  if (MPC.STARTUPS == 0) {

    Make_Directory("SNT");

    if (WISPRNUMBER > 1) {
      if (WISP.NUM != 1)
        WISP.NUM = 1;

      // Open SG Tuport
      OpenTUPort_WISPR(true);
      // Gather all #WISPRNUMBER freespace and sync time.
      GatherWISPRFreeSpace();
    }
  } else
    // Open SG Tuport
    OpenTUPort_WISPR(true);

#ifdef POWERLOGGING
  flogf("\n\t|Check Startup Voltage: %5.2fV", Voltage_Now());
#endif

  // Store Number of Bootups
  MPC.STARTUPS++;
  VEEStoreShort(STARTUPS_NAME, MPC.STARTUPS);

  DataAlarm = false;
  SystemOn = true;
  TickleSWSR();
  Reset_ADCounter();

  create_dtx_file(MPC.FILENUM);

} //____ InitializeRAOS() ____//

/****************************************************************************\
** void Incoming_Data()
** This is the function to gather incoming data during the long duration of the
**    RAOS cycle. This includes serial with WISPR, A->D, and Console (DBG only)
\****************************************************************************/
void Incoming_Data(void) {
  bool incoming = true;

  while (incoming) {

    // Check A to D Power Sampling Buffers
    AD_Check();

    // Console Wake up.
    if (cgetq()) {
      Console(cgetc());
      incoming = true;
    }
    // WISPR Data Check
    else if (tgetq(PAMPort)) {
      if (WISPR_Data() == 1) {
        RTCDelayMicroSeconds(100000L);
        if (tgetq(PAMPort))
          WISPR_Data();
      }
      incoming = true;
    }
#ifdef BLUETOOTH
    // Bluetooth Check if on
    else if (BT_On)
      if (tgetq(BTPort)) {
#ifdef DEBUG
        flogf("bt");
#endif
        Bluetooth_Interface();
        incoming = true;
      } else
        incoming = false;
#endif
    else
      incoming = false;
  }

} //_____ Incoming_Data() _____//

/************************************************************************************************************************\
** void Incoming_Data_AModem()
** This function serves for serial interface when DataAlarm=true and dealing
with the Acoustic Modem data transfer.
** We need this
\************************************************************************************************************************/
void Incoming_Data_AModem() {
  bool incoming = true;
  int amodemreturn = 0;

  while (incoming) {
    // Check A to D Power Sampling Buffers
    if (AD_Check()) {
      ActiveAModemCounter++;
      incoming = true;
    } else if (cgetq()) {
      Console(cgetc());
      incoming = true;
    }

    else if (tgetq(AModemPort)) {
      amodemreturn = AModem_Data();
      incoming = true;
      if (amodemreturn >= 0)
        ActiveAModemCounter = 0;
      if (amodemreturn == 2 && AModemPhase == 4) {

        TURxFlush(AModemPort);
        TUTxFlush(AModemPort);
      } else if (amodemreturn == 2 && AModemPhase == 6)
        DataAlarm = false;
      else if (amodemreturn == 6 && AModemPhase == 6) {
        ActiveAModemCounter = 20;
        Delay_AD_Log(5);
      }

    }
#ifdef BLUETOOTH
    else if (BT_On) {
      if (tgetq(BTPort)) {
        Bluetooth_Interface();
        incoming = true;
      } else
        incoming = false;

    }
#endif
    else
      incoming = false;
  }

} //_____ Incoming_Data_AModem() _____//

/************************************************************************************************************************\
** void Console      Testing Opportunity for Debugging
\************************************************************************************************************************/
void Console(char in) {
  int ret = 0;
#ifdef DEBUG
  flogf("Incoming Char: %c\n", in);
  putflush();
  CIOdrain();
#endif
  switch (in) {
  case 'I':
  case 'i':
    WISPRPower(true);
    break;

  case 'D':
  case 'd':
    WISPRDet(WISP.DETMAX);
    WISPR_Data();
    break;

  case 'F':
  case 'f':
    flogf("\n\t|Console: DFP1");
    WISPRDFP();
    break;

  case 'E':
  case 'e':
    if (!WISPRExit()) {
      if (!WISPRExit()) {
        flogf("Forcing Off");
        WISPRPower(false);
      }
    }
    break;
  case '1':
    SystemOn = false;
    DataAlarm = true;
    break;

  case 'A':
  case 'a':
    DataAlarm = DataAlarm ? false : true;
    RTCDelayMicroSeconds(100000L);
#ifdef DEBUG
    flogf("\n\t|DataAlarm %s", DataAlarm ? "true" : "false");
#endif
    break;
#ifdef BLUETOOTH
  case 'B':
  case 'b': // Low power BT Mode only.
    if (WISPR_Status()) {
      if (!WISPRExit()) {
        if (!WISPRExit()) {
          flogf("Forcing Off");
          WISPRPower(false);
        }
      }
    }
    ret = Bluetooth_LowPower();
    if (ret == 3)
      SystemOn = false;
    else if (ret == 2)
      BIOSReset();

    break;
#endif

  case 'H':
  case 'h':
    AModem_SetPower(true);
    break;

  case 'L':
  case 'l':
    AModem_SetPower(false);
    break;
  }
  ciflush();
}

/******************************************************************************\
**	SleepUntilWoken		Finish up
**
** 1-st release 9/14/99
** 2nd release 6/24/2002 by HM -Changed to use ADS8344/45
\******************************************************************************/
void Sleep(void) {

  IEVInsertAsmFunct(IRQ5_ISR, level5InterruptAutovector); // PAM Interrupt
  IEVInsertAsmFunct(IRQ5_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ2_ISR, level2InterruptAutovector); // BT Interrupt
  IEVInsertAsmFunct(IRQ2_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ4_ISR, level4InterruptAutovector); // Console Interrupt
  IEVInsertAsmFunct(IRQ4_ISR, spuriousInterrupt);

  CTMRun(false);
  SCITxWaitCompletion();
  EIAForceOff(true); // turn off RS232 Transmitters
  QSMStop();
  CFEnable(false);

  PinBus(IRQ2);
  PinBus(IRQ5); // PAM Interrupt
  while ((PinTestIsItBus(IRQ5)) == 0)
    PinBus(IRQ5);
  PinBus(IRQ4RXD); // Console Interrupt
  while ((PinTestIsItBus(IRQ4RXD)) == 0)
    PinBus(IRQ4RXD);

  while (PinRead(IRQ5) && PinRead(IRQ4RXD) && !data && PinRead(IRQ2))
    LPStopCSE(FastStop);

  QSMRun();
  EIAForceOff(false); // turn on the RS232 Transmitters
  CFEnable(true);     // turn on the CompactFlash card
  CTMRun(true);
  PIORead(IRQ4RXD); // Console
  while ((PinTestIsItBus(IRQ4RXD)) != 0)
    PIORead(IRQ4RXD);
  PIORead(IRQ5); // PAM
  while ((PinTestIsItBus(IRQ5)) != 0)
    PIORead(IRQ5);
  PIORead(IRQ2);
#ifdef DEBUG
  flogf(".");
#endif
  RTCDelayMicroSeconds(1000L);

} //____ Sleep() ____//

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
#ifdef CONSOLEINPUT
  IEVInsertAsmFunct(IRQ4_ISR, level4InterruptAutovector); // Console Interrupt
  IEVInsertAsmFunct(IRQ4_ISR, spuriousInterrupt);
#endif

  // CTMRun(false);
  SCITxWaitCompletion();
  EIAForceOff(true);
  // QSMStop();
  CFEnable(false);

  TickleSWSR(); // another reprieve

  PinBus(IRQ3RXX); // AModem Interrupt
  while ((PinTestIsItBus(IRQ3RXX)) == 0)
    PinBus(IRQ3RXX);

#ifdef CONSOLEINPUT
  PinBus(IRQ4RXD); // Console Interrupt
  while ((PinTestIsItBus(IRQ4RXD)) == 0)
    PinBus(IRQ4RXD);

  // Watch Amodem Tuport, Console and AD Logging
  while (PinRead(IRQ3RXX) && PinRead(IRQ4RXD) && !data)
    LPStopCSE(FastStop);

#else

  while (PinRead(IRQ3RXX) && !data)
    LPStopCSE(FastStop);
#endif

  // QSMRun();
  EIAForceOff(false); // turn on the RS232 driver
  CFEnable(true);     // turn on the CompactFlash card
  // CTMRun(true);

  PIORead(IRQ3RXX);
  while ((PinTestIsItBus(IRQ3RXX)) != 0)
    PIORead(IRQ3RXX);
#ifdef CONSOLEINPUT
  PIORead(IRQ4RXD);
  while ((PinTestIsItBus(IRQ4RXD)) != 0)
    PIORead(IRQ4RXD);
#endif
#ifdef DEBUG
  flogf(".");
#endif
  RTCDelayMicroSeconds(5000L);

} //____ Sleep_AModem() ____//

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

/*************************************************************************\
**  static void Irq3ISR(void)
\*************************************************************************/
static void IRQ2_ISR(void) {
  PinIO(IRQ2);
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

/*************************************************************************\
**  static void IRQ5_ISR(void)
\*************************************************************************/
static void IRQ5_ISR(void) {
  PinIO(IRQ5);
  RTE();
} //____ Irq5ISR() ____//

/******************************************************************************\
**	WriteFile      //The Engineering file to be compiled every #minutes
** 1)Detections: Average/Median
** 2)Sample detections
** 3)Power data
\******************************************************************************/
int WriteFile(ulong TotalSeconds) {

  long BlkLength = 1024;
  float voltage = 0.0;
  int filehandle;
  int byteswritten = 0;
  float floater;
  long filenumber;
  uchar *buf;

  buf = (uchar *)malloc(256);
  memset(buf, 0, 256 * (sizeof buf[0]));

  CLK(start_clock = clock();) flogf("\n\t|WriteFile(%s)", uploadfile);
  filehandle = open(uploadfile, O_WRONLY | O_CREAT | O_TRUNC);

  if (filehandle <= 0) {
    flogf("\nERROR  |WriteFile(): open error: errno: %d", errno);
    if (errno != 0)
      return -1;
  }
#ifdef DEBUG
  else
    flogf("\n\t|WriteFile: %s Opened", uploadfile);
#endif
  CLK(stop_clock = clock();
      print_clock_cycle_count(start_clock, stop_clock, "fopen: uploadfile");)
  GetFileName(true, false, &filenumber, "DAT");
  RTCDelayMicroSeconds(25000L);
  sprintf(buf,
          "\n===%s Program %3.2f===\nWrite Time:%s\nFile Number:%ld\n"
          "Files to be sent: %ld\nCF2 Free Space: %ld\nTotal Time:%luSecs\n"
          "Starts:%d of %d\nDetection Interval:%dMins\nData Interval:%dMins",
          MPC.PROGNAME, PROG_VERSION, Time(NULL), MPC.FILENUM, filenumber,
          SystemFreeSpace, TotalSeconds, MPC.STARTUPS, MPC.STARTMAX, MPC.DETINT,
          MPC.DATAXINT);

  byteswritten = write(filehandle, buf, strlen(buf));
#ifdef DEBUG
  flogf("\n\t|Bytes Written: %d", byteswritten);
#endif
#ifdef DEBUG
  flogf("\n%s\n", buf);
  cdrain();
  coflush();
#endif
  RTCDelayMicroSeconds(100000L);
  free(buf);

  // Stop Power Logging, log it, and print to uploadfile
  floater = Power_Monitor(TotalSeconds, filehandle, MPC.BATLOG, MPC.BATCAP);

  if (MPC.BATLOG) {
    if (floater != 0.00) {
      sprintf(MPC.BATCAP, "%7.2f", floater);
      VEEStoreStr(BATTERYCAPACITY_NAME, MPC.BATCAP);
    }
  }

#ifdef WISPR
  WISPRWriteFile(filehandle);
#endif

  RTCDelayMicroSeconds(25000L);

  if (close(filehandle) < 0)
    flogf("\nERROR  |Closing %s, errno: %d", uploadfile, errno);

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

  PITSet51msPeriod(PITOff); // disable timer (drops power)
  CTMRun(false);            // turn off CTM6 module
  SCITxWaitCompletion();    // let any pending UART data finish
  EIAForceOff(true);        // turn off the RS232 driver
  QSMStop();                // shut down the QSM
  CFEnable(false);          // turn off the CompactFlash card

  PinBus(IRQ4RXD); // make it an interrupt pin
  while ((PinTestIsItBus(IRQ4RXD)) == 0)
    PinBus(IRQ4RXD);

  TickleSWSR();      // another reprieve
  TMGSetSpeed(2000); // Changed July 2015
  while (PinTestIsItBus(IRQ4RXD)) {
    //*HM050613 added to reduce current when Silicon System CF card is used
    //*(ushort *)0xffffe00c=0xF000; //force CF card into Card-1 active mode

    LPStopCSE(FullStop); // we will be here until interrupted
    TickleSWSR();        // by break
  }

  CSSetSysAccessSpeeds(nsFlashStd, nsRAMStd, nsCFStd, WTMODE);
  TMGSetSpeed(SYSCLK);

  // CONCLUDE
  PinIO(IRQ4RXD);

  EIAForceOff(false); // turn on the RS232 driver
  QSMRun();           // bring back the QSM
  CFEnable(true);     // turn on the CompactFlash card
  PIORead(IRQ4RXD);   // disable interrupt by IRQ4
  ciflush();          // discard any garbage characters
  flogf("\n%s|Aquisition ended!", Time(NULL));
  putflush(); // tell 'em we're back

} //____ SleepUntilWoken() ____//

/******************************************************************************\
** Vitals
\******************************************************************************/
void Vitals() {

  short status;

  if ((status = Check_Vitals())) {
    switch (status) {
    case 1: // Absolute Min Voltage
      SystemOn = false;
      BT_LP = false;
      DataAlarm = true;
      break;
    case 2: // User Defined Min Voltage
      SystemOn = false;
      BT_LP = true;
      DataAlarm = true;
      break;
    case 3: // Startups Max Surpassed
      SystemOn = false;
      BT_LP = true;
      DataAlarm = true;
      break;
    }
  }
} //____ Vitals() ____//

/******************************************************************************\
** InduceHibernation
\******************************************************************************/
void InduceHibernation() {

  SystemOn = false;
  DataAlarm = DataAlarm ? false : true;
}
