/******************************************************************************\
** 	Autonomous Underwater Hydrophone Program for Multiple Port Controller
**
** MPC Top Version 3.x Controls
**
** -  Long term autonomous underwater recorder
**    >  Single Logfile program: defined in
**    >  Logfile determined by
**    >  Variables can be updated by sending a command string from RUDICS
**       Basestation defined inside the parathesis of the follwoing string:
**
**       $$$MPC(xxxxx)***
**
**    >  These variables are controlled by sending the characters listed
**       below followed by up to 5 numbers (including a '.' for min sys volt)
**       **Note: No "0" zero digit holders necessary
**
**       I:    Detection Interval: period to inquiry WISPR. Defined in minutes
**       A:    AModem Call Interval: rounded up depending on Detection Interval
**       D:    Number of detections to return per detection call.
**             > These will be save to the .dtx file on bottom and appended to
**               the system info before being uploaded to the Top System.
**       S:    Maximum number of Startups
**       v:    Minimum System Voltage (Absolute Minimum 10.5volts for AModem)
**       N:    Detection Return Num: The Number of Detections for RAOSBottom to
induce an AModem Call
**       G:    WISPR Gain
**       C:    DutyCycle: Percentage or decimal? 0.5 vs 50% of ADCounter for
each DETINT
**             >  A DETINT of 60 minutes would produce ADCounter==70;
**             >  The first half of the ADCounter (35 counts) the WISPR would be
left off
**             >  The second half of that 60 minute Detection Interval WISPR
would be on
**       P:    PAM Number: The Pam port in use
**
**
**	December 2015  Alex Turpin AUH Version 3.0
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
#include <MPC_Global.h>
#include <Settings.h> // For VEEPROM settings definitions
#include <TUPORT.h>
#include <WISPR.h>

// WDT Watch Dog Timer definition
short CustomSYPCR = WDT105s | HaltMonEnable | BusMonEnable | BMT32;
#define CUSTOM_SYPCR CustomSYPCR // Enable watch dog  HM 3/6/2014

// Define unused pins here
uchar mirrorpins[] = {1,  15, 16, 17, 18, 19, 21, 24, 25,
                      26, 28, 31, 32, 35, 36, 37, 42, 0};

SystemParameters MPC;
extern WISPRParameters WISP;

void Incoming_Data();
void Console(char);

void Sleep();
void InitializeAUH(ulong *);
// int   WriteFile(float mAh, float voltage, long, ulong TotalSeconds, float);
int WriteFile(ulong);
void Hibernate();
static void IRQ4_ISR(void);
static void IRQ5_ISR(void);
IEV_C_PROTO(ExtFinishPulseRuptHandler);

// LOCAL VARIABLES
bool PutInSleepMode;
bool SystemOn = true;
char *WriteBuffer;
long FileNumber;
static char uploadfile[] =
    "c:00000000.dat"; // 12.9.2015 Can this be a static char?
bool Wispr_On;
long SystemFreeSpace;

// GLOBAL VARIABLES
// extern int ADCounter;
extern bool SystemTimer;
extern bool data;

/******************************************************************************\
**	Main
\******************************************************************************/
void main() {

  ulong PwrOn, PwrOff = 0;
  char string[60];

  InitializeAUH(&PwrOn);
  while (SystemOn) {
    // WISPR Recording, Calling Detections, Waiting for Vitals or External Stop
    while (SystemOn) {

      if (PutInSleepMode)
        Sleep();

      // Check the system timers
      System_Timer(&Wispr_On);

      // Grabs data from TUPort Devices or logs AD_Data
      Incoming_Data();
    }
    // Increment File Number
    FileNumber++;
    sprintf(string, "%08ld", FileNumber);
    VEEStoreStr(FILENUM_NAME, string);

    if (Check_Vitals() == 1)
      SystemOn = false;

    if (!SystemOn && Wispr_On)
      WISPRSafeShutdown();

    // Get SystemTimeOn, Finish Logging Power, Update System of Average Voltage,
    // Start Power Logging
    Time(&PwrOff);
    PwrOff -= PwrOn;

    WriteFile(PwrOff);

    Setup_ADS(true, FileNumber);
  }

  RTCDelayMicroSeconds(100000L);

  PutInSleepMode = true;
#ifdef DEBUG
  flogf("\n%s|Restarting Main() Loop", Time(NULL));
#endif
  TURelease();

  // Low Power Hibernation Mode
  Hibernate();

} //____ Main() ____//

/*********************************************************************************\
** InitializeAUH
\*********************************************************************************/
void InitializeAUH(ulong *PwrOn) {

  ushort SysTimeInt;
  ulong time = NULL;
  bool check = false;
  // Get all Platform Settings
  GetSettings();
  // Get the system settings running
  SetupHardware();

  // Count Down Timer to Start Program or go to PICO dos/Settings
  PreRun();

  // Initialize and open TPU UART
  TUInit(calloc, free);

  // Identify IDs
  flogf("\nProject ID: %s   Platform ID: %s   Location: %s   Boot ups: %d",
        MPC.PROJID, MPC.PLTFRMID, MPC.LOCATION, MPC.STARTUPS);

  // Get Power On Time
  Time(&time);
  *PwrOn = time;

#ifdef POWERLOGGING
  // Turn on A to D Logging for Power Consumption
  SysTimeInt = Setup_ADS(true, FileNumber);
#else
  SysTimeInt = 512;
#endif

  WriteBuffer = (char *)malloc(128);

  // Parse Startup Pameters from System.prm file
  ParseStartupParams("IVSDGPC");

  // Initialize System Timers
  Check_Timers(SysTimeInt);

  // Get Free Disk Space
  SystemFreeSpace = Free_Disk_Space(); // Does finding the free space of a large
                                       // CF card cause program to crash? or
                                       // Hang?

  // Open SG Tuport
  OpenTUPort("PAM", true);

  if (MPC.STARTUPS == 0 && WISPRNUMBER > 1) {
    // Turn on Wispr for recording
    if (Wispr_On)
      WISPRPower(true);
    // Gather all #WISPRNUMBER freespace and sync time.
    GatherWISPRFreeSpace();

  }
#ifdef POWERLOGGING
  // Wait for AD_Check;
  else {
    while (!check) {
      Sleep();

      check = AD_Check();
    }
    if (Wispr_On)
      WISPRPower(true);
  }
#endif

  // Store Number of Bootups
  MPC.STARTUPS++;
  VEEStoreShort(STARTUPS_NAME, MPC.STARTUPS);

  SystemOn = true;
  TickleSWSR();

  ADCounter = 0;

  create_dtx_file((long)FileNumber);

} //____ InitializeAUH() ____//

/****************************************************************************\
** void Incoming_Data()
** This is the function to gather incoming data during the long duration of the
**    MPC cycle. This includes serial with WISPR, A->D, and Console (DEBUG only)
\****************************************************************************/
void Incoming_Data() {
  bool incoming = true;
  int i;
  while (incoming) {

    // Check A to D Power Sampling Buffers
    AD_Check();

    // Console Wake up.
    if (cgetq()) {
      Console(cgetc());
      incoming = true;
    } else if (tgetq(PAMPort)) {
      i = WISPR_Data();
      if (i == 1) {
        RTCDelayMicroSeconds(100000L);
        if (tgetq(PAMPort))
          WISPR_Data();
      } else if (i == 2) {
        UpdateWISPRFRS();
      }
      incoming = true;
    } else {
      incoming = false;
      PutInSleepMode = true;
      RTCDelayMicroSeconds(2500);
    }
  }

} //_____ Incoming_Data() _____//

/************************************************************************************************************************\
** void Console      Testing Opportunity for Debugging
\************************************************************************************************************************/
void Console(char in) {
#ifdef DEBUG
  flogf("Incoming Char: %c\n", in);
  putflush();
  CIOdrain();
#endif
  switch (in) {
  case 'I':
    WISPRPower(true);
    break;

  case 'D':
    WISPRDet(WISP.DETMAX);
    WISPR_Data();
    break;

  case 'F':
    WISPRDFP();
    break;

  case 'E':
    if (!WISPRExit()) {
      if (!WISPRExit()) {
        flogf("Forcing Off");
        WISPRPower(false);
      }
    }
    break;
  case '1':
    SystemOn = false; // DataAlarm = true;
    break;
#ifdef ACOUSTICMODEM
  case 'A':
    AModemAlarm = AModemAlarm ? false : true;
    RTCDelayMicroSeconds(100000L);
#ifdef DEBUG
    flogf("\n\t|AModemAlarm %s", AModemAlarm ? "true" : "false");
#endif
    break;
#endif
  //   case 'P': Power_Monitor(NULL, 300);

  case 'W':
    RTCDelayMicroSeconds(2500000L);
    in = cgetc() - 48;
    if (in > 0 && in < WISPRNUMBER) {
      ChangeWISPR(in);
    }

    break;

  case 'S':
    Setup_ADS(true, FileNumber + 1);
  }

  PutInSleepMode = true;
}

/******************************************************************************\
**	SleepUntilWoken		Finish up
**
** 1-st release 9/14/99
** 2nd release 6/24/2002 by HM -Changed to use ADS8344/45
\******************************************************************************/
void Sleep(void) {

  IEVInsertAsmFunct(IRQ5_ISR, level5InterruptAutovector); // PAMPort Interrupt
  IEVInsertAsmFunct(IRQ5_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ4_ISR, level4InterruptAutovector); // Console Interrupt
  IEVInsertAsmFunct(IRQ4_ISR, spuriousInterrupt);

  CTMRun(false);
  SCITxWaitCompletion();
  EIAForceOff(true); // turn off RS232 Transmitters
  QSMStop();
  CFEnable(false);

  PinBus(IRQ5); // PAMPort Interrupt
#ifdef DEBUG
  PinBus(IRQ4RXD); // Console Interrupt
#ifdef POWERLOGGING
  while (PinRead(IRQ5) && PinRead(IRQ4RXD) && !data)
    LPStopCSE(FastStop);
#else
  while (PinRead(IRQ5) && PinRead(IRQ4RXD))
    LPStopCSE(FastStop);
#endif
  PIORead(IRQ4RXD); // Console
#else
#ifdef POWERLOGGING
  while (PinRead(IRQ5) && !data)
    LPStopCSE(FastStop);
#else
  while (PinRead(IRQ5))
    LPStopCSE(FastStop);
#endif
#endif

  PIORead(IRQ5); // PAMPort

  QSMRun();
  EIAForceOff(false); // turn on the RS232 Transmitters
  CFEnable(true);     // turn on the CompactFlash card
  CTMRun(true);

  PutInSleepMode = false;
#ifdef DEBUG
  flogf(".");
#endif
  RTCDelayMicroSeconds(1000L);

} //____ Sleep() ____//

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
**  static void IRQ4_ISR(void)
** Console Interrupt
\*************************************************************************/
static void IRQ4_ISR(void) {
  PinIO(IRQ4RXD);
  RTE();
} //____ Irq4ISR() ____//

/*************************************************************************\
**  static void IRQ5_ISR(void)
** Hardware Interrupt for PAM Ports
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
  int filehandle;
  char detfname[] = "c:00000000.dtx";
  int byteswritten = 0;
  float kJ = 0.0;

  sprintf(&uploadfile[2], "%08ld.dat", FileNumber);
  filehandle = open(uploadfile, O_WRONLY | O_CREAT | O_TRUNC);

  if (filehandle <= 0) {
    flogf("\nERROR|WriteFile(): open error: errno: %d", errno);
    if (errno != 0)
      return -1;
  }
  RTCDelayMicroSeconds(25000L);

  memset(WriteBuffer, 0, 128 * sizeof(WriteBuffer[0]));
  sprintf(WriteBuffer, "MPC AUH Program Ver:%3.2f\n-SU:%d -SM:%d\n-SFS: "
                       "%ld\n-WT:%s\n-DT:%lu -DI:%3d\0",
          PROG_VERSION, MPC.STARTUPS, MPC.STARTMAX, SystemFreeSpace, Time(NULL),
          TotalSeconds, MPC.DETINT);

  flogf("\n%s", WriteBuffer);

  write(filehandle, WriteBuffer, strlen(WriteBuffer));

//  kJ =Power_Monitor(TotalSeconds, filehandle);

// Adjust and record Battery Capacity
// MPC.BATCAP-=(kJ*POWERERROR);
//  VEEStoreFloat(BATTERYCAPACITY_NAME, MPC.BATCAP);
//   flogf("\n\t|Battery: %7.2fkJ", MPC.BATCAP);

#ifdef WISPR
  WISPRWriteFile(filehandle);
#endif

  sprintf(&detfname[2], "%08d.dtx", FileNumber);
#ifdef DEBUG
  flogf("\n\t|Append File: %s", detfname);
#endif
  Append_Files(filehandle, detfname, false);

  RTCDelayMicroSeconds(25000L);

  close(filehandle);

  RTCDelayMicroSeconds(25000L);

  return 0;

} //____ WriteFile() ____//

/******************************************************************************\
**	SleepUntilWoken		Sleep until IRQ4 is interrupted
**
** 1-st release 9/14/99
** 2nd release 6/24/2002 by HM -Changed to use ADS8344/45
\******************************************************************************/
void Hibernate() {

  ciflush(); // flush any junk
  flogf("\n%s|Hibernate()", Time(NULL));
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
  TMGSetSpeed(1600); // Changed July 2015
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

} //____ Hibernate() ____//
