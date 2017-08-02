/******************************************************************************\
** Long-term Acoustic Real-Time Sensor for Polar Areas (LARA)
**
** LARA.c for control of the NiGK underwater winch and necessary acoustic and
** physical sensors.
**
**	September 2016  Alex Turpin
*****************************************************************************
* PLATFORM.h:  
* short PHASE; // 1=AUH, 2=Ascent, 3=Surface Communication, 4= Descent
* short BUOYMODE;  // 0=stopped 1=ascend 2=descend 3=careful ascent
*        At depth and no velocity. Phase 1 && BuoyMode 0
*        Ascending Phase 2 && BuoyMode 1
*        On surface Phase 3 && BuoyMode 0
*        Descending Phase 4 && BuoyMode 2
**
* Console(char)
*  LARA.PHASE case 1:
*    case 'i': WISPRPower(true);
*    case 'e': WISPRSafeShutdown();
*    case 'd': WISPRDet(c);
*    case 'f': WISPRDFP();
*    case 'w': ChangeWISPR(c);
*    case 'p': LARA.PHASE = c;
*    case 'x': LARA.ON = false;
*    case '2': LARA.DATA = LARA.DATA ? false : true;
*  LARA.PHASE case 2: case 4:
*    case 'w': WinchConsole();
*    case 'p': LARA.PHASE = c;
*    case 't': CTD_Sample();
*    case 'x': LARA.ON = false; LARA.DATA = LARA.DATA ? false : true;
*    case 'a': PrintSystemStatus();
*    case 's': LARA.SURFACED = true;
*  LARA.PHASE case 3:
*    case 'x': LARA.ON = false; // exit from GPSIRID
*    case 'p': LARA.PHASE = c;
***
*
*WISPR BOARD
*** TPU 6    27 PAM1 WISPR TX
*** TPU 7    28 PAM1 WISPR RX
*** TPU 8    29 1= turns on MAX3222
*** TPU 9    30 0= enables MAX3222
*
*Interrupts:
**
** IRQ2 Wakes up from the sleep mode while waiting for ASC string from COM2
** IRQ3
** IRQ5 Interrupt to stop the program
**
*****************************************************************************
** Before deploying, set CF2 time and SM2 time, format SD cards, 
** replace SM2 and CF2 clock batteries
** erase activity.log, set status=0, set startups=0,
** verify boottime is correct, verify pwrondepth and pwroffdepth are correct,
** make sure you have the correct mA multiplier set in the mAh calculation in
** status 5 for the SM2 settings that you are using (compression, kHz, etc.)
** as power consumption varies between those settings
**
\******************************************************************************/

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
#include <CTD.h>
#include <GPSIRID.h>
#include <MPC_Global.h>
#include <PLATFORM.h>
#include <Settings.h> // For VEEPROM settings definitions
#include <WISPR.h>
#include <Winch.h>

#define STRING_SIZE 1024
#define CUSTOM_SYPCR CustomSYPCR // Enable watch dog  HM 3/6/2014
// WDT Watch Dog Timer definition
// Not sure if this watchdog is even working You have to define
short CustomSYPCR = WDT105s | HaltMonEnable | BusMonEnable | BMT32;

//#define LPMODE    FullStop          //NOT USING SLEEP MODE CURRENTLY// Low
//power mode

// VEEPROM Parameter Structures
SystemParameters MPC;
#ifdef WISPR
extern WISPRParameters WISP;
extern bool WISPR_On;
#endif
#ifdef IRIDIUM
extern IridiumParameters IRID;
#endif
#ifdef POWERLOGGING
extern PowerParameters ADS;
#endif
#ifdef NIGKWINCH
extern WINCHParameters NIGK;
extern WinchCalls WINCH;
#endif
#ifdef CTDSENSOR
extern CTDParameters CTD;
#endif

SystemStatus LARA;

IEV_C_PROTO(ExtFinishPulseRuptHandler);

// Define unused pins here
uchar mirrorpins[] = {15, 16, 17, 18, 19, 26, 36, 0};

void shutdown();
void InitializeLARA(ulong *);

void PhaseOne();
void PhaseTwo();
void PhaseThree();
void PhaseFour();
void PhaseFive();
void Console(char);
bool CheckTime(ulong, short, short);
ulong WriteFile(ulong);
static void IRQ2_ISR(void);
static void IRQ3_ISR(void);
static void IRQ4_ISR(void);
static void IRQ5_ISR(void);
void WaitForWinch(short);
void SleepUntilWoken();
bool CurrentWarning();

bool GlobalRestart;
bool PutInSleepMode = false;
static char *returnstr;
static char uploadfile[] =
    "c:00000000.dat"; // 12.9.2015 Can this be a static char?
// static char *WriteBuffer;

/******************************************************************************\
**	Main
\******************************************************************************/
void main() {

  // Two unsigned longs to keep track of system timing.
  ulong PwrOn, PwrOff;

  // Allocation of Space for the Global buffer. Mostly used to write to the
  // uploadfile. Never released.
  WriteBuffer = (char *)calloc(STRING_SIZE, 1);
  // str for anyone to use, i.e. printSystemStatus
  returnstr = (char *)calloc(STRING_SIZE, 1);

  // Platform Specific Initialization Function. PwrOn is the start time of
  // PowerLogging
  InitializeLARA(&PwrOn);

  // Running WISPR Always! //Rebooting TUPort post-Iridium
  OpenTUPort_WISPR(true);
  if (WISP.DUTYCYCL == 100)
    WISPRPower(true);

  // PHASE 5:  Deployment
  if (LARA.PHASE==5) {
    PhaseFive();
  }

  // Main Loop. Always running unless interrupted by User input of 'x' 
  // if We have reached a full shutdown: below absolute minimum voltage.
  while (LARA.ON) {

    switch (LARA.PHASE) {

    // PHASE 1: Recording WISPR
    case 1:
      PhaseOne();
      break;

    // PHASE 2: Ascend ICE Housing
    case 2:
      PhaseTwo();
      break;

    // PHASE 3: Call into Satellite
    case 3:
      Time(&PwrOff);
      PwrOff -= PwrOn;
      // global: static char uploadfile[] = "c:00000000.dat
      // VEEPROM: SystemParameters MPC;
      sprintf(&uploadfile[2], "%08ld.dat", MPC.FILENUM);
      cprintf("\n\t|File Number: %08ld", MPC.FILENUM);
      // writefile 1) MPC 2) Winch Info 3) Winch Status
      // v
      WriteFile(PwrOff);
      // Init New LogFile, set PwrOn which is start of dataxint cycle
      Time(&PwrOn);

      PhaseThree();
      break;

    // PHASE 4:  Descend ICE Housing
    case 4:
      PhaseFour();
      break;
    }
  } // while lara.on

  flogf("\nLARA.ON == false\n");
  free(WriteBuffer);
  free(returnstr);
  shutdown();
} //____ Main() ____/

/***
 * shutdown
 */
void shutdown() {
  WISPRSafeShutdown();

  PIOClear(ANTMODPWR); // Make sure Iridium is Off
  PIOClear(DIFARPWR); // Make sure DIFAR Power Out is off
  PIOClear(AMODEMPWR); // Clear AModem Power

  SleepUntilWoken();
  BIOSReset();
}

/*********************************************************************************\
** InitializeAUH
\*********************************************************************************/
void InitializeLARA(ulong *PwrOn) {
  float velocity, depth;
  ulong time = NULL;
  short prevMode;
  short hour;
  ulong prevTime;
  float volts;
  bool check = false;

  // Get the system settings running
  TUInit(calloc, free);
  PIOMirrorList(mirrorpins);
  PIOSet(DIFARPWR);
  SetupHardware();
  // Count Down Timer to Start Program or go to PICO dos/Settings
  PreRun();
  // Get all Platform Settings
  GetSettings();

  // First Safety Catch. If woken. Reboot
  if (MPC.STARTUPS >= MPC.STARTMAX) {
    SleepUntilWoken();
    BIOSReset(); // Full hardware reset
  }

  // Identify IDs
  flogf("\nProgram: %s  Project ID: %s   Platform ID: %s  Boot ups: %d",
        MPC.PROGNAME, MPC.PROJID, MPC.PLTFRMID, MPC.STARTUPS);

  // Get Power On Time
  Time(&time);
  *PwrOn = time;

  flogf("\nProgram Start time: %s", TimeDate(NULL));

  // WINCH Struct Init
  WINCH.ASCENTCALLS = 0;
  WINCH.ASCENTRCV = 0;
  WINCH.DESCENTCALLS = 0;
  WINCH.DESCENTRCV = 0;
  WINCH.STOPCALLS = 0;
  WINCH.STOPRCV = 0;
  WINCH.BUOYRCV = 0;
  WINCH.WINCHCALLS = 0;
  // LARA Struct Init
  LARA.PHASE = 0; // Decided Below
  LARA.DATA = false;
  LARA.ON = true;
  LARA.BUOYMODE = 0;     // Also Decided Below
  LARA.SURFACED = false; // Decided Below
  LARA.DEPTH = 0;        // Received from first CTD
  LARA.MOORDEPTH = 0;    // Eventually init
  LARA.TOPDEPTH = 0; // depth at start of descent // Eventually Init
  LARA.TDEPTH = 0;       // Decided from Param file
  LARA.AVGVEL = 0;

  // Get Free Disk Space
  Free_Disk_Space(); // Does finding the free space of a large CF card cause
                     // program to crash? or Hang?

  // best to init GPSIRID first
  GPSIRID_Init();
  CTD_Init();
  // startup sets sync mode
#ifdef DEBUGdeploy
#else
  CTD_Start_Up(DEVA, true);
#endif
  CTD_Start_Up(DEVB, true);
  DevSelect(DEVX); // turn antmod off // ??

  // If initializing after reboot... Write previous WriteFile for upload
  if (MPC.STARTUPS > 0) {
    MPC.FILENUM--;
    sprintf(&uploadfile[2], "%08ld.dat", MPC.FILENUM);
    prevTime = WriteFile(0); // By Sending 0 we understand this is the file
                             // being salvaged from a reboot when analyzing over
                             // RUDICS, also this zero is passed to
                             // Power_Monitor to properly calculate previous
                             // startups power consumption.

    MPC.FILENUM++;
  } else // MPC.STARTUPS == 0
    Setup_ADS(true, MPC.FILENUM, BITSHIFT); // not done > 0 ??

  volts = Voltage_Now();
  flogf("\n\t|Check Startup Voltage: %5.2fV", volts);

  // Safety Check. Absoleute Minimum Voltage
  if (volts < MIN_BATTERY_VOLTAGE) {
    flogf("\n\t|Battery Voltage Below Minimum. Activate Hibernation Mode");
    SleepUntilWoken();
    BIOSReset();
  }
  // Else lower than user set minimum.
  else if (volts < atof(ADS.MINVOLT)) {
    WISP.DUTYCYCL = 0; // Booting up with default.cfg will never have a set
                       // value for turning on the WISPR. WISPR can only be
                       // turned on again if
    VEEStoreShort(DUTYCYCLE_NAME, WISP.DUTYCYCL);
    // Possibility of WISPR still being on after reboot. Shut it down.
    OpenTUPort_WISPR(true);
    WISPRSafeShutdown();
    OpenTUPort_WISPR(false);
    LARA.LOWPOWER = true;
  } else
    LARA.LOWPOWER = false;

  CTD_CreateFile(MPC.FILENUM); // for science, descent

  // Initialize More System Parameters
  LARA.PAYOUT = -1;
  LARA.ASCENTTIME = -1;
  LARA.DESCENTTIME = -1;
  LARA.STATUS = -1;
  // LARA.SWITCH;
  LARA.CTDSAMPLES = 0;

  // If First startup, parse system.cfg file and initialize all WISPR Boards.
  if (MPC.STARTUPS == 0) {
    GlobalRestart = false;
    ParseStartupParams(false);
    LARA.BUOYMODE = 0;
    LARA.PHASE = 5; // deploy phase
    LARA.SURFACED = false;
    Make_Directory("SNT");
    Make_Directory("CTD");
    Make_Directory("LOG");
    if (WISPRNUMBER > 1) { // total number
      if (WISP.NUM != 1)
        WISP.NUM = 1; // current used board

      OpenTUPort_WISPR(true);

      // Gather all #WISPRNUMBER freespace and sync time.
#ifdef DEBUGdeploy
#else
      GatherWISPRFreeSpace();
#endif
    }

  }
  // Startups>0 Go to default callmode @ callhour.
  else {
    GlobalRestart = true;
    LARA.PHASE = 0; // Phase 0 when finding the winch status and ctd position as
                    // to place the Buoy in the proper phase
    // Get previous call mode
    prevMode = IRID.CALLMODE;
    flogf("\nLast Irid call mode: %d", prevMode);
    // Depending on callmode, give variable hour the call trigger variable.
    if (prevMode == 0)
      hour = MPC.DATAXINT;
    else
      hour = IRID.CALLHOUR;
    // Parse startups, needed to set target depth back to a safe value.
    ParseStartupParams(true);
    // Force IRID.CALLMODE to one even if default.cfg parses a 0
    IRID.CALLMODE = 1;

    // check for motion
    depth = CTD_AverageDepth(6, &velocity);
    DevSelect(DEVX);

    // Place Buouy in correct state
    if (depth > NIGK.TDEPTH) {
      LARA.SURFACED = false;
      if (velocity < -.05) {
        flogf(
            "\n\t|Ascending Velocity Calculated. Phase 2 && BuoyMode 1"); // Ascending
        LARA.BUOYMODE = 1;
        LARA.PHASE = 2;
      } else if (velocity > 0.5) { // Descending
        flogf("\n\t|Descending Velocity Calculated. Phase 4 && BuoyMode 2");
        LARA.PHASE = 4;
        LARA.BUOYMODE = 2;
      } else {
        flogf(
            "\n\t|At depth and no velocity. Phase 1 && BuoyMode 0"); // Stationary
                                                                     // at depth
        LARA.PHASE = 1;
        LARA.BUOYMODE = 0;
        LARA.MOORDEPTH = LARA.DEPTH;
        CheckTime(prevTime, prevMode, hour);
      }
    } // depth> TD
    else if (depth <= NIGK.TDEPTH) { 
      // Upon restart, Winch must be at or near surface.
      // if((Calltime-hours now)-prevTime)<3600 seconds)
      // try calling... phase 3? What tuports needed?
      LARA.SURFACED = true;
      //   		LARA.BUOYMODE=1; //Removed after first Lake W. Deployment.
      //   This skips the ascent call in phase two
      if (prevTime != 0) {
        if (CheckTime(prevTime, prevMode, hour))
          LARA.PHASE = 3;
        else
          LARA.PHASE = 4;
      } else
        LARA.PHASE = 4;
    }

    PrintSystemStatus();

    if (LARA.PHASE == 0)
      LARA.PHASE = 1;
  }

  LARA.TDEPTH = NIGK.TDEPTH;

  // Store Number of Bootups
  MPC.STARTUPS++;
  VEEStoreShort(STARTUPS_NAME, MPC.STARTUPS);
  if (MPC.STARTUPS >= MPC.STARTMAX) {
    WISPRSafeShutdown();
    PIOClear(ANTMODPWR);
    PIOClear(AMODEMPWR);
    PIOClear(DIFARPWR);
    SleepUntilWoken();
    BIOSReset();
  }
  if (NIGK.RECOVERY) {
    MPC.DATAXINT = 30;
    VEEStoreShort(DATAXINTERVAL_NAME, MPC.DATAXINT);
  }

  LARA.ON = true;
  TickleSWSR();

  ADCounter = 0;

  create_dtx_file(MPC.FILENUM);

/*
  DBG({
  TUChParams *tuch; 
  tuch = TUGetDefaultParams();
  flogf("\nTUChParams: baud %ld, rx %d, tx %d, pf %d",
    tuch->baud, tuch->rxqsz, tuch->txqsz, tuch->tpfbsz);
  tuch->rxqsz=4096;
  tuch->txqsz=4096;
  TUSetDefaultParams(tuch);
  tuch = TUGetDefaultParams();
  flogf("\nTUChParams: baud %ld, rx %d, tx %d, pf %d",
    tuch->baud, tuch->rxqsz, tuch->txqsz, tuch->tpfbsz);
  })
  */
  
/*
 * typedef struct {
 * short bits; // data bits exclusive of start, stop, parity
 * short parity; // parity: 'o','O','e','E', all else is none
 * short autobaud; // automatically adjust baud when clock changes
 * long baud; // baud rate
 * short rxpri; // receive channel TPUPriority
 * short txpri; // transmit channel TPUPriority
 * short rxqsz; // receive channel queue buffer size
 * short txqsz; // transmit channel queue buffer size
 * short tpfbsz; // transmit channel printf buffer size
 * } TUChParams;
 */

} //____ InitializeAUH() ____//
/****************************************************************************\
** PhaseOne
** The initial recording and detecting phase while ICE housing is at/near winch.
\****************************************************************************/
void PhaseOne() {
  flogf("\n\t|PHASE ONE");

  // Initialize System Timers
  Check_Timers(Return_ADSTIME());

  if (WISP.DUTYCYCL < 100)
    if (WISPR_Status())
      WISPRSafeShutdown();

  // Stay here until system_timer says it's time to send data, user input for
  // different phase, NIGK R
  while (!LARA.DATA && LARA.PHASE == 1) {

    Sleep();
    Incoming_Data();
    if (System_Timer() == 2)
      LARA.DATA = true;

    if (!LARA.ON)
      break;
  }

  // This would mean the profiling buoy is at//near surface.
  // if (NIGK.RECOVERY && LARA.DEPTH < NIGK.TDEPTH)
  if (NIGK.RECOVERY && LARA.DEPTH < LARA.TDEPTH)
    LARA.PHASE = 3;
  if (LARA.DATA)
    LARA.PHASE = 2;

} //____ PhaseOne ____//
/****************************************************************************\
** PhaseTwo
** Ascending Phase of Winch.
\****************************************************************************/
void PhaseTwo() {
  ulong AscentStart, AscentStop, timeChange;
  float depthChange;
  float velocity = 0.0;
  int halfway;

  flogf("\n\t|PHASE TWO: Target Depth:%d", LARA.TDEPTH);
  //flogf("\n\t|PHASE TWO: Target Depth:%d", NIGK.TDEPTH);
  // turn off wispr, close wispr ?? here? 
  // check storm warn
  if (WISPR_Status()) { // moved here from p3
    WISPRSafeShutdown();
  }
  OpenTUPort_WISPR(false);
  OpenTUPort_NIGK(true);
  PrintSystemStatus();

  CTD_Select(DEVA);
  LARA.DEPTH = CTD_AverageDepth(6, &velocity);

  // Coming here from phase one. Induced by system_timer==2
  if (LARA.DATA) {
    LARA.MOORDEPTH = LARA.DEPTH;
    LARA.DATA = false;
  }

  // Else, sensor package deeper than target depth. Ascend.
  if (CurrentWarning()) {
  }
  if (LARA.BUOYMODE != 1) {
    AscentStart = Winch_Ascend();
    CTD_Sample();
    WaitForWinch(1);
  }

  if (LARA.DEPTH < LARA.TDEPTH) {
  // if (LARA.DEPTH < NIGK.TDEPTH) {
    flogf("\n\t|Profiling Float Already at target depth");
    LARA.TOPDEPTH = LARA.DEPTH;
    LARA.SURFACED = true;
    LARA.PHASE = 3;
    OpenTUPort_NIGK(false);
    return;
  }

  // Increment Profile number...
  NIGK.PROFILES++;
  VEEStoreShort(NIGKPROFILES_NAME, NIGK.PROFILES);

  // halfway to tdepth, +2 to allow for coasting
  halfway = ((LARA.DEPTH - LARA.TDEPTH) / 2) + LARA.TDEPTH + 2;
  // What's the best way out of this loop? Do we set a time limit for ascent?
  while ((!LARA.SURFACED || LARA.BUOYMODE == 1) && LARA.PHASE == 2) {
    Incoming_Data();

    if (LARA.DEPTH <= halfway) {
      AscentStop = Winch_Stop();
      WaitForWinch(0);
      if (CurrentWarning()) {
      }
      AscentStart = Winch_Ascend();
      CTD_Sample();
      WaitForWinch(1);
      // continue
      halfway=0;
    }

    // What if winch tells us its stopping? What AscentStop time do we get?
    if (LARA.DEPTH <= LARA.TDEPTH) {
    // if (LARA.DEPTH <= NIGK.TDEPTH) {
      cprintf("\n\t|REACHED TARGETDPETH!");
      AscentStop = Winch_Stop();
      WaitForWinch(0);
      LARA.TOPDEPTH = LARA.DEPTH;
      // If we stop at the target Depth
      if (LARA.TOPDEPTH <= LARA.TDEPTH)
      // if (LARA.TOPDEPTH <= NIGK.TDEPTH)
        LARA.SURFACED = true;

      LARA.AVGVEL = CTD_CalculateVelocity();
      if (LARA.AVGVEL == 0.0)
        LARA.AVGVEL = ((float)NIGK.RRATE / 60.0);
      break;
    }
    if (!LARA.ON) break;
  }

  depthChange = LARA.TOPDEPTH - LARA.MOORDEPTH;
  timeChange = AscentStop - AscentStart;
  LARA.PAYOUT = ((float)LARA.ASCENTTIME / 60.0) * NIGK.RRATE;
  flogf("\n\t|Rate of Ascent: %5.2fMeters/Minute",
        (depthChange / ((float)timeChange / 60.0)));
  flogf("\n\t|Calculated Cable Payout: %5.1fMeters", LARA.PAYOUT);
  flogf("\n\t|Time for Ascent: %lu", timeChange);
  PrintSystemStatus();

  if (LARA.SURFACED)
    LARA.PHASE = 3;

  OpenTUPort_NIGK(false);

} //____ PhaseTwo ____//
/****************************************************************************\
** Phase Three
** Testing iridium/gps connection. 
** If failed, release winch cable another meter or two.
** repeat to minimum CTD depth.
\****************************************************************************/
void PhaseThree() {
  short result = 0;
  int gpsFails = 0;
  short count = 0;
  static short IridCallsNoParams = 0;
  char filenum[9] = "00000000";
  flogf("\n\t|PHASE THREE");

  // should do this at boot
  if (GlobalRestart) { 
    // Added 9.28.2016 after first deployment Lake W.
    ParseStartupParams(true); 
  } 

  while (result <= 0) { 
    // -1=false gps, -2=false irid, 1=success 2=fake cmds 3=real cmds
    // DBG( Incoming_Data();)
    result = IRIDGPS(); 

    if (result >= 1 || gpsFails > 4) {
      // IRIDIUM Successful success/fake/real/5th, next phase
      LARA.PHASE = 4;
    }
    if (result == 1 || result == 2) { 
      // Upload Success / Commands
      IridCallsNoParams++; // call sessions w/o Params
      flogf("\n\t|Successful IRID Call: %d", IridCallsNoParams);
      if (IridCallsNoParams > 3) {
        // Load default parameters in "default.cfg" file
        ParseStartupParams(true); 
      } // calls>3
    } // Upload Success / Commands
    else if (result == 3) { 
      // Real Commands
      ParseStartupParams(false);
      IridCallsNoParams = 0;
      flogf("\n\t|Successful IRID Call: %d", IridCallsNoParams);
    } // Real Commands
    else if (result == -2) {
      // GPS Success, IRID Fail
      flogf("\n\t|PhaseThree(): Failed Iridium Transfer");
      IridCallsNoParams++;
      if (IridCallsNoParams > 3) {
        IridCallsNoParams = 0;
        ParseStartupParams(true);
      } // calls>3
      LARA.PHASE = 4;
      break;
    } // GPS Success, IRID Fail
    else if (result == -1) {
      gpsFails++;
      // Bad GPS- GPS fails usually from bad reception.
      flogf("\n\t|PhaseThree(): Failed GPS attempt: %d", gpsFails);
      if (gpsFails >= 5) {
        flogf("\n\t|Exiting PhaseThree()");
        // ?? close ports?
        break;
      } // >=5
      // removed section, go up further
    } // Bad GPS- GPS fails usually from bad reception
  } // while result<=0

  // in recovery, stay on surface 
  if (NIGK.RECOVERY) LARA.PHASE = 1; 
  // NIGK.RECOVERY may be cleared by Params load 
  
  GlobalRestart = false;
  MPC.FILENUM++;
  sprintf(filenum, "%08ld", MPC.FILENUM);
  VEEStoreStr(FILENUM_NAME, filenum);
  create_dtx_file(MPC.FILENUM);
  CTD_CreateFile(MPC.FILENUM); 
  LARA.TDEPTH = NIGK.TDEPTH;

  LARA.DATA = false;

} //____ PhaseThree ____//
/****************************************************************************\
** PhaseFour
\****************************************************************************/
void PhaseFour() {

  float depthChange = 0.0;
  float velocity, descentvelocity;
  ulong timeChange = 0, DescentStart, DescentStop;
  ulong timecheck = 0;
  static float prevDepth = 0;
  int interval = 1;

  flogf("\n%s|Phase_Four():", Time(NULL));
  OpenTUPort_NIGK(true);
  // if (WISP.DUTYCYCL > 50) { // moved here from p3
    // OpenTUPort_WISPR(true);
    // WISPRPower(true);
  // }

  LARA.SURFACED = false;

  PrintSystemStatus();
  // sanity check
  CTD_AverageDepth(6, &velocity);
  if (LARA.BUOYMODE != 0) {
    Winch_Stop();
    WaitForWinch(0);
    flogf("\nErr PhaseFour(): buoy was in motion");
  }
  //
  // turn off antenna, which selects buoy ctd
  DevSelect(DEVX);
  // redundant?
  CTD_Select(DEVB);

  // Now descend.
  if (LARA.BUOYMODE != 2) {
    LARA.TOPDEPTH = LARA.DEPTH;
    DescentStart = Winch_Descend();
    WaitForWinch(2);
    CTD_Sample();
  } else
    CTD_Sample();

  /****
  ADD IN CHECK HERE. if WaitForWinch returns false because of timeout and lack
  of AModem Response: then check CTD Average depth and make sure Velocity>0.25
  (descending.)
  If not, then call Winc_Descend Again. After 10, wait for an hour
  ****/

  prevDepth = LARA.DEPTH;

  while (LARA.BUOYMODE == 2) {
    // reading a sample triggers a new one, in p4
    Incoming_Data();

    // Receive Stop command from Winch...
    if (LARA.BUOYMODE == 0)
      DescentStop = (time(NULL) - (ulong)(NIGK.DELAY));

    timecheck = time(NULL);
    // Check depth change every 60 seconds... This is an out of the while loop.
    // Hopefully it will only come here when TUPort to the AModem stops LARA
    // doesn't hear the Winch serial coming.
    if (timecheck - DescentStart > interval * 180) { // ?? known prob w ctd read
      flogf("\n\t|PhaseFour() Check depth change");
      prevDepth -= LARA.DEPTH;
      if (prevDepth < 0)
        prevDepth = prevDepth * -1.0;
      if (prevDepth < 3) {
        flogf("\nERROR|Depth change less than 3 meters... Winch hasn't "
              "responded STOP");
        DescentStop = Winch_Stop();
        WaitForWinch(0);
      }
      prevDepth = LARA.DEPTH;
      interval++;
    }
  } // while mode==2

  if (LARA.BUOYMODE == 0)
    LARA.MOORDEPTH = LARA.DEPTH;

  // Descent Velocity;
  descentvelocity = CTD_CalculateVelocity();
  flogf("\n\t|Calculated Descent Velocity: %.2fm/s", descentvelocity);

  // Total Vertical depth change. total time change, calculate estimated
  // velocity.
  depthChange = LARA.MOORDEPTH - LARA.TOPDEPTH;
  LARA.DESCENTTIME = (short)(DescentStop - DescentStart);
  flogf("\n\t|Rate of Descent: %fMeters/Minute",
        (depthChange / ((float)LARA.DESCENTTIME / 60.0)));
  flogf("\n\t|User calculated Cable Payout: %fMeters",
        ((float)LARA.DESCENTTIME / 60.0) * NIGK.FRATE);
  flogf("\n\t|Time for Descent: %d", LARA.DESCENTTIME);
  PrintSystemStatus();
  Delay_AD_Log(2);
  OpenTUPort_NIGK(false);

  LARA.PHASE = 1;
  LARA.DATA = false;

} //____ Phase_Four() ____//

/*
 * PhaseFive for deploy time
 */
void PhaseFive() {
  ulong deployT, maxT;
  float nowD=0.0, thenD=0.0;
  int changeless=0;

  deployT = RTCGetTime(NULL, NULL);
  // give up after two hours
  maxT = (ulong) (2*60*60);
  // do nothing for 30minutes
  Delay_AD_Log(30*60);
#ifdef DEBUGdeploy
  CTD_Select(DEVB);
#else
  CTD_Select(DEVA);
#endif
  Delay_AD_Log(9);
  LARA.DEPTH=0.0;
  flogf("\n\s\t|P5: wait until >10m", Time(NULL));
  // wait until deeper than 10m
  while (LARA.DEPTH<10.0) {
    CTD_Sample();
    Delay_AD_Log(3);
    if (!  CTD_Data()) 
      flogf("\nERR in P5 - no CTD data");
    Delay_AD_Log(30);
    if ((RTCGetTime(NULL, NULL) - deployT) > maxT) break; // too long
  }
  flogf("\n\s\t|P5: wait until no depth changes", Time(NULL));
  // check every half minute; if no change for five minutes, then deployed
  while (changeless<10) {
    thenD=LARA.DEPTH;
    CTD_Sample();
    Delay_AD_Log(3);
    if (CTD_Data()) nowD=LARA.DEPTH;
    else  flogf("\nERR in P5 - no CTD data");
    if (abs(nowD-thenD) > 1) { // changed
      flogf("\n\s\t|P5: depth change %4.1f", Time(NULL), (nowD-thenD));
      changeless=0;
    } else changeless++;
    Delay_AD_Log(30);
    if ((RTCGetTime(NULL, NULL) - deployT) > maxT) break; // too long
  }
  // deployed!
  flogf("\n\s\t|P5: deployed", Time(NULL));
  LARA.PHASE=1;
} // PhaseFive()

/****************************************************************************\
** int Incoming_Data()
 * ?? very fragile, caution
\****************************************************************************/
int Incoming_Data() {
  bool incoming = true;
  static int count = 0;
  int value = 0; // Need to update this if we ever need to return a legit value.
                 // -AT 20151215
  switch (LARA.PHASE) {
  // Case 0: Only at startup when MPC.STARTUPS>0
  case 0:
    while (incoming) {
      AD_Check();
      if (tgetq(NIGKPort)) {
        // DBG(flogf("NIGK Incoming");)
        AModem_Data();
      } else if (tgetq(devicePort)) { // ?? very messy handling of ctd
        // DBG(flogf("CTD Incoming");)
        CTD_Data();
        if ((!LARA.SURFACED && LARA.PHASE > 1) || LARA.BUOYMODE > 0 ||
            LARA.PHASE == 0) // if not surfaced (target depth not reached.) and
                             // winch is moving (not stopped)
          CTD_Sample();
      } else if (cgetq()) {
        // DBG(flogf("Console Incoming");)
        Console(cgetc());
      } else
        incoming = false;
    }
    break;

  // CASE 1: MOORED AUH
  case 1:

    while (incoming) {
      AD_Check();
      // Data coming from WISPR Board
      if (tgetq(PAMPort)) {
        // DBG(flogf("WISPR Incoming");)
        WISPR_Data();
      } else if (tgetq(devicePort)) {
        CTD_Data();
      }
      // Console Wake up.
      else if (cgetq()) {
        // DBG(flogf("Console Incoming");)
        Console(cgetc());
      }
      // No more incoming data
      else
        incoming = false;
    }
    break;

  // CASE 2&4: WINCH IN ACTION. Incorporate AMODEM in here
  case 2:
  case 4:

    while (incoming) {
      // ?? does adcheck need to run between each incoming? how often?
      AD_Check();
      if (tgetq(PAMPort)) {
        // DBG(flogf("WISPR Incoming");)
        WISPR_Data();
      } else if (tgetq(NIGKPort)) {
        // DBG(flogf("NIGK Incoming");)
        AModem_Data();
      } else if (tgetq(devicePort)) {
        // DBG(flogf("CTD Incoming");)
        //?? CTD_Data();
        //?? if ((!LARA.SURFACED && (LARA.PHASE == 2 || LARA.PHASE == 4)) ||
            //?? LARA.BUOYMODE > 0) // if not surfaced (target depth not reached.)
                               // and winch is moving (not stopped)
          //?? CTD_Sample();
      } else if (cgetq()) {
        // DBG(flogf("Console Incoming");)
        Console(cgetc());
      }
      // No more incoming data
      else
        incoming = false;
    }
    break;

  // CASE 3: GPSIRID
  case 3:
    while (incoming) {
      AD_Check();
      if (tgetq(PAMPort)) {
        // DBG(flogf("WISPR Incoming");)
        WISPR_Data();
      } else if (cgetq()) {
        // DBG(flogf("Console Incoming");)
        Console(cgetc());
      } else
        incoming = false;
    }
    break;

  default:
    // Lost winch phase. Get LARA System Status decided which phase it best
    // belongs.

    cprintf("default switch");
    break;
  }

  PutInSleepMode = true;

  return value;

} //_____ Incoming_Data() _____//
/************************************************************************************************************************\
** void Console
** Platform Specific Console Communication
\************************************************************************************************************************/
void Console(char in) {
  // are there side effects from any subroutines?
  // shutdown from here
  short c;

  DBG(flogf("Incoming Char: %c", in);)
  Delayms(2);
  switch (LARA.PHASE) {
  case 1:
    switch (in) {
    case 'I':
    case 'i':
      WISPRPower(true);
      break;

    case 'E':
    case 'e':
      WISPRSafeShutdown();
      break;

    case 'D':
    case 'd':
      flogf("\n\t|REQUEST # DTX FROM WISPR?");
      c = cgetc() - 48;
      WISPRDet(c);
      break;

    case 'F':
    case 'f':
      WISPRDFP();
      break;

    case 'W':
    case 'w':
      flogf("\n\t|CHANGE TO WISPR #?");
      c = cgetc() - 48;
      ChangeWISPR(c);
      break;

    case 'P':
    case 'p':
      flogf("\n\t|CHANGE OF PHASE #?");
      c = cgetc() - 48;
      flogf(" Phase: %d", c);
      LARA.PHASE = c;
      break;

    case 'x':
      // LARA.ON = false;
      // LARA.DATA = LARA.DATA ? false : true;
      shutdown();
      break;
    case '2':
      LARA.DATA = LARA.DATA ? false : true;
      break;
    }
    break;

  case 2:
  case 4:
    switch (in) {
    case 'W':
    case 'w':
      WinchConsole();
      break;
    case 'P':
    case 'p':
      flogf("\n\t|CHANGE OF PHASE #?");
      c = cgetc() - 48;
      LARA.PHASE = c;
      break;
    case 'T':
    case 't':
      flogf("\n\t|Take CTD Sample");
      CTD_Sample();
      break;
    case 'x':
      // LARA.ON = false;
      // LARA.DATA = LARA.DATA ? false : true;
      shutdown();
      break;
    case 'a':
    case 'A':
      PrintSystemStatus();
      break;
    case 'S':
    case 's':
      LARA.SURFACED = true;
      break;
    }
    break;

  case 3:
    switch (in) {
    case 'x':
      // LARA.ON = false; // exit from GPSIRID
      shutdown();
      break;
    case 'P':
    case 'p':
      flogf("\n\t|Change of Phase #?");
      c = cgetc() - 48;
      LARA.PHASE = c;
      break;
    }
    break;
  }

  // ?? PutInSleepMode = true;
  return;
}
/******************************************************************************\
**	ExtFinishPulseRuptHandler		IRQ5 logging stop request
interrupt
**
\******************************************************************************/
IEV_C_FUNCT(ExtFinishPulseRuptHandler) {
#pragma unused(ievstack) // implied (IEVStack *ievstack:__a0) parameter

  PinIO(IRQ5);

  PinRead(IRQ5);

} //____ ExtFinishPulseRuptHandler() ____//
/******************************************************************************\
**	SleepUntilWoken		Finish up
**
** 1-st release 9/14/99
** 2nd release 6/24/2002 by HM -Changed to use ADS8344/45
\******************************************************************************/
void Sleep(void) {

  Delayms(10);
  IEVInsertAsmFunct(IRQ4_ISR, level4InterruptAutovector); // Console Interrupt
  IEVInsertAsmFunct(IRQ4_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ5_ISR, level5InterruptAutovector); // PAMPort Interrupt
  IEVInsertAsmFunct(IRQ5_ISR, spuriousInterrupt);

  SCITxWaitCompletion();
  EIAForceOff(true);
  CFEnable(false);

  PinBus(IRQ4RXD); // Console Interrupt
  while ((PinTestIsItBus(IRQ4RXD)) == 0)
    PinBus(IRQ4RXD);
  PinBus(IRQ5); // PAMPort Interrupt
  while ((PinTestIsItBus(IRQ5)) == 0)
    PinBus(IRQ5);

  while (PinRead(IRQ5) && PinRead(IRQ4RXD) && !data)
    LPStopCSE(FastStop); // we will be here until interrupted

  EIAForceOff(false); // turn on the RS232 driver
  CFEnable(true);     // turn on the CompactFlash card

  PIORead(IRQ4RXD);
  while ((PinTestIsItBus(IRQ4RXD)) != 0)
    PIORead(IRQ4RXD);
  PIORead(IRQ5);
  while ((PinTestIsItBus(IRQ5)) != 0)
    PIORead(IRQ5);

  PutInSleepMode = false;

  // DBG2(flogf(".");)
  Delayms(10);

} //____ Sleep() ____//
/******************************************************************************\
**	SleepUntilWoken		Finish up
**
** 1-st release 9/14/99
** 2nd release 6/24/2002 by HM -Changed to use ADS8344/45
\******************************************************************************/
void CTDSleep(void) {

  Delayms(10);
  IEVInsertAsmFunct(IRQ3_ISR, level3InterruptAutovector); // AModem Interrupt
  IEVInsertAsmFunct(IRQ3_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ2_ISR, level2InterruptAutovector); // devicePort/Seaglider
  IEVInsertAsmFunct(IRQ2_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ4_ISR, level4InterruptAutovector); // Console Interrupt
  IEVInsertAsmFunct(IRQ4_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ5_ISR, level5InterruptAutovector); // PAMPort Interrupt
  IEVInsertAsmFunct(IRQ5_ISR, spuriousInterrupt);

  SCITxWaitCompletion();
  EIAForceOff(true);
  CFEnable(false);

  PinBus(IRQ2);    // CTDPort Seaglider Interrupt
  PinBus(IRQ3RXX); // AModem
  PinBus(IRQ4RXD); // Console Interrupt
  PinBus(IRQ5);

  while (PinRead(IRQ2) && PinRead(IRQ5) && PinRead(IRQ3RXX) &&
         PinRead(IRQ4RXD) && !data)
    LPStopCSE(CPUStop); // we will be here until interrupted

  EIAForceOff(false); // turn on the RS232 driver
  CFEnable(true);     // turn on the CompactFlash card
                      /*
                         PIORead(IRQ4RXD);
                              PIORead(IRQ3RXX);
                         PIORead(IRQ2);
                         PIORead(IRQ5);
                      */
  PutInSleepMode = false;

  //   DBG2(flogf(",");)
  Delayms(10);

} //____ Sleep() ____//
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
              //			BIOSResetToPicoDOS();
              //}
} //____ SleepUntilWoken() ____//
/*************************************************************************\
**  static void Irq3ISR(void)
\*************************************************************************/
static void IRQ3_ISR(void) {
  PinIO(IRQ3RXX);
  RTE();
} //____ Irq3ISR ____//
/*************************************************************************\
**  static void Irq2RxISR(void) CTD/Seaglider/ IRIDIUM
\*************************************************************************/
static void IRQ2_ISR(void) {
  PinIO(IRQ2);
  RTE();
} //____ Irq2RxISR ____//
/*************************************************************************\
**  static void IRQ4_ISR(void) CONSOLE
\*************************************************************************/
static void IRQ4_ISR(void) {
  PinIO(IRQ4RXD);
  RTE();
} //____ Irq2ISR() ____//
/*************************************************************************\
**  static void IRQ5_ISR(void) WISPR
\*************************************************************************/
static void IRQ5_ISR(void) {
  PinIO(IRQ5);
  RTE();
} //____ Irq5ISR() ____//
/******************************************************************************\
**	WriteFile      The Data File For Lara
1) Initially upload MPC parameters
2) Winch Info
3) Winch Status
\******************************************************************************/
ulong WriteFile(ulong TotalSeconds) {

  long BlkLength = STRING_SIZE;
  int filehandle;
  struct stat info;
  char detfname[] = "c:00000000.dtx";
  char logfile[sizeof "00000000.log"];
  int byteswritten = 0;
  int bytesWritten = 0;
  long maxupload;
  ulong LoggingTime;

  flogf("\n\t|WriteFile(%s)", uploadfile);
  filehandle = open(uploadfile, O_WRONLY | O_CREAT | O_TRUNC);

  if (filehandle <= 0) {
    flogf("\nERROR  |WriteFile(): open error: errno: %d", errno);
    return -1;
  }
  DBG2(else flogf("\n\t|WriteFile: %s Opened", uploadfile);)

  //*** LARA Write ***//
  sprintf(WriteBuffer, "LARA Program Ver:%.1f\naa:bb.cccc North ddd:ee.ffff "
                       "West\nFileNumber: %ld\nStarts:%d of "
                       "%d\nWriteTime:%s\nSeconds:%lu\nDetection "
                       "Interval:%3d\nData Inteval: %d\n\0",
          PROG_VERSION, MPC.FILENUM, MPC.STARTUPS, MPC.STARTMAX, Time(NULL),
          TotalSeconds, MPC.DETINT, MPC.DATAXINT);

  flogf("\n%s", WriteBuffer);
  bytesWritten = write(filehandle, WriteBuffer, strlen(WriteBuffer));
  DBG2(flogf("\nBytesWritten: %d", bytesWritten);)

  // Only comes here if not rebooted.
  if (TotalSeconds != 0) {
    //*** Winch Info   ***//
    Winch_Monitor(filehandle);
    Delayms(50);
    memset(WriteBuffer, 0, STRING_SIZE);

    //*** Winch Status ***//
    sprintf(WriteBuffer, "%s\n\0", PrintSystemStatus());
    bytesWritten = write(filehandle, WriteBuffer, strlen(WriteBuffer));
    DBG2(flogf("\nBytesWritten: %d", bytesWritten);)
  }
  // Else, coming from reboot. Name the PowerLogging File.
  else {
    ADSFileName(MPC.FILENUM);
    Delayms(50);
  }

  //*** Power Monitoring Upload ***//
  Power_Monitor(TotalSeconds, filehandle, &LoggingTime);
  Delayms(50);
  Setup_ADS(true, MPC.FILENUM + 1, BITSHIFT);
  DOS_Com("del", MPC.FILENUM, "PWR", NULL);
  Delayms(500);

  //*** WISPR File Upload ***//
  WISPRWriteFile(filehandle);
  Delayms(50);

//*** CTD File Upload ***
#ifdef CTDSENSOR
  if (CTD.UPLOAD || TotalSeconds == 0) {
    sprintf(&detfname[2], "%08ld.ctd", MPC.FILENUM);
    Delayms(50);
    DBG(cprintf("\n\t|WriteFile:%ld ctd file: %s", MPC.FILENUM, detfname);)
    stat(detfname, &info);
    if (info.st_size > (long)(IRID.MAXUPL - 1000))
      maxupload = IRID.MAXUPL - 1000;
    else
      maxupload = 0;
    Append_Files(filehandle, detfname, false, maxupload);
  }
  // Despite being upload, move CTD file to archive
  DOS_Com("move", MPC.FILENUM, "CTD", "CTD");
  Delayms(50);
#endif
  //*** MPC.LOGFILE upload ***// Note: occurring only after reboot.
  if (TotalSeconds == 0) { //||MPC.UPLOAD==1)
    sprintf(logfile, "%08ld.log", MPC.FILENUM);
    DBG(cprintf("\n\t|WriteFile: %ld log file: %s", MPC.FILENUM, logfile);)
    stat(logfile, &info);
    if (info.st_size > (long)(IRID.MAXUPL - 2000))
      maxupload = IRID.MAXUPL - 2000;
    else
      maxupload = 0;
    Append_Files(filehandle, logfile, false, maxupload);
    Delayms(50);
  }
  Delayms(50);
  DOS_Com("move", MPC.FILENUM, "LOG", "LOG");

  // Close File
  close(filehandle);
  Delayms(25);

  // Return number of seconds of time on
  if (TotalSeconds == 0)
    return LoggingTime;
  else
    return 0;

} //____ WriteFile() ____//
/**************************************************************************************\
**	PrintSystemStatus()
\**************************************************************************************/
char *PrintSystemStatus() {
  // global returnstr
  sprintf(returnstr, "LARA: "
                        "%d%d%d%d\nMOORDEPTH:%5.2f\nCURRENTDEPTH:%5."
                        "2f\nTOPDEPTH:%5.2f\nTARGETDPETH:%d\nAVG.VEL:%5."
                        "2f\nCTDSAMPLES:%d\n\0",
          LARA.DATA ? 1 : 0, LARA.SURFACED ? 1 : 0, LARA.PHASE, LARA.BUOYMODE,
          LARA.MOORDEPTH, LARA.DEPTH, LARA.TOPDEPTH, LARA.TDEPTH, LARA.AVGVEL,
          LARA.CTDSAMPLES);
  flogf("\n%s", returnstr);
  Delayms(100);

  return returnstr;
}
/**************************************************************************************\
** WaitForWinch
\**************************************************************************************/
void WaitForWinch(short expectedBuoyMode) {
  ulong timenow;
  ulong BreakTime = time(NULL) + 13; // 13 second wait time for Winch Response;

  flogf("\n%s|WaitForWinch(): ExpectedBuoyMode: %d", Time(NULL),
        expectedBuoyMode);
  timenow = time(NULL);
  while (timenow < BreakTime) {

    if (tgetq(NIGKPort)) {
      AModem_Data();
      if (expectedBuoyMode == LARA.BUOYMODE) {
        flogf("\n%s|Received Expected BuoyMode", Time(NULL));
        return;
      } else if (expectedBuoyMode == -1)
        break;
      else if (expectedBuoyMode == 3)
        return;
    } else if (tgetq(devicePort)) {
      CTD_Data();

    } else
      Delayms(250);

    timenow = time(NULL);
  }

  if (LARA.BUOYMODE == expectedBuoyMode) // success
    flogf("\n\t|Successful BUOY Status Correction");

} //____ WaitForWinch() ____//
/**************************************************************************************\
** void CheckTime()
** Only comes here after reboot when MPC.STARTUPS>0
** Check the previous ADS File Time.
** If previous time was close to finishing go ahead and call in: Return True
** If previous time isn't close enough, return false
\**************************************************************************************/
bool CheckTime(ulong prevTime, short mode, short hour) {
  struct tm *info;
  ulong seconds, rawtime;
  int mins;
  short hours;
  bool returnvalue = false;
  int rem;

  time(&rawtime);
  info = gmtime(&rawtime);

  flogf("\n%s|CheckCallTime():", Time(NULL));
  // Time in seconds received from Power_Monitor
  flogf("\n\t|Time to check %lu ", prevTime);

  // Previous mode was set to interval calling
  if (mode == 0) {
    // variable hour here is really minutes in system variable: MPC.DATAXINT
    seconds = (ulong)hour * 60;
    flogf("of expected data interval time: %lu", seconds);
    // calculate difference in seconds until call time.
    if (prevTime > seconds)
      prevTime = seconds;
    seconds -= prevTime;
    flogf("\n\t|Remaining seconds from last startup: %lu", seconds);
    // Get minutes
    seconds = seconds / 60;
    flogf("minutes: %lu", seconds);

    mins = (int)seconds % 60;
    hours = (int)seconds / 60;
    mins = info->tm_min + mins;
    // If minutes now plus estimated minutes remaining is greater than one hour
    if (mins > 59) {
      flogf("\nmins now %d is greater than 59", mins);
      hours++;
    }
    // If hour 23 + 8 hours.
    hours = (short)info->tm_hour + hours;
    // If the system is estimated to call this hour. increment one hour.
    if (info->tm_hour == hours) {
      flogf("\n\t|Hour now and calculated call hour is same...");
      hours++;
      returnvalue = false;
    }
    if (hours > 23)
      hour -= 24;
    flogf("\nhour to call: %d", hours);
    IRID.CALLHOUR = hours;
    if (IRID.CALLHOUR > 23)
      IRID.CALLHOUR = 23;
    else if (IRID.CALLHOUR < 0)
      IRID.CALLHOUR = 0;
    VEEStoreShort(CALLHOUR_NAME, IRID.CALLHOUR);
    flogf("\n\t|New Call Hour: %d", IRID.CALLHOUR);

  }
  // Previous mode set to Time of Day calling
  else if (mode == 1) {
    flogf("\nhour now %d vs call hour %d", info->tm_hour, hour);

    rem = abs(info->tm_hour - hour);
    // If we are within one hour of calling anyways...
    if (rem <= 1 || rem >= 23) {
      flogf("\n\t|CheckTime(Less than an hour until call... call now)");
      // returnvalue= true;
    }
  }
  return returnvalue;

} //____ CheckTime() ____//
/**************************************************************************************\
** LARA_Recovery()
\**************************************************************************************/
void LARA_Recovery() {} //____ LARA_Recovery() ____//

/*
 * CurrentWarning() - current reduces distance between CTD's
 */
bool CurrentWarning() {
  float a, b;
  DBG(flogf("\nCurrentWarning()");)
  CTD_Select(DEVB);
  CTD_Sample();
  CTD_Data();
  b=LARA.DEPTH;
  CTD_Select(DEVA);
  CTD_Sample();
  CTD_Data();
  a=LARA.DEPTH;
  flogf("\n\t|CurrentWarning(): a=%5.2f, b=%5.2f", a, b);
  return false;
}
