/******************************************************************************\
**	Seaglider 3.0 Program
** 6/17/2016
** Alex Turpin
**
** Please view the Readme.txt file
*****************************************************************************
**	STATUS descriptions:
**    1  Waiting to Leave Surface/Waiting for cat/PAM/GPS commands
**    2  Waiting for MIN_WISPR_DEPTH before turning on.
**    3  WISPR on for Descent
**    4  WISPR on for Ascent while >SG.WISPROFFDEP
**    5  Turn WISPR OFF
**    6  Waiting to Reach Surface.
**

SEAGLIDER Communiaction 4800Baud
** TPU 1    22 1=MAX3223 ON
** TPU 2    23 0=SG Comm Select
** TPU 10   31 SEAGLIDER TX
** TPU 11   32 SEAGLIDER RX

WISPR BOARD Communication 9600Baud
** TPU 6    27 PAM WISPR TX
** TPU 7    28 PAM WISPR RX
** TPU 8    29 PAM1&2 Enable
** TPU 9    30 PAM1=0 PAM2=1
** TPU 15   37 PAM Pulse On
** MODCK    42 PAM Pulse Off

IRIDIUM GPS

SEAGLIDER SETTINGS
** SEAG.DIVENUM   Defines the current dive number of the Seaglider. Read in with
each depth reading. Updated at beginning of dive
** SEAG.POWERON   Defines Logger Start Depth
** SEAG.POWEROFF  Defines Logger Stop Depth

WISPR SETTINGS
** WISP.GAIN      WISPR Gain Level [0-3]
** WISP.DETMAX    Maximum # of detections returned per MPC.DETINT Timer [-1 -
MAX_DETECTIONS]
** WISP.DUTYCYCL  Integer number of duty cycle percentage. [0 - 100]
** WISP.DETNUM    # of dtx per call to initiate realtime upload of files. //Not
Implemented on Seaglider
** WISP.MODE      Mode of WISPR Board [1-5]
** WISP.NUM       If MULTIWISPR Defined. [1-4] //Not Implemented on Seaglider

MPC SETTINGS
** MPC.DETINT


INTERRUPTS
** IRQ4 Pin 43 Console Serial Interrupt
** IRQ2 Pin 39 SG Serial Interrupt
** IRQ5 Pin 31 PAM Serial Interrupt
**
\******************************************************************************/

#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions

#include <assert.h>
#include <ctype.h>
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <errno.h>
#include <errno.h>
#include <fcntl.h> // PicoDOS POSIX-like File Access Definitions
#include <float.h>
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
#include <termios.h> // PicoDOS POSIX-like Terminal I/O Definitions
#include <time.h>
#include <unistd.h> // PicoDOS POSIX-like UNIX Function Definitions

#include <dirent.h> // PicoDOS POSIX-like Directory Access Defines
#include <stat.h>   // PicoDOS POSIX-like File Status Definitions

#include <ADS.h>
#include <MPC_Global.h>
#include <PLATFORM.h>
#include <Settings.h> // For VEEPROM settings definitions

#include <WISPR.h>

// WDT Watch Dog Timer definition
// Not sure if this watchdog is even working You have to define
short CustomSYPCR = WDT419s | HaltMonEnable | BusMonEnable | BMT32;
#define CUSTOM_SYPCR CustomSYPCR // Enable watch dog  HM 3/6/2014
#define FIFO_SIZE 7

#define SGBAUD 4800L

// Define unused pins here
uchar mirrorpins[] = {1, 15, 16, 17, 18, 19, 21, 24, 25, 26, 28, 35, 36, 0};

void InitializeSG();
void Sleep();
int WriteFile(ulong);
int UploadFile(TUPort *);
int SG_Data();
void Console(char);
int SG_Depth_Check();
void CompileErrors(int);
void Log_Time(float);
void ParseTime(char *);
float AverageFIFO();
void Incoming_Data();
static void IRQ2_ISR(void);
static void IRQ4_ISR(void);
static void IRQ5_ISR(void);
void OpenTUPort_SG(bool);

SeagliderParameters SEAG;
SystemParameters MPC;

char ParamFile[sizeof "SYSTEM.CFG"];
char logfile[sizeof "00000000.log"];

FILE *file;
bool Shutdown;
bool Surfaced;
bool PutInSleepMode;
int errors[9];
long SystemFreeSpace;

bool NewDive;
extern bool WISPR_On;
bool ReadyToUpload;
float FIFO[FIFO_SIZE];
short FIFO_POS;
short Depth_Samples;

#ifdef WISPR
extern WISPRParameters WISP;
extern bool WISPR_On;
#endif

TUPort *SGPort;
short SG_STATUS;
float Prev_Depth;
float log_dep;
short log_verbose;

float max_dep;
float Depth;
ulong PwrOn, PwrOff;

char uploadfile[] = "c:00000000.dat";

/******************************************************************************\
**	Main
\******************************************************************************/
void main() {

  InitializeSG();
  PwrOff = 0;
  NewDive = true;

  // Main Program Loop
  while (!Shutdown) {

    if (PutInSleepMode == true)
      Sleep(); // Sleep to save power

    else
      Incoming_Data(); // Grabs data from TUPort Devices or logs AD_Data

    if (!Surfaced)
      System_Timer();

    if (Shutdown)
      break;

    // TickleSWSR();
  }

  if (WISPR_On) {
    WISPRSafeShutdown();
    WISPR_On = false;
  }

  flogf(". Program Resetting");

  // OpenTUPort("SG", false);
  // OpenTUPort("PAM", false);
  TURelease();

  RTCDelayMicroSeconds(100000L);

  BIOSReset();

} //____ Main() ____//

/*********************************************************************************\
** InitializeSG
\*********************************************************************************/
void InitializeSG() {
  ulong time;

  int i;

  // Turn off Pins not being used
  PIOMirrorList(mirrorpins);

  // Enable buffered operation
  SCIRxSetBuffered(true);
  SCITxSetBuffered(true);

  // Get Settings, set clock speed, etc.
  SetupHardware();

  // Countdown before program starts
  PreRun();

  // Identify IDs
  flogf("\nProject ID: %s   Platform ID: %s   Location: %s   Boot ups: %d",
        MPC.PROJID, MPC.PLTFRMID, MPC.LOCATION, MPC.STARTUPS);

  GetSettings();

  // Read in configuration file
  ParseStartupParams();

  // Gather Start Time Here
  Time(&time);
  PwrOn = time;

#ifdef POWERLOGGING
  Setup_ADS(true, (long)SEAG.DIVENUM);
#endif

  // Store Number of Bootups
  MPC.STARTUPS++;
  VEEStoreShort(STARTUPS_NAME, MPC.STARTUPS);

  // Initialize System Timers
  Check_Timers(Return_ADSTIME());

  // Initialize and open TPU UART
  TUInit(calloc, free);

  InitializeWISPR();

  OpenTUPort_WISPR(true);

  // Turn System on. Only Exit upon console '1' input or Parsing 'R' from
  // "RAOB(R)"
  Surfaced = true;
  Shutdown = false;
  ReadyToUpload = false; // Waits for "CAT" serial
  ADCounter = 0;

  SystemFreeSpace = Free_Disk_Space();

  create_dtx_file((long)SEAG.DIVENUM);
  OpenTUPort_SG(true);

  // Log Verbose defines how to log the Depth Measurement of Seaglider
  log_verbose = 2;
  log_dep = 0;

  FIFO_POS = 0;
  for (i = 0; i < FIFO_SIZE; i++)
    FIFO[i] = 0.0;
  Depth_Samples = 0;

  TickleSWSR();

} //____ InitializeSG() ____//

/****************************************************************************\
** void Incoming_Data()
\****************************************************************************/
void Incoming_Data(void) {
  bool incoming = true;

  while (incoming) {
    // Check A to D Power Sampling Buffers
    AD_Check();

    Check_Vitals();

    if (tgetq(SGPort) > 5) {
      SG_Depth_Check();
      incoming = true;
    } else if (tgetq(PAMPort) > 4) {
#ifdef DEBUG
      flogf("WISPR INCOMING");
#endif
      WISPR_Data();
      incoming = true;
    } else if (cgetq()) {
#ifdef DEBUG
      flogf("CONSOLE INCOMING");
#endif
      Console(cgetc());
      incoming = true;
    } else {
      incoming = false;
      PutInSleepMode = true;
    }
  }
  // TickleSWSR();

} //_____ Incoming_SeaGlider_Data() _____//

/************************************************************************************************************************\
** void Console      Testing Opportunity for Debugging
\************************************************************************************************************************/
void Console(char in) {

  short c;
#ifdef DEBUG
  flogf("Incoming Char: %c", in);
#endif
  RTCDelayMicroSeconds(2000L);

  switch (in) {

  case 'I':
  case 'i':
    if (PAMPort == NULL)
      OpenTUPort_WISPR(true);
    WISPRPower(true);
    WISPR_On = true;
    break;

  case 'E':
  case 'e':
    if (WISPR_On)
      WISPRSafeShutdown();
    WISPR_On = false;
    break;

  case 'D':
  case 'd':
    flogf("\n\t|REQUEST # DTX FROM WISPR?");
    c = cgetc() - 48;
    if (WISPR_On)
      WISPRDet(c);
    break;

  case 'F':
  case 'f':
    if (WISPR_On)
      WISPRDFP();
    break;

  case 'W':
  case 'w':
    flogf("\n\t|CHANGE TO WISPR #?");
    c = cgetc() - 48;
    if (WISPR_On)
      ChangeWISPR(c);
    break;

  case 'N':
  case 'n':
    flogf("\n\t|CHANGE WISPR GAIN?");
    c = cgetc() - 48;
    flogf("value of c: %d", c);
    if (c <= 3 && c > -1) {
      flogf(" Changed to %d", c);
      WISPRGain(c);
    }
    break;
  case '1':
    Shutdown = true;
    break;
  }

  PutInSleepMode = true;
  return;
}

/******************************************************************************\
**	SleepUntilWoken		Finish up
**
** 1-st release 9/14/99
** 2nd release 6/24/2002 by HM -Changed to use ADS8344/45
\******************************************************************************/
void Sleep(void) {

  //#ifdef DEBUG
  IEVInsertAsmFunct(IRQ4_ISR, level4InterruptAutovector); // Console Interrupt
  IEVInsertAsmFunct(IRQ4_ISR, spuriousInterrupt);
  //      #endif
  IEVInsertAsmFunct(IRQ2_ISR, level2InterruptAutovector); // Seaglider
  IEVInsertAsmFunct(IRQ2_ISR, spuriousInterrupt);
  IEVInsertAsmFunct(IRQ5_ISR, level5InterruptAutovector); // PAM Interrupt
  IEVInsertAsmFunct(IRQ5_ISR, spuriousInterrupt);

  SCITxWaitCompletion();
  EIAForceOff(true); // turn off RS232 Transmitters
  CFEnable(false);

  PinBus(IRQ2);
  PinBus(IRQ4RXD);
  PinBus(IRQ5);

  //#ifdef DEBUG
  while (PinRead(IRQ5) && PinRead(IRQ4RXD) && PinRead(IRQ2) && !data)
    LPStopCSE(FastStop);
  /*#else
     while (PinRead(IRQ5)&&PinRead(IRQ2)&&!data)
        LPStopCSE(FastStop);
  #endif
    */
  EIAForceOff(false); // turn on the RS232 driver
  CFEnable(true);     // turn on the CompactFlash card

  PIORead(IRQ2);
  PIORead(IRQ4RXD);
  PIORead(IRQ5);

  PutInSleepMode = false;

  RTCDelayMicroSeconds(1000L);

} //____ Sleep() ____//

/*************************************************************************\
**  static void Irq2ISR(void)
\*************************************************************************/
static void IRQ2_ISR(void) {
  PinIO(IRQ2);
  RTE();
} //____ Irq2ISR ____//

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
  char detfname[] = "c:00000000.dtx";
  float floater;
  int byteswritten = 0;
  uchar *buf;

  buf = (uchar *)malloc(256);
  memset(buf, 0, 256 * (sizeof buf[0]));

#ifdef DEBUG
  start_clock = clock();
#endif

  // Open Upload File
  //  uploadfile = fopen(fname, "w");
  filehandle = open(uploadfile, O_WRONLY | O_CREAT | O_TRUNC);

  if (filehandle <= 0) {
    flogf("\nERROR|WriteFile(): open error: errno: %d", errno);
    if (errno != 0)
      return -1;
  }
#ifdef DEBUG
  stop_clock = clock();
  print_clock_cycle_count(start_clock, stop_clock, "fopen: uploadfile");
#endif
  RTCDelayMicroSeconds(25000L);
  memset(buf, 0, 256 * sizeof(buf[0]));
  sprintf(buf, "===%s Program %3.2f===\nDiveNum:%d\nStartups:%d of "
               "%d\nWriteTime:%s\nTotalTime:%lu\nMaxDepth:%6.2fM\nDetInt:%3d\0",
          MPC.PROGNAME, PROG_VERSION, SEAG.DIVENUM, MPC.STARTUPS, MPC.STARTMAX,
          Time(NULL), TotalSeconds, max_dep, MPC.DETINT);
  flogf("\n%s", buf);

  byteswritten = write(filehandle, buf, strlen(buf));
#ifdef DEBUG
  flogf("\nBytes Written: %d", byteswritten);
#endif

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

  sprintf(&detfname[2], "%08d.dtx", SEAG.DIVENUM);
#ifdef DEBUG
  flogf("\n\t|Append File: %s", detfname);
#endif
  Append_Files(filehandle, detfname, false);

  RTCDelayMicroSeconds(25000L);

  close(filehandle);

  RTCDelayMicroSeconds(25000L);
  free(buf);
  return 0;

} //____ WriteFile() ____//

/******************************************************************************\
**	UploadFile      Start checking for upload command and send the file off
\******************************************************************************/
int UploadFile(TUPort *Port) {

  struct stat info;
  short BlkLength = 1024, NumBlks, LastBlkLength, BlkNum;
  uchar *buf;

  int filehandle;
  TickleSWSR();

  buf = (uchar *)malloc(BlkLength);

  memset(buf, 0, BlkLength * (sizeof buf[0]));

  cdrain();
  ciflush();
  coflush();

  flogf("\n%s|UPLOAD FILE: %s", Time(NULL), uploadfile);

  if (Port == NULL)
    return -1;

  TUTxFlush(Port);
  TURxFlush(Port);

  // Add compile error string to log file for uploading

  stat(uploadfile, &info);
  NumBlks = info.st_size / BlkLength;
  LastBlkLength = info.st_size % BlkLength;

  filehandle = open(uploadfile, O_RDONLY);
  RTCDelayMicroSeconds(50000L);

  if (filehandle <= 0) {
    flogf("\nERROR|UploadFile() errno: %d", errno);
    if (errno != 0)
      return -1;
  }

  for (BlkNum = 0; BlkNum <= NumBlks; BlkNum++) {

    if (BlkNum == NumBlks)
      BlkLength = LastBlkLength;

    memset(buf, 0, BlkLength * (sizeof buf[0])); // Flush the Global Buffer
    if (buf == NULL) {
      flogf("CAN'T ALLOCATE \"buffer\". Reason:( %s )", strerror(errno));
      RTCDelayMicroSeconds(50000);
      cdrain();
    }

    RTCDelayMicroSeconds(20000L);
    read(filehandle, buf, BlkLength * sizeof(uchar));
    RTCDelayMicroSeconds(50000L);
    // cdrain();
    // coflush();

    TickleSWSR();

    flogf("\n%s|Block %d, Bytes: %d", Time(NULL), BlkNum + 1, BlkLength);

    TUTxFlush(Port);
    TURxFlush(Port);
    TUTxPutBlock(
        Port, buf, BlkLength,
        5000); // Dump the contents of the buffer through the serial line
    TUTxWaitCompletion(Port);
    RTCDelayMicroSeconds(100000L);
  }

  TickleSWSR();

  close(filehandle);
  RTCDelayMicroSeconds(10000L);
  free(buf);

  return 1;

} //____ UploadFile() ____//

/******************************************************************************\
**	SeaGlider_Depth_Check()
** SG Depth Errors:
   1: Large Jump in depths
   2: In Status 6, waiting to upload, starts descending without receiveing
upload
\******************************************************************************/
int SG_Depth_Check() {

  int stringvalue = 0;

  // Get String will return 0 if correct depth reading
  stringvalue = SG_Data();
  TURxFlush(SGPort);
  CompileErrors(stringvalue);
  if (stringvalue != 0) {
    if (stringvalue == 2) { // Since SG2.3 Version: Uploadfile() was placed
                            // here. So efficient it gets called twice. Need
      // to check data on Seagcom to see if it looks correct or if it looks
      // overwritten.
      //**********
      //********
      sprintf(&uploadfile[2], "%08d.dat", SEAG.DIVENUM);
      UploadFile(SGPort);
      RTCDelayMicroSeconds(1000000L);
      return 2;
    } else
      return stringvalue;
  }

  // cprintf("stringvalue: %d, SG_Status: %d", stringvalue, SG_STATUS);cdrain();
  Log_Time(Depth);
  if (Depth_Samples < FIFO_SIZE)
    return 0;

  switch (SG_STATUS) {
  // Incase SG_STATUS is equal to zero
  case 0:
    SG_STATUS++;

  // Waiting for Non-"Surfaced" depth
  case 1:
    // If Descending
    if (AverageFIFO() > 0) {
      if (Prev_Depth > SEAG.ONDEPTH) {
        Surfaced = false;

        SG_STATUS++;
        flogf("\n%s|STATUS: 2", Time(NULL));
      }
    }
    // Else if Ascending
    else {
      if (Prev_Depth > SEAG.OFFDEPTH) {
        Surfaced = false;
        SG_STATUS++;
        flogf("\n%s|STATUS: 2", Time(NULL));
      }

      else if (Prev_Depth < 20.0) {
        Surfaced = true;
        SG_STATUS = 5;
        flogf("\n%s|STATUS: 5", Time(NULL));
      }
    }
    break;

  // Waiting for Correct Depth For Recording
  case 2:
    // Descending
    if (AverageFIFO() > 0) {
      // At or below turn on depth: power on wispr
      if (Prev_Depth > SEAG.ONDEPTH) {
        OpenTUPort_WISPR(false);
        Delay_AD_Log(2);
        OpenTUPort_WISPR(true);
        Delay_AD_Log(2);
        WISPRPower(true); // Power on
        WISPR_On = true;
        ADCounter = 0;
        SG_STATUS++;
        flogf("\n%s|STATUS: 3", Time(NULL));
      }
    }
    // Ascending
    else {
      // Still below turn off depth: power on wispr
      if (Prev_Depth >= SEAG.OFFDEPTH) {
        OpenTUPort_WISPR(false);
        Delay_AD_Log(2);
        OpenTUPort_WISPR(true);
        Delay_AD_Log(2);
        WISPRPower(true);
        WISPR_On = true;
        ADCounter = 0;
        SG_STATUS = 4;
        flogf("\n%s|STATUS: 4", Time(NULL));
      }
      // Above turn off depth: check wispr power
      else if (Prev_Depth <= SEAG.OFFDEPTH) {
        if (WISPR_On) {
          WISPRSafeShutdown();
          WISPR_On = false;
        }
        SG_STATUS = 5;
        flogf("\n%s|STATUS: 5", Time(NULL));
      }
    }
    break;

  // Wait for Glider to Begin Ascending at bottom of dive cycle request WISPR
  // Free Space.
  case 3:

    if (AverageFIFO() < 0) {
      flogf("\n%s|STATUS: 4", Time(NULL));
      flogf("\n%s|MaxDepth: %.2f for dive %d", Time(NULL), max_dep,
            SEAG.DIVENUM);
      log_dep = max_dep;
      SG_STATUS++;
      WISPRDFP();
    }
    break;

  case 4:

    // If max_depth is updated here... Means we are still descending, double
    // check and if needed boot status=3
    if (AverageFIFO() > 0) {
      max_dep = Depth;
      if (AverageFIFO() > 0) {
        flogf("\n%s|STATUS: 3", Time(NULL));
        SG_STATUS = 3;
      }
    }

    // When SeaGlider is near surface and prev change in depth is still going up
    else {
      if (Prev_Depth < SEAG.OFFDEPTH) {
        // if(MPC.DETINT!=0)
        //   WISPRDet(WISP.DETMAX);//One last call
        RTCDelayMicroSeconds(100000L);
        flogf("\n%s|SG STATUS: 5", Time(NULL));
        SG_STATUS++;
        // IF Wispr On, shutdown
        if (WISPR_On) {
          WISPRSafeShutdown();
          WISPR_On = false;
        }
      }
    }

    break;

  case 5:

    if (AverageFIFO() < 0) {
      if (Prev_Depth < 20.0) {
        flogf("\n%s|SG STATUS: 6", Time(NULL));
        SG_STATUS++;
        Time(&PwrOff);
        PwrOff = PwrOff - PwrOn;
        sprintf(&uploadfile[2], "%08d.dat", SEAG.DIVENUM);
        WriteFile(PwrOff);
        DOS_Com("del", SEAG.DIVENUM, "PWR", NULL);
        Setup_ADS(true, SEAG.DIVENUM + 1);
        Time(&PwrOn);
        Surfaced = true;
      }
    }
    break;

  case 6:
    // If Glider begins desending again.
    if (AverageFIFO() > 0) {
#ifdef DEBUG
      flogf("\n%s|Beginning Dive at: %.2f", Time(NULL), Depth);
#endif
      log_dep = Depth;
      NewDive = true;
      flogf("\n%s|SG STATUS: 1", Time(NULL));
      SG_STATUS = 1;
    }

    // else... Glider if found ascending
    else {
      // If Deeper than Logger turn off depth
      if (Prev_Depth > SEAG.OFFDEPTH) {
        flogf("\n%s|SG STATUS: 1", Time(NULL));
        log_dep = Depth;
        SG_STATUS = 1;
      }
    }
    break;

  default:
    SG_STATUS = 0;
    break;
  }
  return 1;
}

/******************************************************************************\
**	GetString
** Receive the data string from the glider including depth
\******************************************************************************/
int SG_Data() {
  short i, k;
  char instring[52];
  char *returnline;
  char newstring[60];
  char *split_depth;
  char *gliderdata;
  char *parsestring;
  char *split_divenum;
  char *split_datetime;
  char *split_trash;
  float depth_check;
  short dive_check;
  char inchar;

  instring[0] = '\0';
  returnline = (char *)calloc(60, 1);

  if (tgetq(SGPort) < 1)
    return -1;

  for (k = 0; k < 100; k++) {
    if (inchar != '+')
      inchar = TURxGetByteWithTimeout(SGPort, 5);
    else
      break;
  }
  if (inchar != '+')
    return -2;

  instring[0] = '+';

  for (i = 0; i < 50; i++) {
    instring[i] = TURxGetByteWithTimeout(SGPort, 200);
    if (instring[i] == '*')
      break;
    else if (instring[i] == -1)
      break;
  }

  gliderdata = strrchr(instring, '+');

  // If Asterisk isn't present, clear TUPort and return
  if (strchr(gliderdata, '*') == NULL) {
    TURxFlush(SGPort);
    TUTxFlush(SGPort);
    return -3;
  }
  // Found Asterisk, get returnline
  else {
    returnline = strtok(gliderdata, "*");
    // DBG(flogf("\n%s|SERIAL: %s", Time(NULL), returnline);   )
    TURxFlush(SGPort);
    TUTxFlush(SGPort);
  }

  // While "At Surface"
  if (Surfaced) {
    // Grab new GPS Time Info
    if ((parsestring = strstr(returnline, "GPS")) != NULL) {
      split_trash = strtok(parsestring, "=");
      split_datetime = strtok(NULL, "*");
      ParseTime(split_datetime);
      //   SG_STATUS=1;
      return 1;
    }
    // Get ready signal to upload file
    else if (strstr(returnline, "cat") != NULL) {
      ReadyToUpload = true;
      //      SG_STATUS=1;
      return 2;
    }
    // Receive new PAM parameters to save
    else if ((parsestring = strstr(returnline, "PAM")) != NULL) {
      memset(newstring, 0, 60);
      strcat(newstring, parsestring + 3);
      SaveParams(
          newstring); // SAVE THESE PARAMETERS TO SYSTEM.CFG FILE. CRC CHECK?
      ParseStartupParams();
      //            SG_STATUS=1;
      return 3;
    } else if (strstr(returnline, "STOP") != NULL) {
#ifdef DEBUG
      flogf("\n%s|STOP STRING RECEIVED!!\n", Time(NULL));
#endif
    }
  }

  split_depth = strtok(returnline, ","); // Tokenize string into seperate datas,
  split_divenum = strtok(
      NULL, "*"); // Tokenize at the comma to get the dive number from string

  depth_check = atof(split_depth + 1); // Intermittent transfer of depth value

  if (depth_check == 0.00) // If depth is exactly 0.00
    return -4;
  else if (depth_check == Prev_Depth) // If we get the exact same depth as least
                                      // time, return bad depth
    return -5;

  Prev_Depth = Depth;
  if (abs(Prev_Depth - Depth) > 100)
    return -4;

  if (Surfaced && NewDive) {
    dive_check = atoi(split_divenum);
    if (dive_check != SEAG.DIVENUM) {
      flogf("\n%s|Updated Dive Number: %d", Time(NULL), dive_check);
      SEAG.DIVENUM = dive_check;
      NewDive = false;
      VEEStoreShort(DIVENUM_NAME, dive_check);
      Open_Avg_File((long)SEAG.DIVENUM);
      create_dtx_file();
    }
  }

  // Prev_Depth=Depth;
  Depth = depth_check;

  return 0;

} //____ SG_Data() ____//

/*******************************************************************************\
** Log Date Time
\*******************************************************************************/
void Log_Time(float depth) {
  float change;
  short remainder = 0;

  // Store depth in our FIFO queue
  remainder = Depth_Samples % FIFO_SIZE;

  if (Depth > Prev_Depth)
    max_dep = Depth;

  Depth_Samples++;
  FIFO[remainder] = depth;
  FIFO_POS = remainder + 1;

  switch (log_verbose) {
  case 3:
    flogf("\n%s|DEPTH: %.2f", Time(NULL), depth);
    break;
  case 2:
    change = depth - log_dep;
    if (abs(change) > 25) {
      log_dep = depth;
      flogf("\n%s|DEPTH: %.2f", Time(NULL), depth);
    } else
      flogf("\n\t|Depth: %.2f", depth);
    break;
  case 1:
    change = depth - log_dep;
    if (abs(change) > 25) {
      log_dep = depth;
      flogf("\n%s|DEPTH: %.2f", Time(NULL), depth);
    }
    break;
  }
  if (Surfaced)
#ifdef DEBUG
    flogf("s");
#endif
  if (NewDive)
#ifdef DEBUG
    flogf("n");
#endif
  RTCDelayMicroSeconds(10000L);
  TURxFlush(SGPort);
  TUTxFlush(SGPort);
}

/***************************************************************************************\
** AverageFIFO
\***************************************************************************************/
float AverageFIFO() {

  int i = 0;
  float direction = 0.0;
  float first;

  first = FIFO[FIFO_POS - 1];
  for (i = FIFO_POS; i < FIFO_SIZE + FIFO_POS; i++) {
    if (i < FIFO_SIZE)
      direction += first - FIFO[i];
    else
      direction += first - FIFO[i - FIFO_SIZE];
  }
#ifdef DEBUG
  flogf("FIFO: %.2f", direction);
#endif
  return direction;

} //____ AverageFIFO() ____//

/***************************************************************************************\
**
\***************************************************************************************/
void ParseTime(char *timein) {

  struct tm t;
  time_t t_of_day;
  int month, day, year, hour, min, sec;

  sscanf(timein, "%d/%d/%d,%d:%d:%d", &month, &day, &year, &hour, &min, &sec);

  t.tm_year = year - 1900;
  t.tm_mon = month - 1; // Month, 0 - jan
  t.tm_mday = day;      // Day of the month
  t.tm_hour = hour;
  t.tm_min = min;
  t.tm_sec = sec;
  t.tm_isdst = -1; // Is DST on? 1 = yes, 0 = no, -1 = unknown
  t_of_day = mktime(&t);
  RTCSetTime(t_of_day, 0);
  flogf("\n%s|New Time Accepted", Time(NULL));
}

/***************************************************************************************\
** CompileErrors
\***************************************************************************************/
void CompileErrors(int error) {

  int i;
  int total = 0;

  switch (error) {
  case -5:
    errors[0]++;
    break;
  case -4:
    errors[1]++;
    break;
  case -3:
    errors[2]++;
    break;
  case -2:
    errors[3]++;
    break;
  case -1:
    errors[4]++;
    break;
  case 0:
    errors[5]++;
    break;
  case 1:
    errors[6]++;
    break;
  case 2:
    errors[7]++;
    break;
  case 3:
    errors[8]++;
    break;
  case 5:
    flogf("\n%s|SG Return String Errors:", Time(NULL));
    for (i = -5; i <= 3; i++) {
      total += errors[i + 5];
      flogf("[%d]:%d ", i, errors[i + 5]);
    }
    flogf("\nTotal: %d", total);
    break;
  }

} //____ CompileErrors()

/***************************************************************************************\
** GetSEAGLIDERSettings()
\***************************************************************************************/
void GetSEAGLIDERSettings() {

  char *p;

  p = VEEFetchData(POWERONDEPTH_NAME).str;
  SEAG.ONDEPTH = atoi(p ? p : POWERONDEPTH_DEFAULT);
#ifdef DEBUG
  uprintf("ONDEPTH=%u (%s)\n", SEAG.ONDEPTH, p ? "vee" : "def");
  cdrain();
#endif

  //"t" 0 or 1
  p = VEEFetchData(POWEROFFDEPTH_NAME).str;
  SEAG.OFFDEPTH = atoi(p ? p : POWEROFFDEPTH_DEFAULT);
#ifdef DEBUG
  uprintf("OFFDEPTH=%u (%s)\n", SEAG.OFFDEPTH, p ? "vee" : "def");
  cdrain();
#endif

  //"t" 0 or 1
  p = VEEFetchData(DIVENUM_NAME).str;
  SEAG.DIVENUM = atoi(p ? p : DIVENUM_DEFAULT);
#ifdef DEBUG
  uprintf("DIVENUM=%u (%s)\n", SEAG.DIVENUM, p ? "vee" : "def");
  cdrain();
#endif
} //____ GetSEAGLIDERSettings() ____//

/***************************************************************************************\
** OpenTUPort_SG()
\***************************************************************************************/
void OpenTUPort_SG(bool on) {

  short rx, tx;
  if (on) {
    rx = TPUChanFromPin(32);
    tx = TPUChanFromPin(31);
    PIOSet(22);
    PIOClear(23);
    SGPort = TUOpen(rx, tx, SGBAUD, 0);
    if (SGPort == 0)
      flogf("\nBad TU Channel: SGPort...");
  }
  if (!on) {
    PIOClear(22);
    TUClose(SGPort);
  }
  return;
}
