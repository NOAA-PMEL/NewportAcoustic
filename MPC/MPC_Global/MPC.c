#include <cfxpico.h> // Persistor PicoDOS Definitions
#include <cfxbios.h> // Persistor BIOS and I/O Definitions

#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <errno.h>
#include <fcntl.h> // PicoDOS POSIX-like File Access Definitions
#include <stat.h>  // PicoDOS POSIX-like File Status Definitions
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h> // PicoDOS POSIX-like Terminal I/O Definitions
#include <unistd.h>  // PicoDOS POSIX-like UNIX Function Definitions

#include <MPC_Global.h>
#include <PLATFORM.h>
#ifdef ACOUSTICMODEM
#include <AMODEM.h>
#endif

#ifdef BLUETOOTH
#include <BT.h>
#endif

#ifdef IRIDIUM
#include <GPSIRID.h>
#endif
#include <Settings.h>
#ifdef WISPR
#include <WISPR.h>
#endif
#ifdef POWERLOGGING
#include <ADS.h>
#endif
#ifdef CTDSENSOR
#include <CTD.h>
#endif

extern SystemParameters MPC;
extern SystemStatus LARA;

#ifdef BLUETOOTH
extern BluetoothParameters BT;
#endif

// Define WISPR VEEPROM Parameters for use
#ifdef WISPR
extern WISPRParameters WISP;
int DutyCycleTicks;
extern float WISPRFreeSpace;
#endif

// Define IRIDIUM VEEPROM Parameters for use
#ifdef IRIDIUM
extern IridiumParameters IRID;
#endif
// Define SEAGLIDER VEEPROM Parameters
#ifdef SEAGLIDER
extern SeagliderParameters SEAG;
#endif
// If using Linkquest UWM2000 Acoustic Modem
#ifdef ACOUSTICMODEM
extern AMODEMParameters AMDM;
#endif

#ifdef POWERLOGGING
extern PowerParameters ADS;
#endif
#ifdef CTDSENSOR
extern CTDParameters CTD;
#endif
#ifdef NIGKWINCH
#include <Winch.h>
extern WINCHParameters NIGK;
#endif

// LOCAL VARIABLES
int DetectionInt;

int DataInterval;
char time_chr[21];

#ifdef TIMING
volatile clock_t start_clock;
volatile clock_t stop_clock;
#endif

/******************************************************************************\
**	PreRun		Exit opportunity before we start the program
\******************************************************************************/
void PreRun(void) {
  short ndelay = 10;
  short i;
  char c;
  char *ProgramDescription = {
      "\n"
      "The program will start in five seconds unless an operator keys in a\n"
      "period '.' character. If a period is received, you have about\n"
      "### seconds to respond to access other software functions on this\n"
      "system before a watchdog reset occurs.\n"
      "\n"};
  time_t nowsecs;
  char strbuf[64];
  struct tm t;
  ushort Ticks;

  nowsecs = RTCGetTime(NULL, &Ticks); // get RTC clock right now
  t = *localtime(&nowsecs);
  strftime(strbuf, sizeof(strbuf), "%m/%d/%Y  %H:%M:%S", &t);

  flogf("\nProgram start time: %s.%.3d [ctime: %lu]\n", strbuf, Ticks / 40,
        nowsecs);
  cprintf(ProgramDescription);
  flogf("You have %d seconds to launch\n", ndelay);

  TickleSWSR();

  for (i = ndelay; i > 0; i--) {
    cprintf("%u..", i);
    c = SCIRxGetCharWithTimeout(1000); // 1 second
#ifdef BLUETOOTH
    if (tgetq(BTPort)) {
      Bluetooth_Interface();
      return;
    }
#endif
    if (c == '.')
      break;
    if (c == -1)
      continue;
    i = 0; // any other key ends the timeout
  }
  if (i <= 0) {
    flogf("\nStarting...\n");
    return; // to start acquisition
  }

  TickleSWSR(); // another reprieve

  QRchar("\nWhat next?(P=PicoDOS, S=Settings)", "%c", false, &c, "PS", true);
  if (c == 'S') {
    settings();

  }

  else if (c == 'P')
    BIOSResetToPicoDOS();

  return; // to start acquisition

} //____ PreRun() ____//
/*********************************************************************************\
** SetupHardware
** Set IO pins, set SYSCLK, if surface tries to open GPS receiver and set RTC
time.
** Set gain.  Return the current phase.
** 10/25/2004 H. Matsumoto
\*********************************************************************************/
void SetupHardware(void) {
  short err = 0;
  short result = 0;
  short waitsFlash, waitsRAM, waitsCF;
  ushort nsRAM, nsFlash, nsCF;
  short nsBusAdj;
  char *p;
  long counter = 0;
  char filenum[] = "00000000";
  char logfile[sizeof "00000000.log"];

#ifdef REALTIME // Placed here 6/20/2016 -AT bcuz SEAGLIDER
  p = VEEFetchData(FILENUM_NAME).str;
  counter = atol(p);
  MPC.FILENUM = counter + 1;
  sprintf(filenum, "%08ld", MPC.FILENUM);
  sprintf(logfile, "%08ld.log", MPC.FILENUM);
  VEEStoreStr(FILENUM_NAME, filenum);
#else
  p = VEEFetchData(LOGFILE_NAME).str;
  strncpy(MPC.LOGFILE, p ? p : LOGFILE_DEFAULT, sizeof(MPC.LOGFILE));
  sprintf(logfile, "%s", MPC.LOGFILE);

#endif
  Initflog(logfile, true);
  PZCacheSetup('C' - 'A', calloc, free);

  CSSetSysAccessSpeeds(nsFlashStd, nsRAMStd, nsCFStd, WTMODE);
  TMGSetSpeed(SYSCLK);

  TickleSWSR();

  flogf("\n----------------------------------------------------------------");
  flogf("\nProgram: %s,  Version: %3.2f,  Build: %s %s", __FILE__, PROG_VERSION,
        __DATE__, __TIME__);
  CSGetSysAccessSpeeds(&nsFlash, &nsRAM, &nsCF, &nsBusAdj);
  flogf("\nSystem Parameters: CF2 SN %05ld, PicoDOS %d.%02d, BIOS %d.%02d",
        BIOSGVT.CF1SerNum, BIOSGVT.PICOVersion, BIOSGVT.PICORelease,
        BIOSGVT.BIOSVersion, BIOSGVT.BIOSRelease);
  CSGetSysWaits(&waitsFlash, &waitsRAM, &waitsCF); // auto-adjusted
  cprintf(
      "\n%ukHz nsF:%d nsR:%d nsC:%d adj:%d WF:%-2d WR:%-2d WC:%-2d SYPCR:%02d",
      TMGGetSpeed(), nsFlash, nsRAM, nsCF, nsBusAdj, waitsFlash, waitsRAM,
      waitsCF, *(uchar *)0xFFFFFA21);
  fflush(NULL);
  coflush();
  ciflush();

  RTS(flogf("\n\t|Starting FileNumber: %ld", MPC.FILENUM);)
  // flogf("\n\t|%s logfile created", MPC.LOGFILE);
  cdrain();
  coflush();

} //____ SetupHardware() ____//
  /****************************************************************************
  ** MakeDirectory()
  ** Setup directories for files not needing to be access anymore.
  ** AT 7/13/2015
  *****************************************************************************/
void Make_Directory(char *path) {
  char DOSCommand[64];
  memset(DOSCommand, 0, 64);
  strncpy(DOSCommand, "mkdir ", 6);
  strncat(DOSCommand, path, 3);

  flogf("\n%s|MakeDirectory() %s", Time(NULL), DOSCommand);
  putflush();
  CIOdrain();
  execstr(DOSCommand);
  RTCDelayMicroSeconds(1000000L);
}
/****************************************************************************
** MakeDirectory()
** Setup directories for files not needing to be access anymore.
** AT 7/13/2015
*****************************************************************************/
void DOS_Com(char *command, long filenum, char *ext, char *extt) {
  char Com[64];
  static char fname[] = "c:00000000.";
  memset(Com, 0, 64);

  if (strstr(command, "copy") != NULL) {
    strncpy(Com, "copy ", 5);
    sprintf(fname, "c:%08ld.", filenum);
    strncat(Com, fname, 11);
    strncat(Com, ext, 3);
    strncat(Com, " c:", 3);
    if (extt == NULL)
      strncat(Com, ext, 3);
    else
      strncat(Com, extt, 3);
    strncat(Com, "\\", 1);
    strncat(Com, &fname[2], 9);
    if (extt == NULL)
      strncat(Com, ext, 3);
    else
      strncat(Com, extt, 3);
  } else if (strstr(command, "del") != NULL) {
    strncpy(Com, "del ", 4);
    sprintf(fname, "c:%08ld.", filenum);
    strncat(Com, fname, 11);
    strncat(Com, ext, 3);
  } else if (strstr(command, "ren") != NULL) {
    strncpy(Com, "ren ", 4);
    sprintf(fname, "c:%08ld.", filenum);
    strncat(Com, fname, 11);
    strncat(Com, ext, 3);
    strncat(Com, " ", 1);
    strncat(Com, fname, 11);
    strncat(Com, extt, 3);
  }

  else if (strstr(command, "move") != NULL) {

    DOS_Com("copy", filenum, ext, extt);
    DOS_Com("del", filenum, ext, NULL);
    return;
  }

  flogf("\n%s|COMDos() %s", Time(NULL), Com);
  putflush();
  CIOdrain();
  execstr(Com);
  RTCDelayMicroSeconds(250000L);
}

/******************************************************************************\
**	Time & Date String
** Get the RTC time seconds since 1970 and convert it to an
** understandable format
\******************************************************************************/
char *Time(ulong *seconds) {

  RTCtm *rtc_time;
  ulong secs = NULL;

  ushort ticks;

  RTCGetTime(&secs, &ticks);
  rtc_time = RTClocaltime(&secs);
  *seconds = secs;
  sprintf(time_chr, "%.2d/%.2d/%.4d %.2d:%.2d:%.2d", rtc_time->tm_mon + 1,
          rtc_time->tm_mday, rtc_time->tm_year + 1900, rtc_time->tm_hour,
          rtc_time->tm_min, rtc_time->tm_sec);
  return time_chr;

} //____ TimeDate() ____//
/************************************************************************************************************************\
** void System_Timer();
**
**    Return value:
**       1: Detection Interval Timer
**       2: Data Interval Timer
**
\************************************************************************************************************************/
int System_Timer() {
  int remainder;
  int adcount = 0;
  time_t rawtime;
  struct tm *info;
  static int DataTimer;
  float vel, depth;

  // Counting the number of ADS file writes, Timing is a function of this
  // number.
  adcount = Get_ADCounter();

  // To avoid Division by zero: we can just return here.
  if (adcount == 0)
    return 0;
  remainder = adcount % DetectionInt;

  // Intermittent check for WISPR Duty Cycle.
  if (DutyCycleTicks > 0)
    if (DetectionInt > DutyCycleTicks && !WISPR_Status())
      if ((adcount % (DetectionInt - DutyCycleTicks)) == 0)
        WISPRPower(true);
      // If DUTYCYCL is zero
      else if (WISPR_Status())
        WISPRPower(false);

  // Regular Detection Interval
  if (remainder == 0) {
    RTS(DataTimer++;)
    flogf("\n%s|System_Timer(%d/%d):", Time(NULL), DataTimer, DataInterval);

// WISPR MEASUREMENTS
#ifdef WISPR
    // If WISPR ON
    if (WISPR_Status()) {
      flogf("\n\t|Detection Call: Requesting %d DTX", WISP.DETMAX);
      WISPRDet(WISP.DETMAX);
      RTCDelayMicroSeconds(100000L);
      WISPRExpectedReturn(3, false);
      if (WISP.DUTYCYCL < 100) {
        flogf("\n\t|WISPR Duty Cycle");
        WISPRSafeShutdown();
      } else {
        WISPRDFP();
        WISPRExpectedReturn(2, true);
      }
    }
#endif

// CTD Measurements
#ifdef CTDSENSOR
    // Get an average of CTD Depth and calculate velocity
    depth = CTD_AverageDepth(3, &vel);
    // If initial average is less than target depth... why?
    if (depth <= NIGK.TDEPTH) {
      // Average more CTD Depth
      depth = CTD_AverageDepth(5, &vel);
      // Still Less than target depth? Set in recovery mode. Set calling
      // interval to 30 minutes.
      if (depth < NIGK.TDEPTH) {
        flogf(
            "\nERROR|System_Timer(CTD): DEPTH LESS THAN TARGET DEPTH!"); // This
                                                                         // will
                                                                         // be
                                                                         // interest
                                                                         // to
                                                                         // happen.
                                                                         // Most
                                                                         // likely
                                                                         // detachment.
        // Set NIGK to recovery?
        NIGK.RECOVERY = 1;
        VEEStoreShort(NIGKRECOVERY_NAME, NIGK.RECOVERY);
        MPC.DATAXINT = 30;
        VEEStoreShort(DATAXINTERVAL_NAME,
                      MPC.DATAXINT); // Call every 30 minutes at surface.
        return 2;
      }
    }
#endif

    // Reset AD Counter for next Detection Interval
    Reset_ADCounter();
  }

// This only happens if our Platform is a REALTIME System.
#ifdef REALTIME
  // If calling on DATATRANSFERINTERVAL
  if (IRID.CALLMODE == 0 || NIGK.RECOVERY) {
    if (DataTimer == 0)
      return 0;
    remainder = DataTimer % DataInterval;
    if (remainder == 0) {
      flogf("\n\t|IRID CALLMODE=0, call now!");
      DataTimer = 0;
      return 2;
    }
  }
  // Else if calling at set tiem of day.
  else if (IRID.CALLMODE == 1) {
    time(&rawtime);
    info = gmtime(&rawtime);
    // DBG(flogf("\n\t|CallTime: %d, Hour Now; %d", IRID.CALLHOUR,
    // info->tm_hour);)
    if (info->tm_hour == IRID.CALLHOUR) {
      DBG(flogf("\n\t|Current Minutes: %d", info->tm_min);)
      if (info->tm_min <= 3) {
        DataTimer = 0;
        flogf("\n\t|IRID CALLMODE=1, Call Now!\n\t|IRID CALLHOUR: %d",
              IRID.CALLHOUR);
        return 2;
      }
    }
  }
#endif

  return 1;

} //____ System_Timer() _____//
/**********************************************************************************************************************\
** void CheckTimerIntervals()
** Check the input values from cmdfile() and makes sure  they are divisible by
SysTimeInt. If not, rounds them up.
** PLI: Power Log Interval
/**********************************************************************************************************************/
float Check_Timers(ushort PLI) {

  float callrate;
  ulong rawtime;
  struct tm *info;
  int minutes, hours, hour;

#ifdef RAOT
  MPC.DETINT += 5;
  MPC.DATAXINT += MPC.DETINT;
#endif

  flogf("\n%s|Check_Timers()", Time(NULL));
  DBG(flogf("\nMPC.DETINT: %d\nMPC.DATAXINT: %d", MPC.DETINT, MPC.DATAXINT);)
  // Rounding of our Detection Timer with the Power Logging Cycle "SysTimeInt"
  DBG(flogf("\n\t|PLI: %hu", PLI);)
  DetectionInt = (((long)(MPC.DETINT * 600L) + (int)(PLI - 1)) / PLI);
  callrate = DetectionInt * (PLI / 600.0);

  flogf("\n\t|Recording Interval: %4.2f minutes", callrate);
  DBG(flogf("\n\t|Detection Int: %d", DetectionInt);)

#ifdef REALTIME
  if (IRID.CALLMODE == 0 || NIGK.RECOVERY) {
    DataInterval = (MPC.DATAXINT + (MPC.DETINT - 1)) / MPC.DETINT;
    callrate = callrate * DataInterval;
    if (NIGK.RECOVERY)
      flogf("\n\t|RECOVERY MODE");
    flogf("\n\t|Data Interval %4.2f minutes", callrate);
    DBG(flogf("\n\t|DataInterval: %d", DataInterval);)
  } else if (IRID.CALLMODE == 1) {
    time(&rawtime);
    info = gmtime(&rawtime);
    flogf("\n\t|CallTime: %d, Hour Now; %d", IRID.CALLHOUR, info->tm_hour);
    if ((int)IRID.CALLHOUR <= info->tm_hour) {
      hour = IRID.CALLHOUR + 23;
      hours = hour - info->tm_hour;
    } else {
      hour = info->tm_hour;
      hours = IRID.CALLHOUR - hour - 1;
    }
    flogf("\n\t|Hours: %d", hours);
    minutes = 60 - info->tm_min;
    flogf("Minutes: %d", minutes);
    minutes = minutes + (60 * hours);
    flogf("\n\t|Time until call: %d minutes", minutes);
  }
#endif

#ifdef RAOT
  MPC.DATAXINT -= MPC.DETINT;
  MPC.DETINT -= 5;
#endif

#ifdef WISPR
  // Calculate Duty Cycle
  DutyCycleTicks = (int)(DetectionInt * ((float)(WISP.DUTYCYCL / 100.0)));
  flogf("\n\t|WISPR Duty Cycle: %d\%", WISP.DUTYCYCL);
  DBG(flogf("\n\t|DutyCycleTicks: %d", DutyCycleTicks);)
// If some kind of Duty Cycle, keep Wispr Off until duty cycle begins

// Real value...

#endif

  // Reset SystemTimer by zeroing out ADCounter
  Reset_ADCounter();

  return callrate;

} //____ Check_Timers() ____//
/***********************************************************************************************\
** Check_Vitals
**    //Voltage: checking the average
**    1- Check Absolute MIN volts
**    2- Check User min volts
**    3- Check Startups
**    4- Check WISPR freespace (RAOSBottom)
**    5- Check CF2 CF freespace
** return -1 on need to shutdown
** return 0 if System is fine
** return 1 if below absolute Min Bat Voltage
** return 2 if below MinVolt
** return 3 if Min Bat Capacity
** return 4 if MIN WISPR FREE Space
** return 5 if No CF2 Free Space
\***********************************************************************************************/
short Check_Vitals() {

  short returnvalue = 0;

#ifdef POWERLOGGING
  float currentvoltage;
  float minvolt;

  minvolt = atof(ADS.MINVOLT); // Grab User defined minimum voltage.

  currentvoltage = Get_Voltage(); // Current Voltage

  if (currentvoltage == 0.00)
    returnvalue = 0;

  else if (currentvoltage <= minvolt) {

    flogf("\n%s|Check_Vitals():", Time(NULL));
    flogf("\n\t|Voltage: %4.2f", currentvoltage);

    if (currentvoltage <= MIN_BATTERY_VOLTAGE) {
      flogf("below absolute min.");
      returnvalue = 1; // System Shutdown
      return returnvalue;
    }

    else if (currentvoltage <= atof(ADS.MINVOLT)) {
      flogf("below user set min.");
      returnvalue = 2; // Hibernate? Stop WISPR?
    }
  }
#endif

  // Free_Disk_Space() somewhere else
  if (SystemFreeSpace < 100) {
    flogf("\nCheck_Vitals(): CF2 Free Space: %ldkB", SystemFreeSpace);
    returnvalue = 5; // Delete Files? Hibernate?
  }

  if (MPC.STARTUPS >= MPC.STARTMAX) {
    flogf("\n%s|Check_Vitals():", Time(NULL));
    flogf("\n\t|Startups surpassed maximum: %d", MPC.STARTUPS);
    returnvalue = 3; // System Shutdown.
  }

  return returnvalue;

} //____ Check_Vitals() ____//
/***************************************************************************************\
** void AppendFile
\***************************************************************************************/
bool Append_Files(int Dest, const char *SourceFileName, bool erase,
                  long maxBytes) {

  int Source;
  int i;
  // char    *buf;
  struct stat fileinfo;
  long BlockLength = 256;
  long NumBlks, LastBlkLength;
  char extension[] = "XXX";
  // char filename[]="c:00000000.xxx";
  int byteswritten;

  RTCDelayMicroSeconds(50000L);

  stat(SourceFileName, &fileinfo);
  if (fileinfo.st_size <= 0) {
    flogf("\nERROR  |Return upon no file to append");
    return false;
  }
  Source = open(SourceFileName, O_RDONLY);
  if (Source <= 0) {
    flogf("\nERROR  |AppendFiles() %s open errno: %d", SourceFileName, errno);
    // ?? if (errno != 0)
    return false;
  }
  DBG(else flogf("\n\t|Append_Files() %s opened", SourceFileName);)

  lseek(Source, 0, SEEK_SET);

  flogf("\n\t|Append_Files %s Size: %ld Bytes", SourceFileName,
        fileinfo.st_size);

  if (maxBytes <= 0)
    NumBlks = fileinfo.st_size / BlockLength;
  else
    NumBlks = maxBytes / BlockLength;

  LastBlkLength = fileinfo.st_size % BlockLength;

  for (i = 0; i <= NumBlks; i++) {

    if (i == NumBlks)
      BlockLength = LastBlkLength;
    //   	buf = (char *) calloc(BlockLength, sizeof(char));
    memset(WriteBuffer, 0, 256 * sizeof(char));

    byteswritten = read(Source, WriteBuffer, BlockLength * sizeof(char));
    DBG(flogf("\n\t|AppendFiles: bytes read: %d", byteswritten);)
    if (i == NumBlks) {
      strcat(WriteBuffer, "\n");
      BlockLength++;
    }
    byteswritten = write(Dest, WriteBuffer, BlockLength * sizeof(char));
    DBG(flogf("\n\t|AppendFiles: bytes written: %d", byteswritten);)
    // free(buf);
  }

  if (erase) {
    if (close(Source) == 0) {
      DBG(flogf("\n\t|Append_Files() %s Closed", SourceFileName);)
      sprintf(extension, "%c%c%c", SourceFileName[11], SourceFileName[12],
              SourceFileName[13]);
      DOS_Com("del", MPC.FILENUM, extension, NULL);
    } else
      flogf("\nERROR  |AppendFiles() %s close errno: %d", SourceFileName,
            errno);
  } else {
    if (close(Source) != 0)
      flogf("\nERROR  |AppendFiles() %s close errno: %d", SourceFileName,
            errno);
    DBG(else flogf("\n\t|AppendFiles() %s Closed", SourceFileName);)
  }

  return true;

} //____ AppendFiles() ____//
  /*************************************************************************
  ** Delay_AD_Log()
  ** AD function with time delay.  Do AD_Log at 5 sec incrment.
  **************************************************************************/
void Delay_AD_Log(short Sec) {
  short i;
  long last, rem;
  last = Sec / 5;
  rem = Sec - last * 5;

  TickleSWSR(); // another reprieve
  for (i = 0; i < last; i++) {

    AD_Check();
    RTCDelayMicroSeconds(5000000L); // Wait until get sat signal gets better
  }
  AD_Check();
  RTCDelayMicroSeconds(rem * 1000000L); // Wait until get sat signal gets better
  TickleSWSR();                         // another reprieve

} ///////////Delay_AD_Log()///////////////////////////////////////
  /****************************************************************
  ** Free_Disk_Space
  **   Returns the free space in kBytes
  ****************************************************************/
long Free_Disk_Space() {

  long freeSpacekB;
  long freeSectors;
  long totalSectors;

  freeSectors = DSDFreeSectors('C' - 'A');
  totalSectors = DSDDataSectors('C' - 'A');
  freeSpacekB = freeSectors / 2;
  flogf("\n\t|FreeDiskSpace %ldkB", freeSpacekB);

  return freeSpacekB;
} //----Free_Disk_Space-----//
/******************************************************************************\
**	GetFileName
**
**	Search the C drive for specific "FileType" i.e. "DAT" or "SNT" or "LOG"
** This function can return the "Lowest" filename of specified FileType for
** Data transferring of the oldest file on the system. or "!Lowest" to see which
** file name is most recent.
**	"incIndex" will increment the filename by one if "!Lowest" is also true.
** A long pointer "fcounter" will be pointed to the filename (8 digit number)
** of which ever file the function is searching for.
**
\******************************************************************************/
char *GetFileName(bool Lowest, bool incIndex, long *fcounter,
                  const char *FileType) {

  long counter = -1; // 2003-08-21
  long filecounter = 0;
  long val;
  long minval = 99999999; // 2003-08-21
  long maxval = -1;       // 2003-08-21
  static char dfname[] = "x:00000000.LOG";
  static char path[] = "x:";
  DIRENT de;
  short err;
  short i;

  //
  path[0] = dfname[0] = 'C'; // C: drive
  DBG(flogf("\n\t|GetFileName(.%3s): %s", FileType,
            Lowest ? "Lowest" : "Highest");)

  // Do this with *log extension next
  if ((err = DIRFindFirst(path, &de)) != dsdEndOfDir) {
    do {
      if (err != 0) {
        flogf("\n%s|GetFileName(): did not find Lowest File.", Time(NULL));
        break;
      }
      if (de.d_name[9] == FileType[0] && de.d_name[10] == FileType[1] &&
          de.d_name[11] == FileType[2]) {
        filecounter++;
        for (i = val = 0; i < 8; i++)
          if (de.d_name[i] >= '0' && de.d_name[i] <= '9')
            val = (val * 10) + de.d_name[i] - '0';
          else
            break;
        if (i == 8) { // all digits
          if (Lowest && val < minval) {
            DBG(flogf("\nNew lowest value: %ld", val);)
            minval = val;
          } else if (val > maxval) {
            DBG(flogf("\nNew highest value: %ld", val);)
            maxval = val;
          }
        }
      }
    } while (DIRFindNext(&de) != dsdEndOfDir);

    if (Lowest) {
      if (minval < 0 && counter < 0 || minval == 99999999) {
        DBG(flogf("\n\t|Did not find any .%3s files", FileType);)
        return NULL;
      } else if (minval > counter) {
        DBG(flogf("\n\t|New min val: %ld", minval);)
        counter = minval;
      }
    } else {
      if (maxval < 0 && counter < 0) // 2003-08-27
        counter = 0;                 // 2003-08-21

      else if (maxval >= counter) { // 2003-08-21
        DBG(flogf("\n\t|New maxval: %ld", maxval);)
        counter = maxval;
        if (incIndex)
          counter++;
      }
    }
  }

  // A new file name
  sprintf(&dfname[2], "%08lu.%3s", counter, FileType);
  if (Lowest)
    *fcounter = filecounter;
  else
    *fcounter = counter;

  DBG(flogf("\n%s|GetFileName(): %s", Time(NULL), dfname);)
  DBG(flogf("\n\t|filecounter: %ld", filecounter);)

  return dfname;

} //____ GetNextDATFileName() ____//
/******************************************************************************\
** SaveParams
\******************************************************************************/
bool SaveParams(const char *Command) {
  int paramfilehandle;
  int byteswritten;
  char params[64];

  memset(params, 0, 64);
  flogf("\n\t|SaveParams(%s)", Command);
  cdrain();
  coflush();
  strncpy(params, Command, strlen(Command));
  flogf("\n\t|param: %s", params);
  paramfilehandle = open("SYSTEM.CFG", O_WRONLY | O_CREAT | O_TRUNC);
  RTCDelayMicroSeconds(25000);
  if (paramfilehandle <= 0) {
    flogf("\nERROR  |SYSTEM.CFG open errno: %d", errno);
    // ?? if (errno != 0)
    return false;
  }
  DBG(else flogf("\n\t|SYSTEM.CFG Opened"); cdrain(); coflush();)

  DBG(flogf("\n\t|string length: %ld", strlen(Command)); cdrain(); coflush();)

  byteswritten = write(paramfilehandle, params, strlen(Command));
  DBG(flogf("\n\t|BytesWritten: %d", byteswritten);)
  RTCDelayMicroSeconds(25000);
  if (close(paramfilehandle) < 0)
    flogf("\nERROR  |SYSTEM.CFG close errno: %d", errno);
  DBG(else flogf("\n\t|SYSTEM.CFG Closed"); cdrain(); coflush();)

  return true;
}
/***********************************************************************************************\
** void parseparam(bool);
** 	-Param1: if true load default.cfg settings file. if false, load
system.cfg
\***********************************************************************************************/
void ParseStartupParams(bool DefaultSettings) {

  char string[5];
  int value, i, j;
  float floatvalue;
  int paramfilehandle;
  char *delimiters2;
  int strlength;
  char SettingsFile[] = "c:SYSTEM.CFG";
  char DefaultSettingsFile[] = "c:DEFAULT.CFG";
  char *token;
  char *param;
  char *paramstring;
  struct stat fileinfo;
  int filelength = 0;
  size_t paramlength, bytesread;
  char *delimiters;
  float floater;
  bool returnval;

  flogf("\n\t|ParseStartupParams():");
  cdrain();
  coflush();

  if (DefaultSettings) {
    stat("DEFAULT.CFG", &fileinfo);
    flogf("DEFAULT.CFG");
    filelength = fileinfo.st_size;
    paramfilehandle = open("DEFAULT.CFG", O_RDONLY);
  }

  else {
    stat("SYSTEM.CFG", &fileinfo);
    flogf("SYSTEM.CFG");
    filelength = fileinfo.st_size;
    paramfilehandle = open("SYSTEM.CFG", O_RDONLY);
  }

  RTCDelayMicroSeconds(25000L);
  if (paramfilehandle < 0) {
    flogf("\nERROR  |ParseStartupParams() open errno: %d", errno);
    return;
  } else {
    returnval = false;
    DBG(flogf("\n\t|ParseStartupParams() opened");)
  }

  param = (char *)calloc(filelength, sizeof(char));
  delimiters = (char *)calloc(sizeof("vgdcpxhrfelmaisut"), sizeof(char));
  sprintf(delimiters, "vgdcpxhrfelmaisut\0");
  strlength = strlen(delimiters);
  delimiters2 = (char *)calloc(strlength * 2, sizeof(char));
  bytesread = read(paramfilehandle, param, (filelength * sizeof(char)));
  DBG(flogf("\n\t|Bytes Read: %ld", bytesread); cdrain(); coflush();)
  if (close(paramfilehandle) == 0) {
    DBG(flogf("\n\t|ParseStartupParams() closed");)
    returnval = true;
  } else {
    DBG(flogf("\nERROR  |ParseStarupParams() errno: %d", errno);)
    returnval = false;
  }

  param = strchr(param, '(');
  paramstring = strtok(param + 1, ")");
  DBG(flogf("\n%s|ParseStartupParam(%s)", Time(NULL), paramstring);)

  for (i = 0, j = 0; i < strlength; i++, j += 2) {

    if ((int)delimiters[i] > 90) {
      delimiters2[j] = delimiters[i] - 32;
      delimiters2[j + 1] = delimiters[i];
    } else {
      delimiters2[j] = delimiters[i];
      delimiters2[j + 1] = delimiters[i] + 32;
    }
  }

  token = strpbrk(paramstring, delimiters2);

  while (token != NULL) {
    paramlength = strspn(token + 1, "1234567890.");
    memset(string, 0, 5);

    if (token[0] == 'v' || token[0] == 'V') {
      strncpy(string, token + 1, paramlength);
      floatvalue = atof(string);
      DBG(flogf("\nvalue for minvolt: %5.2f", floatvalue);)
    } else {
      strncpy(string, token + 1, paramlength);
      value = atoi(string);
      DBG(flogf("\nvalue at %c: %d", token[0], value);)
    }

    switch (token[0]) {
#ifdef WISPR
    // WISPR Gain
    case 'G':
    case 'g':
      if (value < 0 || value > 3) {
        flogf("\n\t|Not Updating WGain...%d", value);
        break;
      }
      if (WISP.GAIN != value) {
        flogf("\n\t|New Gain: %d", value);
        WISP.GAIN = value;
        VEEStoreShort(WISPRGAIN_NAME, value);
      }
      break;
    // Detections Returned
    case 'D':
    case 'd':
      if (value < 0)
        value = 0;
      else if (value > MAX_DETECTIONS)
        value = MAX_DETECTIONS;
      if (value != WISP.DETMAX) {
        flogf("\n\t|New Max Detections: %d", value);
        WISP.DETMAX = value;
        VEEStoreShort(DETECTIONMAX_NAME, value);
      }
      break;
    /*     //Detections to trigger AModem Call
         case 'n':case 'N':
            if(value<0||value>999)
               flogf("\n\t|Not Updating DETNUM... %d", value);
            else if(WISP.DETNUM!=value){
               flogf("\n\t|New DETNUM: %d", value);
               WISP.DETNUM=value;
               VEEStoreShort(DETECTIONNUM_NAME, value);
               }
            break;
      */
    case 'C':
    case 'c':
      if (value <= MIN_DUTYCYCLE)
        value = MIN_DUTYCYCLE;
      else if (value >= MAX_DUTYCYCLE)
        value = MAX_DUTYCYCLE;
      if (value != WISP.DUTYCYCL) {
        flogf("\n\t|New Duty Cycle: %d\%", value);
        WISP.DUTYCYCL = value;
        VEEStoreShort(DUTYCYCLE_NAME, value);
      }
      break;

    case 'P':
    case 'p':
      if (value < 0 || value > 4) {
        flogf("\n\t|Bad PAM Port Value");
      } else if (value != WISP.NUM) {
        WISP.NUM = value;
        flogf("\n\t|WISPR NUMBER: %d", WISP.NUM);
        VEEStoreShort(WISPRNUM_NAME, value);
      }
      break;
#endif
#ifdef CTDSENSOR
    case 'X':
    case 'x':
      if (value == 0 || value == 1)
        if (value != CTD.UPLOAD) {
          CTD.UPLOAD = value;
          flogf("\n\t|CTD UPLOAD %s", CTD.UPLOAD ? "ON" : "OFF");
          VEEStoreShort(CTDUPLOADFILE_NAME, value);
        }
      break;

#endif
#ifdef WINCH

    case 'H':
    case 'h':
      if (value < NIGK_MIN_DEPTH)
        value = NIGK_MIN_DEPTH;
      else if (value > (3 * NIGK.ANTLEN))
        value = (3 * NIGK.ANTLEN);

      if (value != NIGK.TDEPTH) {
        NIGK.TDEPTH = value;
        flogf("\n\t|New Winch Target Depth: %d", NIGK.TDEPTH);
        VEEStoreShort(NIGKTARGETDEPTH_NAME, value);
      }
      break;

    case 'R':
    case 'r':
      if (value < 5)
        value = 5;
      else if (value > 20)
        value = 20;
      if (value != NIGK.RRATE) {
        NIGK.RRATE = value;
        flogf("\n\t|New User defined Rise Rate: %d", NIGK.RRATE);
        VEEStoreShort(NIGKRISERATE_NAME, value);
      }
      break;

    case 'F':
    case 'f':
      if (value < 5)
        value = 5;
      else if (value > 20)
        value = 20;
      if (value != NIGK.FRATE) {
        NIGK.FRATE = value;
        flogf("\n\t|New User defined Fall Rate: %d", NIGK.FRATE);
        VEEStoreShort(NIGKFALLRATE_NAME, value);
      }
      break;

    case 'E':
    case 'e':
      if (value == 1 || value == 0)
        if (value != NIGK.RECOVERY) {
          NIGK.RECOVERY = value;
          flogf("\n\t|NIGK RECOVERY %s", NIGK.RECOVERY ? "ON" : "OFF");
          VEEStoreShort(NIGKRECOVERY_NAME, value);
        }
      break;

#endif
#ifdef SEAGLIDER
    case 'e':
    case 'E':
      if (value < MIN_OFF_DEPTH)
        value = MIN_OFF_DEPTH;

      if (value != SEAG.OFFDEPTH) {
        SEAG.OFFDEPTH = value;
        flogf("\n\t|New SEAGLIDER PAM Off Depth: %d", SEAG.OFFDEPTH);
        VEEStoreShort(POWERONDEPTH_NAME, value);
      }
      break;

    case 'O':
    case 'o':
      if (value < MIN_ON_DEPTH) {
        value = MIN_ON_DEPTH;
      }
      if (value != SEAG.ONDEPTH) {
        SEAG.ONDEPTH = value;
        flogf("\n\t|New SEAGLIDER PAM On Depth: %d", SEAG.ONDEPTH);
        VEEStoreShort(POWEROFFDEPTH_NAME, value);
      }
      break;

#endif

#ifdef BLUETOOTH
    case 'b':
    case 'B':
      if (value < 0 || value > 1) {
        flogf("\nERROR  |Bad BT Value: %d", value);

      } else if (value != BT.ON) {
        BT.ON = value;
        VEEStoreShort(BLUETOOTH_NAME, value);
      }
      break;
#endif
    // Hour at which to call at if in Callmode==1
    case 'L':
    case 'l':
      if (value < 0)
        value = 0;
      else if (value > 23)
        value = 23;
      if (value != IRID.CALLHOUR) {
        flogf("\n\t|New Iridium Call Hour: %d", value);
        IRID.CALLHOUR = value;
        VEEStoreShort(CALLHOUR_NAME, value);
      }
      break;

    // Mode at which to call RUDICS. 0 = dataxint, 1 = call hour
    case 'M':
    case 'm':
      if (value == 1 || value == 0)
        if (value != IRID.CALLMODE) {
          flogf("\n\t|Changed Iridium Call Mode: %s",
                value ? "Time of Day" : "Data Interval");
          IRID.CALLMODE = value;
          VEEStoreShort(CALLMODE_NAME, value);
        }
      break;

    // Time between data transfers in Minutes
    case 'A':
    case 'a':
      if (value < MIN_DATAX_INTERVAL) {
        value = MIN_DATAX_INTERVAL;
        sprintf(string, "%d", value);
      } else if (value > MAX_DATAX_INTERVAL) {
        value = MAX_DATAX_INTERVAL;
        sprintf(string, "%d", value);
      }
      if (MPC.DATAXINT != value) {
        flogf("\n\t|New Data transfer Interval: %d", value);
        MPC.DATAXINT = value;
        VEEStoreShort(DATAXINTERVAL_NAME, value);
      }
      break;
    // Detection Interval Time in Minutes
    case 'I':
    case 'i':
      if (value < MIN_DETECTION_INTERVAL) {
        value = MIN_DETECTION_INTERVAL;
        sprintf(string, "%2d", value);

      } else if (value > MAX_DETECTION_INTERVAL) {
        value = MAX_DETECTION_INTERVAL;
        sprintf(string, "%2d", value);
      }
      if (MPC.DETINT != value) {
        flogf("\n\t|New Detection Interval: %d", value);

        MPC.DETINT = value;
        VEEStoreShort(DETECTIONINT_NAME, value);
      }
      break;

    // Minimum System Voltage
    case 'v':
    case 'V':
      floater = atof(ADS.MINVOLT);
      if (floatvalue < MIN_BATTERY_VOLTAGE) {
        floatvalue = MIN_BATTERY_VOLTAGE;
      }
      if (floatvalue != floater) {
        flogf("\n\t|New Minimum Voltage: %4.2f", floatvalue);
        floater = floatvalue;
        sprintf(ADS.MINVOLT, "%4.2f", floatvalue);
        VEEStoreStr(MINSYSVOLT_NAME, ADS.MINVOLT);
      }
      break;

    case 'T':
    case 't':
      if (value >= 0 && value <= 1) {
        if (value != ADS.BATLOG) {
          flogf("\n\t|New Battery Calculator status: %s", value ? "On" : "Off");
          ADS.BATLOG = value;
          VEEStoreShort(BATTERYLOGGER_NAME, value);
        }
      }
      break;

    // Maximum Startups allowed.
    case 'S':
    case 's':
      /*if(value<MPC.STARTUPS){
              value=MPC.STARTUPS+1;
              }*/
      if (value > MAX_STARTUPS)
        value = MAX_STARTUPS;
      if (value != MPC.STARTMAX) {
        flogf("\n\t|New Max Startups: %d", value);
        MPC.STARTMAX = value;
        VEEStoreShort(STARTMAX_NAME, value);
      }
      break;

    // Max Upload Size in Bytes
    case 'U':
    case 'u':
      if (value > MAX_UPLOAD)
        value = MAX_UPLOAD;
      else if (value < 1000)
        value = 1000;
#ifdef IRIDIUM
      if (value != IRID.MAXUPL) {
        flogf("\n\t|New Max Upload Size: %d", value);
        IRID.MAXUPL = value;
        VEEStoreShort(MAXUPLOAD_NAME, value);
      }
#endif
#ifdef ACOUSTICMODEM
      if (value != AMDM.MAXUPL) {
        AMDM.MAXUPL = value;
        flogf("\n\t|New Max Amodem Upload Size: %d", value);
        VEEStoreShort(AMODEMMAXUPLOAD_NAME, value);
      }
#endif
      break;

      cdrain();
      coflush();
    }

    token = strpbrk(token + 1, delimiters2);
  }

} //____ parseparam() ____//
/**************************************************************************************\
** VEEStoreShort(char*)
\**************************************************************************************/
void VEEStoreShort(char *veename, short value) {

  char string[sizeof "00000"];

  memset(string, 0, 5);
  sprintf(string, "%d", value);
  VEEStoreStr(veename, string);

} //____ VEEStoreShort() ____//
/*********************************************************************************\
** VEEPROM GetSettings     Read settings from VEE or use defaults if not
available
\*********************************************************************************/
void GetSettings(void) {

  char *p;

  p = VEEFetchData(PROG_NAME).str;
  strncpy(MPC.PROGNAME, p ? p : PROG_DEFAULT, sizeof(MPC.PROGNAME));
  DBG(uprintf("PROGNAME=%s (%s)\n", MPC.PROGNAME, p ? "vee" : "def"); cdrain();)

  p = VEEFetchData(PROJID_NAME).str;
  strncpy(MPC.PROJID, p ? p : PROJID_DEFAULT, sizeof(MPC.PROJID));
  DBG(uprintf("PROJID=%s (%s)\n", MPC.PROJID, p ? "vee" : "def"); cdrain();)

  p = VEEFetchData(PLTFRMID_NAME).str;
  strncpy(MPC.PLTFRMID, p ? p : PLTFRMID_DEFAULT, sizeof(MPC.PLTFRMID));
  DBG(uprintf("PLTFRMID=%s (%s)\n", MPC.PLTFRMID, p ? "vee" : "def"); cdrain();)

  p = VEEFetchData(LONGITUDE_NAME).str;
  strncpy(MPC.LONG, p ? p : LONGITUDE_DEFAULT, sizeof(MPC.LONG));
  DBG(uprintf("LONGITUDE=%s (%s)\n", MPC.LONG, p ? "vee" : "def"); cdrain();)

  p = VEEFetchData(LATITUDE_NAME).str;
  strncpy(MPC.LAT, p ? p : LATITUDE_DEFAULT, sizeof(MPC.LAT));
  DBG(uprintf("LATITUDE=%s (%s)\n", MPC.LAT, p ? "vee" : "def"); cdrain();)
  /*
          p = VEEFetchData(LOCATION_NAME).str;
          strncpy(MPC.LOCATION, p ? p : LOCATION_DEFAULT, sizeof(MPC.LOCATION));
          DBG( uprintf("LOCATION=%s (%s)\n", MPC.LOCATION, p ? "vee" : "def");
     cdrain();)
          */
  p = VEEFetchData(STARTUPS_NAME).str;
  MPC.STARTUPS = atoi(p ? p : STARTUPS_DEFAULT);
  DBG(uprintf("STARTUPS=%d (%s)\n", MPC.STARTUPS, p ? "vee" : "def"); cdrain();)

  //"s" 0- 999 Ideally this would be a small number
  p = VEEFetchData(STARTMAX_NAME).str;
  MPC.STARTMAX = atoi(p ? p : STARTMAX_DEFAULT);
  DBG(uprintf("STARTMAX=%d (%s)\n", MPC.STARTMAX, p ? "vee" : "def"); cdrain();)
  if (MPC.STARTMAX > MAX_STARTUPS) {
    uprintf("STARTMAX changed from %d to %d\n", MPC.STARTMAX, MAX_STARTUPS);
    MPC.STARTMAX = MAX_STARTUPS;
    VEEStoreShort(STARTMAX_NAME, MPC.STARTMAX);
  }
  /*
if(MPC.STARTUPS>MPC.STARTMAX){
  //Hibernation Mode!
  }*/

  //"I" 0-360 minutes
  p = VEEFetchData(DETECTIONINT_NAME).str;
  MPC.DETINT = atoi(p ? p : DETECTIONINT_DEFAULT);
  DBG(uprintf("DETINT=%u (%s)\n", MPC.DETINT, p ? "vee" : "def"); cdrain();)
  if (MPC.DETINT < MIN_DETECTION_INTERVAL) {
    MPC.DETINT = MIN_DETECTION_INTERVAL;
    VEEStoreShort(DETECTIONINT_NAME, MPC.DETINT);
  } else if (MPC.DETINT > MAX_DETECTION_INTERVAL) {
    MPC.DETINT = MAX_DETECTION_INTERVAL;
    VEEStoreShort(DETECTIONINT_NAME, MPC.DETINT);
  }

  //"A"
  p = VEEFetchData(DATAXINTERVAL_NAME).str;
  MPC.DATAXINT = atoi(p ? p : DATAXINTERVAL_DEFAULT);
  DBG(uprintf("DATAXINT=%u (%s)\n", MPC.DATAXINT, p ? "vee" : "def"); cdrain();)
  if (MPC.DATAXINT < MIN_DATAX_INTERVAL) {
    MPC.DATAXINT = MIN_DATAX_INTERVAL;
    VEEStoreShort(DATAXINTERVAL_NAME, MPC.DATAXINT);
  } else if (MPC.DATAXINT > MAX_DATAX_INTERVAL) {
    MPC.DATAXINT = MAX_DATAX_INTERVAL;
    VEEStoreShort(DATAXINTERVAL_NAME, MPC.DATAXINT);
  }
/*
p = VEEFetchData(HIBERNATE_NAME).str;
MPC.HIBERNATE = atoi( p ? p : HIBERNATE_DEFAULT);
DBG(uprintf("HIBERNATE=%u (%s)\n", MPC.HIBERNATE, p ? "vee" : "def"); cdrain();)
*/

#ifdef POWERLOGGING
  GetPowerSettings();
#endif

#ifdef CTDSENSOR
  GetCTDSettings();
#endif

#ifdef BLUETOOTH
  GetBTSettings();
#endif

// GET WISPR PARAMETERS
#ifdef WISPR
  GetWISPRSettings();
#endif

// GET IRIDIUM PARAMETERS
#ifdef IRIDIUM
  GetIRIDIUMSettings();
#endif

#ifdef NIGKWINCH
  GetWinchSettings();
#endif

// Get Seaglider Parameters
#ifdef SEAGLIDER
  GetSEAGLIDERSettings(); // Version 2.5
#endif

#ifdef ACOUSTICMODEM
  GetAMODEMSettings();
#endif

} //____ RAOBGetSettings() ____//
void print_clock_cycle_count(clock_t start, clock_t stop, char *label) {

  flogf("\n%f seconds for %s",
        ((double)(stop - start)) / (double)CLOCKS_PER_SEC, label);
}
