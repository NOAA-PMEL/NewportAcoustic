#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions

#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <fcntl.h>
#include <stat.h>    // PicoDOS POSIX-like File Status Definitions
#include <termios.h> // PicoDOS POSIX-like Terminal I/O Definitions
#include <unistd.h>

#include <assert.h>
#include <cfxad.h>
#include <ctype.h>
#include <errno.h>
#include <float.h>
#include <float.h>
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
#include <time.h>

#include <ADS.h>
#include <MPC_Global.h>
#include <PLATFORM.h>
#include <Settings.h>
#include <WISPR.h>

bool SendWISPRGPS = false;
static int WISPRGPSSends;
bool WISPR_On;
float WISPRFreeSpace = 0.0;
int TotalDetections;
int dtxrqst;

// SYSTEM PARAMETERS
#ifdef SEAGLIDER
extern SeagliderParameters SEAG;
#endif

extern SystemParameters MPC;

WISPRParameters WISP;

// char* WISPRString;
// short WISPRStringLength=64;
// PAM TUPORT Setup
TUPort *PAMPort;
short PAM_RX, PAM_TX;

char *GetWISPRInput(float *);
void AppendDetections(char *, int);

static char wisprfile[] = "c:WISPRFRS.DAT";
static char WisprString[64];

bool WISPR_Status() { return WISPR_On; }
/*************************************************************************\
** void WISPRInit(short)
Power is applied to the wispr board and after the program starts it waits
for GPS time and location. That's when we send it.
\*************************************************************************/
void WISPRPower(bool power) {
  // must call OpenTUPort_WISPR first
  if (power) {

    flogf("\n%s|WISPR: ON", Time(NULL));
    PIOSet(WISPR_PWR_ON);
    Delayms(10);
    PIOClear(WISPR_PWR_ON);
    WISPR_On = true;
    WISPRGPSSends = 0;
    SendWISPRGPS = false;
  } else {
    flogf("\n%s|WISPR: OFF", Time(NULL));
    PIOSet(WISPR_PWR_OFF);
    Delayms(10);
    PIOClear(WISPR_PWR_OFF);
    WISPR_On = false;
    Delayms(1000);
  }

} //_____ WISPRInit() _____//
/***************************************************************************\
** void WISPR_Data()
** Incoming WISPR ASCII Communication. Looks for certain commands starting with
** '$' and ending with '*'
** Return values:
**    1: GPS
**    2: DFP
**    3: DXN
**    4: DTX
**    5: NGN
**    6: FIN
**    0: NULL
**   -1: No Match
**   -2: <MIN FREE SPACE
\***************************************************************************/
short WISPR_Data() {
  float returndouble;
  int i;
  int WISPRFile;
  short returnvalue = 0;
  char DTXFilename[] = "c:00000000.DTX";

  // memset(WISPRString, 0, WISPRStringLength*(sizeof WISPRString[0]));

  // WisprString =
  GetWISPRInput(
      &returndouble); // changed to 1.25 second wait time to receive the '$'

  if (strncmp(WisprString, "$GPS", 4) == 0) {
    WISPRGPS(124.5, 45);
    return 1;
  }
  /*   else if(strncmp(WisprString, "$TFP", 4)==0){

        flogf("\n\t|WISPR%d: %0.2f%% Total Space", WISP.NUM, returndouble);
        if(returndouble ==0.0){
           DBG(flogf("\n\t|Bad TFP return... tryaing again");)
           WISPRTFP();
           return -3;
           }
        //Calculate what minimum free space should be for given detint.
        }
    */
  else if (strncmp(WisprString, "$DFP", 4) == 0) {

    flogf("\n\t|WISPR%d: %.2f%% Free Space", WISP.NUM, returndouble);

    if (returndouble == 0.0) {
      DBG(flogf("\n\t|Bad DFP return...trying again");)
      WISPRDFP();
      return -2;
    }

    WISPRFreeSpace = returndouble;

#ifndef SEAGLIDER
    UpdateWISPRFRS();
#endif

    if (returndouble < MIN_FREESPACE) {

      flogf("\n\t|WISPR%d FreeSpace below Minimum", WISP.NUM);
      if (WISP.NUM < WISPRNUMBER) {
        flogf("\n\t|Incrementing WISPRNumber");
        ChangeWISPR(WISP.NUM + 1);
        // Only location when to permanently increment wispr number: When free
        // space runs out. or from start script.
        VEEStoreShort(WISPRNUM_NAME, WISP.NUM);
      }

      else {
        flogf("\n\t|Exceeded WISPRNumber");
        WISP.DUTYCYCL = 0;
        VEEStoreShort(DUTYCYCLE_NAME, 0);
        flogf("\n\t|WISPR Shutdown");
        // WISPRSafeShutdown(); //This creates a never ending loop.
        if (!WISPRExit())
          if (!WISPRExit()) {
            flogf(": Forcing Off");
            // ?? fetch storm warning
            WISPRPower(false);
          }
        Check_Timers(Return_ADSTIME());
        return 2;
      }

      return 2;
    }

    if (!SendWISPRGPS) {
      WISPRGPS(124.5, 45);
      Delayms(150);
      TUTxFlush(PAMPort);
      TURxFlush(PAMPort);
      WISPRGain(-1);
    }

    return 2;
  }

  else if (strncmp(WisprString, "$DXN", 4) == 0) {

    TotalDetections += (int)returndouble;
    flogf("\n\t|Total Detections: %d", TotalDetections);
#ifdef SEAGLIDER
    sprintf(&DTXFilename[2], "%08d.dtx", SEAG.DIVENUM);
#else
    sprintf(&DTXFilename[2], "%08ld.dtx", MPC.FILENUM);
#endif

    DBG(flogf("\n\t|DTX file: %s", DTXFilename);)
    WISPRFile = open(DTXFilename, O_APPEND | O_RDWR | O_CREAT);
    Delayms(25);
    if (WISPRFile <= 0)
      flogf("\nERROR  |WISPR_Data() %s open errno: %d", DTXFilename, errno);
    DBG(else flogf("\n\t|WISPR_Data() %s opened", DTXFilename);)

    if (WISPRFile <= 0) {
      flogf("\nERROR|WISPR_Data() open errno: %d", errno);
      return -3;
    }

    if ((int)returndouble < dtxrqst)
      dtxrqst = (int)returndouble;
    if ((int)returndouble == 0) {
      flogf("\n\t|Writing %d detections to %s", dtxrqst, DTXFilename);
      if (close(WISPRFile) < 0)
        flogf("\nERROR |WISPR_Data() %s close errno %d", DTXFilename, errno);
      DBG(else flogf("\n\t|WISPR_Data() %s closed", DTXFilename);)
      return 3;
    }

    else if (dtxrqst > 0) {
      DBG(flogf("\n%s", WisprString);)
      AppendDetections(WisprString, WISPRFile);
      if (dtxrqst > 0) {
        for (i = 0; i < dtxrqst; i++) {
          TickleSWSR();
          // memset(WisprString, 0, 64*(sizeof WisprString[0]));
          GetWISPRInput(&returndouble);
          if ((int)returndouble == -1) {
            flogf("\nERROR|Bad Return from GetWISPRInput()");
            break;
          }
          AppendDetections(WisprString, WISPRFile);
        }
        TURxFlush(PAMPort);
#ifdef ACOUSTICMODEM // This if defined statement should be based on whether an
                     // AMODEM is implemented but rather a real-time detection
                     // turn around.
        if (WISP.DETNUM > 0)
          if (returndouble >= WISP.DETNUM)
            flogf("\n\t|Detection Number surpassed");
#endif
      }
    }
    close(WISPRFile);

    return 3;
  }

  else if (strncmp(WisprString, "$DTX", 4) == 0) {
    DBG(flogf("\n%s", WisprString);)
    return 4;
  }

  else if (strncmp(WisprString, "$NGN", 4) == 0) {
    if (!SendWISPRGPS) {
      WISPRGPS();
      Delayms(50);
    }

    WISPRGain(-1);

    return 5;
  }

  else if (strncmp(WisprString, "$FIN", 4) == 0) {
    flogf(": Found Exit");
    Delayms(2000);     // Gives a little bit of time to WISPR to umount /mnt
    WISPRPower(false); // Powers off Wispr
    return 6;
  } else if (strcmp(WisprString, NULL) == 0) {
    return 0;
  } else
    return -1;

  return 0;

} //_____ WISPR_Data() _____//
/*************************************************************************\
** int WISPRDet()
This is where we inquire about the total number of detections every so often.
This function will call to the wispr board with max_detections. It first
repsonds with a $DXN...* com line then lists each actual detection
afterward with a $DTX...* com line. Each are up to ~60 bytes per line

\*************************************************************************/
void WISPRDet(int dtx) {

  if (dtx < 0)
    return;                   // if negative number then don't call detections
  else if (dtx > WISP.DETMAX) // if received dtx reqeust > Maximum user set.
    dtx = WISP.DETMAX;

  dtxrqst = dtx;

  Delayms(10);
  TUTxFlush(PAMPort);
  TURxFlush(PAMPort);
  TUTxPrintf(PAMPort, "$DX?,%d*\n", dtx);
  TUTxWaitCompletion(PAMPort);

  Delayms(500);

} //_____ WISPRDet() _____//
/*************************************************************************\
** int WISPRGPS()
This is where we send the GPS Time and Location to WISPR Board at startup
\*************************************************************************/
void WISPRGPS() {

  float minutes;
  float decimal;
  float LAT, LONG;
  char LATITUDE[17];
  char LONGITUDE[17];

  flogf("\n%s|WISPRGPS(%d):", Time(NULL), WISP.NUM);
  strcpy(LATITUDE, MPC.LAT);
  strcpy(LONGITUDE, MPC.LONG);
  flogf("\n\t|LAT: %s, LONG: %s", LATITUDE, LONGITUDE);
  LONG = atof(strtok(LONGITUDE, ":")); // Get Degree
  minutes = atof(strtok(NULL, " NS")); // Get decimal minutes
  decimal = minutes / 60.0;
  LONG += decimal;
  LAT = atof(strtok(LATITUDE, ":"));
  minutes = atof(strtok(NULL, " WE"));
  decimal = minutes / 60.0;
  LAT += decimal;

  flogf("\n\t|Sending GPS: %.4f %.4f", LONG, LAT);
  TUTxFlush(PAMPort);
  TUTxPrintf(PAMPort, "$GPS,%ld,%.4f,%.4f*\n", RTCGetTime(NULL, NULL), LONG,
             LAT);
  TUTxWaitCompletion(PAMPort);

  SendWISPRGPS = true;

} //_____ WISPRGPS() _____//
/*************************************************************************\
** int WISPRGain()
We can update the gain parameters for the WISPR Board.
Might want to do this at the start up of WISPR Program when WISPR Requests
for gain values.

\*************************************************************************/
void WISPRGain(short c) {

  if (c < 0 || c > 3)
    c = WISP.GAIN;

  flogf(" & Gain: %d", c);
  TUTxFlush(PAMPort);
  TUTxPrintf(PAMPort, "$NGN,%d*\n", c);
  TUTxWaitCompletion(PAMPort);
  Delayms(2);

} //_____ WISPRGain() _____//
/*************************************************************************\
** int WISPRDFP()
\*************************************************************************/
void WISPRDFP() {

  DBG(flogf("\n\t|WISPRDFP(%d)", WISP.NUM);)
  TURxFlush(PAMPort);
  TUTxFlush(PAMPort);
  TUTxPrintf(PAMPort, "$DFP*\n");
  TUTxWaitCompletion(PAMPort);
  Delayms(250);

} //_____ WISPRDFP() _____//
/*************************************************************************\
** int WISPRTFP()
\*************************************************************************/
void WISPRTFP() {

  DBG(flogf("\n\t|WISPRTFP(%d)", WISP.NUM);)
  TURxFlush(PAMPort);
  TUTxFlush(PAMPort);
  TUTxPrintf(PAMPort, "$TFP*\n");
  TUTxWaitCompletion(PAMPort);
  Delayms(250);

} //_____ WISPRDFP() _____//
/*************************************************************************\
** int WISPRExit()
\*************************************************************************/
bool WISPRExit() {

  flogf("\n\t|WISPRExit()");

  TUTxFlush(PAMPort);
  TURxFlush(PAMPort);
  TUTxPrintf(PAMPort, "$EXI*\n");
  TUTxWaitCompletion(PAMPort);
  Delay_AD_Log(150);  Delayms(200);

  WISPR_Data();

  if (!WISPR_On) {

    return true;
  } else
    return false;

} //_____ WISPRExit() _____//
/*******************************************************************************\
** char* GetWISPRInput()
1. Gets incoming serial data from ActivePAM on MPC
2. If it is -1 it breaks and exits, once it sees a '*' it stops taking serial
3. Look for the appropriate WISPR Command (DFP, EXI, DXN, DTX)
4. returns numchars if we are looking for DFP
5. will return number of detections if we get DXN. for which we will run a for
loop to get each detection in this same function
6. Should return char* of DTX if we need to get each detection.

\*******************************************************************************/
char *GetWISPRInput(float *numchars) {
  // global WisprString;
  const char *asterisk;
  int stringlength;
  bool good = false;
  int i = 0, k;
  char in;
  char r[65];
  char inchar;
  short count = 0;

  memset(WisprString, 0, 64 * sizeof(char));

  inchar = TURxGetByteWithTimeout(PAMPort, 250);
  for (k = 0; k < 100; k++) {
    if (inchar != '$')
      inchar = TURxGetByteWithTimeout(PAMPort, 25);
    else
      break;
  }

  if (inchar != '$')
    return NULL;

  r[0] = '$';
  r[1] = TURxGetByteWithTimeout(PAMPort, 100);
  if (r[1] == -1) {
    DBG(flogf("\nThe first input character is negative one");
        Delayms(20);)
    return NULL;
  }
  for (i = 2; i < 64; i++) { // Up to 59 characters
    in = TURxGetByteWithTimeout(PAMPort, 100);
    if (in == '*') {
      r[i] = in; // TURxFlush(PAMPort);
      break;
    } // if we see an * we call that the last of this WISPR Input
    else if (in == -1) {
      TURxFlush(PAMPort);
      return NULL;
    } else
      r[i] = in;
  }

  // Looks for end of WISPRInput, gets length, appends to returnline
  asterisk = strchr(r, '*');
  if (asterisk == NULL) {
    *numchars = -1;
    return NULL;
  }
  stringlength = asterisk - r + 1; // length from '$' to '*'
  strncat(WisprString, r, stringlength);
  strcat(WisprString, "\n\0");

  // If calling for number of detections return detections.
  if (strncmp("DXN", r + 1, 3) == 0) {

    *numchars = atoi(WisprString + 5);
  } else if (strncmp("DTX", r + 1, 3) == 0) {
    *numchars = 0;
  }

  // Sending Free Space, return value between , and *
  else if (strncmp("DFP", r + 1, 3) == 0) {
    *numchars = atof(strtok(WisprString + 5, "*"));
    // if(numchars==0.00) return false?
  }
  /*
  else if(strncmp("TFP", r+1, 3)==0){
     *numchars = atof(strtok(WisprString+5, "*"));

     }
     */

  return WisprString;

} //_____ GetIRIDInput() _____//
  /***************************************************************************************
  ** void ChangeWISPR()
  ****************************************************************************************/
void ChangeWISPR(short wnum) {

  // Shut off WISPR
  if (!WISPRExit()) {
    if (!WISPRExit()) {
      flogf(": Forcing Off");
      WISPRPower(false);
    }
  }

  OpenTUPort_WISPR(false);
  Delayms(100);
  WISP.NUM = wnum;
  OpenTUPort_WISPR(true);
  WISPRPower(true);
}
/**********************************************************************************************
** void GetWISPRSettings()
**********************************************************************************************/
void GetWISPRSettings() {
  char *p;
  /*****************WISPR PARAMETERS**************************************/
  p = VEEFetchData(DETECTIONMAX_NAME).str;
  WISP.DETMAX = atoi(p ? p : DETECTIONMAX_DEFAULT);
  DBG(uprintf("DETMAX=%u (%s)\n", WISP.DETMAX, p ? "vee" : "def"); cdrain();)
  if (WISP.DETMAX > MAX_DETECTIONS) {
    WISP.DETMAX = MAX_DETECTIONS;
    VEEStoreShort(DETECTIONMAX_NAME, WISP.DETMAX);
  }

  //"g" 0-3 gain values
  p = VEEFetchData(WISPRGAIN_NAME).str;
  WISP.GAIN = atoi(p ? p : WISPRGAIN_DEFAULT);
  DBG(uprintf("WISPRGAIN=%u (%s)\n", WISP.GAIN, p ? "vee" : "def"); cdrain();)
  if (WISP.GAIN > 3) {
    WISP.GAIN = 3;
    VEEStoreShort(WISPRGAIN_NAME, WISP.GAIN);
  } else if (WISP.GAIN < 0) {
    WISP.GAIN = 0;
    VEEStoreShort(WISPRGAIN_NAME, WISP.GAIN);
  }

  // "N" Number of detections per DETINT to trigger AModem
  p = VEEFetchData(DETECTIONNUM_NAME).str;
  WISP.DETNUM = atoi(p ? p : DETECTIONNUM_DEFAULT);
  DBG(uprintf("DETECTNUM=%u (%s)\n", WISP.DETNUM, p ? "vee" : "def"); cdrain();)

  //"C" dutycycle
  p = VEEFetchData(DUTYCYCLE_NAME).str;
  WISP.DUTYCYCL = atoi(p ? p : DUTYCYCLE_DEFAULT);
  DBG(uprintf("DUTYCYCLE=%d (%s)\n", WISP.DUTYCYCL, p ? "vee" : "def");
      cdrain();)
  if (WISP.DUTYCYCL > MAX_DUTYCYCLE) {
    WISP.DUTYCYCL = MAX_DUTYCYCLE;
    VEEStoreShort(DUTYCYCLE_NAME, WISP.DUTYCYCL);
  } else if (WISP.DUTYCYCL < MIN_DUTYCYCLE) {
    WISP.DUTYCYCL = MIN_DUTYCYCLE;
    VEEStoreShort(DUTYCYCLE_NAME, WISP.DUTYCYCL);
  }

  // "x" wisprnum
  p = VEEFetchData(WISPRNUM_NAME).str;
  WISP.NUM = atoi(p ? p : WISPRNUM_DEFAULT);
  DBG(uprintf("WISPRNUM=%d (%s)\n", WISP.NUM, p ? "vee" : "def"); cdrain();)
  if (WISP.NUM < 1) {
    WISP.NUM = 1;
    VEEStoreShort(WISPRNUM_NAME, WISP.NUM);
  } else if (WISP.NUM > WISPRNUMBER) {
    WISP.NUM = WISPRNUMBER;
    VEEStoreShort(WISPRNUM_NAME, WISP.NUM);
  }

} //____ GetWISPRSettings() ___//
/***********************************************************************************************\
** void WISPRSafeShutdown()
\***********************************************************************************************/
void WISPRSafeShutdown() {

  int i;

  if (WISPR_On) {
    WISPRDFP();
    i = WISPR_Data();
    if (i != 2) {
      WISPRDFP();
      i = WISPR_Data();
    }

    // Shut off WISPR for Data Transmission
    if (!WISPRExit()) {
      if (!WISPRExit()) {
        flogf(": Forcing Off");
        WISPRPower(false);
      }
    }

  } else
    return;

} //___ WISPRSafeShutdown() ___//
/***********************************************************************************************\
** Void WISPRWriteFile()
\***********************************************************************************************/
void WISPRWriteFile(int uploadfilehandle) {
  char detfname[] = "c:00000000.dtx";
  int byteswritten;

  memset(WriteBuffer, 0, 256 * sizeof(char));

  flogf("\n\t|WISPRWriteFile()");

  if (WISP.DUTYCYCL > 0) {
    sprintf(WriteBuffer, "\n---WISPR%d---\nGPS Sends: %d\nGain:%d\nFree "
                         "Space:%4.2f%%\nDuty Cycle:%d%%\nMax "
                         "Detections:%02d\nTotal Detections:%d\n\0",
            WISP.NUM, WISPRGPSSends, WISP.GAIN, WISPRFreeSpace, WISP.DUTYCYCL,
            WISP.DETMAX, TotalDetections);

    DBG(flogf("\n%s", WriteBuffer);)
    /*
            #ifdef REALTIME
               if(WISP.DETNUM>0){
               sprintf(stringadd, "\nCall upon %04d Detections\n", WISP.DETNUM);
               strncat(buf, stringadd, 27*sizeof(char));
               }
            #endif
            */

  } else {
    sprintf(WriteBuffer, "\nWISPR-OFF\n\0");
  }

  byteswritten = write(uploadfilehandle, WriteBuffer, strlen(WriteBuffer));
  DBG(flogf("\nWISPRWrite File: Number of Bytes written: %d", byteswritten);)

  // If more than one wispr, add total free space of all WISPRs to Write File.
  if (WISPRNUMBER > 1)
    Append_Files(uploadfilehandle, "C:WISPRFRS.DAT", false, 0);
  if (TotalDetections > 0) {
#ifdef SEAGLIDER
    sprintf(&detfname[2], "%08d.dtx", SEAG.DIVENUM);
#else
    sprintf(&detfname[2], "%08ld.dtx", MPC.FILENUM);
#endif
    DBG(flogf("\n\t|Append File: %s", detfname);)
    Append_Files(uploadfilehandle, detfname, true, 0);
  } else {
#ifdef SEAGLIDER
    DOS_Com("del", (long)SEAG.DIVENUM, "dtx", NULL);
#else
    DOS_Com("del", MPC.FILENUM, "dtx", NULL);
#endif
  }

  TotalDetections = 0;

} //____ WISPRWriteFile() ____//
/***********************************************************************************************\
** float GetWISPRFreeSpace
** Read "wisprfs.bat" file. return free space of current WISPR? ||  return free
space of last wispr
\***********************************************************************************************/
float GetWISPRFreeSpace() {

  return WISPRFreeSpace;

} //____ GetWISPRFreeSpace() ____//
/*************AppendDetections ********************/
void AppendDetections(char *DTXString, int FileDescriptor) {
  int i;

  DBG(flogf("\nAppendDetections() dtxstring length: %lu", strlen(DTXString));)
  i = write(FileDescriptor, DTXString, strlen(DTXString));
}
/*************create dtx file*********************/
void create_dtx_file(long fnum) {
  int filehandle;
  char SourceFileName[] = "c:00000000.dtx";

  sprintf(&SourceFileName[2], "%08ld.dtx", fnum);
  filehandle = creat(SourceFileName, 0);
  Delayms(500);
  if (filehandle < 0) {
    flogf("\nERROR  |Create_DTX_File() errno: %d", errno);
  }
  DBG(else flogf("\n\t|create_dtx_file() %s opened", SourceFileName);)

  if (close(filehandle) < 0)
    flogf("\nERROR  |create_dtx_file(): %s close errno: %d", SourceFileName,
          errno);
  DBG(else flogf("\n\t|create_dtx_file(): %s closed", SourceFileName);)

  Delayms(10);
}
/***********************************************************************************************\
** GatherWISPRFreeSpace()
** Only should come here upon MPC.STARTUPS==0 && WISPRNUMBER>1
\***********************************************************************************************/
void GatherWISPRFreeSpace() {
  short wret = 0, i, count = 0, wnum = 1;
  int byteswritten = 0;
  int writenum = 0;
  bool gain = false, dfp = false;
  int wisprfilehandle;
  char wisprbuff[16];
  static char *filename = "C:WISPRFRS.DAT";

  if (WISPR_On) {
    WISPRExit();
    Delayms(2500);
  }
  WISPRPower(true);

  wisprfilehandle = open(filename, O_CREAT | O_TRUNC | O_RDWR);
  if (wisprfilehandle <= 0) {
    flogf("file handle: %d", wisprfilehandle);
    flogf("errno: %d", errno);
    return;
  }
  for (i = 1; i <= WISPRNUMBER; i++) {
    sprintf(wisprbuff, "W%d:00.00%,", i); // sprintf adds trailing \0
    byteswritten =
        write(wisprfilehandle, wisprbuff, strlen(wisprbuff) * sizeof(char));
    flogf("\n\t|Bytes written: %d", byteswritten);
  }
  close(wisprfilehandle);

  while (wnum <= WISPRNUMBER) {
    while (!dfp && count <= 3) {

      Sleep();

      if (AD_Check()) {
        count++;
        if (gain || count == 2) {
          DBG(flogf("\n\t|GWFS: DFP2");)
          WISPRDFP();
          Delayms(150);
          if (WISPR_Data() == 2)
            dfp = true;
        }
      }

      if (tgetq(PAMPort)) {
        wret = WISPR_Data();
        if (wret == 1) {
          Delayms(150);
          wret = WISPR_Data();
          if (wret == 5) {
            gain = true;
            Delayms(150);
            DBG(flogf("\n\t|GWFS: DFP1");)
            WISPRDFP();
            Delayms(150);
            if (WISPR_Data() == 2)
              dfp = true;
          }
        }
      }
    }

    TURxFlush(PAMPort);
    TUTxFlush(PAMPort);
    if (!dfp)
      WISPRFreeSpace = 00.00;

    dfp = false;
    gain = false;
    count = 0;
    wnum++;
    if (wnum > WISPRNUMBER)
      break;
    //      if(WISPRFreeSpace>=MIN_FREESPACE)
    ChangeWISPR(wnum);
  }
  DBG(flogf("\n\t|wnum: %d, WISP.NUM: %d", wnum, WISP.NUM);)
  wnum = 1;

  // Shut off WISPR && Close TUPort
  if (!WISPRExit()) {
    if (!WISPRExit()) {
      flogf(": Forcing Off");
      WISPRPower(false);
    }
  }

  OpenTUPort_WISPR(false);
  Delayms(100);
  WISP.NUM = wnum;
  VEEStoreShort(WISPRNUM_NAME, WISP.NUM);

} //____GatherWISPRFreeSpace() ____//
/***********************************************************************************************\
** void UpdateWISPRFRS()
**
**    Write to file: WISPRFRS.DAT with the current wispr's free space
\***********************************************************************************************/
void UpdateWISPRFRS() {
  // global *WisprString;
  int wisprfilehandle;
  struct stat fileinfo;
  char *wispnum = "W0:";
  char *p;
  int length;
  int bytes;
  long filesize;

  flogf("\n%s|Update %s ", Time(NULL), wisprfile);
  Delayms(10);
  // sprintf(&wisprfile[2], "WISPRFRS.DAT");
  Delayms(20);
  if (stat(wisprfile, &fileinfo) != 0) {
    flogf("%s file does not exist. making file...", wisprfile);
    GatherWISPRFreeSpace();
    stat(wisprfile, &fileinfo);
  }
  filesize = fileinfo.st_size;

  DBG(flogf("\n\t|File size: %ld", filesize);)

  wisprfilehandle = open(wisprfile, O_RDWR);
  Delayms(25);
  if (wisprfilehandle <= 0)
    flogf("\nERROR  |UpdateWISPRFRS(): file open errno: %d", errno);
  DBG(else flogf("\n\t|UpdateWISPRFRS() %s opened", wisprfile);)

  read(wisprfilehandle, WisprString, fileinfo.st_size);
  // flogf("\n\t|%s", WisprString);
  if (WISP.NUM > WISPRNUMBER)
    WISP.NUM = WISPRNUMBER;
  sprintf(wispnum, "W%d:", WISP.NUM);

  p = strstr(WisprString, wispnum);
  if (p == NULL)
    flogf("\nERROR  |No String search found for: %s", wispnum);
  length = p - WisprString;
  if (length > 30) {
    flogf("\nERROR |Returning due to bad filde position for %s", wisprfile);
    close(wisprfilehandle);
    return;
  }

  p = strtok(p + 3, ",");

  DBG(flogf("\nUpdating WISPRFRS at position %d from %5.2f to %5.2f", length,
            atof(p), WISPRFreeSpace);)

  sprintf(wispnum, "%5.2f", WISPRFreeSpace);
  lseek(wisprfilehandle, length + 3, SEEK_SET);
  bytes = write(wisprfilehandle, wispnum, 5);
  DBG(flogf("\nBytes written: %d", bytes);)
  lseek(wisprfilehandle, 0, SEEK_SET);
  read(wisprfilehandle, WisprString, fileinfo.st_size);
  flogf("%s", WisprString);

  close(wisprfilehandle);

} //____ UpdateWISPRFRS() ____//
/****************************************************************************************************************\
** void OpenTUPort_WISPR()
\****************************************************************************************************************/
void OpenTUPort_WISPR(bool on) {
  // global WisprStr
  int WisprNum;

  WisprNum = WISP.NUM;
  flogf("\n\t|%s WISPR%d TUPort", on ? "Open" : "Close", WisprNum);
  if (on) {
    PAM_RX = TPUChanFromPin(28);
    PAM_TX = TPUChanFromPin(27);
    PAMPort = TUOpen(PAM_RX, PAM_TX, BAUD, 0);
  } else if (!on) {
    TUTxFlush(PAMPort);
    TURxFlush(PAMPort);
    TUClose(PAMPort);
    Delayms(1000);
  }

  PIOClear(WISPR_PWR_ON);
  PIOClear(WISPR_PWR_OFF);

  // PAM 1
  if (WisprNum == 1) {
    if (on) {
      PIOSet(WISPRONE);
      PIOClear(WISPRTWO);
    } else {
      PIOClear(WISPRONE);
      PIOClear(WISPRTWO);
    }
  }
  // PAM 2
  else if (WisprNum == 2) {
    if (on) {
      PIOSet(WISPRONE);
      PIOSet(WISPRTWO);
    } else {
      PIOClear(WISPRONE);
      PIOClear(WISPRTWO);
    }
  }
  // PAM 3
  else if (WisprNum == 3) {
    if (on) {
      PIOSet(WISPRTHREE);
      PIOClear(WISPRFOUR);
    } else {
      PIOClear(WISPRTHREE);
      PIOClear(WISPRFOUR);
    }

  }
  // PAM 4
  else if (WisprNum == 4) {
    if (on) {
      PIOSet(WISPRTHREE);
      PIOSet(WISPRFOUR);
    } else {
      PIOClear(WISPRTHREE);
      PIOClear(WISPRFOUR);
    }
  } else if (WisprNum == 0) {
    flogf("\n\t|WISPR Zero. Run out of space?");
  }
  // Bad PAM
  else {
    if (PAMPort == 0)
      printf("\nBad TU Channel: PAM...");
    flogf("\n\t|Wrong PAM Port...");
    TUClose(PAMPort);
  }
  Delayms(100);
}
/*
** bool WISPRExpectedReturn(short)
*/
bool WISPRExpectedReturn(short expected, bool reboot) {

  if (WISPR_Data() != expected) {
    OpenTUPort_WISPR(false);
    Delay_AD_Log(2);
    OpenTUPort_WISPR(true);
    Delay_AD_Log(1);
    switch (expected) {
    case 1:
      WISPRGPS();
      break;
    case 2:
      WISPRDFP();
      break;
    case 3:
      WISPRDet(WISP.DETMAX);
      break;
    case 6:
      WISPRExit();
      break;
    }
    if (WISPR_Data() != expected)
      if (reboot) {
        WISPRSafeShutdown();
        Delay_AD_Log(2);
        WISPRPower(true);
      }
    return false;
  } else
    return true;

} //____ WISPRExpectedReturn ____//
