/******************************************************************************\
**	GPSIRID.c				Iridium communication and
IridFile transfer
**
** 02/25/2014  Sorqan Chang-Gilhooly
** 04/01/2015  Haru Matsumoto, Alex Turpin
*****************************************************************************
**	Upload a file to Rudics per a command received.  Currently max number
of commands
** can be sent from Rudics is 10. So up to 10 files can be uploaded per one
** telemetry session (or connection).
**
**
****************************************************************'************
**
**
\******************************************************************************/

#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions
#include <errno.h>
#include <float.h>
#include <stdarg.h>
#include <stddef.h>

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

#include <PLATFORM.h>
#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <fcntl.h>    // PicoDOS POSIX-like File Access Definitions
#include <stat.h>     // PicoDOS POSIX-like File Status Definitions
#include <termios.h>  // PicoDOS POSIX-like Terminal I/O Definitions
#include <unistd.h>   // PicoDOS POSIX-like UNIX Function Definitions

#include <ADS.h>
#include <GPSIRID.h>

#include <MPC_Global.h>
#include <Settings.h>

#define HANDSHAKE 3
#define ANTSWPIN 1 // Antenna swith pin number
#define RecallTimeout 300
#define NUMCOM 7 // Number of commmands can be receive per telemetry
#define MAX_RESENT 3
#define GPS_TRIES 10 // num of tries to get min GPS sat
#define STRING_SIZE 1024
#define RUDICSBLOCK 1000
// #define RUDICSBLOCK 2000


IridiumParameters IRID;
extern SystemParameters MPC;

bool SatComOpen = false;
bool LostConnect = false;
short Max_No_SigQ_Chk = 7;

// int BlkLength = 2000; // Irid file block size. 1kB to 2kB, 1024-2048
int BlkLength = RUDICSBLOCK; 
short IRIDWarm = 27;  // Irid Modem warm-up. 45 IS NORMAL, 1 FOR TESTING ///
short MinSQ;

static char ProjID[5];
static char PltfrmID[5];
static char PhoneNum[14];

short TX_Success = 0;
short Num_Resent;

ulong TimingDelayCount;
bool TimeExpired = false;
static int IRIDFileHandle;
static short IRIDStatus;
static char IRIDFilename[sizeof "c:00000000.dat"];

void OpenTUPort_CTD(bool); // CTD.c
void OpenTUPort_AntMod(bool);
void shutdown(); // defined in lara.c
bool GetUTCSeconds();
short Connect_SendFile_RecCmd(const char *);
bool InitModem(int);
bool CheckSignal(void);
short CallStatus(void);
short PhoneStatus(void);
short PhonePin(void);
short UploadFiles();

short SignalQuality(short *signal_quality);
bool Call_Land(void);
bool SendProjHdr(void);
short Send_File(bool FileExist, long);
int Send_Blocks(char *bitmap, uchar NumOfBlks, ushort BlkLength,
                ushort LastBlkLength);
int Receive_Command();
int Calc_Crc(uchar *buf, int cnt);
void Convert_BitMap_To_CharBuf(ulong val0, ulong val1, char *bin_str);
short Check_If_Cmds_Done_Or_Resent(ulong *val0, ulong *val1);
bool HangUp(void);
short GetIRIDInput(char *, short, uchar *, int *, short wait);
char *GetGPSInput(char *, int *);
void SendString(const char *);
bool GetGPS_SyncRTC();
short SwitchAntenna(char);
void SelectModule(char);
short StringSearch(char *, char *, uchar *);
void StatusCheck();
bool CompareCoordinates(char *, char *);
DBG( void ConsoleIrid(); ) // check console for interrupt, redirect 

// IRIDUM TUPORT Setup
TUPort *AntModPort;
short ANTMOD_RX, ANTMOD_TX;

uchar *inputstring, *first, *scratch;
//
/********************************************************************************\
** GPSIRID-3.0 12/15/2015-AT
**
** Incorporating Sleep Mode, So this communications protocol will be run in
stages.
** Each time there is down time, the program reverts back to the main file and
sleep before
** more is serial transfer is needed to be dealt with.
// Get GPS, sync RTC, call Rudics phone number, send platform ID, log on server,
// send a file and receive a command.
//
//   Block size should be 1024 or 2048
//   Cyclic Redundancy Check, CRC-CCITT(0x1D0F)
//   return: -1 on hw fail, -2 "lostCarrier" on any upload fail, else UploadFiles result
*********************************************************************************/
short IRIDGPS() {

  short TX_Result;

  if (!PowerOn_GPS()) {
    OpenTUPort_AntMod(false);
    return -1;
  }

  TX_Result = UploadFiles();
  if (TX_Result >= 1) {
    flogf("\n\t|ANTMOD SUccess");
    OpenTUPort_AntMod(false);
    return TX_Result;
  }

  else {
    flogf("\nLOST CARRIER");
    OpenTUPort_AntMod(false);
    return -2;
  }

} //____ IRIDGPS() ____//
/*********************************************************************************\
** UploadFiles();
\*********************************************************************************/
short UploadFiles() {
  // ProjID, PltfrmID, PhoneNum
  short TX_Result;
  static char fname[] = "c:00000000.dat";
  SwitchAntenna('I');

  MinSQ = IRID.MINSIGQ; // Min signal quality

  // three global vars - not used anywhere ??
  strcpy(ProjID, MPC.PROJID);
  strcpy(PltfrmID, MPC.PLTFRMID);
  strcpy(PhoneNum, IRID.PHONE);

  // Prepare current file name
  // Find highest .dat file number
  //   GetFileName(false, false, &fnum, "dat");
  // if(fnum==0) flogf("\n\t|Highest filenum is zero");
  sprintf(&fname[2], "%08ld.dat", MPC.FILENUM);

  TX_Result = Connect_SendFile_RecCmd(fname);

  return TX_Result;

} //____ UploadFiles() ____//
/*********************************************************************************\
** PowerOn_GPS()
\*********************************************************************************/
bool PowerOn_GPS() {
  // Open the GPS/IRID satellite com...
  OpenTUPort_AntMod(true);
  SwitchAntenna('G');

  if (GetGPS_SyncRTC(IRID.OFFSET)) // false if no GPS sat
    return true;
  else
    return false;
}
/**********************************************************************************\
 * SwitchAntenna(G|I|S)
 * Switch antenna module between SBE39 TD, Iridium and GPS.  
 * ^A Antenna G|I * ^B Binary byte * ^C Connect I|S * ^D powerDown I|S * ^E binary lEngth
\**********************************************************************************/
short SwitchAntenna(char r) {
  char a, d;
  DBG(flogf("\n\t|SwitchAntenna(%c)", r);)
  // select antMod vs SBE16, switch antMod device
  switch (r) {
    case 'G': { a='G'; d='I'; break; }
    case 'I': { a='I'; d='I'; break; }
    case 'S': { a='0'; d='S'; break; }
    default: { // bad case
      flogf("\nError SwitchAntenna(%c): bad choice", r);
      return -1;
    }
  }

  TUTxPutByte(AntModPort, 3, true);  // connect
  TUTxPutByte(AntModPort, d, true);  // I S
  if (a) {
    TUTxPutByte(AntModPort, 1, true);  // antenna
    TUTxPutByte(AntModPort, a, true);  // G I
    RTCDelayMicroSeconds(1000000L); // wait 1 sec to settle antenna switch noise
  }
  return 0;
} //____ SwitchAntenna ____//

/***
 * SelectModule(A|S) - select antMod vs SBE16
 */
void SelectModule(char m) {
  // select is slow, but com port speeds differ
  switch (m) {
    case 'A': OpenTUPort_CTD(false); OpenTUPort_AntMod(true); break;
    case 'S': OpenTUPort_AntMod(false); OpenTUPort_CTD(true); break;
    default: flogf("\nError SelectModule(%c)", m);
  }
  Delayms(500);
} // SelectModule()

/******************************************************************************\
** Connect_SendFile_RecCmd
** 1) Switch antenna to IRID
**    If connection is lost anytime, goto 6) and restart from 2).
** 2) Call, Connect, and Log on
** 3) Send Platform ID and get ACK
** 4) Send a file
** 5) Receive command (until receives 'done' from land)
**    R    - Resend
**    cmds - TX was success. Check if it is a real command, receive it and save
it,
**             if it is Senddata(), discard the command
**    done - TX was success. No command is comming -> goto 6)
**
**    Check if there is a next file,
**          Yes- send the file -> goto 4)
**          No - send 'done'   -> goto 5)
** 6) Hung up
**
** Three kinds of response from Rudics land
** 'R' for Resend, 'cmds' for command to receive, and 'done' for no more commands
** Two kinds of cmds: Real =2, fake (Senddata)=1.  Otherwise =0
**
** Returns TX_Success
** TX_Success=-2, Lost carrier
**            -1, Garbled
**             0, resent request received. This should not happen.
**             1, "done" received
**             2  "cmds" received. Senddata.
**             3  parameters
** Can upload up to ComMax (=10) files per connection
**
\******************************************************************************/
short Connect_SendFile_RecCmd(const char *filename) {

  short icall = 0;
  short CmdType = 0;
  bool ACK = false;
  bool FileExist = true;
  bool NewCMDS = false;
  long filenumber = 0;
  struct stat info;
  // const char* FileType;
  char currentfile[9];
  int status = 0;

  // FileType="DAT";
  DBG(flogf("\n%s|Connect_SendFile_RecCmd()", Time(NULL)); putflush();
      CIOdrain();)

  TickleSWSR(); // another reprieve

  memset(currentfile, 0, (size_t) 9);

  strncpy(IRIDFilename, filename, 14);
  // Figure out file size and how many blocks need to be sent
  stat(IRIDFilename, &info);
  // Only need to do this once... Unless we Power off the modem.
  PhonePin();

  // Register, call and check SQ, connect the Rudics and login PMEL:
  while (icall <= IRID.MAXCALLS && FileExist) {
    // not really init - whole connect sequence
    ACK = InitModem(status);

    // ACK received from PMEL. Send a file & check the land resp.
    // Loop to send multiple files
    while (ACK && TX_Success != 1 && TX_Success >= -1) {

      TX_Success = Send_File(FileExist, info.st_size);
      // TX_Success=-4, Garbled Response
      //	-3, No Response From Land
      //	-2, Lost carrier
      //        -1, Garbled
      //         0, resent request received. This should not happen.
      //         1, "done" received.
      //         2  "SendData" received. 
      //	 3, Real Commands

      AD_Check();

      // TX Received Done From Land, Time to Hang up.
      if (TX_Success == 1) {

        // flogf("\n\t|Received Done");putflush();CIOdrain();
        if (FileExist) {
          strncpy(currentfile, filename + 2, 8);
          filenumber = atol(currentfile);
          DOS_Com("move", filenumber, "DAT", "SNT");
        }
      }
      // If we received new commands
      else if (TX_Success >= 2) {
        if (TX_Success == 3)
          NewCMDS = true;
        strncpy(currentfile, filename + 2, 8);
        filenumber = atol(currentfile);
        DOS_Com("move", filenumber, "DAT", "SNT");
        filename = GetFileName(IRID.LOWFIRST, false, NULL, "DAT");
        if (filename == NULL)
          FileExist = false;

        if (FileExist) { // More file(s) to send
          SendString("data");
          strncpy(IRIDFilename, filename, 14);
          // Figure out file size and how many blocks need to be sent
          stat(IRIDFilename, &info);
          // flogf("\n\t|Another file to send %s",
          // IRIDFilename);putflush();CIOdrain(); //Loop back to while
        } else {
          SendString("done"); // I think done needs to be sent to hang up.
          flogf("\n\t|All files sent!");
          putflush();
          CIOdrain(); // But still commands may be coming
        }

      }

      else if (TX_Success <= -2) {
        ACK = false;
        status = 0;
        break;
      }
    }

    if (TX_Success == 1)
      break;

    else if (TX_Success <= -1 || LostConnect || icall == IRID.MAXCALLS) {
      flogf("\n\t|Restart while loop due to:");
      if (TX_Success == -2)
        flogf(" No Carrier.");
      else if (TX_Success == -3)
        flogf(" No response from Land");
      else if (TX_Success == -4)
        flogf(" Garbled Response");
      if (LostConnect) {
        flogf(" Lost Connect.");
      } // status=3;}
      if (icall == IRID.MAXCALLS) {
        flogf(" Max Number of Iridium Calls.");
        putflush();
        CIOdrain();
      } else if (icall < IRID.MAXCALLS) {
        TickleSWSR(); // another reprieve
        flogf("\n%s|Connect_SendFile_RecCmd() Rest for %d sec", Time(NULL),
              IRID.REST);
        cdrain();
        if (TX_Success != 1)
          Delay_AD_Log(IRID.REST); // give some time to recover from bad TX
        TickleSWSR();              // another reprieve
      }
    }

    TX_Success = 0;
    icall++;
    LostConnect = false;
    ACK = false; // Reset
  }

  // OpenTUPort_AntMod(false);

  if (NewCMDS)
    TX_Success = 3;

  return TX_Success;

} //____ Connect_SendFile_RecCmd ____//
/******************************************************************************\
** bool GetGPS_SyncRTC)
** Get GPS loc, time and synchronizes CF2 RTC with offset seconds
\******************************************************************************/
bool GetGPS_SyncRTC() {
  // char* Location;
  char *Latitude;
  char *Longitude;
  char *Coordinates;
  short sat_num = 0;
  short count = 0;
  ulong total_seconds = 0;
  int byteswritten = 0;
  static bool firstGPS = true;
  int datafilehandle;
  char uploadfname[] = "c:00000000.bot";
  bool pole = false;

  bool returnvalue = true;
  char* first;
  first = scratch;

  if (MPC.STARTUPS == 0 && firstGPS)
    firstGPS = true;
  else
    firstGPS = false;

  // while sat number is less than 4
  flogf("\n\t|GetGPS_SyncRTC()");
  while (sat_num < 6 && count < GPS_TRIES) {
    SendString("AT+PD");
    RTCDelayMicroSeconds(10000); // HM
    GetGPSInput(NULL, &sat_num);
    flogf("\n\t|Sat num: %d", sat_num);
    cdrain();
    Delay_AD_Log(8);
    count++;
  }
  if (count > 9)
    return false;
  // When sat number is minimum to be acceptable

  if (sat_num >= 6) {

    // freed?
    // Location=(char*)calloc(100,1);
    Latitude = (char *)calloc(16, 1);
    Longitude = (char *)calloc(16, 1);
    Coordinates = (char *)calloc(33, 1);

    // Synchronize the RTC time with GPS
    if (GetUTCSeconds())
      flogf("\n%s|Successful New Time", Time(NULL));
    else
      flogf("\n\t|Failed gathering GPS Time");
    cdrain();

    SendString("AT+PL");
    RTCDelayMicroSeconds(100000); // blk - seems to miss part of reply
    if (GetGPSInput("PL", &sat_num) != NULL) {
      Latitude = strtok(first, "|");
      Longitude = strtok(NULL, "|");
      sprintf(Coordinates, "%s %s", Latitude, Longitude);
      flogf("\n\t|Coords: %s", Coordinates);
      if (!CompareCoordinates(Latitude, Longitude)) {
        SendString("AT+PL");
        if (GetGPSInput("PL", &sat_num) != NULL) {
          Latitude = strtok(first, "|");
          Longitude = strtok(NULL, "|");
          sprintf(Coordinates, "%s|%s", Latitude, Longitude);
          flogf("\n\t|Coords: %s", Coordinates);
          if (!CompareCoordinates(Latitude, Longitude) && !firstGPS) {
            LARA_Recovery();
          }
        }
      }
      strncpy(MPC.LAT, Coordinates, 16);
      //	   	VEEStoreStr(LATITUDE_NAME, strtok(Coordinates, "|"));
      strncpy(MPC.LONG, &Coordinates[17], 16);
      //	   	VEEStoreStr(LONGITUDE_NAME, strtok(NULL, "|"));
      firstGPS = false;
      flogf("\n\t|LAT: %s, LONG: %s", MPC.LAT, MPC.LONG);

      sprintf(&uploadfname[0], "c:%08ld.dat", MPC.FILENUM);
      DBG(flogf("\n\t|Write to file: %s", uploadfname);)
      datafilehandle = open(uploadfname, O_RDWR);
      RTCDelayMicroSeconds(25000L); // ??
      if (datafilehandle <= 0) {
        flogf("\nERROR|GetGPS_SyncRTC(): %s open errno: %d", uploadfname,
              errno);
        // not a fatal err, did not log ??
        returnvalue = false;
      } else {
        DBG2(flogf("\n\t|GetGPS_SyncRTC() %s Opened", uploadfname);)
        lseek(datafilehandle, 21, SEEK_SET); // 22 is for LARA data file
        RTCDelayMicroSeconds(25000L); // ??
        byteswritten = write(datafilehandle, Coordinates, strlen(Coordinates));
        DBG2(flogf("\n\t|Coordinate bytes written: %d", byteswritten);)
        if (close(datafilehandle) != 0)
          flogf("\nERROR  |GetGPS_SyncRTC: File Close error: %d", errno);
        DBG2(else flogf("\n\t|GetGPS_SyncRTC: File Closed");)
      }
    }
    cdrain();
  }

  return returnvalue;
}
//_____ GetGPS_SyncRTC) _____//
/******************************************************************************\
** ulong GetUTCSeconds(char*, char*)
* Returns seconds since 1970
\******************************************************************************/
bool GetUTCSeconds() {

  struct tm t;
  time_t time_seconds;
  char *hours, *minutes, *seconds;
  char *time;
  long difference = 0;
  ulong time_now = 0;

  time = (char *)calloc(24, sizeof(char));
  TURxFlush(AntModPort);
  SendString("AT+PD");
  strncpy(time, GetGPSInput("PD", NULL), 10);
  CLK(start_clock = clock();)
  DBG2(flogf("\n%s|TimeNow", Time(NULL));)
  SendString("AT+PT");
  strcat(time, "|");
  strncat(time, GetGPSInput("PT", NULL), 12);

  DBG2(flogf("\n%s|GetUTCSeconds(): %s", Time(NULL), time);)
  cdrain();

  t.tm_mon = atoi(strtok(time, "-")) - 1;
  t.tm_mday = atoi(strtok(NULL, "-"));
  t.tm_year = atoi(strtok(NULL, "|")) - 1900;

  if ((hours = strtok(NULL, ":")) != NULL) {
    t.tm_hour = atoi(hours);
    if ((minutes = strtok(NULL, ":")) != NULL) {
      t.tm_min = atoi(minutes);
      if ((seconds = strtok(NULL, ".")) != NULL)
        t.tm_sec = atoi(seconds);
      else
        return 0;
    } else
      return 0;
  } else
    return 0;

  time_seconds = mktime(&t);

  Time(&time_now);
  if (time_seconds != -1) {
    // time_seconds=time_seconds+(ulong)IRID.OFFSET;
    difference = time_now - (time_seconds + (ulong)IRID.OFFSET);
    DBG2(flogf("\n\t|Time Difference: %ld", difference);)
    if (difference <= 300L && difference >= -300L) {

      CLK(stop_clock = clock(); print_clock_cycle_count(
              start_clock, stop_clock, "GetUTCSeconds(success)");
          cdrain();)
      RTCSetTime(time_seconds + (ulong)IRID.OFFSET, NULL);

      return true;
    }
  }

  CLK(stop_clock = clock();
      print_clock_cycle_count(start_clock, stop_clock, "GetUTCSeconds(failed)");
      cdrain();)

  return false;
} //_____ GetUTCSeconds() _____//
/*******************************************************************************************
** CopmareCoordinates()
\*******************************************************************************************/
bool CompareCoordinates(char *LAT, char *LONG) {

  int degrees, Degrees;
  float minutes, Minutes, f, F;
  static short GPSChange;
  bool returnvalue = true;
  char Lat[] = "aa:bb.cccc NORTH";
  char Long[] = "dd:ee.ffff WEST";

  strcpy(Lat, MPC.LAT);
  strcpy(Long, MPC.LONG);
  flogf("\nLat: %s, MPC.LAT: %s", LAT, Lat);
  degrees = atoi(strtok(LAT, ":"));
  minutes = atof(strtok(NULL, " "));
  Degrees = atoi(strtok(MPC.LAT, ":"));
  Minutes = atof(strtok(NULL, " NS"));

  flogf("\n\t|%d:%4.2f - %d:%4.2f", degrees, minutes, Degrees, Minutes);
  f = minutes - Minutes;
  if (f < 0.0)
    f = f * -1.0;
  flogf("\n\t|Lat diff: %d:%4.2f", abs(degrees - Degrees), f);

  flogf("\nLong: %s, MPC.LONG: %s", LONG, Long);
  degrees = atoi(strtok(LONG, ":"));
  minutes = atof(strtok(NULL, " "));
  Degrees = atoi(strtok(MPC.LONG, ":"));
  Minutes = atof(strtok(NULL, " EW"));

  flogf("\n\t|%d:%4.2f - %d:%4.2f", degrees, minutes, Degrees, Minutes);
  F = minutes - Minutes;
  if (F < 0.0)
    F = F * -1.0;
  flogf("\n\t|Long diff: %d:%4.2f", abs(degrees - Degrees), F);

  if (f > MAX_GPS_CHANGE || F > MAX_GPS_CHANGE) {
    flogf("\n\t|GPS Change greater than max: %f", MAX_GPS_CHANGE);
    returnvalue = false;
  }

  /*
  IF DIFFERENCE IS 00:01.00 OR GREATER GO TO RECOVERY MODE.
  ELECTRONICS PACKAGE HAS PROBABLY DETACHED.
  THIS MEASUREMENT DEPENDS ON MOORING LINE HEIGHT. IF 1KM MOORING DEPTH,
  DIFFERENCE OF 00:02.00 OR GREATER MIGHT BE NECESSARY
  */

  return returnvalue;

} //____ CompareCoordinates ____//
/******************************************************************************\
** Close COM2 for Iridium/GPS unit and power down
\******************************************************************************/
void OpenTUPort_AntMod(bool on) {

  short wait = 10000;
  int warm;

  DBG2(flogf("  ..OpenTUPort_AntMod() ");)
  if (on) {
    ANTMOD_RX = TPUChanFromPin(ANTMODRX);
    ANTMOD_TX = TPUChanFromPin(ANTMODTX);

    // Power ON
    PIOSet(ANTMODPWR);
    PIOSet(ANTMODCOM);

    AntModPort = TUOpen(ANTMOD_RX, ANTMOD_TX, IRIDBAUD, 0);
    if (AntModPort == 0) {
      flogf("\n\t|Bad IridiumPort");
      return;
    }
    TUTxFlush(AntModPort);
    TURxFlush(AntModPort);

    warm = IRID.WARMUP;
    DBG(flogf("\n%s|Warmup GPS for %d Sec", Time(NULL), warm); cdrain();)
    // connect to gps
    SwitchAntenna('G');

    inputstring = (char *)calloc(STRING_SIZE, 1);
    scratch = (char *)calloc(STRING_SIZE, 1);

    Delay_AD_Log(warm);

    TURxFlush(AntModPort);
    SatComOpen = true;

  } else if (!on) {

    flogf("\n%s|PowerDownCloseComANTMOD() ", Time(NULL));
    SendString("AT*P");

    Delay_AD_Log(3);

    PIOClear(ANTMODCOM);
    PIOClear(ANTMODPWR);
    TUClose(AntModPort);

    free(inputstring);
    free(scratch);

    SatComOpen = false;
  }
}
/******************************************************************************\
** CheckSignal
   returns true if good signal.
   false if too many bad hits.
\******************************************************************************/
bool CheckSignal() {

  short signal_quality = 0, no_sigQ_chk = 0;
  short good_sigQ = 0;
  short last_SQ = 0.;

  flogf("\n\t|CheckSignal() ");

  while ((no_sigQ_chk < Max_No_SigQ_Chk)) {

    TickleSWSR();                   // another reprieve
    SignalQuality(&signal_quality); // Check IRIDIUM signal quality
    flogf("\n\t|CSQ: %d", signal_quality);
    putflush();
    CIOdrain();
    if (signal_quality >= MinSQ && last_SQ >= MinSQ)
      return true;
    last_SQ = signal_quality;
    Delay_AD_Log(2);
    no_sigQ_chk++;
  }

  return false;

} //____ CheckSignal() ____//
/******************************************************************************\
** InitModem()
** Initialize the Irid modem port.
\******************************************************************************/
bool InitModem(int status) {
  bool ACK;
  short ret;

  TX_Success = 0;
  LostConnect = false;

  flogf("\n%s|Modem Initialization", Time(NULL));

  if (!SatComOpen) {
    OpenTUPort_AntMod(true);
    PhonePin();
  }
  ACK = false;
  // bizarre switch without break
  switch (status) {
  case 0:
    ret = PhoneStatus();
    if (ret == 0) // ready
      status++;
    // should wait if ringing or in data call ??
    else
      return false;
  case 1:
    if (CallStatus() == 6)
      status++;
    else
      return false;
  case 2:
    if (CheckSignal())
      status++;
    else
      return false;
  case 3:
    if (Call_Land())
      status++;
    else
      return false;

  case 4:
    ACK = SendProjHdr();
    break;
  }

  return ACK;

} //_____ InitModem _____//
/******************************************************************************\
** Call Status
**    A.8.77 +CLCC - Request Current Call Status
**       Exec Command: +CLCC
**
**       Retern: +CLCC: <stat>
**
** stat: 0  Active
**       1  Call Held
**       2  Dialing (MO Call)
**       4  Incoming (MT Call)
**       6  Idle
**
\******************************************************************************/
short CallStatus() {

  int status = -1;
  short wait = 6000;

  flogf("\n\t|CallStatus()");
  putflush();
  CIOdrain();
  SendString("AT+CLCC");
  RTCDelayMicroSeconds(150000L);
  if (GetIRIDInput(":00", 4, "OK", &status, wait) == 1) {
    switch (status) {
    case 0:
      flogf(": Active");
      break;

    case 1:
      flogf(": Call Held");
      break;

    case 2:
      flogf(": Dialing (MO Call)");
      break;

    case 4:
      flogf(": Incoming (MT Call)");
      break;

    case 6:
      flogf(": Idle");
      break;
    }
    return status;
  } else if (GetIRIDInput(":00", 4, "OK", &status, wait) != 1) {
    flogf(": not ready: %d", status);
    cdrain();
  }

  return status;

} //____ CallStatus() ____//
/******************************************************************************\
** PhoneStatus
**    A.8.29 +CPAS - Modem Activity Status
**       Exec Command: +CPAS<x>
**
**    x: 0  Ready
**       1  Unavailable
**       2  Unknown
**       3  Data Call Ringing
**       4  Data Call In Progress
**
** Check whether a call is in progress or if commands are accepted.
** CPAS status, which is 0 if "Ready"
** Returns -1 CPAS query fails, i.e. CarrierLoss/Error
** Oregon State University, 2/15/2015
\******************************************************************************/
short PhoneStatus() {
  int status;
  short wait = 10000;

  flogf("\n\t|PhoneStatus()");
  putflush();
  CIOdrain();
  SendString("AT+CPAS");
  if (GetIRIDInput(":00", 4, "OK", &status, wait) == 1) {
    switch (status) {
    case 0:
      flogf(": Ready");
      break;

    case 1:
      flogf(": Unavailable");
      break;

    case 2:
      flogf(": Unknown");
      break;

    case 3:
      flogf(": Data Call Ringing");
      break;

    case 4:
      flogf(": Data Call in Progress");
      break;
    }
    return status;
  } else
    flogf(": not ready: %d", status);
  cdrain();

  return -1;

} //____ PhoneStatus() ____//
/******************************************************************************\
** PhonePin
**    A.8.48 +CPIN - Enter PIN
**       Read Command: +CPIN?
**       Results: +CPIN: <code>
**
**    Different Values:
**       READY    not waiting for password
**       PH PIN   waiting for Phone Unlock Code
**       SIM PIN  waiting for SIM card PIN1 code
**       SIM PUK  waiting for SIM PUK code (SIM PIN1 is blocked)
**       SIM PIN2 waiting for SIMPIN2*
**       SIM PUK2 waiting for SIM PUK2  code (SIM PIN2 is blocked)
**
**       *1) PIN1 successfully entered == READY
**        2) No PIN1 required ==READY
**        3) waiting for PIN2
**
** Check whether a call is in progress or if commands are accepted.
** Returns 0 if "Ready", "Data Call Ringing", " Data Call In Progress".
** Returns -1 if "Unavailable" or "Unknown."
** Oregon State University, 2/17/2015
\******************************************************************************/
short PhonePin(void) {
  short wait = 10000;
  flogf("\n\t|no PhonePin on this sim card");
  return 0;
  // ?? sim card setup - add into params
  flogf("\n\t|PhonePin()");
  SendString("AT+CPIN=\"1111\"");
  Delay_AD_Log(2);
  GetIRIDInput("CPIN=", 5, NULL, NULL, 5000);
  if (GetIRIDInput("OK", 2, NULL, NULL, wait) == 1) {
    flogf("SIM PIN Set");
    Delay_AD_Log(3);
    return 0;
  } else
    flogf("\nERROR  |SIM PIN unsuccessful...");

  putflush();
  CIOdrain();

  return 1;
} //____ PhonePin() ____//

//*****************************************************************************
// SignalQuality
// Check the Iridium signal quality (0 to 5)
// H. Matsumoto
//*****************************************************************************
short SignalQuality(short *signal_quality) {

  short returnvalue = 0;
  int sig = -1;

  SendString("AT+CSQ");
  RTCDelayMicroSeconds(10000L);
  TURxFlush(AntModPort);
  TURxGetByteWithTimeout(AntModPort, 5000);
  RTCDelayMicroSeconds(3000000L);

  GetIRIDInput("CSQ:", 5, NULL, &sig, 20000);

  *signal_quality = (short)sig;
  if (*signal_quality < 1)
    returnvalue = -1;

  return returnvalue;
}
/******************************************************************************\
** void SendString()
\******************************************************************************/
void SendString(const char *StringIn) {
  DBG(flogf("\n\t|SendString(%s)", StringIn); cdrain();)
  strcpy(scratch, StringIn);
  strcat(scratch, "\r");
  TUTxPutBlock(AntModPort, scratch, (long) strlen(scratch), (short) 10000);
} //_____ SendString() _____//

/******************************************************************************\
** Call_Land
**
** Call phone number specified.
** Haru Matsumoto, 4/15/2005, NOAA
** Revised by Sorqan Chang-Gilhooly, 12/04/2013
\******************************************************************************/
bool Call_Land(void) {
  short j = 0;
  bool CallOK = false;
  char call[] = "ATD";
  short wait = 12000;
  short length;
  short num = 0;
  short status;

  TickleSWSR(); // another reprieve

  length = strlen(PhoneNum);
  flogf("\n%s|Call_Land()", Time(NULL));
  putflush();
  CIOdrain();
  strncat(call, PhoneNum, length);

  SendString(call);

  // Looks for ATD followed by phonenum
  if (GetIRIDInput("ATD", 3, PhoneNum, NULL, 5000) != 1) {
    CallOK = false;
    DBG2(flogf("\n\t|Call did not make it"); cdrain();)
    // StatusCheck();
    if ((status = PhoneStatus()) == 0) {
      if (CallStatus() == 6) {
        flogf("\n\t|Call_Land Try call again.");
        SendString(call);
        if (GetIRIDInput("ATD", 3, PhoneNum, NULL, 5000) == 1)
          CallOK = true;
      }
    } else if (status == 8192) // still connected
      return true;
  } else
    CallOK = true;

  if (CallOK) {
    num = GetIRIDInput("CONNECT", 7, "192", NULL, wait);
    if (num == 1)
      flogf("\n\t|Connected!");

    else if (num == 0) {
      CallOK = false;
      if (GetIRIDInput("CONNECT", 7, "192", NULL, wait) != 1) {
        flogf("\nERROR  |No Connect...");
        SendString(call);
        if (GetIRIDInput("ATD", 3, "0881600", NULL, 12000) != 1) {
          CallOK = false;
          DBG2(flogf("\n\t|Call did not make it, again");)
        }
        num = GetIRIDInput("CONNEC", 6, "192", NULL, wait);
        if (num == 1) {
          DBG2(flogf("\n\t|Connected!");)
          CallOK = true;
        }
      } else {
        flogf("\n\t|Connected!");
        CallOK = true;
      }
    } else if (num == -1) {
      flogf("\nERROR  |CallLand() Failed");
      CallOK = false;
    }
  }

  fflush(NULL);

  return CallOK; // successful

} //____ Call_Land() ____//
/******************************************************************************\
** SendProjHdr
** After connection is established, send platform ID twice and check if
** ACK is received. If successful, return true.
** HM, NOAA, CIMRS
\******************************************************************************/
bool SendProjHdr() {
  bool Ack = false;
  short Num_ACK = 0;
  short AckMax = 20;
  short wait = 2500; // in millisec //was 1500 8.29.2016
  short Status = 0;
  int crc, crc1, crc2;
  // need an extra char for null terminated
  unsigned char buf[16], proj[8], bmode[4];

  // Flush IO Buffers
  TUTxFlush(AntModPort); TURxFlush(AntModPort);
  DBG(flogf("\n%s|SendProjHdr(%4s%4s)", Time(NULL), MPC.PROJID, MPC.PLTFRMID);)

  sprintf(proj, "%4s%4s", MPC.PROJID, MPC.PLTFRMID);
  crc = Calc_Crc(proj, 8);
  crc1 = crc;
  crc2 = crc;
  sprintf(buf, "???%c%c%c%c%c%c%c%c%c%c%c%c", 
    (char)((crc1 >> 8) & 0x00FF), (char)(crc2 & 0x00FF), 
    proj[0], proj[1], proj[2], proj[3], 
    proj[4], proj[5], proj[6], proj[7]);

  // antMod blockmode for header // ^B, 2 byte length
  sprintf(bmode, "%c%c%c", (char)2, (char)0, (char)13 );

  while (Ack == false && Num_ACK < AckMax) { // Repeat
    AD_Check();
    TUTxPutBlock(AntModPort, bmode, 3, 10000);
    TUTxPutBlock(AntModPort, buf, 13, 10000); // 13 bytes + crlf
    RTCDelayMicroSeconds(16 * 3333L);
    // give rudics a chance to reply
    TickleSWSR(); // another reprieve

    Status = GetIRIDInput("ACK", 3, NULL, NULL, wait);
    if (Status == 1) {
      flogf("\n\t|ACK Received");
      LostConnect = false;
      Ack = true;
    } else {
      if (Status == -1) { // NO CARRIER
        flogf("\n\t|ACK Failed");
        LostConnect = true;
      }
    }
    Num_ACK++;
  }

  if (Num_ACK >= AckMax && Status != 1) {
    flogf("\n\t|ACK inquirey reached max = %d", AckMax);
    cdrain();
    if (LostConnect) {
      flogf("\n\t|LostConnection");
      cdrain();
      StatusCheck();
      RTCDelayMicroSeconds(20000L);
    } else {
      TUTxFlush(AntModPort);
      TURxFlush(AntModPort);

      // Exit in-call data mode to check phone status.
      TUTxPrintf(AntModPort, "+++");  // ??
      TUTxWaitCompletion(AntModPort);
      RTCDelayMicroSeconds(25000L);
      if (GetIRIDInput("OK", 2, NULL, NULL, 3500) == 1) {
        StatusCheck();
        HangUp();
        // StatusCheck(); //deal with phone status
        // if(CallStatus()==4){//enter back into the call?
      } else StatusCheck(); // What happens in this scenario?
    } // LostConnect

    AD_Check();
    flogf("\n%s|SendProjHdr() PLATFORM ID TX FAILED.", Time(NULL));
    putflush();
    CIOdrain();
    TX_Success = -1;
    RTCDelayMicroSeconds(20000L);
  } // Num_ACK >= AckMax
  RTCDelayMicroSeconds(10000L);
  return Ack;

} //____ SendProjHdr() ____//
/******************************************************************************\
** Send_File
** Send the ASC data file after connection and ACK. After the successful file
** upload, it checks if "cmds", "done" or resend request from land. Returns the
** land request as:
**
**	LandResp  = 2, "cmds" Prepare to receice the command
**           = 1, "done" Prepare to send done
**           = 0, @@@ string.  Resend request came. Prepare to resend blocks.
**		       =-1, something else, usually message garbled. Reconnect.
**           =-2, No carrier.  Power off/on
\******************************************************************************/
short Send_File(bool FileExist, long filelength) {
  char bitmap[64]; // 63-byte array corresponds to 64-bit map, val0 and val1 for
                   // resending the data block
  uchar NumOfBlks; // Number of data blocks to send <=63
  long filesize;
  ushort LastBlkLength;
  short i;
  long oldLength;
  ulong val0, val1; // 64-bit bit map for resending the data block
  short Reply = 0;
  // short Delay;
  DBG2(flogf(" .Send_File() ");)
  if (FileExist) {

    if (filelength > (long)IRID.MAXUPL + 500) {
      oldLength = filelength;
      filelength = (long)IRID.MAXUPL + 500;
      flogf("\n\t|Only sending first %ld of %ld bytes", filelength, oldLength);
    }

    NumOfBlks = (uchar)(filelength / (long)BlkLength);

    filesize = (long)BlkLength * NumOfBlks;
    if (filesize < filelength) // If block length*number of blocks is less than
                               // IridFile length
    {
      LastBlkLength =
          (ushort)(filelength -
                   filesize); // Make another block to be sent that has the end
      NumOfBlks += 1;         // Number of blocks = number of blocks + 1
    } else if (filesize == filelength)
      LastBlkLength = (ushort)BlkLength; // True IridFile size same

    // Limit to 64 blocks
    if (NumOfBlks > 63) {
      flogf("\n\t|Send_File() File too big. Only sending first 63 blocks.");
      putflush();
      NumOfBlks = 63;
      LastBlkLength = BlkLength;
    }

    flogf("\n%s|Send_File(%s):\n\t|%ld Bytes, %d BLKS", Time(NULL),
          IRIDFilename, filelength, NumOfBlks);
    cdrain(); // Log the IridFile length
    AD_Check();

    // Send the data IridFile in blocks
    for (i = 0; i < NumOfBlks; i++)
      bitmap[63 - i] = '1'; // Set bitmap all to '1s' to send all blocks
    // DBG(flogf("\n\t|Check First Bitmap: %s", bitmap); putflush();)

    Send_Blocks(bitmap, NumOfBlks, BlkLength, LastBlkLength);
    // note- S_B delays per output size, flushes input
  } // FileExist
  // Delay = (short)LastBlkLength / 1000;
  // if (Delay == 0) Delay = 1;
  // delay for land to reply
  Delay_AD_Log(5);

  Reply = Check_If_Cmds_Done_Or_Resent(&val0, &val1);
  // Check if the send-data went OK
  // Reply = 0 resent request
  // Reply = 1 done. no more commands coming
  // Reply = 2 Fake Commands (To prompt multiple file upload
  // Reply = 3 Real Commands
  // Reply = -1 bad TX
  //      = -2 No carrier
  //      = -5 Inaccurate reply from land

  Num_Resent = 0; // Number of resent trials
  while (Reply <= 0 && Num_Resent < MAX_RESENT) {

    AD_Check();

    // For errors returned from Rudics Basestation
    if (Reply <= 0) {
      Num_Resent++;
      flogf("\n\t|TX INCOMPLETE. Reply=%d", Reply);
      putflush();
      CIOdrain();
      RTCDelayMicroSeconds(10000L);
      if (Reply == 0) { // Request to resend bad blocks
        Convert_BitMap_To_CharBuf(val0, val1, &bitmap);
        DBG2(flogf("\n\t|Resend"); cdrain();
            RTCDelayMicroSeconds(20000L);)
        Send_Blocks(bitmap, NumOfBlks, BlkLength, LastBlkLength);
      }
      //-1 Someting bad happened. TX Failed. Restart Modem
      else if (Reply == -1) {
        Num_Resent = MAX_RESENT;
        flogf("\n\t|Something else");
        putflush();
        CIOdrain();
      }
      //-2 No Carrier response from Modem.
      else if (Reply <= -2) {
        LostConnect = true;
        break; // NO CARRIER
      }
    }
  }

  // Resent effort failed more than XX times
  if (Num_Resent >= MAX_RESENT) {
    flogf("\n%s|Send_File() Resends exceeded Max: %d. ", Time(NULL),
          Num_Resent);
    putflush();
    CIOdrain();
    return -1;
  }

  else if (Reply == 0) {
    flogf("\n%s|Send_File() Land sent resend req.", Time(NULL));
    putflush();
    CIOdrain();
  }

  else if (Reply == -1) {
    flogf("\n%s|Send_File() Land TX garbled.", Time(NULL));
    putflush();
    CIOdrain();
  }

  else if (Reply <= -2) { // No Carrier.  Call again
    flogf("\n%s|Send_File() Lost carrier.", Time(NULL));
    LostConnect = true;
  }

  return Reply;
} //____ Send_File() ____//

/******************************************************************************\
** Send_Blocks
** Send or resend the data. The blocks to be sent are in bitmap field.
\******************************************************************************/
int Send_Blocks(char *bitmap, uchar NumOfBlks, ushort BlockLength,
                ushort LastBlkLength) {

  uchar *buf, bmode[4];
  uchar BlkNum;
  long mlength;
  ushort blklen;
  uchar mlen[2];
  int crc_calc;
  long bytesread;
  const short dataheader = 10; 

  DBG(flogf(" .Send_Blocks() ");)
  IRIDFileHandle = open(IRIDFilename, O_RDONLY);
  if (IRIDFileHandle <= 0) {
    flogf("\nError: Send_Blocks: failed open(%s)", IRIDFilename);
    return -6;
  }
  DBG2(else flogf("\n\t|Send_Blocks: open(%s)", IRIDFilename);)
  //buf = (uchar *)malloc((BlockLength+20) * sizeof(char));
  buf = (uchar *)calloc((int) (BlockLength+20), 1);

  crc_calc = 0x0000;
  for (BlkNum = 1; BlkNum <= NumOfBlks;
       BlkNum++) { // For loop to send all blocks
    AD_Check();
    if (BlkNum == NumOfBlks)
      BlockLength = LastBlkLength; // If Last Blcok, get length of last block
    mlength = BlockLength + dataheader;    // PMEL IRID block size + 5
    blklen = BlockLength + 5;
    mlen[0] = (blklen & 0xFF00) >> 8; // Convert an integer to
    mlen[1] = (blklen & 0x00FF);      // 2-byte uchar.
    DBG1(flogf(", clear %d bytes", blklen + 5); cdrain();)
    memset(buf, 0, (size_t) (blklen + 5)); // Flush the buffer

    bytesread = read(IRIDFileHandle, buf + dataheader, (size_t) BlockLength);
    DBG(flogf("\n\t|Bytes Read: %ld", bytesread); cdrain(); )
    if (bitmap[64 - BlkNum] != '0') { // Send in reverse order
      DBG(flogf("\n\t|SENDING BLK #%d %ld BYTES", BlkNum, mlength); )
      AD_Check();
      buf[5] = mlen[0]; // Block length
      buf[6] = mlen[1];
      buf[7] = 'T'; // Data type
      buf[8] = (uchar)BlkNum;
      buf[9] = (uchar)NumOfBlks;
      crc_calc = Calc_Crc(buf + 5, blklen); // PMEL site crc include first 5
      buf[0] = '@';
      buf[1] = '@';
      buf[2] = '@';
      buf[3] = (uchar)((crc_calc & 0xFF00) >> 8);
      buf[4] = (uchar)((crc_calc & 0x00FF));

      // TUTxFlush(AntModPort);
      // TURxFlush(AntModPort);
      // antMod blockmode for header // ^B, 2 byte length
      sprintf(bmode, "%c%c%c", (uchar)2, 
        (uchar)((mlength >> 8) & 0x00FF), (uchar)(mlength & 0x00FF));
      TUTxPutBlock(AntModPort, bmode, (long) 3, 1000);
      TUTxPutBlock(AntModPort, buf, mlength, 10000);
      // let transmission complete // satellite @ 2400baud*10bit bytes = 240/s
      // 1/240 = 4.167millisec
      RTCDelayMicroSeconds(mlength * 4167L);

      AD_Check();
    } // if bitmap[]
    // pause that refreshes

#ifdef DEBUG
    if (tgetq(AntModPort)) {
      printsafe( TURxGetBlock(AntModPort, scratch, 
          (long) STRING_SIZE, (short) 100),
        scratch);
    }
#else
    TURxFlush(AntModPort);
    cdrain();
#endif
    Delayms(1000);
  }

  free(buf);
  if (close(IRIDFileHandle) != 0)
    flogf("\nERROR  |Send_Blocks: File Close error: %d", errno);
  return 0;

} //____Send_Blocks____//


/********************************************************************************************\
** Calc_Crc
** Calculate 16-bit CRC of contents of buf. Use long integer for calculation,
but returns 16-bit int.
** Converted from P. McLane's C code.
** H. Matsumoto
\********************************************************************************************/
int Calc_Crc(uchar *buf, int cnt) {
  long accum;
  int i, j;
  accum = 0x00000000;

  if (cnt <= 0)
    return 0;
  while (cnt--) {
    accum |= *buf++ & 0xFF;

    for (i = 0; i < 8; i++) {
      accum <<= 1;

      if (accum & 0x01000000)
        accum ^= 0x00102100;
    }
  }

  /* The next 2 lines forces compatibility with XMODEM CRC */
  for (j = 0; j < 2; j++) {
    accum |= 0 & 0xFF;

    for (i = 0; i < 8; i++) {
      accum <<= 1;

      if (accum & 0x01000000)
        accum ^= 0x00102100;
    }
  }

  return (accum >> 8);

} //____Calc_Crc()____//

/******************************************************************************\
** Convert_BitMap_To_CharBuf     Conver the BitMap to a readable Char string
\******************************************************************************/
void Convert_BitMap_To_CharBuf(ulong val0, ulong val1, char *bin_str) {
  ulong remainder;
  int count, type_size;
  short NumBadBlks;

  NumBadBlks = 0;
  type_size = sizeof(ulong) * 8;

  for (count = 0; count < type_size; count++) {
    remainder = val0 % 2;
    if (remainder) {
      bin_str[count] = '1';
      NumBadBlks++;
    } else
      bin_str[count] = '0';
    val0 /= 2;
  }
  for (count = type_size; count < type_size * 2; count++) {
    remainder = val1 % 2;
    if (remainder) {
      bin_str[count] = '1';
      NumBadBlks++;
    } else
      bin_str[count] = '0';
    val1 /= 2;
  }

  // bin_str[count]='\0';
  // Debug
  flogf("\n%s|Convert_BitMap_To_CharBuf() NUM OF BAD BLKS=%d", Time(NULL),
        NumBadBlks);
  putflush();
  CIOdrain(); // RTCDelayMicroSeconds(10000L);
  //	flogf("\n%s", bin_str); RTCDelayMicroSeconds(20000L);
  // debug end

} //____Convert_BitMap_To_CharBuf____//
/******************************************************************************\
** Check_If_Cmds_Done_Or_Resent
** Check if all data received. A "done" or a "cmds" string tells the data was
OK.
**	Returns 2, if "cmds"
**         1, if "done"
**         0, if @@@ string.  Resend request.
**		    -1, something else, usually message garbled.
**        -2, NO CARRIER
**        -3, No Queue... Wait longer?
**        -4, Resend Request Garbled
**        -5, Inaccurate reply from land
\******************************************************************************/
short Check_If_Cmds_Done_Or_Resent(ulong *val0, ulong *val1) {

  uchar hbuf[16], bfo[8];
  short done = 0, j, cmds = 0;
  int resent = 0;
  long qsize;
  long leninput;
  int len;
  short i, buf_size = 11, hbuf_size = 3;
  int crc_rec, crc_chk; // crc
  int k = 1;

  DBG(flogf("\n%s|Waiting for Land", Time(NULL)); cdrain();)
  memset(inputstring, 0, (size_t) STRING_SIZE);
  CLK(start_clock = clock();)

  AD_Check();

  // recode ?? getblock with longer timeout
  // how long are we waiting here?
  Delay_AD_Log(1);
  inputstring[0] = TURxGetByteWithTimeout(AntModPort, 15000);
  Delay_AD_Log(1);
  qsize = (long)tgetq(AntModPort);

  DBG(flogf("\n\t|Check Queue: %ld", qsize); cdrain();)
  RTCDelayMicroSeconds(10000L);
  if (qsize < 7) {
    inputstring[1] = TURxGetByteWithTimeout(AntModPort, 3000);
    k = 2;
    qsize = (long)tgetq(AntModPort);
    DBG(flogf("\n\t|Check Queue: %ld", qsize); cdrain();)
  }

  CLK(stop_clock = clock();
      print_clock_cycle_count(start_clock, stop_clock, "GetIRIDInput");
      cdrain();)
  AD_Check();

  leninput = TURxGetBlock(AntModPort, inputstring + k, qsize,
                           TUBlockDuration(AntModPort, qsize)) + k;
  DBG1(printsafe((long) leninput, inputstring);) 
  len = strspn(inputstring, "\r\ncmdsoneNO C");
  DBG(flogf("\n\t|Len of command characters: %d of %ld", len, leninput);)

  if (len > 1) {
    strncpy(hbuf, inputstring, len);

    if (StringSearch(hbuf, "cmds", NULL) == 1) {
      cmds = Receive_Command((int) leninput);

    } else if (StringSearch(hbuf, "done", NULL) == 1)
      done = 1;
    else if (StringSearch(hbuf, "NO C", NULL) == 1)
      LostConnect = true;
  } else {
    len = strspn(inputstring, "\r\n@ ");
    DBG(flogf("\n\t|Len of possible resendString: %d", len);)
    if (StringSearch(hbuf, "@@@", NULL) == 1)
      resent = 1;
  }

  if (cmds > 0) { //"cmds" string is confirmed get ready to receive a command
    DBG(flogf("\n\t|Incoming Commands"); putflush(); CIOdrain();)
    return cmds + 1;
  } else if (done) //"done" string is confirmed //Moved this else if here
                   //because for some reason we were receiving @@@ in cmds, and
                   //done returns.
  {
    flogf("\n\t|TX COMPLETED, done");
    putflush();
    CIOdrain();
    RTCDelayMicroSeconds(10000L);
    return 1;
  } else if (resent) { //@@@ Received from RUDICS, Need to Resend
    hbuf[hbuf_size] = TURxGetByteWithTimeout(AntModPort, 5000);
    qsize = tgetq(AntModPort);
    for (i = hbuf_size + 1; i < qsize; i++) { // Get other 11-bytes from header
      hbuf[i] = TURxGetByteWithTimeout(AntModPort, 300);
      if (hbuf[i] == '-1') {
        DBG(flogf("\n\t|resent buffer incomplete: %s", hbuf);) return -3;
      } // If any of the 11 are bad, return fail
    }

    crc_rec = hbuf[3] << 8 | hbuf[4]; // Save the received CRC, crc MSB<<8+MSB
    crc_chk = Calc_Crc(hbuf + 5, 9);  // Create a new CRC

    if (hbuf[5] == 'R' &&
        crc_chk == crc_rec) { // Compare new and old CRC, Header OK, proceed.
      flogf("\n\t|Resend CRC Matches");
      putflush();
      CIOdrain(); // RTCDelayMicroSeconds(20000L);
      for (i = 0; i < 8; i++) {
        bfo[i] = 0x00; // clear
        for (j = 0; j < 8; j++)
          bfo[i] |= (uchar)((uchar)hbuf[i + 3] >> j & 0x01)
                    << (7 - j); // reverse the order within the byte
      }
      *val0 = bfo[0] | ((ulong)bfo[1]) << 8 | ((ulong)bfo[2]) << 16 |
              ((ulong)bfo[3]) << 24;
      *val1 = bfo[4] | ((ulong)bfo[5]) << 8 | ((ulong)bfo[6]) << 16 |
              ((ulong)bfo[7]) << 24;
      return 0;
    } else {
      DBG(flogf("\n\t|Resend request garbled: %s", hbuf); putflush();
          CIOdrain(); RTCDelayMicroSeconds(10000L);)
      return -4;
    }
  }

  else {
    DBG(flogf("\n\t|Inaccurate reply from land: %s", hbuf);)
    return -5;
  }

} //____Check_If_Cmds_Done_Or_Resent____//
/******************************************************************************\
**	Receive_Command   Here is where we receive the command header from the
RUDICS
** system after sending the Platform ID. The 10 byte header consists of:
** 1: "@@@" (3-bytes of 0x40)
** 2: CRC Checksum (2-byte)     -CRC includes everything below here \/ \/ \/ \/
\/
** 3: Message length (2-byte)   -Includes everything below here\/ \/ \/ \/ \/
** 4: Message Type (1-byte)     - should be ASCII
** 5: Block Number (1-byte)     - should be one
** 6: Number of Blocks (1-byte) - should be one
** 7: Command: should be less than 64 bytes
**
** LOGICS
**   Receive command (until receives 'done' from land
**   else if it is a real command, save it,
**   else if it is Senddata(), prepare to send another file if there is one
**   TX of the file iss success, rename and move it another direcory
**   Check if there is a next file,
**          Yes- send 'data' and upload the file
**          No - send 'done'
**   If received 'done' break out
**   Return 1 if Senddata(),
**          2 if it is a real command.
**          0 if no command received
\******************************************************************************/
int Receive_Command(int len) {
  int cmds = 1;
  uchar *commands;
  int crc_chk, crc_irid;
  int cmdLength;
  int queue = 0;
  int count = 0;
  int offset = 10;

  commands = (uchar *)calloc(75, 1);

  AD_Check();
  while (queue == 0 && count < 3) {
    queue = tgetq(AntModPort);
    if (queue > 0) {
      DBG(flogf("\n\t|strlen of inputstring: %d", len);)
      TURxGetBlock(AntModPort, inputstring + len, (long) queue,
                   TUBlockDuration(AntModPort, queue));
      break;
    }
    count++;
    Delay_AD_Log(1);
  }

  flogf("\n%s|Receive_Command()", Time(NULL));

  if ((commands = strstr(inputstring, "@@@")) != NULL) {
    DBG(flogf("\n\t|%s", commands + 10);)
    crc_irid = commands[3] << 8 | commands[4];  // Save the received CRC
    cmdLength = commands[5] << 8 | commands[6]; // Get command length;
    DBG(flogf("\n\t|Given CRC: %#4x, CmdLength: %d", crc_irid, cmdLength);)

    offset = 10;
  } else if ((commands = strstr(inputstring, "@@")) != NULL) {
    DBG(flogf("\n\t|%s", commands + 9);)
    crc_irid = commands[2] << 8 | commands[3];
    cmdLength = commands[4] << 8 | commands[5];
    DBG(flogf("\n\t|Given CRC: %#4x, CmdLength: %d", crc_irid, cmdLength);)
    offset = 9;
  }

  flogf("\n\t|initial command string: %s", commands + offset);
  cdrain();

  // Here we get the command length given by the first two bytes of buf (or 6th
  // and 7th bytes of header)

  // Calc_CRC needs everything after crc bytes in header + length/#of bytes
  crc_chk = Calc_Crc(commands + (offset - 5), cmdLength);
  DBG(flogf("\n\t|Calc CRC: %#4x", crc_chk);)
  RTCDelayMicroSeconds(20000L);

  if (crc_chk == crc_irid) { // If our crc values match up...
    flogf("\n\t|%s", commands + 10);
    putflush();
    CIOdrain();
    if (strstr(commands + 10, "Sen") != NULL) { // It's Senddata!!
      cmds = 1; // Senddata() received. Fake command to send data file if there
                // is more.
    } else if (strstr(commands + 10, "$$") != NULL) {
      cmds = 2;
      // strncpy(commands+5,Resp, comlength);   //Store the command.
      putflush();
      CIOdrain();
      SaveParams(commands + 10);
    }
  } else {
    flogf("\n%s|Receive_Command() crc did not match", Time(NULL));
    flogf("\n\t|But lets look at them and accept them anyways?\n\t|%s",
          commands +
              10); // This caused a bad pass of parameters to the bottom Modem
    if (strstr(commands + 10, "$$") != NULL ||
        strstr(commands + 10, "LARA") != NULL) {
      cmds = 2;
      SaveParams(commands + 10);

    } else
      cmds = 0;
  }

  return cmds;
} //____ Receive_Command() ____//
/******************************************************************************\
** HangUp
**  Escape sequence and hang up the call.  Just hang up.  It does not
**  power off.
\******************************************************************************/
bool HangUp(void) {
  short wait = 5000;
  short count = 0;
  short status = 0;
  flogf("\n%s|HangUp() TX_Success=%d. Making sure it hangs up properly",
        Time(NULL), TX_Success);
  putflush();
  CIOdrain();
  RTCDelayMicroSeconds(20000L);

  SendString("ATH");
  while (status != 1 && count < 2) {
    AD_Check();
    status = GetIRIDInput("OK", 2, NULL, NULL, wait);
    if (status == 1) {
      flogf("\n\t|Hang up", Time(NULL));
      putflush();
      CIOdrain();
      return false;
    } else {
      flogf("\n\t|Sending +++");
      cdrain();
      TUTxFlush(AntModPort);
      TURxFlush(AntModPort);

      TUTxPrintf(AntModPort, "+++"); // ??
      TUTxWaitCompletion(AntModPort);
      RTCDelayMicroSeconds(25000L);

      GetIRIDInput("OK", 2, NULL, NULL, wait);
      SendString("ATH");
    }
    count++;
  }

  RTCDelayMicroSeconds(20000L);

  AD_Check();

  return false;

} //____ HangUp() ____//
/*******************************************************************************\
** GetIRIDInput()
** 1: Grabs IRID incoming char stream from the turport
** 2: Look for a character in Template such as ':' and then grab the char
**    (number=num_char_to_read) proceeding chars
** 3: From within here, we can see if we need to return a short pointer for
**    SigQual, and returns
** 4: Compares Iridium buf[] to compstring. If it sees the same string,
 * numchars is optional output, atoi 
**
** @RETURN: if match: 1, no match: 0 and NoCarrier/Error: -1.
** 2/27/2015
\*******************************************************************************/
short GetIRIDInput(char *Template, short num_char_to_reads, uchar *compstring,
                   int *numchars, short wait) {
  short Match = 0;
  int i = 0;
  short stringlength;
  long len;
  long lenreturn;
  char *first;

  first = scratch; // blk - first is a problem

  DBG(flogf("\n\t|GetIRIDInput(%s, %s)", Template, compstring); )

  DBG(ConsoleIrid();)  // check if console is asking to stop

  memset(inputstring, 0, (size_t) STRING_SIZE);
  memset(first, 0, (size_t) STRING_SIZE);

  CLK(start_clock = clock();)

  // Wait up to wait milliseconds to grab next byte from iridium/gps
  // 3.25.14 up to possibly 20 seconds...
  TickleSWSR(); // another reprieve
  inputstring[0] = TURxGetByteWithTimeout(AntModPort, wait);
  RTCDelayMicroSeconds(100000L);

  len = (long) tgetq(AntModPort);
  if (len>STRING_SIZE) len=(long)STRING_SIZE;
  lenreturn = TURxGetBlock(AntModPort, inputstring + 1, len,
                           TUBlockDuration(AntModPort, len));
  if (len == 0)
    return 0;

  DBG1(printsafe(lenreturn, inputstring);)
  // RTCDelayMicroSeconds(25000L);

  TickleSWSR(); // another reprieve


  // If we are looking for the string Template
  if (Template != NULL) {
    // if we did not find Template
    // first!! ptr into inputstring
    if ((first = strstr(inputstring, Template)) == NULL) {
      RTCDelayMicroSeconds(50000L);
      Match = 0;
    }
    // successful Template search
    else {
      Match = 1;
      DBG(flogf("\n%s|Found %s", Time(NULL), Template);)
      CLK(stop_clock = clock();
          print_clock_cycle_count(start_clock, stop_clock, "GetIRIDInput");
          cdrain();)
      if (numchars != NULL) {
        // overwrite begin of inpstr with match .. num_char
        // ouput numchars = int found after end of match
        strncpy(inputstring, first, num_char_to_reads);
        *numchars = atoi(inputstring + strlen(Template));
      }
    }
  } // template

  // If we are looking for the String compstring
  if ((compstring != NULL)) {

    // Get String Length
    stringlength = strlen(compstring);

    if (!Template)
      // no template, so first has not been assigned
      strncpy(first, strstr(inputstring, compstring), stringlength);
    else
      // first points into inputstring at match Template
      strncat(first, strstr(inputstring, compstring), stringlength);

    // If compstring was successfully added to inputstring
    // i.e. if strstr(inputstring, compstring) 
    if (strstr(first, compstring) != NULL)
      Match = 1;
    else {
      Match = 0;
      first[2] = 0;
      RTCDelayMicroSeconds(20000L);
    } // strstr(first, compstring)
  } // (compstring != NULL)
  if (Match > 0)
    return Match;

  if (strstr(inputstring, "NO CARRIER") != NULL) {
    LostConnect = true;
    flogf("\n%s|No Carrier!", Time(NULL));
    putflush();
    CIOdrain();
    Match = -1;
  } else if (strstr(inputstring, "UUUUUU") != NULL) {
    LostConnect = true;
    flogf("\n%s|No Carrier!", Time(NULL));
    putflush();
    CIOdrain();
    Match = -1;
  } else if (strstr(inputstring, "ERROR") != NULL) {
    flogf("\nERROR  |GetIRIDInput();");
    Match = -1;
  }
  if (Match == 0) {
    DBG(flogf("\n\t|GetIRID() Buf: %s", inputstring);)
    Match = StringSearch(inputstring, Template, compstring);
  }

  RTCDelayMicroSeconds(20000L);

  return Match;
} //_____ GetIRIDInput() _____//
/********************************************************************************************\
** StringSearch
**    Powerful search function to look through each string input from Iridium
**    and determine if the characters match at least half of what we expect
\********************************************************************************************/
short StringSearch(char *inString, char *Template, uchar *compstring) {
  short tempLength;
  short compLength;
  int Match = 0;
  int i, Count = 0;

  tempLength = strlen(Template);
  compLength = strlen(compstring);

  if (Template != NULL) {
    for (i = 0; i < tempLength; i++) {
      if (strchr(inString, Template[i]) != NULL)
        Count++;
    }
    DBG(flogf("\n\t|StringSearch(Count: %d of %d)", Count, tempLength);)
    if (Count >= (tempLength / 2) + (tempLength % 2))
      Match = 1;
  }
  Count = 0;
  if (compstring != NULL) {
    for (i = 0; i < compLength; i++) {
      if (strchr(inString, compstring[i]) != NULL)
        Count++;
    }
    DBG(flogf("\n\t|StringSearch(Count: %d of %d)", Count, compLength);)
    if (Count >= (compLength / 2) + (compLength % 2))
      Match = 1;
  }

  return Match;

} //____ IRIDString Search() ____//
/*******************************************************************************\
** char* GetGPSInput()
** 1: Grabs whatever ANTMOD data is incoming from the turport
** 2: Can look for a character "chars" such as ':' and then grab the number
(numchars) proceeding chars
** 2.1:  From within here, we can see if we need to return a short pointer for
SigQual, and returns
** 3: Compares that string str1 to input char* compstring and returns "true"
\*******************************************************************************/
char *GetGPSInput(char *chars, int *numsats) {
  // global inputstring first 
  bool good = false;
  int count = 0;
  long len;
  long lenreturn;

  DBG(ConsoleIrid();)  // check if console is asking to stop

  first = scratch; // blk - first is a problem

  memset(inputstring, 0, (size_t) STRING_SIZE);
  memset(first, 0, (size_t) STRING_SIZE);

  inputstring[0] = TURxGetByteWithTimeout(
      AntModPort, 5000); // Wait for first character to come in.

  RTCDelayMicroSeconds(50000L);
  len = (long)tgetq(AntModPort);
  RTCDelayMicroSeconds(20000L);
  // ?? why flush here
  cdrain();

  lenreturn = TURxGetBlock(AntModPort, inputstring + 1, len,
                           TUBlockDuration(AntModPort, len));
  DBG(flogf("\nGPS<< %s", inputstring);)

  if (chars != NULL) {
    strtok(inputstring, "=");

    if (chars == "PL") {
      strncpy(first, strtok(NULL, "="), 16); // concatenates lat
      strcat(first, "|"); // Places a | for distinction between lat and long
      strncat(first, strtok(NULL, "="),
              16); // concatenates long by tokenizing next '='
      RTCDelayMicroSeconds(20000L);
    }

    else if (chars == "PD")
      strncpy(first, strtok(NULL, "="), 10);

    else if (chars == "PT")
      strncpy(first, strtok(NULL, "="), 12);

    else
      first = NULL;

  }

  else if (strstr(inputstring, "Used=") != NULL) {
    *numsats = atoi(strrchr(inputstring, '=') + 1);
    RTCDelayMicroSeconds(20000L);
    cdrain();
  }
  DBG(flogf("\n\t|first string: %s", first);)
  TURxFlush(AntModPort);
  TUTxFlush(AntModPort);

  return first;

} //_____ GetGPSInput() _____//
  /*******************************************************************************************
  ** void GetIRIDIUMSettings()
  **********************************************************************************************/
void GetIRIDIUMSettings() {

  char *p;

  //"u" maxupload
  p = VEEFetchData(MAXUPLOAD_NAME).str;
  IRID.MAXUPL = atoi(p ? p : MAXUPLOAD_DEFAULT);
  DBG(uprintf("MAXUPL=%d (%s)\n", IRID.MAXUPL, p ? "vee" : "def"); cdrain();)

  p = VEEFetchData(ANTSW_NAME).str;
  IRID.ANTSW = atoi(p ? p : ANTSW_DEFAULT);

  //"P"  0088160000519
  /* Never know if you might need to change the communication number*/
  p = VEEFetchData(IRIDPHONE_NAME).str;
  strncpy(IRID.PHONE, p ? p : IRIDPHONE_DEFAULT, sizeof(IRID.PHONE));

  //"c" number of times to try calling
  p = VEEFetchData(MINSIGQ_NAME).str;
  IRID.MINSIGQ = atoi(p ? p : MINSIGQ_DEFAULT);

  //"c" IRID GPS warm up perido in sec
  p = VEEFetchData(WARMUP_NAME).str;
  IRID.WARMUP = atoi(p ? p : WARMUP_DEFAULT);

  // Maximum number of calls to Iridium
  p = VEEFetchData(MAXCALLS_NAME).str;
  IRID.MAXCALLS = atoi(p ? p : MAXCALLS_DEFAULT);

  // Offset in seconds of syncing CF2 clock with GPS input signal
  p = VEEFetchData(OFFSET_NAME).str;
  IRID.OFFSET = atoi(p ? p : OFFSET_DEFAULT);

  // Time in seconds for Iridium Modem to rest between calls
  p = VEEFetchData(IRIDREST_NAME).str;
  IRID.REST = atoi(p ? p : IRIDREST_DEFAULT);

  // l
  p = VEEFetchData(CALLHOUR_NAME).str;
  IRID.CALLHOUR = atoi(p ? p : CALLHOUR_DEFAULT);

  // m?
  p = VEEFetchData(CALLMODE_NAME).str;
  IRID.CALLMODE = atoi(p ? p : CALLMODE_DEFAULT);
}

/*******************************************************************************************
** void StatusCheck()
*******************************************************************************************/
void StatusCheck() {

  flogf("\n\t|PhoneStatus: %d", PhoneStatus());
  cdrain();

  flogf("\n\t|Call Status: %d", CallStatus());
  cdrain();
  RTCDelayMicroSeconds(25000L);

} //____ StatusCheck() ____//

/***
 * ConsoleIrid()
 * check for console input during IRID modem use
 */
DBG(
void ConsoleIrid() {
  // flush pending, look at most recent char
  char c=0;
  if (!cgetq()) return;
  while (cgetq()) {c = cgetc();}
  switch (c) {
  case 'x': shutdown();
    break;
  }
}
)
