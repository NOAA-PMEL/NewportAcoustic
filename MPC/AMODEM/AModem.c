

#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions

#include <ADS.h>
#include <MPC_Global.h>
#include <PLATFORM.h>
#include <Settings.h>
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

#include <AMODEM.h>

#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <fcntl.h>    // PicoDOS POSIX-like File Access Definitions
#include <stat.h>     // PicoDOS POSIX-like File Status Definitions
#include <termios.h>  // PicoDOS POSIX-like Terminal I/O Definitions
#include <unistd.h>   // PicoDOS POSIX-like UNIX Function Definitions

// LOCAL VARIABLES

// EXTERNAL VARIABLES
extern int AModemPhase;

int ResendRequest = 0;

const char *GetHeader(char *, short, int *);
const char *AModemCom();
void AModemUpload(const char *);
ushort CalcCrc(uchar *, ushort);
void AModemUploadParameters();
void AModemSleep();
static void IRQ3_ISR();

uchar *AModemBuffer;
char *ComString;
int ResendCount = 0;

// AMODEM TUPORT Setup
TUPort *AModemPort;
short AModem_RX, AModem_TX;

extern SystemParameters MPC;
extern bool data;
// Include Acoustic Modem Paramaeters
AMODEMParameters AMDM;
static char fname[] = "c:00000000.bot";
/***************************************************************************\
** void AModem_Data(void)
**
** This function will be much like how WISPR com messages are read in.
** $...*
** returns value depending on status of AModemPort communication
** AModemPort

** Still to do: Implement "RESEND %S" filenumber. Example: RAOT did not
** receive some # of file and requests it.
\***************************************************************************/
short AModem_Data() {
  static bool NewSystemParams = false;
  static short DataCount;
  char fname[sizeof "c:00000000.dat"];

  AModemCom();
  flogf("\n%s|AModem_Data(%s)", Time(NULL), ComString);

  // if ACK, other modem is ready for data
  if (strstr(ComString, "ACK") != NULL) {
    if (AModemPhase == 0)
      AModemPhase++;
    if (AModemPhase == 1) {
      DataCount = 0;
      NewSystemParams = false;
      AModemSend("ACK");
      AModemPhase++;
    } else if (AModemPhase == 2) {
      AModemSend("DATA");
      AModemPhase = 3;
      return 2;
    } else if (AModemPhase == 4) {
      AModemSend("ACK");
      AModemPhase++;
    } else if (AModemPhase >= 5) {
#ifdef RAOT
      AModemUploadParameters();
      AModemPhase = 6;
#endif
#ifdef RAOB
      AModemSend("DONE");
      AModemPhase = 6;
      Delay_AD_Log(5);
      return 2;
#endif
    }
    return 1;

  }
  // if Done Close tuport
  else if (strstr(ComString, "DONE") != NULL) {
    if (AModemPhase == 2) {
      AModemSend("DATA");
      AModemPhase++;
    } else if (AModemPhase == 3) {
      AModemSend("DONE");
      AModemPhase++;
    } else if (AModemPhase == 4) {
      TUTxFlush(AModemPort);
      TURxFlush(AModemPort);
      // AModemReset(); //AModemPort Reset to help save power. Instead of
      // wasting
      // AModemAlarm=false; //Operating in Mode 1 (Near Real-Time operation) we
      // can to Shutoff AModemPort after receiving new Parameters from IRID
    }

    else if (AModemPhase >= 5)
      AModemPhase = 6;

    return 2;
  } else if (strstr(ComString, "RAOS") != NULL) {
    AModemSend("ACK");
    if (SaveParams(ComString))
      NewSystemParams = true;
    else
      NewSystemParams = false;
    AModemPhase = 5;
    return 3;
  } else if (strstr(ComString, "DATA") != NULL) {

    if (AModemPhase < 3 && AModemPhase > 0) {
      sprintf(fname, "c:%08ld.dat", MPC.FILENUM);

      DBG(flogf("\n\t|amodem upload file name: %s", fname);)
      AModemPhase = 3;
      AModemUpload(fname);
      AModemPhase = 4;

      DBG(flogf("\n\t|Post AModemUPload");)
    }

    return 4;
  }
  // if RESEND, resend last block
  else if (strstr(ComString, "RESEND") != NULL) {
    TUTxFlush(AModemPort);
    TURxFlush(AModemPort);
    return 5;
  }

  else if (strstr(ComString, "GPS") != NULL) {
    AModemSend("DONE");
    if (NewSystemParams)
      ParseStartupParams();
    Delay_AD_Log(8);
    AModemPhase = 6;
    return 6;
  }

  // AModemPort TUPort Timeout, returned -1 while waiting for incoming serial
  else if (strstr(ComString, "-1") != NULL) {
    return -1;
    TURxFlush(AModemPort);
    TUTxFlush(AModemPort);
  } else if (strstr(ComString, "-2") != NULL)
    return -2;
  else if (strstr(ComString, "-3") != NULL)
    return -3;

  DBG(flogf("\nData Fail: %s", ComString);)

  if (ResendCount >= MAX_RESENDS) {
    AModemPhase = 6;
    return -6;
  }

  return -4;

} //____ AModem_Data() ____//
/******************************************************************\
** void AModemUpload() Upload new parameters to Bottom Side
\******************************************************************/
void AModemUpload(const char *fname) {

  int amodemfile;
  struct stat fileinfo;
  ulong BlockLength;
  long filelength;
  ushort LastBlkLength;
  uchar NumBlks;
  uchar BlkNum;
  int count = 0, amodemreturn = 0;
  ushort crc_calc, blklen;
  long mlength;
  uchar mlen[2];
  bool ResendBlock = false;
  ushort cmdLength;
  uchar crc_buf[2];
  long fileposition = 0;
  long maxuploadremainder;
  char currentfile[9];
  long filenumber = 0;

  memset(currentfile, 0, 9);

  maxuploadremainder = (long)AMDM.MAXUPL;

  while (fname != NULL) {

    flogf("\n%s|AModemUpload(%s)", Time(NULL), fname);

    stat(fname, &fileinfo);
    DBG(flogf("\n\t|File Size: %ld\n\t|Maxuploadremaining: %ld",
              fileinfo.st_size, maxuploadremainder);) // Log the IridFile length

    // Add in the MAXUPL setting here
    if (fileinfo.st_size >= maxuploadremainder)
      filelength = maxuploadremainder;
    else
      filelength = fileinfo.st_size;

    BlockLength = AMDM.BLKSIZE;
    BlockLength -= 7;
    NumBlks = filelength / BlockLength;
    LastBlkLength = filelength % BlockLength;
    NumBlks++;
    DBG(flogf("\n\t|Number of Blocks: %d \n\t|Last Block Length: %hu", NumBlks,
              LastBlkLength);)

    BlkNum = 1;
    while (BlkNum <= NumBlks) {
      if (!ResendBlock) {

        fileposition = (BlkNum - 1) * BlockLength;
        flogf("\n\t|File Position: %ld", fileposition);
        flogf("\n\t|File Length: %ld", filelength);

        amodemfile = open(fname, O_RDONLY);
        if (amodemfile <= 0) {
          flogf("\nERROR |AModemUpload(): %s open errno: %d", fname, errno);
          return;
        }
        DBG(else flogf("\n\t|AModemUpload() %s Opened", fname);)

        lseek(amodemfile, fileposition, SEEK_SET);

        if (BlkNum == NumBlks)
          BlockLength = LastBlkLength;
        mlength = BlockLength + 7;
        blklen = BlockLength + 5;
        mlen[0] = (blklen & 0xFF00) >> 8;
        mlen[1] = (blklen & 0x00FF);

        // AModemBuffer = (uchar*)malloc(BlockLength+5);
        memset(AModemBuffer + 7, 0, AMDM.BLKSIZE * (sizeof AModemBuffer[0]));

        read(amodemfile, AModemBuffer + 7, BlockLength);
        RTCDelayMicroSeconds(10000L);

        if (close(amodemfile) < 0)
          flogf("\nERROR  |AModemUpload() %s close error: %d", fname, errno);
        DBG(else flogf("\n\t|AModemUpload() %s Closed", fname);)

        RTCDelayMicroSeconds(10000L);

        AModemBuffer[2] = mlen[0]; // Block length
        AModemBuffer[3] = mlen[1];
        AModemBuffer[4] = 'I'; // Data type
        AModemBuffer[5] = BlkNum;
        AModemBuffer[6] = NumBlks;

        crc_calc = CalcCrc(AModemBuffer + 2, blklen);
        cmdLength = mlen[0] << 8 | mlen[1];
        flogf("\n\t|Sending Block: %d, %hu Bytes", BlkNum, cmdLength);
        DBG(flogf("\n\t|crc: %#4x\n\t|Blklen: %d", crc_calc, blklen);
            putflush(); CIOdrain(); RTCDelayMicroSeconds(20000L);)
        crc_buf[1] = (uchar)(crc_calc & 0x00FF);
        crc_buf[0] = (uchar)((crc_calc >> 8) & 0x00FF);

        AModemBuffer[0] = crc_buf[0];
        AModemBuffer[1] = crc_buf[1];
      }

      cdrain();
      coflush();

      TUTxPrintf(AModemPort, "@@@");

      TUTxPutBlock(AModemPort, AModemBuffer, mlength, 30 * mlength);
      TUTxWaitCompletion(AModemPort);
      RTCDelayMicroSeconds(200000L);

      // DBG(  flogf("\n%s", AModemBuffer); )
      RTCDelayMicroSeconds(200000L);

      // Waiting for Response from RAOT
      while (amodemreturn <= 0 && count < 3) {

        AModemSleep();

        if (AD_Check())
          count++;
        if (tgetq(AModemPort))
          amodemreturn = AModem_Data();

        if (amodemreturn == -4)
          AModemSend("RESEND");
      }

      count = 0;
      BlkNum++;

      if (amodemreturn == 1) { // If "ACK"

        ResendBlock = false;
        // if done sending this file
        if (BlkNum == (NumBlks + 1)) {
          maxuploadremainder -= filelength;
          /*moved next three lines below to here*/
          strncpy(currentfile, fname + 2, 8);
          filenumber = atol(currentfile);
          DOS_Com("move", filenumber, "DAT", "SNT");

          if (maxuploadremainder < (long)AMDM.BLKSIZE) {
            AModemSend("DONE");
            RTCDelayMicroSeconds(200000L);
            return;
          }
          /*from here*/
          fname = GetFileName(true, false, NULL, "DAT");
          if (fname == NULL) {
            AModemSend("DONE");
            RTCDelayMicroSeconds(200000L);
            return;
          }
          amodemreturn = 0;
        } else
          amodemreturn = 0;

      } else if (amodemreturn == 4) { // If "DATA"
        flogf("amodemreturn =4");
        amodemreturn = 0;
      } else if (amodemreturn == 5) { // If "RESEND"
        flogf("\n%s|AModem_Upload(): Resending Last Block", Time(NULL));
        ResendBlock = true;
        Delay_AD_Log(5);
        TURxFlush(AModemPort);
        TUTxFlush(AModemPort);
        amodemreturn = 0;
        BlkNum--;
      } else if (amodemreturn < 0) // Bad Return
        flogf("\n%s|AModem_Upload(): Bad return from AModem_Data(): %d",
              Time(NULL), amodemreturn);

      else
        flogf("\n%s|AModem_Upload(): AModemPort unresponsive: %d", Time(NULL),
              amodemreturn);
    }

    RTCDelayMicroSeconds(200000L);
  }
} //____ AModemUpload() ____//
/********************************************************************************************\
** CalcCrc
** Calculate 16-bit CRC of contents of buf. Use long integer for calculation,
but returns 16-bit int.
** Converted from P. McLane's C code.
** H. Matsumoto
\********************************************************************************************/
ushort CalcCrc(uchar *buf, ushort cnt) {
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

} //____CalcCrc()____//
/**************************************************************************************\
** AModemStream
** Stream in AModemPort data
** 10 byte header followed by Blocklength - 10. "@@@CRBL***"
\**************************************************************************************/
int AModemStream(int filehandle) {

  const char *string;
  int i;
  long qsize = 0;
  ushort cmdLength;
  int byteswritten;
  ushort crc_chk;
  ushort crc_comp;
  short charnums = 10;
  bool command = false;
  uchar input = 0;
  uchar ml[10] = {""};
  long bytesreceived = 0;
  int qadd = 0;
  short count = 0;

  //  flogf("\n\t|AModemStream()"); cdrain(); coflush();

  while (input != '@' && input != '$') {
    input = TURxGetByteWithTimeout(AModemPort, 20);
    count++;
    if (count > AMDM.BLKSIZE) {
      command = true;
      string = "ERR";
      break;
    }
  }
  if (input == '$') {
    command = true;
    string = AModemCom();
  }

  if (input == '@') {
    ml[0] = '@';

    for (i = 1; i <= charnums + 2; i++) {
      input = TURxGetByteWithTimeout(AModemPort, 1000);
      if (input == 'I')
        charnums = i;
      ml[i] = input;
    }

    crc_chk = ml[charnums - 4] << 8 | ml[charnums - 3]; // Save the received CRC
    // crc=ml[3]<<8|ml[4]; 			    //Save the received CRC

  }

  // If Bottom Modem Sends Done Command
  else if (command) {
    if (strstr(string, "DONE") != NULL) {
      AModemPhase = 4;
      AModemSend("DONE");
      flogf("\n%s|AModem_Data(DONE)", Time(NULL));
      AD_Check();
      return 1;
    } else if (strstr(string, "RESEND") != NULL) {
      AModemResend();
    } else if (strstr(string, "ERR") != NULL) {
      flogf("\nERROR|AModemStream(): No Header");
      AModemSend("DATA");
      return -1;
    } else if (strstr(string, "DATA") != NULL) {
      flogf("\n%s|AModemStream(): Next Data File", Time(NULL));
      return 4;
    }
  }

  i = 0;
  // Find Block Length
  cmdLength = ml[charnums - 2] << 8 | ml[charnums - 1];
  qsize = (long)tgetq(AModemPort);
  DBG(flogf("\n\t|charnums: %d", charnums);)
  DBG(flogf("\n\t|header: %s", ml);)
  DBG(flogf("\n\t|cmdLength: %d, qsize: %ld", cmdLength, qsize);)

  if (cmdLength > AMDM.BLKSIZE) {
    flogf("\nERROR|Block larger than AMDM.BLKSIZE");
    while (qsize != (long)tgetq(AModemPort)) {
      qsize = (long)tgetq(AModemPort);
      RTCDelayMicroSeconds(20000L);
    }
    DBG(flogf("\n\t|False cmdLength of: %hu, changed to bytes in queue: %ld",
              cmdLength, qsize);)
    cmdLength = (ushort)qsize;
    qadd = 5;
  } else
    qadd = 0;
  if (cmdLength <= 1)
    return -1;

  flogf("\n\t|Streaming %hu Bytes", cmdLength);
  RTCDelayMicroSeconds(200000L);

  memset(AModemBuffer, 0, AMDM.BLKSIZE * (sizeof AModemBuffer[0]));

  for (i = 0; i < 5; i++)
    AModemBuffer[i] = ml[charnums - 2 + i];

  // flogf("\n\t|Block %d of %d", AModemBuffer[3], AModemBuffer[4]);

  //  flogf("\n\t|Grabbing %hu Byte", cmdLength);

  bytesreceived =
      TURxGetBlock(AModemPort, AModemBuffer + 5, (long)cmdLength - 5,
                   20 * cmdLength); // Testing 7.15.2015, commented section
                                    // below works well. Trying to streamline

  DBG(flogf("\n\t|bytesreceived: %ld", bytesreceived);)
  // DBG(flogf("\n\n%s", AModemBuffer);)
  crc_comp = CalcCrc(AModemBuffer, cmdLength);
  DBG(flogf("\nCalculated Crc: %#4x, Received Crc: %#4x", crc_comp, crc_chk);
      putflush(); CIOdrain(); RTCDelayMicroSeconds(20000L);)

  if (crc_comp == crc_chk) {
    flogf("\n%s|Successful Amodem Download Block %X of %hu", Time(NULL),
          AModemBuffer[3], AModemBuffer[4]);

    byteswritten = write(filehandle, AModemBuffer + 5, (ulong)cmdLength - 5);
    if (byteswritten == -1)
      flogf("\nERROR  |attempt to write... errno: %d", errno);
    else
      DBG(flogf("\n\t|Bytes Written: %d", byteswritten);)
    cdrain();
    coflush();
    RTCDelayMicroSeconds(1000000L);
    AModemSend("ACK");

  } else {
    flogf("\nERROR|Bad CRC Check. Resend Last Block");
    cdrain();
    coflush();
    AModemSend("RESEND");
    Delay_AD_Log(1);
    TUTxFlush(AModemPort);
    TURxFlush(AModemPort);
    if (ResendRequest > 3)
      return -3;
    else
      return -2;
  }

  return 0;

} //_____ AModemStream() _____//
/*******************************************************************************\
** char* GetCmdsHeader()
** 1: Grabs whatever IRIDGPS data is incoming from the turport
** 2: Can look for a character "chars" such as ':' and then grab the number
(numchars) proceeding chars
** 2.1:  From within here, we can see if we need to return a short pointer for
SigQual, and returns
** 3: Compares that string str1 to input char* compstring and returns "true"
\*******************************************************************************/
const char *GetHeader(char *chars, short charnums, int *numchars) {
  int i = 0;
  uchar input;
  uchar ml[5];
  int crc;
  short count = 0;

  if (chars == "@@@") {
    while (input != '@' && input != '$') {
      input = TURxGetByteWithTimeout(AModemPort, 150);
      count++;
      if (count > AMDM.BLKSIZE)
        return "ERR";
    }
    if (input == '$') {
      return AModemCom();
    }

    RTCDelayMicroSeconds(20000L);
    while (input == '@') {
      ml[0] = input;
      input = TURxGetByteWithTimeout(AModemPort, 150);
    }

    for (i = 3; i < charnums; i++) {
      input = TURxGetByteWithTimeout(AModemPort, 1000);

      ml[i] = input;
    }

    crc = ml[3] << 8 | ml[4]; // Save the received CRC

    *numchars = crc;
  }
  return ml;

} //_____ GetCmdsHeader() _____//
/***************************************************************************\
** bool AModemSend()
\***************************************************************************/
void AModemSend(char *send) {

  TUTxFlush(AModemPort);
  TUTxPrintf(AModemPort, "$$$%s***", send);
  TUTxWaitCompletion(AModemPort);
  RTCDelayMicroSeconds(10000L);
  TURxFlush(AModemPort);
  flogf("\n\t|AModemSend(%s)", send);

  if (strstr(send, "ACK") != NULL)
    ResendRequest = 1;
  if (strstr(send, "DONE") != NULL)
    ResendRequest = 2;
  if (strstr(send, "DATA") != NULL)
    ResendRequest = 3;
  if (strstr(send, "RESEND") != NULL) {
    ResendRequest = 4;
    ResendCount++;
  }

  Delay_AD_Log(5);
}
/******************************************************************************\
** AModemCom()
** 	Will return a const char* if a recognized command is received: $xxx*
**		Else a return of "-1": Nothing in TUPort
**		"-2": Found "$" but nothing after.
** 	"-3": Did not find a "*" in TUPort
**		"
\******************************************************************************/
const char *AModemCom() {

  const char *asterisk;
  char *pointer;
  char *gps;
  int stringlength;
  bool good = false;
  int i = 0, k;
  char in;
  char r[61];
  char inchar;
  short count = 0;
  ulong total_seconds;
  ulong gps_time;

  // ComString=(char*)calloc(60,1);
  memset(ComString, 0, 60 * sizeof(ComString[0]));
  inchar = TURxGetByteWithTimeout(AModemPort, 100);

  for (k = 0; k < 100; k++) {
    if (inchar != '$')
      inchar = TURxGetByteWithTimeout(AModemPort, 200);
    else
      break;
  }
  if (k > 98)
    return "-1";
  r[0] = '$';
  r[1] = TURxGetByteWithTimeout(AModemPort, 100);
  if (r[1] == -1) {
    DBG(flogf("\n%s|AModem_Data():The first input character is negative one",
              Time(NULL));
        RTCDelayMicroSeconds(20000L);)
    return "-2";
  }
  for (i = 2; i < 64; i++) { // Up to 62 characters
    in = TURxGetByteWithTimeout(AModemPort, 100);
    if (in == '*') {
      r[i] = in;
      break;
    } // if we see an * we call that the last of this WISPR Input
    else if (in == -1) {
      return "-3";
    } else
      r[i] = in;
  }

  // Looks for end of AModemPort input, gets length, appends to ComString
  asterisk = strchr(r, '*');
  stringlength = asterisk - r + 1; // length from '$' to '*'
  strncat(ComString, r, stringlength);

  if (strstr(ComString, "GPS") != NULL) {
    gps = (char *)calloc(11, 1);
    pointer = strchr(ComString, 'S');
    gps = strtok(pointer + 1, ",");
    //      longitude = strtok(NULL, ",");
    //      latitude = strtok(NULL, "*");
    //      flogf("\n\t|Long: %s, Lat: %s", longitude, latitude);
    total_seconds = (ulong)atol(gps);
    // if new time is greater than a few minutes. 5 minute difference?
    gps_time = RTCGetTime(NULL, NULL);
    if ((total_seconds < gps_time - 300) || (total_seconds > gps_time + 300))
      return ComString;

    else {
      flogf("\n%s|Time Now", Time(NULL));
      RTCSetTime(total_seconds + 6, NULL);
      flogf("\n%s|New Time", Time(NULL));
    }
  }

  return ComString;
}
/******************************************************************************\
** AModemUploadParameters()
**    The function should only be called by the Buoy.
**    Purpose is to update the subsurface mooring with new system parameters
\******************************************************************************/
void AModemUploadParameters() {

  int file;
  char *parameterstring;
  char RAOSParamFile[] = "c:SYSTEM.CFG";
  size_t bytesread;
  struct stat fileinfo;
  int paramlength;
  char *string = {""};
  char gps[sizeof "gps1234567890"];

  flogf("\n%s|AModemUploadParameters()", Time(NULL));

  sprintf(RAOSParamFile, "c:SYSTEM.CFG");
  stat(RAOSParamFile, &fileinfo);
  paramlength = fileinfo.st_size;
  parameterstring = (char *)malloc(paramlength);
  file = open(RAOSParamFile, O_RDONLY);
  if (file <= 0) {
    flogf("\n\t|AModemUploadParameters(): file open errno: %d", errno);
    return;
  } else
    flogf("\n\t|AModemUploadParameters() %s opened", RAOSParamFile);

  bytesread = read(file, parameterstring, paramlength);
  DBG(flogf("\n\t|Bytes Read from %s: %ld", RAOSParamFile, bytesread); cdrain();
      coflush();)
  if (close(file) < 0)
    flogf("\nERROR  |%s close errno: %d", RAOSParamFile, errno);

  DBG(else flogf("\n\t|%s closed", RAOSParamFile);)

  strncpy(string, parameterstring, bytesread);
  RTCDelayMicroSeconds(100000L);
  AModemSend(string);

  // NEED TO IMPLEMENT
  // AMODEM SLEEP FUNCTION

  Delay_AD_Log(5);

  if (strstr(AModemCom(), "ACK")) {
    sprintf(gps, "GPS%10ld", RTCGetTime(NULL, NULL));
    RTCDelayMicroSeconds(100000);
    flogf("\n%s|TimeNow");
    cdrain();
    coflush();
    AModemSend(gps);
    flogf("\n\t|Successful Parameter update");
    ParseStartupParams();
  } else {
    flogf("\n\t|Unsuccessful Parameter update... Trying again");
    AD_Check();
    AModemSend(string);
    if (strstr(AModemCom(), "ACK")) {
      ParseStartupParams();
      flogf("\n\t|Second Try was a success");
    } else {
      flogf("\n\t|Second Try was unsuccessful");
    }
  }
}
/*******************************************************************************\
**
\********************************************************************************/
void AModemResend() {
  DBG(flogf("\n\t|AModemResend()");)
  if (ResendRequest == 1)
    AModemSend("ACK");
  else if (ResendRequest == 3)
    AModemSend("DATA");
}
/*******************************************************************************\
** GetAMODEMSettings
\*******************************************************************************/
void GetAMODEMSettings() {

  char *p;

  p = VEEFetchData(AMODEMOFFSET_NAME).str;
  AMDM.OFFSET = atoi(p ? p : AMODEMOFFSET_DEFAULT);
  DBG(uprintf("AMODEMOFFSET=%u (%s)\n", AMDM.OFFSET, p ? "vee" : "def");
      cdrain();)

  p = VEEFetchData(AMODEMMAXUPLOAD_NAME).str;
  AMDM.MAXUPL = atoi(p ? p : AMODEMMAXUPLOAD_DEFAULT);
  DBG(uprintf("AMODEMMAXUPLOAD=%u (%s)\n", AMDM.MAXUPL, p ? "vee" : "def");
      cdrain();)

  p = VEEFetchData(AMODEMBLOCKSIZE_NAME).str;
  AMDM.BLKSIZE = atoi(p ? p : AMODEMBLOCKSIZE_DEFAULT);
  DBG(uprintf("AMODEMBLOCKSIZE=%u (%s)\n", AMDM.BLKSIZE, p ? "vee" : "def");
      cdrain();)

} //____ GetAMODEMSettings() ____//
/******************************************************************************\
**	SleepUntilWoken		Finish up
**
** 1-st release 9/14/99
** 2nd release 6/24/2002 by HM -Changed to use ADS8344/45
** 3rd release 2014 AT
\******************************************************************************/
void AModemSleep(void) {

  RTCDelayMicroSeconds(5000L);

  IEVInsertAsmFunct(IRQ3_ISR, level3InterruptAutovector); // AModem Interrupt
  IEVInsertAsmFunct(IRQ3_ISR, spuriousInterrupt);

  // CTMRun(false);
  SCITxWaitCompletion();
  EIAForceOff(true);
  // QSMStop();
  CFEnable(false);

  TickleSWSR(); // another reprieve

  PinBus(IRQ3RXX); // AModem Interrupt

  while (PinRead(IRQ3RXX) && !data)
    LPStopCSE(FastStop);

  // QSMRun();
  EIAForceOff(false); // turn on the RS232 driver
  CFEnable(true);     // turn on the CompactFlash card
  // CTMRun(true);

  PIORead(IRQ3RXX);

  DBG(flogf(",");)
  RTCDelayMicroSeconds(5000L);

} //____ Sleep_AModem() ____//
/*************************************************************************\
**  static void Irq3ISR(void)
\*************************************************************************/
static void IRQ3_ISR(void) {
  PinIO(IRQ3RXX);
  RTE();
} //____ Irq3ISR ____//
/*************************************************************************\
**   void OpenTUPort_AModem
\*************************************************************************/
void OpenTUPort_AModem(bool on) {

  if (on) {
    AModemBuffer = (uchar *)malloc(AMDM.BLKSIZE + 10);
    ComString = (char *)malloc(60);
    ResendCount = 0;

    AModem_RX = TPUChanFromPin(AMODEMRX);
    AModem_TX = TPUChanFromPin(AMODEMTX);

    PIOClear(AMODEMPWR);
    RTCDelayMicroSeconds(250000L);
    PIORead(48);
    PIOSet(AMODEMPWR);
    AModemPort = TUOpen(AModem_RX, AModem_TX, AMODEMBAUD, 0);
    if (AModemPort == 0)
      flogf("\n\t|Bad AModem port\n");
    else {
      TUTxFlush(AModemPort);
      TURxFlush(AModemPort);
      Delay_AD_Log(5); // Wait 5 seconds for AModemPort to power up
    }
  }
  if (!on) {
    Delay_AD_Log(5);
    PIOClear(AMODEMPWR);
    TUClose(AModemPort);
    free(AModemBuffer);
    free(ComString);
  }
  return;
}
/*
*
*/
void AModem_SetPower(bool high) {

  long waittime = 10000;

  if (AModemPort == 0) {
    OpenTUPort_AModem(true);
  }

  TUTxPrintf(AModemPort, "~");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "\%");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "L");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "Q");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "U");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "W");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "M");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, ".");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "C");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "O");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "M");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "D");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "3");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "2");
  RTCDelayMicroSeconds(waittime);
  TUTxPrintf(AModemPort, "0");
  RTCDelayMicroSeconds(waittime);
  if (high) {
    TUTxPrintf(AModemPort, "1");
    RTCDelayMicroSeconds(waittime);
  } else {
    TUTxPrintf(AModemPort, "0");
    RTCDelayMicroSeconds(waittime);
  }
}