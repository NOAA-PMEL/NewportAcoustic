#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions
#include <MPC_Global.h>
#include <PLATFORM.h>
//#include <Winch.h>
#include <CTD.h>

#include <ADS.h>
#include <Settings.h>
#include <Winch.h>
#include <assert.h>
#include <ctype.h>
#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <errno.h>
#include <fcntl.h> // PicoDOS POSIX-like File Access Definitions
#include <float.h>
#include <float.h>
#include <limits.h>
#include <locale.h>
#include <math.h>
#include <setjmp.h>
#include <signal.h>
#include <stat.h> // PicoDOS POSIX-like File Status Definitions
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h> // PicoDOS POSIX-like Terminal I/O Definitions
#include <time.h>
#include <unistd.h> // PicoDOS POSIX-like UNIX Function Definitions

time_t CTD_VertVel(time_t);

int Sea_Ice_Algorithm(); // Returns reason: 1=Stop, should be at surface and
                         // ready to send.
// 2: Stop due to low Temp or other reason
// 3: Ascend2, careful ascent
// 4: Descend, definitely ice
// 5: Stop due to little change in salinity
// CTDParameters CTD;

extern SystemParameters MPC;
extern SystemStatus LARA;

CTDParameters CTD;

bool SyncMode;

TUPort *CTDPort;

float SummedVelocity;
short CTDSamples;

char CTDLogFile[] = "c:00000000.ctd";

// simulation variables: #ifdef LARASIM
float descentRate = 0.1923; // m/s
float ascentRate = 0.26;

char *stringin;
char *stringout;
char *writestring;
/*****************************************************************************\
** CTD_Start_Up()
\*****************************************************************************/
bool CTD_Start_Up(bool settime) {
  bool returnval = false;

  // CTD_CreateFile(MPC.FILENUM);

  if (CTDPort == NULL)
    OpenTUPort_CTD(true);

  CTD_SampleBreak();
  Delay_AD_Log(1);

  if (CTD_GetPrompt()) {
    cprintf("successful startup");
    returnval = true;
  } else {

    if (!CTD_GetPrompt()) {
      OpenTUPort_CTD(false);
      RTCDelayMicroSeconds(100000);
      OpenTUPort_CTD(true);
      CTD_SampleBreak();
      if (CTD_GetPrompt()) {
        returnval = true;
        cprintf("successful startup3");
      }
    } else {
      returnval = true;
      cprintf("sucessful startup2");
    }
  }
  if (settime)
    CTD_DateTime();

  TURxFlush(CTDPort);
  TUTxFlush(CTDPort);

  return returnval;

} //_____ CTD_Start_Up() _____//
/******************************************************************************\
** CTD_CreateFile()
\******************************************************************************/
void CTD_CreateFile(long filenum) {

  int filehandle;

  sprintf(&CTDLogFile[2], "%08ld.ctd", filenum);
  filehandle = open(CTDLogFile, O_RDWR | O_CREAT | O_TRUNC);
  RTCDelayMicroSeconds(50000L);
  if (filehandle < 0) {
    flogf("\nERROR  |CTD_CreateFile() errno: %d", errno);
  }

  if (close(filehandle) < 0)
    flogf("\nERROR  |CTD_CreateFile(): %s close errno: %d", CTDLogFile, errno);

  RTCDelayMicroSeconds(10000L);

} //____ CTD_CreateFile() _____//
/********************************************************************************\
** CTD_DateTime()
\********************************************************************************/
void CTD_DateTime() {

  time_t rawtime;
  struct tm *info;
  char buffer[15];

  time(&rawtime);

  info = gmtime(&rawtime);

  //	strftime(buffer, 15, "%m%d%Y%H%M%S", info);
  sprintf(buffer, "%02d%02d%04d%02d%02d%02d", info->tm_mon, info->tm_mday,
          info->tm_year + 1900, info->tm_hour, info->tm_min, info->tm_sec);
  printf("\nCTD_DateTime(): %s\n");

  TUTxPrintf(CTDPort, "DATETIME=%s\n", buffer);
  RTCDelayMicroSeconds(250000L);
  while (tgetq(CTDPort))
    cprintf("%c", TURxGetByte(CTDPort, true));

} //_____ CTD_DateTime() ____//
/********************************************************************************\
** CTD_GetPrompt()
\********************************************************************************/
bool CTD_GetPrompt() {

  short count = 0;
  short LastByteInQ;

  memset(stringin, 0, 128 * sizeof(char));

  LastByteInQ = TURxPeekByte(CTDPort, (tgetq(CTDPort) - 1));
  while (((char)LastByteInQ != '>') && count < 2) { // until a > is read in
                                                    // command line for CTDPort
                                                    // or 7 seconds pass
    TUTxPrintf(CTDPort, "\n");
    RTCDelayMicroSeconds(1000000);
    LastByteInQ = TURxPeekByte(CTDPort, (tgetq(CTDPort) - 1));
    count++;
  }

  if (count == 2) {
    TURxGetBlock(CTDPort, stringin, 128 * sizeof(uchar), 1000);
    if (strstr(stringin, "S>") != NULL) {
      cprintf("\nPrompt from CTDBlock");
      TURxFlush(CTDPort);
      return true;
    }
    return false;
  }

  else {
    cprintf("\nPrompt from CTD");
    TURxFlush(CTDPort);
    return true;
  }
}
/********************************************************************************\
** CTD_Sample()
\********************************************************************************/
void CTD_Sample() {

  if (SyncMode) {
    // cprintf("pulse");
    TUTxPrintf(CTDPort, "pulse\n");
  } else {
    // cprintf("ts");
    TUTxPrintf(CTDPort, "TS\n");
  }

  RTCDelayMicroSeconds(250000);

} //____ CTD_Sample() ____//
/********************************************************************************\
** CTD_SampleSleep()
\********************************************************************************/
void CTD_SyncMode() {

  TUTxPrintf(CTDPort, "Syncmode=y\n");
  TUTxWaitCompletion(CTDPort);
  RTCDelayMicroSeconds(500000);
  TUTxPrintf(CTDPort, "QS\n");
  TUTxWaitCompletion(CTDPort);
  RTCDelayMicroSeconds(1000000);

  SyncMode = true;

  TURxFlush(CTDPort);

} //____ CTD_Sample() ____//
/********************************************************************************\
** CTD_Sample()
\********************************************************************************/
void CTD_SampleBreak() {

  TUTxBreak(CTDPort, 5000);

  SyncMode = false;

} //____ CTD_Sample() ____//
/************************************************************************************\
** void Sea_Ice_Algorithm(void)
** The sea ice algorthim starts a careful ascent when the buoy reaches the
threshold depth
** When first leaving winch, grab initial temperature and salinity.
** When the depth threshold is passed:
**    1. if change in temperature and salinity are less than given amount, stop
**    2. if temp is warmer than min temp: careful ascent, less than min: descend
**    3.
** if (t_change of <.5 && Sal_change <.5). stop. change to continuous sampling,
ascend
** if TEMP @10-20m <-1.5 stop, and descend
** Water Temperature at Mooring M8(The influence of sea ice... Sullivan et. al)
** can be as low as -1.8
**
\************************************************************************************/
int Sea_Ice_Algorithm() {
  static int ice;
  static float temp_change, current_temp, next_temp;
  static float sal_change, current_sal, next_sal;
  /*
   //Do this only one time for each Buoy Ascent
   if(CTD_Docked==true){
   cprintf("\nSetting initial Temp and Sal in SIA");
   ice=0;
   current_temp=Initial_Temp;
   current_sal=Initial_Sal;
   temp_change=0;
   sal_change=0;
   CTD_Docked=false;
   }

   if(sim==true){    //Only if Simulation is true
   next_temp=SimTemp;
   next_sal=SimSal;
   }
   else{
   next_temp=temp;
   next_sal=salinity;
   }

   temp_change=(temp_change+ (next_temp-current_temp)); //Calculate TEMP change
   sal_change=(sal_change+ (next_sal-current_sal));    //Calculate salinity
  change

   flogf("\ndT: %f, dS: %f", temp_change, sal_change);

   current_sal=next_sal;
   current_temp=next_temp;




   //Simulation Mode
   if(sim==true){
      if(SimDepth<=Depth_Threshold&&BuoyMode==1){  cprintf("\nSIA: Depth
  Threshold Reached");

         if(temp_change<=0.5&&(abs(sal_change)<=0.5)){
            cprintf("\nSIA: Small variation between TEMP and sal ");
            CommandWinch("\#S,01,00", true);   RTCDelayMicroSeconds(2000000);
  //try get command to get reult back? so no delay needed
            if(temp<=Temp_Threshold){ CommandWinch("\#F,01,00", true);
               RTCDelayMicroSeconds(2000000);       cprintf("\nSIA: Minimum
  Temperature Recognized ");
               ice=4; }
            else if(temp>Temp_Threshold&&BuoyMode==0){
  cprintf("\nSIA: Careful Ascent ");
               ice=3;              CommandWinch("\#R,01,02", true);
  RTCDelayMicroSeconds(2000000);     }
         }
         else if(SimDepth<=Min_Depth){
            CommandWinch("\#S,01,00", true);
            //Surfaced=true; ice=1;
         }


      }
      else if(BuoyMode==3){  if(abs(sal_change)<0.4){//salinity becomes less at
  buoy ascends
            cprintf("\nSIA: Continuing Careful Ascent ");   ice=3;    }
         else if(abs(sal_change)<0.4){  cprintf("\nSIA: Temp:OK, Sal Change: Bad
  ");
            ice=5;   CommandWinch("\#S,01,00", true);
  RTCDelayMicroSeconds(2000000);
            CommandWinch("\#F,01,00", true);  return ice;   }
         if(current_temp<Temp_Threshold){   cprintf("\nSIA: Temp: Too Low ");
  ice=2;
            CommandWinch("\#S,01,00", true);  RTCDelayMicroSeconds(2000000);
            CommandWinch("\#F,01,00", true);  return ice;   }
         if(PRES<=Min_Depth){   CommandWinch("\#S,01,00", true);
            RTCDelayMicroSeconds(2000000);      cprintf("\nSIA: Reached Minimum
  Depth ");
            //Surfaced=true;
            ice=1;  }
      }
   }




   //Real Dive Sea Ice Algorithm Mode
   else if(pres<=Depth_Threshold&&BuoyMode==1){          //Depth threshold
  reached
      if(temp_change<0.5&&sal_change<0.5){               //if change in ctd
  readings is small
         CommandWinch("\#S,01,00", true);                     //Stop winch
         if(temp<=Temp_Threshold){                       //if temp is below
  threshold
            CommandWinch("\#F,01,00", true);                  //descend
            cprintf("SIA: Minimum Temperature Recognized ");
            ice=4;        }
            else if(temp>Temp_Threshold&&BuoyMode==0){   //if temp is above
  threshold
            CommandWinch("\#R,01,02", true);                  //put in careful
  ascent mode
            cprintf("SIA: Careful Ascent ");
            ice=3;   }}
      else if(pres<=Min_Depth){                          //if change in ctd
  readings is large, and buoy reaches minimum depth
         CommandWinch("\#S,01,00", true);                     //stop buoy
         cprintf("SIA: Minimum Depth Reached ");
        // Surfaced=true;
        }   }                           //surfaced = true. go to iridium

  else if(BuoyMode==3){                                  //careful ascent mode
      if(current_temp>Temp_Threshold){                   //temperatures reads ok
         cprintf("\nSIA: Continuing Careful Ascent ");
         ice=3; }                                        //no ice
      else if(abs(sal_change)<0.4){                           //change in
  salinity is too small
         CommandWinch("\#S,01,00", true);                     //stop
         ice=2;                                          //sal change bad
         CommandWinch("\#F,01,00", true);  }                  //descend
      else if(pres<=Min_Depth){                          //buoy reached minimum
  depth
         CommandWinch("\#S,01,00", true);                     //Should it ascend
  more if still no signal? depends on our iridium float attached to the buoy
        // Surfaced=true;                                  //surfaced =true go
  to iridium
         ice=1;}
  }
  */
  return ice;
} //____ bool Sea_Ice_Algorithm(void) ____/
/******************************************************************************\
** void CTD_data()
* CTD with fluro, par
* Temp, conductivity, depth, fluromtr, PAR, salinity, time
* 16.7301,  0.00832,    0.243, 0.0098, 0.0106,   0.0495, 14 May 2017 23:18:20
* The response is approximately 2 sec.
* FLS and PAR data in between the depth and salinity. Ignore the conductivity.
\******************************************************************************/
bool CTD_Data() {
  bool log = false;
  bool prompt = false;
  bool returnvalue = false;
  char *split_temp;
  char *split_cond;
  char *split_pres;
  char *split_flu;
  char *split_par;
  char *split_sal;
  char *split_date;
  float temp;
  char *mon;
  float sal;
  char charin;
  int filehandle;
  int i = 0, count = 0, byteswritten, month;
  struct tm info;
  time_t secs = 0;

  memset(stringin, 0, 128 * sizeof(char));

  while (count < 3 && i < 128) {
    charin = TURxGetByteWithTimeout(CTDPort, 250);
    if (charin == -1)
      count++;
    else {
      stringin[i] = charin;
      i++;
    }
  }

  // expect to see stringin start with " *# "
  if (strchr(stringin, '#') != NULL) {
    log = true;
  } else if (strchr(stringin, '>') != NULL)
    prompt = true;
  else if (strchr(stringin, '<') != NULL)
    prompt = true;

  if (prompt) {
    flogf("\nERROR|CTD_Data(): Received >s rather than #, sending into sleep "
          "mode.");
    CTD_SyncMode();
    Delay_AD_Log(3);
    TUTxPrintf(CTDPort, "TS\n");
    TUTxWaitCompletion(CTDPort);
    return returnvalue;
  }

  // error here if log!=true
  flogf("\n%s", stringin);

  // Split data string up into separate values
  // Example: # 20.6538,  0.01145,    0.217,   0.0622, 01 Aug 2016 12:16:50
  strtok(stringin, "# ");
  split_temp = strtok(NULL, " ");
  if (split_temp == NULL) {
    cprintf("\nNo Commas in CTD Data");
  }
  strtok(NULL, " "); // erase cond.
  split_cond = strtok(NULL, " "); // ignor cond
  split_pres = strtok(NULL, " ");
  split_flu = strtok(NULL, " ");
  split_par = strtok(NULL, " ");
  split_sal = strtok(NULL, " ");
  split_date = strtok(NULL, ",");

  temp = atof(strtok(split_temp, ","));

  LARA.DEPTH = atof(strtok(split_pres, ","));

  sal = atof(strtok(split_sal, ","));

  info.tm_mday = atoi(strtok(split_date, " "));
  info.tm_mon = -1;
  mon = strtok(NULL, " ");
  if (strstr(mon, "Jan") != NULL)
    info.tm_mon = 0;
  else if (strstr(mon, "Feb") != NULL)
    info.tm_mon = 1;
  else if (strstr(mon, "Mar") != NULL)
    info.tm_mon = 2;
  else if (strstr(mon, "Apr") != NULL)
    info.tm_mon = 3;
  else if (strstr(mon, "May") != NULL)
    info.tm_mon = 4;
  else if (strstr(mon, "Jun") != NULL)
    info.tm_mon = 5;
  else if (strstr(mon, "Jul") != NULL)
    info.tm_mon = 6;
  else if (strstr(mon, "Aug") != NULL)
    info.tm_mon = 7;
  else if (strstr(mon, "Sep") != NULL)
    info.tm_mon = 8;
  else if (strstr(mon, "Oct") != NULL)
    info.tm_mon = 9;
  else if (strstr(mon, "Nov") != NULL)
    info.tm_mon = 10;
  else if (strstr(mon, "Dec") != NULL)
    info.tm_mon = 11;

  if (info.tm_mon == -1) {
    flogf("\nERROR|CTD_Data(): Date incorrect. flush tuport ");
    TURxFlush(CTDPort);
    return returnvalue;
  }

  info.tm_year = (atoi(strtok(NULL, " ")) - 1900);
  info.tm_hour = atoi(strtok(NULL, ":"));
  info.tm_min = atoi(strtok(NULL, ":"));
  info.tm_sec = atoi(strtok(NULL, " "));

  month = info.tm_mon + 2;
  sprintf(split_date, ", %d.%d.%d %d:%d:%d", info.tm_mday, month,
          info.tm_year - 100, info.tm_hour, info.tm_min, info.tm_sec);

  secs = mktime(&info);

  // Log WriteString
  // error here (and above) if !log
  if (log) {
    memset(writestring, 0, 128 * sizeof(char));
    sprintf(writestring, "#%.4f,", temp);
    strcat(writestring, split_pres);
    LARA.DEPTH = atof(strtok(split_pres, ","));
    strcat(writestring, ","); // added 9.28 after deploy 2, data 1. 02UTC of
                              // 9.29
    strcat(writestring, split_sal);
    strcat(writestring, split_date);
    strcat(writestring, "\n");

    // this might be an error, done in VertVel
    LARA.CTDSAMPLES++;
    TURxFlush(CTDPort);
    filehandle = open(CTDLogFile, O_APPEND | O_CREAT | O_RDWR);
    if (filehandle <= 0) {
      flogf("\nERROR  |CTD_Logger() %s open errno: %d", CTDLogFile, errno);
      if (errno != 0) {
        // CTD_CreateFile(MPC.FILENUM);
        return true;
      }
    }

    flogf("\nLog: %s", writestring);
    // this should not be in log section
    if (LARA.BUOYMODE != 0)
      CTD_VertVel(secs);

    byteswritten = write(filehandle, writestring, strlen(writestring));
    // cprintf("\nBytes Written: %d", byteswritten);

    if (close(filehandle) != 0)
      flogf("\nERROR  |CTD_Logger: File Close error: %d", errno);

  } else {
    cprintf("\nNo \# found: %s", stringin);
    CTD_Start_Up(true);
    CTD_SyncMode();
  }
  return true;

} //____ CTD_Data() _____//
/******************************************************************************\
** void CTD_VertVel()
\******************************************************************************/
time_t CTD_VertVel(time_t seconds) {

  static ulong pastTime = 0;
  static float pastDepth = 0;
  float vel = 0.0;
  ulong timechange = 0;

  timechange = seconds - pastTime;

  if (timechange > 180 || pastTime == 0) {
    pastTime = seconds;
    pastDepth = LARA.DEPTH;
    DBG(flogf(
            "\n\t|CTD_VertVel: No previous recent measurements, reinitialize");)
    return 0;
  } else {
    pastTime = seconds;
    vel = (float)((LARA.DEPTH - pastDepth) / (float)timechange);
    SummedVelocity += vel;
    CTDSamples++;
    // flogf("\n\t|time change: %lu", timechange);
    flogf(", %4.2f m/s", vel);

    pastDepth = LARA.DEPTH;
  }

  return timechange;

} //____ CTD_VertVel() ____//
/******************************************************************************\
** void OpenTUPort_CTD(bool);
** !! calloc/free may not match, poor use of globals
\******************************************************************************/
void OpenTUPort_CTD(bool on) {
  // writestring, stringin
  short CTD_RX, CTD_TX;
  flogf("\n\t|%s CTD TUPort", on ? "Open" : "Close");
  if (on) {
    CTD_RX = TPUChanFromPin(32);
    CTD_TX = TPUChanFromPin(31);
    PIOSet(22);
    PIOClear(23);
    CTDPort = TUOpen(CTD_RX, CTD_TX, BAUD, 0);
    RTCDelayMicroSeconds(20000L);
    if (CTDPort == 0)
      flogf("\nBad TU Channel: CTDPort...");
    writestring = (char *)calloc(128, sizeof(char));
    stringin = (char *)calloc(128, sizeof(char));
  }
  if (!on) {

    free(writestring);
    free(stringin);

    PIOClear(22);
    TUClose(CTDPort);
  }
  return;

} //____ OpenTUPort_CTD() ____//
/******************************************************************************\
** void GetCTDSettings();
\******************************************************************************/
void GetCTDSettings() {
  char *p;

  p = VEEFetchData(CTDUPLOADFILE_NAME).str;
  CTD.UPLOAD = atoi(p ? p : CTDUPLOADFILE_DEFAULT);
  DBG(uprintf("CTD.UPLOAD=%u (%s)\n", CTD.UPLOAD, p ? "vee" : "def"); cdrain();)

} //____ GetCTDSettings() ____//
/******************************************************************************\
** float CTD_AverageDepth(int)
        -Usually Called while waiting for Winch Response.
        @Param1: int i, number of CTD Samples to average
        @Param2: float* to current average velocity
        @Return: Average Position from this sampling
\******************************************************************************/
float CTD_AverageDepth(int i, float *velocity) {
  int j;
  float depth[11];
  float depthchange;
  float vel;
  bool firstreading = true;
  float returnValue;
  ulong starttime = 0, stoptime = 0;

  if (tgetq(CTDPort))
    TURxFlush(CTDPort);
  // This for loop is to understand the profiling buoy's starting position prior
  // to ascending the water column.
  Delay_AD_Log(1);
  // TUTxPrintf(CTDPort, "\n");
  CTD_Sample();

  for (j = 0; j < i; NULL) {

    if (AD_Check()) {
      if (j == 0)
        CTD_Sample();
      j++;
      if (j == i) {
        *velocity = 0.0;
        return 0.0;
      }
    }
    if (tgetq(CTDPort)) {
      if (CTD_Data()) {
        if (firstreading)
          starttime = RTCGetTime(NULL, NULL);
        depth[j] = LARA.DEPTH;
        j++;
      }
      if (j == i) {
        stoptime = RTCGetTime(NULL, NULL);
        break;
      }
      CTD_Sample();
    }
  }
  // Very basic velocity calculation
  depthchange = depth[j - 1] - depth[0];
  flogf("\nDepthChange: %f over %d samples", depthchange, j - 1);
  vel = depthchange / ((float)j - 1.0);
  *velocity = vel;
  for (j = 1; j < i; j++)
    depth[0] = depth[0] + depth[j];
  depth[0] = depth[0] / (float)i;
  // if movement of more than 1 meter in duration of function.
  if (abs(depth[0] - depth[i - 1]) < 1.0) {
    flogf("\n\t|Profiling Buoy Stationary at %5.2f at %5.2fm/sample", depth[0],
          vel);
    returnValue = depth[0];
  } else {
    flogf("\n\t|Profiling Buoy Moving currently at %5.2f at %5.2fm/sample",
          depth[0], vel);
    returnValue = 0.0;
  }

  return returnValue;
}
/******************************************************************************\
** float CTD_CalculateVelocity()
        -Called at end of Ascent or Descent
        @Return: Calculated Velocity since previous calculation.
\******************************************************************************/
float CTD_CalculateVelocity() {
  float vel;
  vel = (SummedVelocity / (float)CTDSamples);
  // if(vel<0) vel=vel*-1.0;
  flogf("\nSummedVel: %5.2f, samples: %d", SummedVelocity, CTDSamples);
  // if(vel<=0.0) return 0.0;
  flogf("\n\t|CTD_CalculatedVelocity(): %3.2f", vel);
  SummedVelocity = 0.0;
  CTDSamples = 0;
  return vel;
}
/*+++++++++++++++++++++++++
===========================
END NEVER-ENDING WHILE LOOP
===========================
+++++++++++++++++++++++++*/
