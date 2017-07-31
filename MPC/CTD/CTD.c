/** handling CTD in buoy, seabird SBE 16plus
 * general note: ctd wants \r only for input
 * Bug:! TUTxPrint translates \n into \r\n, which sorta kinda works if lucky
 */
#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions
#include <MPC_Global.h>
#include <PLATFORM.h>
#include <CTD.h>
#include <ADS.h>
#include <Settings.h>
#include <Winch.h>
#include <GPSIRID.h>

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

#define STRING_SIZE 1024

time_t CTD_VertVel(time_t);
int Sea_Ice_Algorithm(); 

extern SystemParameters MPC;
extern SystemStatus LARA;
CTDParameters CTD;
bool SyncMode;
extern TUPort *devicePort; // GPSIRID
int sbeID=DEVB;  // DEVA:antenna DEVB:buoy
float SummedVelocity;
short CTDSamples;
char CTDLogFile[] = "c:00000000.ctd";
static char *stringin;
static char *stringout;

// simulation variables: #ifdef LARASIM
float descentRate = 0.1923; // m/s
float ascentRate = 0.26;

/* 
 * CTD_Init() initialize file
 */
int CTD_Init() {
  // global char *stringin, *stringout;  // used by CTD.c
  stringin = (char *)calloc(STRING_SIZE, 1);
  stringout = (char *)calloc(STRING_SIZE, 1);
  return 0;
}

/*****************************************************************************\
** CTD_Start_Up()
* open port if needed, send break, get prompt, flush; opt settime
\*****************************************************************************/
bool CTD_Start_Up(int sbe, bool settime) {
  // global int sbeID;
  bool returnval = false;
  DBG( flogf("\n\t|. CTD_Start_Up(%d, %d) ", sbe, settime); )

  sbeID=sbe;
  DevSelect(sbe);

  // CTD_CreateFile(sbe, MPC.FILENUM);  // called from lara.c
  // leave sync mode
  CTD_SampleBreak();
  if (CTD_GetPrompt()) 
    cprintf(" successful startup");
  else {
    CTD_SampleBreak();
    Delay_AD_Log(1);
    if (CTD_GetPrompt())
      cprintf(" successful startup1");
    else {
      DevSelect(DEVX);
      Delay_AD_Log(1);
      DevSelect(sbe);
      CTD_SampleBreak();
      Delay_AD_Log(1);
      if (CTD_GetPrompt()) 
        cprintf(" successful startup2");
    } 
  }
  if (settime)
    CTD_DateTime();
  CTD_SyncMode();
  TURxFlush(devicePort);
  TUTxFlush(devicePort);
  return true;
} //_____ CTD_Start_Up() _____//

/******************************************************************************\
** CTD_CreateFile()
\******************************************************************************/
void CTD_CreateFile(long filenum) {
  int filehandle;
  sprintf(&CTDLogFile[2], "%08ld.ctd", filenum);
  filehandle = open(CTDLogFile, O_RDWR | O_CREAT | O_TRUNC);
  Delayms(50);
  if (filehandle < 0) {
    flogf("\nERROR  |CTD_CreateFile() errno: %d", errno);
  }
  if (close(filehandle) < 0)
    flogf("\nERROR  |CTD_CreateFile(): %s close errno: %d", CTDLogFile, errno);
  Delayms(10);
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
  sprintf(buffer, "%02d%02d%04d%02d%02d%02d", info->tm_mon+1, info->tm_mday,
          info->tm_year + 1900, info->tm_hour, info->tm_min, info->tm_sec);
  printf("\nCTD_DateTime(): %s\n");

  TUTxPrintf(devicePort, "DATETIME=%s\r", buffer);
  Delayms(250);
  while (tgetq(devicePort))
    cprintf("%c", TURxGetByte(devicePort, true));

} //_____ CTD_DateTime() ____//

/********************************************************************************\
** CTD_GetPrompt()
\********************************************************************************/
bool CTD_GetPrompt() {
  // global char *stringin;
  short count = 0;
  short LastByteInQ;

  memset(stringin, 0, STRING_SIZE);

  LastByteInQ = TURxPeekByte(devicePort, (tgetq(devicePort) - 1));
  while (((char)LastByteInQ != '>') && count < 2) { // until a > is read in
                                                    // command line for devicePort
                                                    // or 7 seconds pass
    TUTxPrintf(devicePort, "\r");
    Delayms(1000);
    LastByteInQ = TURxPeekByte(devicePort, (tgetq(devicePort) - 1));
    count++;
  }

  if (count == 2) {
    TURxGetBlock(devicePort, stringin, (long) STRING_SIZE, (short) 1000);
    if (strstr(stringin, "S>") != NULL) {
      cprintf("\nPrompt from CTDBlock");
      TURxFlush(devicePort);
      return true;
    }
    return false;
  }

  else {
    cprintf("\nPrompt from CTD");
    TURxFlush(devicePort);
    return true;
  }
}

/********************************************************************************\
** CTD_Sample()
\********************************************************************************/
void CTD_Sample() {
  DBG(cputc('+');)
  DBG2( flogf("\n . CTD_Sample"); )
  if (SyncMode) {
    TUTxPrintf(devicePort, "+\r");
  } else {
    TUTxPrintf(devicePort, "TS\r");
  }
  Delayms(250);
} //____ CTD_Sample() ____//

/********************************************************************************\
** CTD_SampleSleep()
\********************************************************************************/
void CTD_SyncMode() {
  DBG(flogf("\n\t|CTD_SyncMode()");)
  TUTxPrintf(devicePort, "Syncmode=y\r");
  TUTxWaitCompletion(devicePort);
  Delayms(500);
  TUTxPrintf(devicePort, "QS\r");
  TUTxWaitCompletion(devicePort);
  Delayms(1000);
  SyncMode = true;
  TURxFlush(devicePort);
} //____ CTD_Sample() ____//

/********************************************************************************\
** CTD_Sample()
\********************************************************************************/
void CTD_SampleBreak() {
  TUTxBreak(devicePort, 5000);
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
// Returns reason: 
// 1: Stop, should be at surface and ready to send.
// 2: Stop due to low Temp or other reason
// 3: Ascend2, careful ascent
// 4: Descend, definitely ice
// 5: Stop due to little change in salinity
// CTDParameters CTD;
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
            CommandWinch("\#S,01,00", true);   Delayms(2000);
  //try get command to get reult back? so no delay needed
            if(temp<=Temp_Threshold){ CommandWinch("\#F,01,00", true);
               Delayms(2000);       cprintf("\nSIA: Minimum
  Temperature Recognized ");
               ice=4; }
            else if(temp>Temp_Threshold&&BuoyMode==0){
  cprintf("\nSIA: Careful Ascent ");
               ice=3;              CommandWinch("\#R,01,02", true);
  Delayms(2000);     }
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
  Delayms(2000);
            CommandWinch("\#F,01,00", true);  return ice;   }
         if(current_temp<Temp_Threshold){   cprintf("\nSIA: Temp: Too Low ");
  ice=2;
            CommandWinch("\#S,01,00", true);  Delayms(2000);
            CommandWinch("\#F,01,00", true);  return ice;   }
         if(PRES<=Min_Depth){   CommandWinch("\#S,01,00", true);
            Delayms(2000);      cprintf("\nSIA: Reached Minimum
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
* !! too many functions here
* !! sets lara.depth lara.temp
* !! CTD_vertvel
* !! log scientific
* waits up to 8 seconds - best called after tgetq()
\******************************************************************************/
bool CTD_Data() {
  // global stringin CTDLogFile
  char *strin;  // pointer into stringin
  int filehandle;
  int i = 0, byteswritten, month;
  long len;
  char *split_temp, *split_cond, *split_pres;
  char *split_flu, *split_par, *split_sal;
  float temp, cond, pres, flu, par, sal;
  char *split_date, *mon;
  struct tm info;
  time_t secs = 0;

  DBG2( flogf("\n. CTD_Data()"); )
  memset(stringin, 0, STRING_SIZE);

  // waits up to 8 seconds - best called after tgetq()
  len = GetStringWait(stringin, 8000);
  DBG2( flogf("\n%s", stringin);)

  // expect to see stringin start with ".*# "
  // do better sanity checking
  strin = strchr(stringin, '#');
  if (strin == NULL) {
    // no data #
    if ((strchr(stringin, '>') != NULL)
        || (strchr(stringin, '<') != NULL)) {
      // prompt
      flogf("\nERROR|CTD_Data(): got <|> want #, set sync mode");
      CTD_SyncMode();
    } else {
      // no < > #, don't know what
      flogf("\nERROR|CTD_Data(): No prompt found, reset ctd");
      CTD_Start_Up(sbeID, true);
      CTD_SyncMode();
    }
    return false;
  } // no #

  // Split data string up into separate values
  if (sbeID==DEVB) { // buoy sbe
    // Example: # 20.6538,  0.01145,    0.217,   0.0622, 01 Aug 2016 12:16:50
    split_temp = strtok(strin, "#, ");
    split_cond = strtok(NULL, ", "); 
    split_pres = strtok(NULL, ", ");
    split_flu = strtok(NULL, ", ");
    split_par = strtok(NULL, ", ");
    split_sal = strtok(NULL, ", ");
    split_date = strtok(NULL, "\r\n"); 

    temp = atof(split_temp);
    cond = atof(split_cond);
    pres = atof(split_pres);
    flu = atof(split_flu);
    par = atof(split_par);
    sal = atof(split_sal);
  } else { // antMod sbe
    // Example: # 20.6538,  0.217,   01 Aug 2016 12:16:50
    split_temp = strtok(strin, "#, ");
    split_pres = strtok(NULL, ", ");
    split_date = strtok(NULL, "\r\n"); 

    temp = atof(split_temp);
    pres = atof(split_pres);
  }
  // convert date time to secs
  info.tm_mday = atoi(strtok(split_date, " "));
  mon = strtok(NULL, " ");
  info.tm_year = (atoi(strtok(NULL, " ")) - 1900);
  info.tm_hour = atoi(strtok(NULL, ":"));
  info.tm_min = atoi(strtok(NULL, ":"));
  info.tm_sec = atoi(strtok(NULL, " "));

  info.tm_mon = -1;
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

  // need better sanity checks
  if (info.tm_mon == -1) {
    flogf("\nERROR|CTD_Data(): month %s incorrect. flush tuport ", mon);
    flogf("\nERROR|CTD_Data(): %s ", stringin );
    flogf("\nERROR|CTD_Data(): %f, %f, %f, %f, %f, %f, %d, %s, %d ", 
      temp, cond, pres, flu, par, sal, info.tm_mday, mon, info.tm_year);
    TURxFlush(devicePort);
    return false;
  }

  month = info.tm_mon + 2;
  sprintf(split_date, ", %d.%d.%d %d:%d:%d", info.tm_mday, month,
          info.tm_year - 100, info.tm_hour, info.tm_min, info.tm_sec);

  secs = mktime(&info);

  if (sbeID==DEVB) { // buoy sbe
    // Log WriteString
    memset(stringout, 0, STRING_SIZE);
    sprintf(stringout, "#%.4f,", temp);
    strcat(stringout, split_pres);
    strcat(stringout, ","); // added 9.28 after deploy 2, data 1. 02UTC of
    strcat(stringout, split_sal);
    strcat(stringout, split_date);
    strcat(stringout, "\n");

    TURxFlush(devicePort);
    filehandle = open(CTDLogFile, O_APPEND | O_CREAT | O_RDWR);
    if (filehandle <= 0) {
      flogf("\nERROR  |ctdlogfile '%s' fd %d", CTDLogFile, filehandle);
      flogf("\nERROR  |CTD_Logger() %s open errno: %d", CTDLogFile, errno);
      flogf("\nLog: %s", stringout);
      return false;
    }
    byteswritten = write(filehandle, stringout, strlen(stringout));
    DBG2( cprintf("\nBytes Written: %d", byteswritten);)
    if (close(filehandle) != 0) {
      flogf("\nERROR  |CTD_Logger: File Close error: %d", errno);
      return false;
    }
  } 
  LARA.DEPTH = pres;
  // LARA.TEMP = temp;
  // this incr looks strange, but lara.ctd is not ctdsamples, not part of averaging
  LARA.CTDSAMPLES++;
  if (LARA.BUOYMODE != 0) CTD_VertVel(secs);
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
    DBG(flogf("\n\t|CTD_VertVel: No previous recent measurements, reinitialize");)
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
** void CTD_GetSettings();
\******************************************************************************/
void CTD_GetSettings() {
  char *p;

  p = VEEFetchData(CTDUPLOADFILE_NAME).str;
  CTD.UPLOAD = atoi(p ? p : CTDUPLOADFILE_DEFAULT);
  DBG(uprintf("CTD.UPLOAD=%u (%s)\n", CTD.UPLOAD, p ? "vee" : "def"); cdrain();)

} //____ CTD_GetSettings() ____//

/******************************************************************************\
** float CTD_AverageDepth(int)
        -Usually Called while waiting for Winch Response.
        @Param1: int i, number of CTD Samples to average
        @Param2: float* to current average velocity
        @Return: Average Position from this sampling
\******************************************************************************/
float CTD_AverageDepth(int i, float *velocity) {
  int j;
  float first, last, diff, vel;
  ulong dur, starttime = 0, stoptime = 0;

  DBG1( flogf("\n . CTD_AverageDepth"); )
  CTD_Start_Up(DEVA, false);
  TURxFlush(devicePort);

  for (j = 0; j < i; j++) {
    CTD_Sample();
    if (!CTD_Data()) return 0.0; // waits up to 8 sec
    if (j==0) {
      starttime = RTCGetTime(NULL, NULL);
      first = LARA.DEPTH;
    }
  } // for j
  stoptime = RTCGetTime(NULL, NULL);
  last = LARA.DEPTH;
  diff = first - last;
  // Very basic velocity calculation
  flogf("\nDepthChange: %5.2f over %d samples", diff, i);
  dur = stoptime-starttime;
  vel = diff / (float) dur;
  *velocity = vel;
  // if movement of more than 1 meter in 6s
  if (dur<6) 
    flogf("\n\t|not enough sample time to profile buoy movement");
  else
    if (abs(diff) < 1.0) {
      flogf("\n\t|Profiling Buoy Stationary at %5.2f, %5.2fm/s", last, vel);
      LARA.BUOYMODE=0;
    } else {
      flogf("\n\t|Profiling Buoy Moving currently at %5.2f, %5.2fm/s", last, vel);
      if (vel>0) LARA.BUOYMODE=1;
      else LARA.BUOYMODE=2;
    }
  return last;
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
