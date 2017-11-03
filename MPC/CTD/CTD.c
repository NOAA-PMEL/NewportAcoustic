/* handling CTD in buoy, seabird SBE 16plus
 * general note: ctd wants \r only for input
 * Bug:! TUTxPrint translates \n into \r\n, which sorta kinda works if lucky
 */
#include <common.h>
#include <CTD.h>

extern SystemParameters MPC;
extern SystemStatus LARA;
struct CTDParameters{
  bool syncMode;
  short delay;          //Delay in seconds between polled samples
  TUPort port;
} ctd;


/* 
 */
bool CTD_Init() {
  DBG0( flogf("\n\t|. CTD_Init"); )

  CTD_SampleBreak();
  if (!CTD_GetPrompt(stringin)) {
    DBG2(flogf(" fail start getprompt");)
    CTD_SampleBreak();
    DevSelect(sbe);
    Delay_AD_Log(3);
    if (!CTD_GetPrompt(stringin)) {
      DBG2(flogf(" fail start1 getprompt");)
      if (sbe==DEVA) DevSelect(DEVX);
      Delay_AD_Log(1);
      DevSelect(sbe);
      CTD_SampleBreak();
      Delay_AD_Log(3);
      if (!CTD_GetPrompt(stringin)) {
        DBG2(flogf(" fail start2 getprompt");)
      }
    } 
  }
  CTD_DateTime();
  // CTD_SyncMode();
  return true;
} // CTD_Init



/*
 */
void CTD_DateTime() {

  time_t rawtime;
  struct tm *info;
  char buffer[15];
  DBG1(flogf("\n\t|CTD_DateTime()");)
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

} // CTD_DateTime() 

/*
 * CTD_GetPrompt - poke buoy CTD, look for prompt
 */
bool CTD_GetPrompt() {
  static char str[32];
  bool r = false;
  // looking for S>
  TURxFlush(ctd.port);
  TUTxPutByte(ctd.port, '\r', true);
  GetStringWait(ctd.port, 6, str);
  if (strstr(str, ">") != NULL) r=true;
  return r;
}

/*
 */
void CTD_Sample() {
  char ch;
  DBG1( flogf("\n . CTD_Sample"); )
  //if (SyncMode) {
    //TUTxPrintf(devicePort, "+\r");
    //Delayms(20);
    //TURxFlush(devicePort);
  //} else {
  // TUTxPrintf(devicePort, "TS\r");
  TUTxPutByte(devicePort, 'T', true);
  ch=TURxGetByteWithTimeout(devicePort, 10);
  TUTxPutByte(devicePort, 'S', true);
  ch=TURxGetByteWithTimeout(devicePort, 10);
  TUTxPutByte(devicePort, '\r', true);
  ch=TURxGetByteWithTimeout(devicePort, 10);
  ch=TURxGetByteWithTimeout(devicePort, 10);
  //}
} //____ CTD_Sample() ____//

/*
 */
void CTD_SyncMode() {
  DBG0(flogf("\n\t|CTD_SyncMode()");)
  // CTD_SampleBreak();
  TUTxPrintf(devicePort, "Syncmode=y\r");
  Delayms(500);
  TUTxPrintf(devicePort, "QS\r");
  Delayms(500);
  SyncMode = true;
  TURxFlush(devicePort);
} 

/*
 */
void CTD_SampleBreak() {
  TUTxBreak(devicePort, 5000);
  SyncMode = false;
}


/*
 * bool CTD_Data() 
 * Temp, conductivity, depth, fluromtr, PAR, salinity, time
 * 16.7301,  0.00832,    0.243, 0.0098, 0.0106,   0.0495, 14 May 2017 23:18:20
 * The response is approximately 2 sec.
 * FLS and PAR data in between the depth and salinity. Ignore the conductivity.
 * !! too many functions here
 * !! sets lara.depth lara.temp
 * !! CTD_vertvel
 * !! log scientific
 * waits up to 8 seconds - best called after tgetq()
 */
bool CTD_Data() {
  // global stringin CTDLogFile
  char *strin;  // pointer into stringin
  int filehandle;
  int i=0;
  int byteswritten, month;
  long len;
  char *split_temp, *split_cond, *split_pres;
  char *split_flu, *split_par, *split_sal;
  float temp, cond, pres, flu, par, sal;
  char *split_date, *mon;
  struct tm info;
  time_t secs = 0;

  DBG0( flogf("\n. CTD_Data()"); )
  memset(stringin, 0, BUFSZ);

  // waits up to 8 seconds - best called after tgetq()
  len = GetStringWait(stringin, (short) 8000);
  DBG2( printsafe(len, stringin);)
#ifdef DEBUG3
  printsafe(len, stringin);
#endif

  TURxFlush(devicePort);
  strin=stringin;

  memset(stringout, 0, BUFSZ);
  // Split data string up into separate values
  if (sbeID==DEVB) { // buoy sbe
    // Example: # 20.6538,  0.01145,    0.217,   0.0622, 01 Aug 2016 12:16:50
    // Example: 20.6538,  0.01145,    01 Aug 2016, 12:16:50
    split_temp = strtok(strin, "\r\n# ");
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
    sprintf(stringout, "%.4f,%.5f,%.3f,%.4f,%.4f,%.4f", 
      temp, cond, pres, flu, par, sal);
  } else { // antMod sbe
    // Example: # 20.6538,  0.217,   01 Aug 2016 12:16:50
    split_temp = strtok(strin, "\r\n#, ");
    split_pres = strtok(NULL, ", ");
    split_date = strtok(NULL, "\r\n"); 

    temp = atof(split_temp);
    pres = atof(split_pres);
    sprintf(stringout, "%.4f,%.3f", temp, pres);
  }
  LARA.DEPTH = pres;
  // LARA.TEMP = temp; //??
  DBG1(flogf("\nctd->%s", stringout);)

  // buoy sbe vvvvv
  // convert date time to secs
  info.tm_mday = atoi(strtok(split_date, ", "));
  mon = strtok(NULL, " ");
  info.tm_year = (atoi(strtok(NULL, " ,")) - 1900);
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
    flogf("\nERROR|CTD_Data(): month %s incorrect. ", mon);
    flogf("\n\t|''%s''", stringin);
    return false;
  } 
  else month = info.tm_mon + 2;

  memset(stringin, 0, 32);
  sprintf(stringin, ",%d/%d/%d,%02d:%02d:%02d", month, info.tm_mday,
          info.tm_year - 100, info.tm_hour, info.tm_min, info.tm_sec);
  strcat( stringout, stringin );
  DBG1(flogf("%s", stringin);)

  secs = mktime(&info);

  // if antMod DEVA, we are done, no log
  if (sbeID==DEVA) { return true; }

  // Log WriteString
  filehandle = open(CTDLogFile, O_APPEND | O_CREAT | O_RDWR);
  if (filehandle <= 0) {
    flogf("\nERROR  |ctdlogfile '%s' fd %d", CTDLogFile, filehandle);
    flogf("\nERROR  |CTD_Logger() %s open errno: %d", CTDLogFile, errno);
    return false;
  }
  byteswritten = write(filehandle, stringout, strlen(stringout));
  DBG2( flogf("\nBytes Written: %d", byteswritten);)
  if (close(filehandle) != 0) {
    flogf("\nERROR  |CTD_Logger: File Close error: %d", errno);
    return false;
  }
  // this incr looks strange, but lara.ctd is not ctdsamples, not part of averaging
  LARA.CTDSAMPLES++;
  if (LARA.BUOYMODE != 0) CTD_VertVel(secs); // ??
  return true;
} //____ CTD_Data() _____//

