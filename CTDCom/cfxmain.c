/******************************************************************************\
**	cfxmain.c				Persistor and PicoDOS starter C file  
**	
*****************************************************************************
**	
**	
*****************************************************************************
**	
**	
\******************************************************************************/

#include	<cfxbios.h>	// Persistor BIOS and I/O Definitions
#include	<cfxpico.h>	// Persistor PicoDOS Definitions

#include	<assert.h>
#include	<ctype.h>
#include	<errno.h>
#include	<float.h>
#include	<limits.h>
#include	<locale.h>
#include	<math.h>
#include	<setjmp.h>
#include	<signal.h>
#include	<stdarg.h>
#include	<stddef.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include	<time.h>

#include	<dirent.h>	// PicoDOS POSIX-like Directory Access Defines
#include	<dosdrive.h>	// PicoDOS DOS Drive and Directory Definitions
#include	<fcntl.h>	// PicoDOS POSIX-like File Access Definitions
#include	<stat.h>	// PicoDOS POSIX-like File Status Definitions
#include	<termios.h>	// PicoDOS POSIX-like Terminal I/O Definitions
#include	<unistd.h>	// PicoDOS POSIX-like UNIX Function Definitions
#include <PLATFORM.h>


#define NIGK_MIN_DEPTH 6

//if ACOUSTICMODEM defined

#define  AMODEMRX       33
#define  AMODEMTX       35
#define  AMODEMBAUD 		4800L

#define  BAUD        9600L

#define  WISPRONE       29
#define  WISPRTWO       30
#define  WISPRTHREE     24
#define  WISPRFOUR      25

#define WISPR_PWR_ON    37
#define WISPR_PWR_OFF   42


#define  STARTDEPTH_NAME         "DEPTHSTART"
#define  STARTDEPTH_DEFAULT      "40"	//meters
#define  STARTDEPTH_DESC         \
"Start Depth\n" \

#define  DEPTHINTERVAL_NAME       "DEPTHINT"
#define  DEPTHINTERVAL_DEFAULT    "25"	//cm/s
#define  DEPTHINTERVAL_DESC       \
"Change in depth\n" \

//ctd
bool CTD_Start_Up (bool settime);
bool CTD_GetPrompt ();
void CTD_DateTime ();
void CTD_Data (void);
void CTD_Send ();
void CTD_Sample ();
void Console (char);
void CTD_SampleBreak ();
void CTD_CreateFile (long);
void CTD_SyncMode ();
void OpenTUPort_CTD (bool);


bool AModem_Data (short *);
void AModem_Send (short, bool);

ulong Winch_Ascend ();
ulong Winch_Descend ();
ulong Winch_Stop ();
void Buoy_Status ();
void WinchConsole ();
void OpenTUPort_NIGK (bool);
void OpenTUPort_NIGK (bool);
void OpenTUPort_WISPR (bool on);
char *Time (ulong *);

#define  BAUD        9600L


bool SyncMode;
TUPort *PAMPort;
TUPort *CTDPort;
TUPort *NIGKPort;

float SummedVelocity;
short CTDSamples;

char CTDLogFile[] = "c:00000000.ctd";

//simulation variables: #ifdef LARASIM
float descentRate = 0.1923;	//m/s
float ascentRate = 0.26;

char time_chr[21];

int STARTDEPTH;
int DEPTHINT;
static float depth;
bool SystemOn;
short WinchMode;
static char *writestring;
static char *stringin;
static char *stringout;
/******************************************************************************\
**	main
\******************************************************************************/
void
main ()
{

  ulong secs = 0;
  ulong time = NULL;
  int count = 0;
  bool up = true;
  short reply = 0;		//0 = stop, 1 = ascend, 2 = 
  bool command = false;
  bool tr = false, trr = false;
  char *p;
  time_t nowsecs;
  char strbuf[64];
  struct tm t;
  ushort Ticks;


  Initflog ("activity.log", true);
  PZCacheSetup ('C' - 'A', calloc, free);

  CSSetSysAccessSpeeds (nsFlashStd, nsRAMStd, nsCFStd, WTMODE);
  TMGSetSpeed (SYSCLK);

  writestring = (char *) calloc (128, sizeof (char));
  stringin = (char *) calloc (32, sizeof (char));
  stringout = (char *) calloc (64, sizeof (char));

  WinchMode = 0;
  // Initialize and open TPU UART
  TUInit (calloc, free);

  nowsecs = RTCGetTime (NULL, &Ticks);	// get RTC clock right now
  t = *localtime (&nowsecs);
  strftime (strbuf, sizeof (strbuf), "%m/%d/%Y  %H:%M:%S", &t);

  flogf ("\nProgram start time: %s.%.3d [ctime: %lu]\n", strbuf, Ticks / 40,
	 nowsecs);

  //OpenTUPort_NIGK(true);
  OpenTUPort_CTD (true);
  OpenTUPort_WISPR (true);

  p = VEEFetchData (STARTDEPTH_NAME).str;
  STARTDEPTH = atoi (p ? p : STARTDEPTH_DEFAULT);

  p = VEEFetchData (DEPTHINTERVAL_NAME).str;
  DEPTHINT = atoi (p ? p : DEPTHINTERVAL_DEFAULT);

  SystemOn = true;
  depth = (float) STARTDEPTH;
  TUTxPrintf (CTDPort, "QS\n");

  while (SystemOn)
    {


      if (tgetq (CTDPort))
	CTD_Data ();
      if (tgetq (PAMPort))
	{
	  CTD_Send ();
	  TURxFlush (PAMPort);
	}
      if (cgetq ())
	{
	  Console (cgetc ());
	}

    }



}				//____ main() ____//

/******************************************************************************\
**	Time & Date String
** Get the RTC time seconds since 1970 and convert it to an
** understandable format
\******************************************************************************/
char *
Time (ulong * seconds)
{

  RTCtm *rtc_time;
  ulong secs = NULL;

  ushort ticks;

  RTCGetTime (&secs, &ticks);
  rtc_time = RTClocaltime (&secs);
  *seconds = secs;
  sprintf (time_chr, "%.2d/%.2d/%.4d %.2d:%.2d:%.2d",
	   rtc_time->tm_mon + 1, rtc_time->tm_mday, rtc_time->tm_year + 1900,
	   rtc_time->tm_hour, rtc_time->tm_min, rtc_time->tm_sec);
  return time_chr;

}				//____ TimeDate() ____//

/******************************************************************************\
** void Console(char);
\******************************************************************************/
void
Console (char in)
{

  char c;

  switch (in)
    {
    case 'w':
    case 'W':
      cprintf ("\nWinch Mode: Select [0-2]");
      c = cgetc ();
      if (c <= '2' && c >= '0')
	{
	  switch (c)
	    {
	    case '0':
	      WinchMode = 0;
	      break;
	    case '1':
	      WinchMode = 1;
	      break;
	    case '2':
	      WinchMode = 2;
	      break;
	    }
	  cprintf ("Winch Mode: %d", WinchMode);
	}
      else
	{
	  cprintf ("Bad call number %c", c);
	}
      break;
    case 'e':
    case 'E':
      SystemOn = false;
      break;

    }
}

/******************************************************************************\
** void OpenTUPort_CTD(bool);
\******************************************************************************/
void
OpenTUPort_CTD (bool on)
{

  short CTD_RX, CTD_TX;
  flogf ("\n\t|%s CTD TUPort", on ? "Open" : "Close");
  if (on)
    {
      CTD_RX = TPUChanFromPin (32);
      CTD_TX = TPUChanFromPin (31);
      PIOSet (22);
      PIOClear (23);
      CTDPort = TUOpen (CTD_RX, CTD_TX, BAUD, 0);
      RTCDelayMicroSeconds (20000L);
      if (CTDPort == 0)
	flogf ("\nBad TU Channel: CTDPort...");



    }
  if (!on)
    {



      PIOClear (22);
      TUClose (CTDPort);
    }
  return;

}				//____ OpenTUPort_CTD() ____//

/****************************************************************************\
** void OpenTUPort_NIGK(bool)
\****************************************************************************/
void
OpenTUPort_NIGK (bool on)
{
  static bool once = true;
  short AModem_RX, AModem_TX;
  flogf ("\n\t|%s NIGK Winch TUPort", on ? "Open" : "Close");

  if (on)
    {
      AModem_RX = TPUChanFromPin (AMODEMRX);
      AModem_TX = TPUChanFromPin (AMODEMTX);

      RTCDelayMicroSeconds (250000L);
      PIORead (48);
      //PIOSet(AMODEMPWR);    //Powers up the DC-DC for the Acoustic Modem Port
      NIGKPort = TUOpen (AModem_RX, AModem_TX, AMODEMBAUD, 0);
      if (NIGKPort == 0)
	flogf ("\n\t|Bad Winch TUPort\n");
      else
	{
	  TUTxFlush (NIGKPort);
	  TURxFlush (NIGKPort);
	  RTCDelayMicroSeconds (5000000L);	//Wait 5 seconds for NIGKPort to power up
	  TUTxPrintf (NIGKPort, "\n");
	  RTCDelayMicroSeconds (250000L);
	  TUTxFlush (NIGKPort);
	  TURxFlush (NIGKPort);

	}
    }
  else
    {
      RTCDelayMicroSeconds (500000L);
      TUClose (NIGKPort);
    }
  return;

}				//____ OpenTUPort_NIGK() ____//


/********************************************************************************\
** CTD_Sample()
\********************************************************************************/
void
CTD_SampleBreak ()
{

  TUTxBreak (CTDPort, 5000);

  SyncMode = false;

}				//____ CTD_Sample() ____//

/******************************************************************************\
** void CTD_data()
\******************************************************************************/
void
CTD_Data ()
{
  bool log = false;
  //char* stringin;
  //char* writestring;
  char *split_temp;
  char *split_pres;
  char *split_cond;
  char *split_SAL;
  float depthchange;
  char charin;
  static ulong prevTime = 0;
  ulong secs, timeChange;
  int i = 0, count = 0;


  memset (stringout, 0, 128 * sizeof (char));

  while (count < 2 && i < 128)
    {
      charin = TURxGetByteWithTimeout (CTDPort, 100);
      if (charin == -1)
	count++;
      else
	{
	  stringout[i] = charin;
	  i++;
	}
    }

  if (strchr (stringout, '#') == NULL)
    {
      cprintf ("\nFrom CTD: %s", stringout);
      TUTxPrintf (PAMPort, "%s\n", stringout);
      TUTxWaitCompletion (PAMPort);
      return;
    }
  memset (writestring, 0, 64 * sizeof (char));
  // Split data string up into separate values
  // Example: # 20.6538,  0.01145,    0.217,   0.0622, 01 Aug 2016 12:16:50
  split_temp = strtok (stringout, ",");
  split_cond = strtok (NULL, ",");


  if (WinchMode == 1)
    {				//going up.
      depthchange = (float) (DEPTHINT + ((rand () >> 12) - 8));
      flogf ("\ndepthChange: %f", depthchange);
      secs = RTCGetTime (NULL, NULL);
      timeChange = secs - prevTime;
      prevTime = secs;
      if (timeChange > 180)
	{
	  cprintf ("\nSeconds larger than 180: %lu", timeChange);
	}
      else
	{
	  depth -= depthchange / 100.0 * timeChange;
	  //prevTime=secs;
	  flogf ("\nsecs: %lu, New Depth: %f", timeChange, depth);
	}
    }
  else if (WinchMode == 2)
    {				//Going Down.
      depthchange = (float) (DEPTHINT + ((rand () >> 12) - 8));
      flogf ("\ndepthChange: %f", depthchange);
      secs = RTCGetTime (NULL, NULL);
      timeChange = secs - prevTime;
      prevTime = secs;
      if (timeChange > 180)
	cprintf ("\nSecs larger than 180: %lu", timeChange);
      else
	{
	  depth += depthchange / 100.0 * timeChange;
	  flogf ("\nsecs: %lu, New Depth: %f", timeChange, depth);
	}
    }


  //Pressure
  split_pres = strtok (NULL, ",");



  //Salinity
  split_SAL = strtok (NULL, "\n\r");

  sprintf (writestring, "#%s, %s,  %6.3f, %s\n", split_temp, split_cond,
	   depth, split_SAL);

  cprintf ("\nCTD to LARA:%s", writestring);
  TUTxPrintf (PAMPort, "%s\n", writestring);
  TUTxWaitCompletion (PAMPort);

}				//____ CTD_Data() _____// 

/************************************************************************************\
** void CTD_Send()
\************************************************************************************/
void
CTD_Send ()
{
  int i = 0, count = 0;
  char charin;
  //char* stringin;

  //stringin=(char*) calloc(32, sizeof(char));
  memset (stringin, 0, 32 * sizeof (char));
  cprintf ("\n%s|CTD_Send():", Time (NULL));
  while (count < 2 && i < 31)
    {
      charin = TURxGetByteWithTimeout (PAMPort, 50);
      cprintf ("%c", charin);
      if (charin != -1)
	{

	  stringin[i] = charin;
	  if (charin == '\n' || charin == '\r')
	    break;		//maybe return;?
	  i++;
	}
      else if (charin == -1)
	{
	  if (count > 0)
	    {
	      cprintf ("\nSending Break to CTD");
	      TUTxBreak (CTDPort, 4000);
	      return;
	    }
	  count++;
	}
    }
  TURxFlush (PAMPort);
  stringin[i + 1] = '\0';

  TUTxPrintf (CTDPort, "%s", stringin);
  TUTxWaitCompletion (CTDPort);
  cprintf ("\nTo CTD: %s", stringin);


}				//____ CTD_Send() ____//

/*************************************************************\
** void AModem_Data(void)
returns true if recieved a command (means AModem_Send will be a response)
points at reply 0: stop 1:ascend 2:descend 3: winch 4: buoy
\*************************************************************/
void
AModem_Send (short reply, bool command)
{


  switch (reply)
    {
    case 0:
      if (command)
	TUTxPrintf (NIGKPort, "#S,02,00\n");
      else
	TUTxPrintf (NIGKPort, "%%S,02,00\n");
      break;

    case 1:
      if (command)
	TUTxPrintf (NIGKPort, "#R,02,00\n");
      else
	TUTxPrintf (NIGKPort, "%%R,02,00\n");
      break;

    case 2:
      if (command)
	TUTxPrintf (NIGKPort, "#F,02,00\n");
      else
	TUTxPrintf (NIGKPort, "%%F,02,00\n");
      break;

    case 3:
      if (command)
	TUTxPrintf (NIGKPort, "#W,02,00\n");
      else
	TUTxPrintf (NIGKPort, "%%W,02,00\n");
      break;

    case 4:
      if (command)
	TUTxPrintf (NIGKPort, "#B,02,00\n");
      else
	TUTxPrintf (NIGKPort, "%%B,02,00\n");
      break;

    }
  TUTxWaitCompletion (NIGKPort);

}				//____ AModem_Send() ____//

/*************************************************************\
** void Buoy(void)
** A function call to the Buoy to get the status of equipment. 
** The return call through NIGKPort sends 2 bytes of data as the 
** response to the Buoy CommandWinch from the Deck Unit
\*************************************************************/
void
Buoy_Status (void)
{
  char B_Status[3] = "00";	//Base return call

  flogf ("\n%s|Buoy Status:", Time (NULL));


  TUTxPrintf (NIGKPort, "%%B,01,%c%c\n", B_Status[0], B_Status[1]);	//Send rest of status via acoustic remote to deck unit
  TUTxWaitCompletion (NIGKPort);
  RTCDelayMicroSeconds (5000000);	//Make sure all the Buoy Calls have been received....
  TURxFlush (NIGKPort);		//Before clearing the NIGKPort Rx


}				//____ Buoy() ____//

/****************************************************************************************************************\
** void OpenTUPort_WISPR()
\****************************************************************************************************************/
void
OpenTUPort_WISPR (bool on)
{
  short PAM_RX, PAM_TX;

  flogf ("\n\t|%s PAM TUPort", on ? "Open" : "Close");
  if (on)
    {
      PAM_RX = TPUChanFromPin (28);
      PAM_TX = TPUChanFromPin (27);
      PAMPort = TUOpen (PAM_RX, PAM_TX, BAUD, 0);
    }
  else if (!on)
    {
      TUTxFlush (PAMPort);
      TURxFlush (PAMPort);
      TUClose (PAMPort);
      RTCDelayMicroSeconds (1000000L);
    }

  if (on)
    {
      PIOSet (WISPRONE);
      PIOClear (WISPRTWO);
    }
  else
    {
      PIOClear (WISPRONE);
      PIOClear (WISPRTWO);
    }





}
