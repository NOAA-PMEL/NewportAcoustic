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

#define SYSCLK    1920

void UpdateTime ();
void UpdateParameters ();
void DelaySeconds (int);
void DownloadFile (int);
void Console (char);

TUPort *SeaGlider;


short DETMAX, DETINT, PWRONDEPTH, PWROFFDEP, WGAIN, MINVOLT;
float MaxDepth;

/******************************************************************************\
**	main
\******************************************************************************/
void
main ()
{

  float depth;
  short divenum = 0;
  short SeaGlider_RX, SeaGlider_TX;
  //  TUChParams *TUCP;
  bool poweron = true;



  DETMAX = 0;
  DETINT = 0;
  PWRONDEPTH = 0;
  PWROFFDEP = 0;
  MINVOLT = 0;
  WGAIN = 0;
  MaxDepth = 999.9;
  Initflog ("activity.log", true);
  TMGSetSpeed (SYSCLK);
  // Initialize and open TPU UART
  TUInit (calloc, free);

  SeaGlider_RX = TPUChanFromPin (32);
  SeaGlider_TX = TPUChanFromPin (31);

  PIOSet (22);
  PIOClear (23);


  SeaGlider = TUOpen (SeaGlider_RX, SeaGlider_TX, 4800, 0);
  if (SeaGlider == 0)
    flogf ("\nBad TU Channel: SG...");
//      else    
  //    TUCP =TUGetCurrentParams(SeaGlider, TUCP );


  depth = 0.0;
  flogf ("\nEnter Gain:");
  Console ('N');

  flogf ("\nEnter DETINT:");
  Console ('d');

  flogf ("\nEnter DETMAX:");
  Console ('D');

  flogf ("\nEnter PWRONDEPTH:");
  Console ('I');

  flogf ("\nEnter PWROFFDEP:");
  Console ('E');

  flogf ("\nMax Depth of this Dive:");
  Console ('M');


  while (poweron)
    {
      divenum++;
      UpdateTime ();		//"%r+%3+%3+%3++GPS=%{%m/%d/%Y,%H:%M:%S***}%r%r"

      DelaySeconds (12);

      UpdateParameters ();

      for (depth = 0.0; depth < MaxDepth; depth = depth + 0.52)
	{
	  TUTxPrintf (SeaGlider, "+++%.2f,%d***", depth, divenum);
	  flogf ("\ndepth=%.2f", depth);
	  DelaySeconds (5);
	  if (cgetq ())
	    {
	      Console (cgetc ());
	    }
	}
      for (depth = MaxDepth; depth >= 1.0; depth = depth - 0.49)
	{
	  TUTxPrintf (SeaGlider, "+++%.2f,%d***", depth, divenum);
	  flogf ("\ndepth=%.2f", depth);
	  DelaySeconds (5);
	  if (cgetq ())
	    {
	      Console (cgetc ());
	    }
	}

      DelaySeconds (30);

      DownloadFile (divenum);



    }


}

/******************************************************************************\
**	UpdateTime()
\******************************************************************************/
void
UpdateTime ()
{

  RTCtm *rtc_time;
  ulong secs = NULL;


  ushort ticks;

  RTCGetTime (&secs, &ticks);
  rtc_time = RTClocaltime (&secs);

  //sprintf(time_chr,"%.2d/%.2d/%.4d %.2d:%.2d:%.2d",
  //rtc_time->tm_mon + 1, rtc_time->tm_mday, rtc_time->tm_year + 1900,\
  //rtc_time->tm_hour,rtc_time->tm_min,rtc_time->tm_sec);
  TUTxPrintf (SeaGlider, "++GPS=%.2d/%.2d/%.4d,%.2d:%.2d:%.2d**",
	      rtc_time->tm_mon + 1, rtc_time->tm_mday,
	      rtc_time->tm_year + 1900, rtc_time->tm_hour, rtc_time->tm_min,
	      rtc_time->tm_sec);

  flogf ("\nNew Time Set:%.2d/%.2d/%.4d,%.2d:%.2d:%.2d**",
	 rtc_time->tm_mon + 1, rtc_time->tm_mday, rtc_time->tm_year + 1900,
	 rtc_time->tm_hour, rtc_time->tm_min, rtc_time->tm_sec);

}

/***************
** UpdateParameters()
*************/
void
UpdateParameters ()
{


  TUTxPrintf (SeaGlider, "+++PAM(G%dv11.0d%02dD%02dI%03dE%03d)***", WGAIN,
	      DETMAX, DETINT, PWRONDEPTH, PWROFFDEP);
  flogf ("\nUpdated Paremeters to: G1 v11.0 d1 D30 I025 E025");

}

/***
** DelaySeconds
***/
void
DelaySeconds (int seconds)
{

  RTCDelayMicroSeconds (seconds * 1000000);
}

/***
** DownloadFile();
***/
void
DownloadFile (int divenum)
{
  FILE *divefile;
  uchar *buffer;
  int elements;
  //  char* fname;
  char fname[sizeof "0000.dat"];

  buffer = (uchar *) malloc (1024);
  sprintf (fname, "%04d.dat", divenum);


  divefile = fopen (fname, "w");

  TUTxPrintf (SeaGlider, "+++cat,%d***", divenum);
  DelaySeconds (5);
  TUTxPrintf (SeaGlider, "+++cat,%d***", divenum);
  DelaySeconds (1);

  while (tgetq (SeaGlider))
    {
      if (tgetq (SeaGlider) > 1024)
	{
	  elements = 1024;
	  memset (buffer, 0, elements);
	  TURxGetBlock (SeaGlider, buffer, elements, 1000);
	  fwrite (buffer, 1, elements, divefile);
	  flogf ("\nWriting %d bytes to %s", elements, fname);
	}
      else
	{
	  elements = tgetq (SeaGlider);
	  memset (buffer, 0, 1024);
	  TURxGetBlock (SeaGlider, buffer, elements, 1000);
	  fwrite (buffer, 1, elements, divefile);
	  flogf ("\nWriting %d bytes to %s", elements, fname);
	}

    }
  fclose (divefile);
  free (buffer);

}

/************************************************************************************************************************\
** void Console      Testing Opportunity for Debugging
\************************************************************************************************************************/
void
Console (char in)
{

  short first;
  short second;
  short third;
  short gain;
  char *returnnum = "00";
  short detections;


  flogf ("\n%c ", in);

  if (in == 'E')
    {
      first = cgetc ();
      second = cgetc ();
      third = cgetc ();
      detections = (first - 48) * (second - 48) * (third - 48);
      PWROFFDEP = detections;
      flogf (" %d", PWROFFDEP);
    }
  else if (in == 'I')
    {
      first = cgetc ();
      second = cgetc ();
      third = cgetc ();
      detections = (first - 48) * (second - 48) * (third - 48);
      PWRONDEPTH = detections;
      flogf (" %d", PWRONDEPTH);
    }
  else if (in == 'N')
    {
      gain = cgetc ();
      WGAIN = gain - 48;
      flogf (" %d", WGAIN);
    }
  //'D' followed by two numbers to be multiplied to get return detections.
  else if (in == 'D')
    {
      first = cgetc ();
      second = cgetc ();
      detections = (first - 48) * (second - 48);

      DETMAX = detections;
      flogf (" %d", DETMAX);
    }
  else if (in == 'd')
    {

      first = cgetc ();
      second = cgetc ();
      detections = (first - 48) * (second - 48);
      DETINT = detections;
      flogf (" %d", DETINT);
    }
  else if (in == 'M')
    {
      first = cgetc ();
      second = cgetc ();
      third = cgetc ();
      flogf (" %c%c%c", first, second, third);
      first = first - 48;
      second = second - 48;
      third = third - 48;
      MaxDepth = (first * 100) + (second * 10) + third;
    }



}
