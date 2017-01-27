/******************************************************************************\
**	CSACComTest.c				Persistor and PicoDOS
starter C file
**
** Synchronize the CF2 RTC to CSAC clock
** There is 2ms delay due to CF2 CPU response time.  As a result CF2 RTC time
** is 2ms behind CSAC.
*****************************************************************************
**
**
*****************************************************************************
**
**
\******************************************************************************/

#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions

#include <assert.h>
#include <ctype.h>
#include <errno.h>
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

#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <fcntl.h>    // PicoDOS POSIX-like File Access Definitions
#include <stat.h>     // PicoDOS POSIX-like File Status Definitions
#include <termios.h>  // PicoDOS POSIX-like Terminal I/O Definitions
#include <unistd.h>   // PicoDOS POSIX-like UNIX Function Definitions

ulong TimingRuptCount;
bool TimeExpired = false;
ulong SecondsSince1970;

// define the TPU pins for CSAC COM
#define RX 33
#define TX 34
#define CF2CSAC 23
#define PCCSAC 22
#define PPSIN 24
#define PPSENAB 25
#define RxTIMEOUT 999 // milliseconds
#define TICKS_TO_MS(x) ((ushort)(x / (CFX_CLOCKS_PER_SEC / 1000)))
bool ProgramFinished = false;

PinIODefineTPU(RX, 12);     // Pin 33
PinIODefineTPU(TX, 13);     // Pin 34
PinIODefineTPU(CF2CSAC, 2); // Pin 23
PinIODefineTPU(PCCSAC, 1);  // Pin 22
PinIODefineTPU(PPSIN, 3);   // Pin 24
PinIODefineTPU(PPSENAB, 4); // Pin 25

void TUSimpleTerm(void);
void Delay(short centisecs);
bool GetSyncRTCbyCSAC(void);
bool CheckRTC_CSACTime(void);
void LowPowerDelay(ulong nDelay);
IEV_C_PROTO(TimingRuptHandler);
ulong WhatTime(char *time_chr, ushort *rtcms);
IEV_C_PROTO(ExtFinishPulseRuptHandler);
void RTCCSACSync(ulong secs, short detect);

/******************************************************************************\
**	main
\******************************************************************************/
int main(void) {
  // short i;
  short result = 0; // no errors so far
  char time_chr[22];
  short rtcms;

  // Identify the progam and build
  printf("\nProgram: %s: %s %s \n", __FILE__, __DATE__, __TIME__);
  // Identify the device and its firmware
  printf("Persistor CF%d SN:%ld   BIOS:%d.%02d   PicoDOS:%d.%02d\n", CFX,
         BIOSGVT.CFxSerNum, BIOSGVT.BIOSVersion, BIOSGVT.BIOSRelease,
         BIOSGVT.PICOVersion, BIOSGVT.PICORelease);
  // Identify the arguments
  WhatTime(time_chr, &rtcms);

  uprintf("RTC time now %s\n", time_chr);
  // TUSimpleTerm();
  GetSyncRTCbyCSAC();
  cdrain();
  coflush();

  CheckRTC_CSACTime();
  cdrain();
  coflush();

  uprintf("Out\n");
  WhatTime(time_chr, &rtcms);
  uprintf("%s\n", time_chr);

  // BIOSResetToPicoDOS(); // full reset, but jump to 0xE10000 (PicoDOS)
  return result;

} //____ main() ____//

//***************************************************************************************
// TUSimpleTerm
// Simple COM2 term routine
// 03/27/2013 H. Matsumoto NOAA/OSU
//***************************************************************************************
void TUSimpleTerm(void) {
  TUPort *tuport;
  short rxch, txch;
  long baud = 57600;

  uprintf("\nA simple CSAC communication program\n");
  cdrain();
  coflush();
  //      uprintf("First term is for GPS.  To move on to Iridium term, press
  //      interrupt button.\n");
  cdrain();
  coflush();
  //
  //      INITIALIZE THE TPU UART MODULE
  //              This really belongs in your main() function should generally
  //              only
  //              be called once at the start of your program. Repeat calls to
  //              TUInit
  //              release any existing TPU UART channels and attempts to use
  //              them
  //              without reopening will fail or crash the system.
  //
  TUInit(calloc, free); // give TU manager access to our heap

  //
  //      OPEN THE TPU UART PORT
  //              This is the simplest form of opening a port since we
  //              implicitly
  //              accept all of the default configuration settings defined by
  //              the
  //              TUChParams structure which is defined in <cfxpico.h> by
  //              passing
  //              zero for the last parameter. See the TUAltConsoleCmd for an
  //              example showing how to supply custom configuration parameters
  //              for an open call.
  //
  //              The Rx and Tx TPU channels are selected by their associated
  //              pin
  //              number on the CF2's connector C. The actual TPU channels are
  //              never directly referenced after opening, and the pin numbers,
  //              or well named macros, are much more intuitive. In this case,
  //              we're using the pins that connect to the auxiliary RS232
  //              driver for connector AUX1 on a fully loaded PicoDAQ2
  //              RecipeCard
  //              which is why also setup pins C29 and C30 to enable the
  //              MAX3222.

  PIOSet(CF2CSAC);
  PIOClear(PCCSAC);
  PIORead(48);
  rxch = TPUChanFromPin(RX); // AUX RS232 receiver on CSAC board
  txch = TPUChanFromPin(TX); // AUX RS232 transmitter on CSAC board
  cprintf("Enabling the CF2-CSAC Com port\n");

  cprintf("Delaying 0.5 sec\n");
  Delay(5);

  cprintf("Opening CSAC com port\n");
  tuport = TUOpen(rxch, txch, baud, 0);
  if (tuport == 0) {
    cprintf("\n!!! Unexpected error opening TU channel, aborting\n");
    goto exitTUST;
  }

  //
  //      PREPARE FOR POSSIBLE FAST BLOCK TRANSFERS
  //              The TPU UARTs only work with interrupt driven queues, but the
  //              main SCI UART run in polled mode until explicitly directed to
  //              switch to queued interrupt driven mode. We'll do that now in
  //              case we get some big block requests from actions like paste
  //              or send text files.
  SCIRxSetBuffered(true); // switch SCI to buffered receive mode
  SCITxSetBuffered(true); // switch SCI to buffered transmit mode

  //
  //      ENTER THE TERMINAL LOOP
  //              For as long a the PBM button isn't pushed (or pin-C39
  //              grounded,
  //              we alternately check the TPU UART receive queue for data, and
  //              if
  //              found, grab it and send it to the main SCI UART for viewing.
  //              Then
  //              we check the main SCI UART's queue to see if any keyboard data
  //              has come in, and if so, grab that and send it out to whatever
  //              is
  //              on the receiving end of the TPU UART.
  //
  //              Note that we used the abbreviated console I/O style macro
  //              names
  //              (defined in <cfxpico.h> instead of the longer and more
  //              descriptive
  //              actual function names. You are free to use whatever you
  //              prefer.
  //      TUTxPrintf(tuport,"AT+CBST=7\n");

  uprintf("Entering the terminal loop.  To get out, press PBM.\n");
  while (PIORead(IRQ5)) // run until PBM button pushed
  {
    if (tgetq(tuport))      // we've got data from afar
      cputc(tgetc(tuport)); // grab it and show it

    if (cgetq())              // we've got data from the keyboard
      tputc(tuport, cgetc()); // send it off
  }

exitTUST:
  if (tuport)
    TUClose(tuport);

  PinIO(IRQ5);

  PIOClear(CF2CSAC);
  PIOSet(PCCSAC);
  cprintf("Exit out\n");
  cdrain();
  coflush();
  // return 0;

} //____ TUSimpleTerm ____//

//*************************************************************************************
// Delay
// Delay 0.1 * centisecs [Sec] approximately
// Use more power than the low power version
// Walter Hannah (July, 2004)
//*************************************************************************************
void Delay(short centisecs)
// delay this many hundredths of a second
{
  short i;
  for (i = 0; i < 100 * centisecs; i++)
    Delay1ms();

  // RTCDelayMicroSeconds(dsecs * 100000);
}

//*****************************************************************************************
// GetSyncRTCbyCSAC()
// Get CSAC time in sec since 1970, Jan 1.  If set == true, lock RTC time to the
// CSAC time.
// If set==false, just returns the CSAC sec.
// Char array time_chr contains time.
// H. Matsumoto 03-27-2013
//*****************************************************************************************
bool GetSyncRTCbyCSAC(void)
// Get CSAC time, set the real time clock if set is true
{
  TUPort *tuport;
  short rxch, txch;
  long baud = 57600L;
  short detect = 1;

  short i;
  short imax = 10; // No. of characters expect from CSAC
  char r[62] = {NULL};
  RTCtm *timenow; // CFX's real time clock
  ushort ticks = 0;

  TUInit(calloc, free); // give TU manager access to our heap
  PIOSet(CF2CSAC);
  PIOClear(PCCSAC);
  PIORead(
      48); // This statement is necessary since RX pin is connected to pin 48.
  PIOSet(PPSENAB); // Enable reading 1-PPS in

  rxch = TPUChanFromPin(RX); // AUX RS232 receiver on CSAC board
  txch = TPUChanFromPin(TX); // AUX RS232 transmitter on CSAC board
  tuport = TUOpen(rxch, txch, baud, 0);

  cprintf("Enabling the CF2-CSAC Com port\n");
  cprintf("Delaying 0.5 sec\n");
  Delay(5);

  cprintf("Opening CSAC com port\n");

  if (tuport == 0) {
    cprintf("\n!!! Unexpected error opening TU channel, aborting\n");
    goto exitSYNC;
  }

  // ProgramFinished=false;

  IEVInsertCFunct(&ExtFinishPulseRuptHandler, level5InterruptAutovector);
  PinBus(IRQ5);

  // while(!ProgramFinished)
  //{
  TUTxFlush(tuport);
  TURxFlush(tuport); // Need this for the next TU serial command

  // TickleSWSR();
  cprintf("\nSending T to CSAC to sync");
  TUTxPrintf(tuport, "T"); // Get time in sec since 1970
  TUTxWaitCompletion(tuport);

  while (!PinRead(PPSIN))
    ;
  for (i = 0; i < imax; i++) {
    if ((r[i] = TURxGetByteWithTimeout(tuport, RxTIMEOUT + 10)) == -1) {
      goto exitSYNC;
    }
  }

  // Received time in sec since 1970 from CSAC.
  // cprintf("%s\n",r);cdrain();coflush();
  SecondsSince1970 = atol(r) + 1L;

  RTCCSACSync(SecondsSince1970, detect);

  // cprintf("%ld\n",SecondsSince1970);
  timenow = localtime(&SecondsSince1970);
  cprintf("\nTime set at %d %d:%d:%d:%d:%.3u\n", timenow->tm_year,
          timenow->tm_yday + 1, timenow->tm_hour, timenow->tm_min,
          timenow->tm_sec, TICKS_TO_MS(ticks));

  TUTxFlush(tuport);
  TURxFlush(tuport); // Need this for the next TU serial command
  //}
  RTCDelayMicroSeconds(50L);
  TUTxFlush(tuport);
  TURxFlush(tuport);
  TUClose(tuport);
  PIOClear(CF2CSAC);
  PIOSet(PCCSAC);
  PIOClear(PPSENAB); // Disable reading 1-PPS in
  PinRead(IRQ5);

  cdrain();
  coflush();
  uprintf("Synched successfully!! \n");
  return true;
exitSYNC:

  TickleSWSR();

  TUTxFlush(tuport);
  TURxFlush(tuport);
  TUClose(tuport);
  PIOClear(CF2CSAC);
  PIOSet(PCCSAC);
  PIOClear(PPSENAB); // Disable reading 1-PPS in

  cprintf("\nError in TPUTerm. Not synched.\n"); // for timing purpose
  return false;
}

//**CheckRTC_CSACTime()************************************************************
//*****************************************************************************************
// CheckRTC_CSACTIme()
// Compare CFx RTC and CSAC time after sync.
// H. Matsumoto 04-02-2013
//*****************************************************************************************
bool CheckRTC_CSACTime(void)
// Get GPS time, set the real time clock if set is true
{
  TUPort *tuport;
  short rxch, txch;
  long baud = 57600L;

  short i;
  short imax = 10;
  char r[62] = {NULL};
  ulong rtcsec, csacsec;
  RTCtm *rtctime;  // CFX's real time clock
  RTCtm *csactime; // CSAC time
  ushort *RTCms = 0;
  ushort ticks = 0;

  TUInit(calloc, free); // give TU manager access to our heap
  cprintf("Connecting the CF2-CSAC COM ports\n");
  PIOSet(CF2CSAC);  // Enable  CSAC to CF com
  PIOClear(PCCSAC); // Disable PCC to CSAC com
  PIORead(48);      // This statement is necessary RX pin is connected to pin 48
  PIOSet(PPSENAB);

  rxch = TPUChanFromPin(RX); // AUX RS232 receiver on CSAC board
  txch = TPUChanFromPin(TX); // AUX RS232 transmitter on CSAC board
  cprintf("Checks RTC and CSAC times\n");
  cprintf("Opening CSAC com port\n");

  tuport = TUOpen(rxch, txch, baud, 0);
  cprintf("Delaying 0.5 sec to make sure RS232 port is ready...\n");
  Delay(5);

  if (tuport == 0) {
    cprintf("\n!!! Unexpected error opening TU channel, aborting\n");
    goto exitTPU;
  }

  // uprintf("%d\n",PIORead(IRQ5));
  // Enable PBM(IRQ5) interrupt
  ProgramFinished = false;
  IEVInsertCFunct(&ExtFinishPulseRuptHandler, level5InterruptAutovector);
  PinBus(IRQ5);

  // while(!ProgramFinished)
  //{
  TUTxFlush(tuport);
  TURxFlush(tuport); // Need this for the next TU serial command
  // TickleSWSR();
  RTCDelayMicroSeconds(500000L);

  // Get CSAC seconds since Jan 1, 1970
  cprintf("\nSending T to CSAC.\n");

  TUTxPrintf(tuport, "T"); // Get time in sec since 1970
  TUTxWaitCompletion(tuport);

  while (!PinRead(PPSIN))
    ;
  *RTCms = TICKS_TO_MS(ticks);

  for (i = 0; i < imax; i++) {
    if ((r[i] = TURxGetByteWithTimeout(tuport, RxTIMEOUT + 10)) == -1) {
      goto exitTPU;
    }
  }

  csacsec = atol(r);
  rtcsec = RTCGetTime(NULL, &ticks);

  rtctime = localtime(&rtcsec);
  cprintf("RTC  %d %d:%d:%d:%d:%.3u\n", rtctime->tm_year, rtctime->tm_yday + 1,
          rtctime->tm_hour, rtctime->tm_min, rtctime->tm_sec,
          *RTCms); // TICKS_TO_MS(ticks));

  csactime = localtime(&csacsec);
  cprintf("CSAC %d %d:%d:%d:%d\n", csactime->tm_year, csactime->tm_yday + 1,
          csactime->tm_hour, csactime->tm_min, csactime->tm_sec);

  TURxFlush(tuport); // Need this for the next TU serial command
  //}

  RTCDelayMicroSeconds(50L);
  PinRead(IRQ5);
  TUTxFlush(tuport);
  TURxFlush(tuport);
  TUClose(tuport);
  PIOClear(CF2CSAC);
  PIOSet(PCCSAC);
  PinRead(IRQ5);
  uprintf("Get out of the 2nd loop\n");
  return true;
exitTPU:

  // TickleSWSR();
  TUTxFlush(tuport);
  TURxFlush(tuport);
  TUClose(tuport);
  PIOClear(CF2CSAC);
  PIOSet(PCCSAC);
  PinRead(IRQ5);
  PIOClear(PPSENAB);

  cprintf("Error in TPUTerm\n"); // for timing purpose
  return false;
} //**CheckRTC_CSACTime()*

//*************************************************************************************
// LowPowerDelay
// The following uses 51msec periodic interrupt to delay time (low power)
// Low power 1 to 2mA. 1L delays 102msec. 10L delays 1.020sec, etc.
// Haru Matsumoto (NOAA) 10/13/2004
//*************************************************************************************
void LowPowerDelay(ulong nDelay) {
  ulong TempCount = 0L;
  ulong passtime;
  RTCTimer rt;

  SCITxWaitCompletion(); // let any pending UART data finish
                         //      TMGSetSpeed(8000);

  TimingRuptCount = 0L;
  PITSet51msPeriod(PITOff); // disable timer
  PITRemoveChore(0);        // get rid of all current chores
  IEVInsertCFunct(&TimingRuptHandler, pitVector); // replacement fast routine

  // Shut down other unnecessary operations
  CFEnable(false);
  EIAForceOff(true);
  CTMRun(false);
  // QSMStop();

  RTCElapsedTimerSetup(&rt);
  PITSet51msPeriod(2); // A pulse every 102msec
  while (!TimeExpired) {
    if (TimingRuptCount == TempCount) {
      LPStopCSE(FastStop);
    } else {
      TempCount = TimingRuptCount;
      if (TimingRuptCount >= nDelay) // 0.515 sec delay
      {
        TimeExpired = true;
      }
    }
  }
  passtime = RTCElapsedTime(&rt);
  TimeExpired = false;

  EIAForceOff(false);
  CFEnable(true);
  QSMRun();
  PITSet51msPeriod(PITOff);
  PITRemoveChore(0);

  // WhatTime(strbuf);
  // printf("%ld us\n",passtime);
} // LowPowerDelay

/*************************************************************************************
**	WhatTime
**
**	Returns 20-char long current timein RTC time in GMT format.
**
**	|RTC__--------------|
**	104 245:13:34:45:999
**  123456789012345678901
**           1         2         3         4
**	NOAA, Nerport, OR
**	Haru Matsumoto 1-st version 10/26/04	hm
*************************************************************************************/
ulong WhatTime(char *GMT_char, ushort *RTCms) {
  RTCtm *rtc_time; // CFX's real time clock
  ushort Ticks;

  SecondsSince1970 = RTCGetTime(&SecondsSince1970, &Ticks);
  rtc_time = RTClocaltime(&SecondsSince1970);
  *RTCms = TICKS_TO_MS(Ticks);
  sprintf(GMT_char, "%.3d %.3d:%.2d:%.2d:%.2d:%.3u", rtc_time->tm_year,
          rtc_time->tm_yday + 1, rtc_time->tm_hour, rtc_time->tm_min,
          rtc_time->tm_sec, *RTCms);
  return SecondsSince1970;
} // WhatTime

//***************************************************************************************
// Interrupt routine for PIT51msPeriod (low power)
//***************************************************************************************
IEV_C_FUNCT(TimingRuptHandler) {
#pragma unused(ievstack) // implied (IEVStack *ievstack:__a0) parameter

  if (!TimeExpired) {
    TimingRuptCount++;
  }
} //____ TimingRuptHandler() ____//

/******************************************************************************\
**	ExtFinishPulseRuptHandler		IRQ5 logging stop request
interrupt
**
\******************************************************************************/
IEV_C_FUNCT(ExtFinishPulseRuptHandler) {
#pragma unused(ievstack) // implied (IEVStack *ievstack:__a0) parameter

  PinIO(IRQ5);
  ProgramFinished = true;

} //____ ExtFinishPulseRuptHandler() ____//

#if CFX == CF2
/******************************************************************************\
**	CustomRTCSyncStart	// Simple custom patch that syncs on PPSIN
pin going high
\******************************************************************************/
static short TimeStartLevelDetect;
void CustomRTCSyncStart(void);
void CustomRTCSyncStart(void) {

  TickleSWSR();                                  // another reprieve
  while (PinRead(PPSIN) != TimeStartLevelDetect) // now we wait...
    ;
  // cprintf("RTCSync: exiting\n");
}

//____ RTCSync() ____//
#endif

/*******************************************************************************
**  RTCCSACSync  // From example how to synchronize the RTC**
**  This program assumes that argument secs contains the time in seconds since
**  midnight, 1/1/70, which will become valid on the leading edge of the 1-PPS
**  pulse we will be monitoring on pin 24 (PPSIN). It also assumes that there's
**  nothing else important happening, as we may be turning off all interrupts
**  while we wait for the synchronization signal.**
**
**	NOAA/OSU, Newport, OR
**	1-st Release 9/08/99  by HM
** 2nd  Release 04/02/2013 HM
*******************************************************************************/
void RTCCSACSync(ulong secs, short detect) {
  ushort saveSR; // to save the interrupt mask level

  SecondsSince1970 = secs;
  saveSR = IEVSaveSRThenDisable(); // all interrupts now disabled
  PIOSet(PPSENAB);                 // Enable 1-PPS reading at pin 24

#if CFX == CF1
  RTCStopClock();                  // stop the clock
  RTCSetTime(secs, NULL);          // load the time into the stopped clock
  TickleSWSR();                    // another reprieve
  while (PinRead(PPSIN) != detect) // now we wait...
    ;
  RTCStartClock(); // start the clock running again
#elif CFX == CF2
  {
    vfptr savedRTCSyncStart = BIOSPatchInsert(RTCSyncStart, CustomRTCSyncStart);
    TimeStartLevelDetect = detect;
    RTCSetTime(secs, NULL); // load the time into the stopped clock
    BIOSPatchInsert(RTCSyncStart, savedRTCSyncStart);
  }
#endif

  IEVRestoreSavedSR(saveSR); // restore the entry mask level
  TickleSWSR();              // another reprieve
} /* RTCCSACSync() */
