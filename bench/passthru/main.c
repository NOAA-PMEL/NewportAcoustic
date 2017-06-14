/******************************************************************************\
**	main.c				Persistor and PicoDOS starter C file
**
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

#define DEBUG
#ifdef DEBUG
#define DBG(X) X // template:	DBG( cprintf("\n"); )
#pragma message("!!! "__FILE__                                                 \
                ": Don't ship with DEBUG compile flag set!")
#else
#define DBG(X) // nothing
#endif

// Definitions of uMPC TPU ports
#define SB39PWR 23 // SB#39plus TD power
#define SB39RX 32  // Tied to IRQ2
#define SB39TX 31

#define PAMPWR 24     // PAM PWR on/off
#define PAMZEROBIT 29 // PAM selecton
#define PAMONEBIT 30  // PAM selection
#define PAMRX 28
#define PAMTX 27

#define ADCPWR 19

#define A3LAPWR 21 // IRIDGPS PWR
#define A3LARX 33  // IRIDGPS tied to /IRQ3
#define A3LATX 35

#define COM4PWR 22 // COM4 Enable
#define COM4RX 26  // Tied to /IRQ5
#define COM4TX 25
#define SYSCLK 14000            // choose: 160 to 32000 (kHz)
#define WTMODE nsStdSmallBusAdj // choose: nsMotoSpecAdj or nsStdSmallBusAdj
//#define	LPMODE	FastStop			// choose: FullStop or
//FastStop or CPUStop

TUPort *SB39Port;                 // SB39 TD TUport
TUPort *PAMPort;                  // PAM TUport
TUPort *IRIDGPSPort;              // IRID GPS port
TUPort *COM4Port;                 // COM4 port
char *LogFile = {"activity.log"}; // Activity Log
static char *stringin;
short sb39rxch, sb39txch;
short pamrxch, pamtxch;
short iridrxch, iridtxch;
short com4rxch, com4txch;

// SB39 data over the serial line
static char *SB39receive; // Pointer of received input from SB39 TD
static char *SB39send;    // Section to send to LARA main

void OpenSB39Port(bool on);
void SB39Data();
void OpenPAMPort(bool on);
void OpenIRIDGPSPort(bool on);
void OpenCOM4Port(bool on);
static void Irq5RxISR(void);
static void Irq2RxISR(void);

/******************************************************************************\
**	main
\******************************************************************************/
int main() {
  short result = 0; // no errors so far
  uchar chin = 0xA;
  uchar chSB = 0xB;
  // uchar  test;
  // ushort chin=10;
  // ushort chSB=10;
  RTCTimer tmtset;
  short test = 0;

  bool connectCOM4andSB39 = false;
  // short numt;

  // Identify the progam and build
  printf("\nProgram: %s: %s %s \n", __FILE__, __DATE__, __TIME__);
  printf("Persistor CF%d SN:%ld   BIOS:%d.%02d   PicoDOS:%d.%02d\n", CFX,
         BIOSGVT.CFxSerNum, BIOSGVT.BIOSVersion, BIOSGVT.BIOSRelease,
         BIOSGVT.PICOVersion, BIOSGVT.PICORelease);

  // I am here because the main eletronics powered this up and this *.app
  // program started.
  //
  // Make sure all the device powers and COM ports down
  PIOClear(A3LAPWR);
  PIOClear(SB39PWR);
  PIOClear(COM4PWR);
  PIOClear(PAMPWR);
  PIOClear(ADCPWR);

  // what happens here?
  CSSetSysAccessSpeeds(nsFlashStd, nsRAMStd, nsCFStd, WTMODE);
  // set sys clock freq kHz
  TMGSetSpeed(SYSCLK);

  SB39receive = calloc(128, sizeof(uchar));
  SB39send = calloc(128, sizeof(uchar));
  stringin = (char *)calloc(128, sizeof(uchar));

  // Initialize activity logging
  /// Initflog(LogFile, true);

  /// flogf("Program to test serial com.\n");
  /// flogf("At COM4, type ^B to connect SB39 and COM4.Type 'ts' to get the T-D\n");
  /// flogf("At COM4, type '^C' to connect IRIDGPS and COM4.\n");
  /// flogf("To exit, type ^D\n");

  // Initialize to open TPU UART
  TUInit(calloc, free);

  /// OpenSB39Port(true); // Open SBE39 port
  OpenIRIDGPSPort(true);
  /// OpenCOM4Port(true); // Open port to communicate with the main elec

  connectCOM4andSB39 = false;

  /// flogf("Foreign host<->COM4<->SB39\n");
  /// flogf("To switch from SB39 to IRID/GPS, type ^C\n");
  putflush();
  /// // DBG(RTCElapsedTimerSetup(&tmtset));
  /// // DBG(flogf("Elapsed time %ld us\n", RTCElapsedTime(&tmtset)));

  /// // Install the COM4 IRQ5 interrupt handlers from RS232 RX input.
  /// IEVInsertAsmFunct(Irq5RxISR, level5InterruptAutovector); // COM4
  /// IEVInsertAsmFunct(Irq5RxISR, spuriousInterrupt);         // COM4

  // Enable the IRQ5 interrupt
  /// PinBus(IRQ5); // enable COM4 (IRQ5) interrupt
  // PinBus(IRQ2);
  TickleSWSR(); // another reprieve

  while (PinRead(IRQ5)) {
    LPStopCSE(FastStop); // we will be here until interrupted
    TickleSWSR();        // by break
  }                      // Go find out what letter is

  PinRead(IRQ5);
  chin = tgetc(COM4Port);
  while (chin != 4) { //= CTRL D to exit
    if (chin == 2) {  // CTRL B to communicate with SBE39
      CSSetSysAccessSpeeds(nsFlashStd, nsRAMStd, nsCFStd, WTMODE);
      TMGSetSpeed(8000);

      flogf("CTL B entered\n");
      chin = 0x0A; // clear chin
      DBG(RTCElapsedTimerSetup(&tmtset));
      while (chin > 7) {     // 25 us  //printable ASCII
        if (tgetq(SB39Port)) // receive data from SB39
        {
          tputc(COM4Port, tgetc(SB39Port));
          // cputc(chin);// grab it and show it
        }

        if (tgetq(COM4Port)) { // we've got data at COM4 port from the main
          chin = tgetc(COM4Port);
          if (chin > 7) { // printable ASCII
            tputc(SB39Port, chin);
            // cputc(tgetc(SB39Port));
          }
        }
        if (test == 0)
          DBG(flogf("Elapsed time %ld us\n", RTCElapsedTime(&tmtset)));
        test = 1;
      }
    }

    if (chin == 3) { // CTRL C
      flogf("IRID/GPS mode\n");
      chin = 0x0A; // clear chin
      while (chin > 7) {
        if (tgetq(IRIDGPSPort)) // receive data from IRIDGPS
        {
          tputc(COM4Port, tgetc(IRIDGPSPort));
          // cputc(chin);// grab it and show it
        }

        if (tgetq(COM4Port)) { // we've got data COM4 port
          chin = tgetc(COM4Port);
          if (chin > 7) { // normal ASCII
            tputc(IRIDGPSPort, chin);
            // cputc(tgetc(SB39Port));
          }
        }
      }
    }
  }
  if (SB39Port != 0)
    OpenSB39Port(false);
  if (IRIDGPSPort != 0)
    OpenIRIDGPSPort(false);
  if (COM4Port != 0)
    OpenCOM4Port(false);
  PIOClear(COM4PWR);
  cdrain();
  coflush();
  free(SB39send);
  free(SB39receive);

  return result;
} //____ main() ____//

void OpenSB39Port(bool on) {
  long baud = 9600L;

  // uMPC
  if (on) {
    PIOSet(SB39PWR); // turn on SB39 serial term power
    sb39rxch = TPUChanFromPin(SB39RX);
    sb39txch = TPUChanFromPin(SB39TX);
    // RTCDelayMicroSeconds(500000L);

    // PinBus(IRQ2);

    // Define SB39 TD tuporst
    SB39Port = TUOpen(sb39rxch, sb39txch, baud, 0);
    RTCDelayMicroSeconds(100000L);

    // Enable buffered operation
    SCIRxSetBuffered(true);
    SCITxSetBuffered(true);

    if (SB39Port == 0 && on) // SB39
      flogf("\n!!! Error opening SB39 channel...");
  } else if (!on) {

    PIOClear(SB39PWR); // SB39 TD
    putflush();
    TUClose(SB39Port);
  }
} /*OpenSB39Port(bool on)*/

/*************************************************************************
* OpenPAMPort(bool on)
* If on=true, open the com.
* If on=false, close the com.
**************************************************************************/
void OpenPAMPort(bool on) {
  long baud = 19200L;

  if (on) {
    // TUInit(calloc, free);
    PIOSet(PAMPWR);       // PAM COM ON
    PIOClear(PAMZEROBIT); // PAM1 select
    PIOClear(PAMONEBIT);  // PAM1 select
    pamrxch = TPUChanFromPin(PAMRX);
    pamtxch = TPUChanFromPin(PAMTX);

    RTCDelayMicroSeconds(1000000L);

    // Define PAM tuporst
    PAMPort = TUOpen(pamrxch, pamtxch, baud, 0);
    RTCDelayMicroSeconds(100000L);

    // Enable buffered operation
    SCIRxSetBuffered(true);
    SCITxSetBuffered(true);

    TUTxFlush(PAMPort);
    TURxFlush(PAMPort);
    RTCDelayMicroSeconds(1000000L);

    if (PAMPort == 0 && on) // SB39
      flogf("\n!!! Error opening PAM channel...");
  } else if (!on) {
    TUClose(PAMPort);
    PIOClear(PAMPWR); // Shut down PAM COM
    flogf("Close PAM term\n");
    putflush();
  }
} /*OpenPAMPort(bool on)*/
/*************************************************************************
* OpenIRIDGPSPort(bool on)
* If on=true, open the com.
* If on=false, close the com.
**************************************************************************/
void OpenIRIDGPSPort(bool on) {
  long baud = 19200L;

  if (on)
    flogf("Opening the high speed IRID/GPS port\n");
  if (on) {
    PIOSet(A3LAPWR);                   // PWR ON
    iridrxch = TPUChanFromPin(A3LARX); //
    iridtxch = TPUChanFromPin(A3LATX);

    // PIORead(48);
    PIORead(50);

    // PIORead(33); //Make IRQ3 read
    // PIORead(35);
    RTCDelayMicroSeconds(1000000L); // wait for warm up

    // Define PAM tuporst
    IRIDGPSPort = TUOpen(iridrxch, iridtxch, baud, 0);
    RTCDelayMicroSeconds(100000L);

    // Enable buffered operation
    SCIRxSetBuffered(true);
    SCITxSetBuffered(true);

    TUTxFlush(IRIDGPSPort);
    TURxFlush(IRIDGPSPort);
    RTCDelayMicroSeconds(1000000L);

    if (IRIDGPSPort == 0 && on) // IRID
      flogf("\n!!! Error opening IRIDGPS channel...");
  } else if (!on) {
    TUClose(IRIDGPSPort);
    PIOClear(A3LAPWR); // Shut down IRID PWR
    flogf("Close IRIDGPS term\n");
    putflush();
  }
} /*OpenIRIDGPSPort(bool on)*/

/*************************************************************************
* OpenCOM4Port(bool on) for uMPC. On MPC it is AMODEM com
* If on=true, open the com.
* If on=false, close the com.
**************************************************************************/
void OpenCOM4Port(bool on) {
  long baud = 19200L;

  if (on)
    flogf("Opening the high speed COM4 port\n");
  if (on) {

    com4rxch = TPUChanFromPin(COM4RX);
    com4txch = TPUChanFromPin(COM4TX);

    PIOSet(COM4PWR); // PWR On COM4 device

    RTCDelayMicroSeconds(1000000L);

    // Define COM4 tuport
    COM4Port = TUOpen(com4rxch, com4txch, baud, 0);
    RTCDelayMicroSeconds(100000L);

    // Enable buffered operation
    SCIRxSetBuffered(true);
    SCITxSetBuffered(true);

    TUTxFlush(COM4Port);
    TURxFlush(COM4Port);
    RTCDelayMicroSeconds(100000L);

    if (COM4Port == 0 && on) // COM4
      flogf("\n!!! Error opening COM4 port...");
  } else if (!on) {
    TUClose(COM4Port);
    PIOClear(COM4PWR); // Shut down COM4 device
    flogf("Close COM4 port\n");
    putflush();
  }
} /*OpenCOM4Port(bool on)*/

/******************************************************************************\
** void SB39Data()
\******************************************************************************/
void SB39Data() {
  char charin = ' ';
  int i = 0, count = 0;
  float depth, temp;

  bool log = false;
  char *first_junk;
  char *split_temp;
  char *split_pres;
  // char* split_cond;
  char *split_date;
  char *split_time;
  char string_to_send[128];

  memset(SB39receive, 0, 128 * sizeof(char));
  memset(string_to_send, 0, 128 * sizeof(char));

  while (count < 50 && i < 128) {
    charin = TURxGetByteWithTimeout(SB39Port, 20);
    if (charin == -1)
      count++;
    else {
      SB39receive[i] = charin;
      i++;
    }
  }

  // Example
  //  25.6035,  0.046, 09 May 2017, 16:20:58
  //<Executed/>
  // flogf("%d chars\n", i);
  sprintf(string_to_send, "%s", SB39receive + 5);
  cprintf("new %s\n", string_to_send);

  uprintf("%s\n", SB39receive);

  first_junk = strtok(string_to_send, " ");
  split_temp = strtok(NULL, " ,");
  // split_temp = strtok(SB39receive, " ,");
  split_pres = strtok(NULL, " ,");

  // date
  split_date = strtok(NULL, ",");
  split_time = strtok(NULL, "\n\r");

  memset(SB39send, 0, 128 * sizeof(char));
  sprintf(SB39send, "$ATD %s, %s, %s, %s*", split_temp, split_pres, split_date,
          split_time);

  count = 0;
  for (i = 1; i <= 128; ++i) {
    if (SB39send[i] != '*') {
      ++count;
    } else
      break;
  }
  count += 2;

  flogf("Char to send to LARA:%d, %s\n", count, SB39send);
  flogf("Temp %s\n", split_temp);
  flogf("depth %s\n", split_pres);
  flogf("date %s\n", split_date);
  flogf("time %s\n", split_time);

  temp = atof(split_temp);
  flogf("T =%9.4f\n", temp);
  depth = atof(split_pres);
  flogf("D =%9.3f\n");
  putflush();

  // TUTxPrintf(PAMPort, "%s\n", writestring);
  // TUTxWaitCompletion(PAMPort);

} // SB39Data()
/*************************************************************************\
**  static void Irq2ISR(void)
\*************************************************************************/
static void Irq2RxISR(void) {
  PinIO(IRQ2);
  RTE();
} //____ Irq2ISR ____//

/******************************************************************************\
**	Irq5RxISR			Interrupt handler for IRQ5 (tied to
CMOS RxD)
**
**	This single interrupt service routine handles both the IRQ4 interrupt
**	and the very likely spurious interrupts that may be generated by the
**	repeated asynchronous and non-acknowledged pulses of RS-232 input.
**
**	The handler simply reverts IRQ4 back to input (to prevent further level
**	sensative interrupts) and returns. It's assumed the routine that set
this
**	up is just looking for the side effect of breaking the CPU out of STOP
**	or LPSTOP mode.
**
**	Note that this very simple handler is defined as a normal C function
**	rather than an IEV_C_PROTO/IEV_C_FUNCT. We can do this because we know
**	(and can verify by checking the disassembly) that is generates only
**	direct addressing instructions and will not modify any registers.
\******************************************************************************/
static void Irq5RxISR(void) {

  PinIO(IRQ5); // 39 IRQ5 (tied to Rx)
  RTE();

} //____ Irq4RxISR() ____//
