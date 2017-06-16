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

#define WTMODE nsStdSmallBusAdj // choose: nsMotoSpecAdj or nsStdSmallBusAdj
//#define	LPMODE	FastStop			// choose: FullStop or
//FastStop or CPUStop

// IRIDUM TUPORT Setup
TUPort *IRIDGPSPort;
short IRIDGPS_RX, IRIDGPS_TX;
// Definitions of MPC TPU ports
#define IRIDPWR 23 // IRIDGPS PWR
#define IRIDSEL 22 
#define IRIDRX 32  // IRIDGPS tied to /IRQ2
#define IRIDTX 31


char *LogFile = {"activity.log"}; // Activity Log
static char *stringin;

// data over the serial line
static char *S_receive; // Pointer of received input from SB39 TD
static char *S_send;    // Section to send to LARA main

void OpenTUPort_IRIDGPS(bool on);
static void Irq5RxISR(void);
static void Irq2RxISR(void);

/******************************************************************************\
**	main
\******************************************************************************/
int main() {
  short result = 0; // no errors so far
  int run = true; // yes
  uchar chin = 0xA;
  uchar chSB = 0xB;

  // Identify the progam and build
  printf("\nProgram: %s: %s %s \n", __FILE__, __DATE__, __TIME__);
  printf("Persistor CF%d SN:%ld   BIOS:%d.%02d   PicoDOS:%d.%02d\n", CFX,
         BIOSGVT.CFxSerNum, BIOSGVT.BIOSVersion, BIOSGVT.BIOSRelease,
         BIOSGVT.PICOVersion, BIOSGVT.PICORelease);

  S_receive = calloc(128, sizeof(uchar));
  S_send = calloc(128, sizeof(uchar));
  stringin = (char *)calloc(128, sizeof(uchar));

  // Initialize
  IEVInsertAsmFunct(Irq2RxISR, level2InterruptAutovector);//IR
  IEVInsertAsmFunct(Irq2RxISR, spuriousInterrupt);        //IR
  PinBus(IRQ2);                                   // IR

  TUInit(calloc, free);
  OpenTUPort_IRIDGPS(run);

  putflush();


  return result;
} //____ main() ____//

/*************************************************************************
* OpenTUPort_IRIDGPS(bool on)
* If on=true, open the com.
* If on=false, close the com.
**************************************************************************/
void OpenTUPort_IRIDGPS(bool on) {

  short wait = 10000;
  int warm;
  if (on) {
    IRIDGPS_RX = TPUChanFromPin(IRIDRX);
    IRIDGPS_TX = TPUChanFromPin(IRIDTX);

    // Power ON
    PIOSet(IRIDPWR);
    PIOSet(IRIDSEL);
    IRIDGPSPort = TUOpen(IRIDGPS_RX, IRIDGPS_TX, 19200, 0);
    if (IRIDGPSPort == 0)
      flogf("\n\t|Bad IridiumPort");

    warm = 45;
    flogf("\n%s|Warming up GPS/IRID Unit for %d Sec", Time(NULL), warm);
    putflush();
    CIOdrain();
    TUTxFlush(IRIDGPSPort);

    inputstring = (char *)calloc(128, 1);
    first = (char *)calloc(128, 1);

    // Delay_AD_Log(warm);
    RTCDelayMicroSeconds(warm * 1000000L); 

    TURxFlush(IRIDGPSPort);

  } else if (!on) {

    flogf("\n%s|PowerDownCloseComIRIDGPS() ", Time(NULL));
    putflush();
    CIOdrain();
    SendString("AT*P");
    GetIRIDInput("OK", 2, NULL, NULL, wait);

    // Delay_AD_Log(3);
    RTCDelayMicroSeconds(3 * 1000000L); 

    PIOClear(IRIDGPSCOM);
    PIOClear(IRIDGPSPWR);
    TUClose(IRIDGPSPort);
  }
} // OpenTUPort_IRIDGPS



/*************************************************************************\
**  static void Irq2xISR(void)
\*************************************************************************/
static void Irq2RxISR(void) {
  PinIO(IRQ2);
  RTE();
} //____ Irq2xISR ____//

