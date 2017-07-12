/******************************************************************************\
**	main.c				Persistor and PicoDOS starter C file
**
**  Program to communicate with T-D sensor and Iridium/GPS modem.
**  Main electronics serial<->COM4 of this CF2 <->SB39
**  or
**  Main electronics serial<->COM4 of this CF2 <->Iridium/GPS
**  To switch between SB39 and Iridium/GPS, send the following CTRL char to COM4
**  ^b to connect SB39plus and send 'ts' to get the T-D data.
**  ^f to connect GPS. Antenna is switched to GPS side.
**  ^d to connect Iridium. Antenna is sitched to IRID.
**  To exit, ^e.
**  You can go back to SB39plus whenever you send ^b.
**  CF2 sends/receives 2 bytes at a time. So sends the even number of character
data
**
**  July 3, 2017 HM
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

//#define DEBUG
#ifdef DEBUG
#define DBG(X) X // template:   DBG( cprintf("\n"); )
#pragma message("!!! "__FILE__                                                 \
                ": Don't ship with DEBUG compile flag set!")
#else               /*  */
#define DBG(X)      // nothing
#endif              /*  */
#define VERSION 1.2 // keep this up to date!!! - always V.R
// blk 1.2 power mgmt, 9600

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
#define ANTSW 1

#define A3LAPWR 21 // IRIDGPS PWR
#define A3LARX 33  // IRIDGPS tied to /IRQ3
#define A3LATX 35

#define COM4PWR 22 // COM4 Enable
#define COM4RX 26  // Tied to /IRQ5
#define COM4TX 25
//#define       SYSCLK   8000           // choose: 160 to 32000 (kHz)
#define SYSCLK 16000            // choose: 160 to 32000 (kHz)
#define WTMODE nsStdSmallBusAdj // choose: nsMotoSpecAdj or nsStdSmallBusAdj
//#define       LPMODE  FastStop                        // choose: FullStop or
// FastStop or CPUStop
TUPort *SB39Port;                 // SB39 TD TUport
TUPort *PAMPort;                  // PAM TUport
TUPort *IRIDGPSPort = 0;          // IRID GPS port
TUPort *COM4Port;                 // COM4 port
char *LogFile = {"activity.log"}; // Activity Log

char chin = 0x0A;

// SB39 data over the serial line
static char *SB39receive; // Pointer of received input from SB39 TD
static char *SB39send;    // Section to send to LARA main
bool CharInterrupt = false;

// short         CustomSYPCR = WDT419s | HaltMonEnable | BusMonEnable | BMT32;
//#define CUSTOM_SYPCR
void OpenSB39Port(bool on);
void SB39Data();
void OpenPAMPort(bool on);
void OpenIRIDGPSPort(bool on);
void OpenCOM4Port(bool on);
static void Irq5RxISR(void);
static void Irq2RxISR(void);
IEV_C_PROTO(CharRuptHandler);
void PreRun(void);

/******************************************************************************\
**	main
\******************************************************************************/
int main() {
  short result = 0; // no errors so far
  uchar chSB = 0x0B;

  // RTCTimer tmtset;
  short test = 0;
  bool connectCOM4andSB39 = false;
  long del = 0L;

  // Identify the progam and build
  printf("\nProgram: %s: 2.1-%f, %s %s \n", __FILE__, (float)VERSION, __DATE__,
         __TIME__);
  printf("Persistor CF%d SN:%ld   BIOS:%d.%02d   PicoDOS:%d.%02d\n", CFX,
         BIOSGVT.CFxSerNum, BIOSGVT.BIOSVersion, BIOSGVT.BIOSRelease,
         BIOSGVT.PICOVersion, BIOSGVT.PICORelease);

  // I am here because the main eletronics powered this unit up and *.app
  // program started.
  //
  // Make sure unnecessary devices are powered down
  PIOClear(A3LAPWR);
  PIOClear(SB39PWR);
  PIOClear(COM4PWR);
  PIOClear(PAMPWR);
  PIOClear(ADCPWR);
  CSSetSysAccessSpeeds(nsFlashStd, nsRAMStd, nsCFStd, WTMODE);
  TMGSetSpeed(SYSCLK);
  SB39receive = calloc(128, sizeof(uchar));
  SB39send = calloc(128, sizeof(uchar));

  // Initialize activity logging
  Initflog(LogFile, true);
  PreRun();

  // Initialize to open TPU UART
  TUInit(calloc, free);

  // Open the com ports
  OpenSB39Port(true); // Open SBE39 port
  OpenCOM4Port(true); // Open COM4 to prepart to talk to the main elec
  connectCOM4andSB39 = true;

  /*
     //Keyboard input. Test only.
     if (connectCOM4andSB39==false){//direct
     cprintf("user keyboard<->SB39. Type . to stop\n");

     while(chin !='.')
     {
     if (tgetq(SB39Port))
     {                                           // we've got data from
     SB39
     cputc(tgetc(SB39Port)); // grab it and show it
     }

     if (cgetq())
     {// we've got data from the keyboard
     if (chin !='.')
     tputc(SB39Port, chin=cgetc());     // send it off
     //tputc(SB39Port, chin);// send it off
     }
     }
     }else{//control SB39 from other COM port input
   */

  // Install the COM4 IRQ5 interrupt handlers from RS232 RX input.
  IEVInsertAsmFunct(&CharRuptHandler,
                    level5InterruptAutovector); // COM4 IRQ5 handler
  TURxFlush(COM4Port);
  TUTxFlush(COM4Port);

  // TickleSWSR();                                      // another reprieve
  RTCDelayMicroSeconds(5500L);
  chin = 0x0A;

  // Enable the COM4 IRQ5 interrupt
  PinBus(IRQ5);                  // enable COM4 (IRQ5) interrupt
  while (chin > 6 || chin < 2) { // Wait until get CTRL char
    // while (tgetq(COM4Port))
    while (!CharInterrupt) {
      LPStopCSE(FastStop); // we will be here until interrupted
      RTCDelayMicroSeconds(10L);

      // TickleSWSR();
    } // Go find out what character is
    chin = tgetc(COM4Port);
  }
  CharInterrupt = false;
  PinIO(IRQ5);
  if (chin < 7 && chin > 1) {
    while (chin != 5) {
    // if it is CTRL e, exit out, and end the program

      // CTRL b to communicate with SBE39plus
      if (chin == 2) {
        if (SB39Port == 0)
          OpenSB39Port(true); // Open SBE39 port
        flogf("SBE39plus mode\n");
        chin = 0x0A; // clear chin
        DBG(RTCElapsedTimerSetup(&tmtset));
        while (chin > 7) {     // 25 us  //printable ASCII
          // receive data from SB39
          if (tgetq(SB39Port)) {
            TUTxPutByte(COM4Port, tgetc(SB39Port), true);
            DBG(cputc(chin)); // debug grab it and show it
          }
          if (tgetq(COM4Port)) { // we've got data at COM4 port from the main
            chin = tgetc(COM4Port);
            if (chin > 7) { // printable ASCII
              TUTxPutByte(SB39Port, chin, true);
              DBG(cputc(chin)); // Debug
            }
          }
          DBG(flogf("Elapsed time %ld us\n", RTCElapsedTime(&tmtset)));
          test = 1;
        }
      } // end of SB39plus session
      TURxFlush(COM4Port);

      // IRID/GPS sesssion starts
      if (chin == 6 || chin == 4) { // CTRL f for GPS or CTRL D Iridium
        if (IRIDGPSPort == 0)
          OpenIRIDGPSPort(true);
        if (chin == 6) {
          flogf("GPS mode\n");
          PIOClear(1); // Antenna switch to GPS
          tiflush(COM4Port);
        } else if (chin == 4) {
          flogf("Iridium mode\n");
          PIOSet(1); // Antenna SW to Iridium
          tiflush(COM4Port);
        }
        chin = 0x10; // replace with ASCII > 7

        // There is a chance that Iridium modem sends NULL char (=0)
        while (chin > 7 ||
               chin < 2) {        // handle both GPS and Iridium serial stream
          if (tgetq(IRIDGPSPort)) // receive char from IRID/GPS
          {
            chin = tgetc(IRIDGPSPort);
            TUTxPutByte(COM4Port, chin, true);

            // DBG(cputc(chin));// grab it and show it
          }
          if (tgetq(COM4Port)) { // we've got data COM4 port
            chin = tgetc(COM4Port);
            if (chin > 7) { // normal ASCII
              TUTxPutByte(IRIDGPSPort, chin, true);

              // DBG(cputc(chin));
            }
          }
        }
      } // IRID/GPS session ends
    }
  } else
    flogf("Did not get CTLR char. Exit.\n");
  if (SB39Port != 0)
    OpenSB39Port(false);
  if (IRIDGPSPort != 0)
    OpenIRIDGPSPort(false);
  if (COM4Port != 0)
    OpenCOM4Port(false);
  TURelease();
  PIOClear(COM4PWR);
  PIOClear(ADCPWR);
  cdrain();
  coflush();
  free(SB39send);
  free(SB39receive);
  return result;
} //____ main() ____//

void OpenSB39Port(bool on) {
  long baud = 9600L;
  short sb39rxch, sb39txch;

  // uMPC
  if (on) {
    PIOSet(SB39PWR); // turn on SB39 serial term power
    sb39rxch = TPUChanFromPin(SB39RX);
    sb39txch = TPUChanFromPin(SB39TX);

    // RTCDelayMicroSeconds(500000L);

    // Define SB39 TD tuporst
    SB39Port = TUOpen(sb39rxch, sb39txch, baud, 0);
    RTCDelayMicroSeconds(100000L);
    if (SB39Port == 0 && on) // SB39
      flogf("\n!!! Error opening SB39 channel...");
  } else if (!on) {
    PIOClear(SB39PWR); // SB39 TD
    putflush();
    TUClose(SB39Port);

    // TURelease();
  }
} /*OpenSB39Port(bool on) */

/*************************************************************************
* OpenPAMPort(bool on)
* If on=true, open the com.
* If on=false, close the com.
**************************************************************************/
void OpenPAMPort(bool on) {
  long baud = 19200L;
  short pamrxch, pamtxch;
  if (on) {

    // TUInit(calloc, free);//Need to be executed only once
    PIOSet(PAMPWR);       // PAM COM ON
    PIOClear(PAMZEROBIT); // PAM1 select
    PIOClear(PAMONEBIT);  // PAM1 select
    pamrxch = TPUChanFromPin(PAMRX);
    pamtxch = TPUChanFromPin(PAMTX);
    RTCDelayMicroSeconds(1000000L);

    // Define PAM tuporst
    PAMPort = TUOpen(pamrxch, pamtxch, baud, 0);
    RTCDelayMicroSeconds(100000L);
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
} /*OpenPAMPort(bool on) */

/*************************************************************************
* OpenIRIDGPSPort(bool on)
* If on=true, open the com.
* If on=false, close the com.
* IRQ3
**************************************************************************/
void OpenIRIDGPSPort(bool on) {
  long baud = 19200L;
  short iridrxch, iridtxch;
  if (on)
    flogf("Opening the high speed IRID/GPS port\n");
  if (on) {
    PIOSet(A3LAPWR);                   // PWR ON
    iridrxch = TPUChanFromPin(A3LARX); //
    iridtxch = TPUChanFromPin(A3LATX);
    PIOSet(1); // Antenna switch initialize
    RTCDelayMicroSeconds(1000000L);
    PIOClear(1); // Antenna switch to GPS
    RTCDelayMicroSeconds(1000000L);
    PIORead(
        48); // Important!! This is connected to TXin of the internal RS232 IC
    PIORead(33);
    PIORead(IRQ3RXX);               // Make IRQ3 read. Not bus.
    RTCDelayMicroSeconds(1000000L); // wait for IRID/GPS unit to warm up

    // Define PAM tuporst
    IRIDGPSPort = TUOpen(iridrxch, iridtxch, baud, 0);
    RTCDelayMicroSeconds(100000L);
    TUTxFlush(IRIDGPSPort);
    TURxFlush(IRIDGPSPort);
    RTCDelayMicroSeconds(1000000L);
    if (IRIDGPSPort == 0 && on) // IRID
      flogf("\n!!! Error opening IRIDGPS channel...");
  } else if (!on) {
    TUClose(IRIDGPSPort);
    IRIDGPSPort = 0;
    PIOClear(A3LAPWR); // Shut down IRID PWR
    flogf("Close IRIDGPS term\n");
    putflush();
  }
} /*OpenIRIDGPSPort(bool on) */

/*************************************************************************
* OpenCOM4Port(bool on) for uMPC. On MPC it is for AMODEM com
* If on=true, open the com.
* If on=false, close the com.
* Uses IRQ5
**************************************************************************/
void OpenCOM4Port(bool on) {
  long baud = 19200L;
  short com4rxch, com4txch;
  if (on)
    flogf("Opening the high speed COM4 port\n");
  if (on) {
    com4rxch = TPUChanFromPin(COM4RX);
    com4txch = TPUChanFromPin(COM4TX);
    PIOSet(COM4PWR); // PWR On COM4 device
    RTCDelayMicroSeconds(1000000L);
    PIORead(IRQ5);

    // Define COM4 tuport
    COM4Port = TUOpen(com4rxch, com4txch, baud, 0);
    RTCDelayMicroSeconds(100000L);
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
} /*OpenCOM4Port(bool on) */

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

  // memset(SB39receive, 0, 128*sizeof(char));
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

/******************************************************************************\
**	ExtPulseRuptHandler		IRQ5 request interrupt
**
\******************************************************************************/
IEV_C_FUNCT(CharRuptHandler) {

#pragma unused(ievstack) // implied (IEVStack *ievstack:__a0) parameter
  CharInterrupt = true;
  PinIO(IRQ5);

  // flogf("Interrupted by IRQ5 \n", time_chr);cdrain();coflush();
} //____ ExtFinishPulseRuptHandler() ____//

/******************************************************************************\
**	PreRun		Exit opportunity before we start the acquisition program
**
**
\******************************************************************************/
void PreRun(void) {
  short standby = 4;
  short i;
  char c;
  char what = 'n';
  char *ProgramDescription = {
      "\n"
      "Serial interface program to control and communiate with SB39 and "
      "Iridium/GPS.\n"
      "From another computer to COM4 of this CF2 via 19200 \n"
      "send ^b to connect SB39.Type 'ts' to get the T-D from SB39plus.\n"
      "send ^f to connect GPS. Antenna SW is GPS side.\n"
      "send ^d to connect Iridium/GPS. Antenna to IRID side.\n"
      "send ^e to exit.\n"
      "To stop this program from starting, hit . during the countdown.\n"};
  cprintf(ProgramDescription, standby, standby);
  for (i = standby; i > 0; i--) {
    cprintf("%u..", i);
    c = SCIRxGetCharWithTimeout(1000); // 1 second
    if (c == '.')
      break;
    if (c == -1)
      continue;
    i = 0; // any other key ends the timeout
  }
  if (i <= 0) {
    cprintf("\nStarting...\n");
    return; // to start acquisition
  }

  //    TickleSWSR();   // another reprieve
  QRchar("\nWhat next?(P=PicoDOS, BIOS reset)", "%c", false, &c, "PS", true);
  if (c == 'S') {
    BIOSReset(); // force clean restart
  } else if (c == 'P')
    BIOSResetToPicoDOS();
  return; // to start acquisition
} //____ PreRun() ____//
