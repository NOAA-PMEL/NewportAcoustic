/*
 * antmain.c    Antenna module, LARA
**  Program to communicate with T-D sensor and Iridium/GPS modem.
**  Main electronics serial<->COM4 of this CF2 <->SBE
**  July 12, 2017 blk
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
#define DBG(X) X // template:   DBG( cprintf("\n"); )
#pragma message("!!! "__FILE__                                                 \
                ": Don't ship with DEBUG compile flag set!")
#else               /*  */
#define DBG(X)      // nothing
#endif              /*  */
#define VERSION 2.0 
// keep this up to date!!! 

// Definitions of uMPC TPU ports
#define SBEPWR 23 // SB#39plus TD power
#define SBERX 32  // Tied to IRQ2
#define SBETX 31

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
#define BUOY_BAUD 9600L
#define IRID_BAUD 19200L

#define BUOY 0
#define SBE 1
#define IRID 2

//#define       LPMODE  FastStop                        // choose: FullStop or
// FastStop or CPUStop
//
//
// short         CustomSYPCR = WDT419s | HaltMonEnable | BusMonEnable | BMT32;
//#define CUSTOM_SYPCR
//static void Irq5RxISR(void);
//static void Irq2RxISR(void);
//IEV_C_PROTO(CharRuptHandler);
TUPort* OpenSbePt(bool on);
TUPort* OpenIridPt(bool on);
TUPort* OpenBuoyPt(bool on);
short getByte(TUPort *tup);
short power(short c, bool on);
void connect(char c);
short char2id(short ch);
void init();
void help();
void status(short command);
void antennaSwitch(char c);

char *LogFile = {"activity.log"}; 
char antSw;
bool run=true; 
struct { char *name, c; bool power; TUPort *port; } dev[3] = {
  { "BUOY", 'B', false, NULL },
  { "SBE", 'S', false, NULL },
  { "IRID", 'I', false, NULL } };
short devID; // ID of upstream device, 1-2
TUPort *buoy=NULL, *devPort=NULL; // dev port of connnected upstream device


/******************************************************************************\
**	main
\******************************************************************************/
void main() {
  short ch, command=0;
  int binary=0; // count of binary chars to pass thru
  bool commandMode=false, binaryMode=false;

  // set up hw
  init();
  // initial connection is SBE
  power('S', true);
  connect('S');

  // run remains true, exit via biosreset{topicodos}
  while (run) {
    // look for chars on both sides, process
    // read bytes not blocks, to see any rs232 errs
    // note: using vars buoy,devport is faster than dev[id].port
    // get all from dev upstream
    while (devPort && TURxQueuedCount(devPort)) {
      ch=getByte(devPort);
      TUTxPutByte(buoy, ch, true); // block if queue is full
    } // char from device

    // get all from buoy
    while (buoy && TURxQueuedCount(buoy)) {
      ch=getByte(buoy);
      // binaryMode, commandMode, new command, character
      if (binaryMode) {
        TUTxPutByte(devPort, ch, true);
        if (--binary<1) binaryMode=false;
        DBG(printf(binary ? "." : ".\n"); putflush();)
      // endif (binaryMode)
      } else if (commandMode) {
        switch (command) { // set by previous byte
          case 1: // ^A Antenna G|I
            antennaSwitch(ch);
            break;
          case 2: // ^B Binary byte
            TUTxPutByte(devPort, ch, true);
            break;
          case 3: // ^C Connect I|S
            connect(ch);
            break;
          case 4: // ^D powerDown I|S
            power(ch, false);
            break;
          case 5: // ^E binary lEngth 1Byte
            // up to 64K not needed for this version
            // convert last byte and next byte to a integer, 0-64K
            // binary=ch<<8 + getByte(buoy);
            binary=ch;
            binaryMode=true;
            // DBG(ch=binary;)
            break;
          case 6: // ^F unused
            break;
          case 7: // ^G unused
            break;
        } // switch (command)
        DBG(printf("cmd:%d arg:%d\n", command, ch);)
        commandMode=false;
        command=0;
      // endif (commandMode)
      } else if (ch<8) {
          commandMode=true;
          command=ch;
      // endif (ch<8)
      } else { 
        // regular char
        TUTxPutByte(devPort, ch, true);
      }
    } // while buoy

    // console
    if (SCIRxQueuedCount()) {
      // probably going out to PICO
      ch=SCIRxGetChar();
      // SCIR is masked ch=ch & 0x00FF;
      switch (ch) {
        case 'x': 
          BIOSResetToPicoDOS(); break;
        case 's':
          status(command); break;
        default:
          help();
      } // switch (ch)
    } // if console 
  } // while run
} // main()




/*
 * OpenSbePt(true)
 */
TUPort* OpenSbePt(bool on) {
  TUPort *sbePort=NULL;
  long baud = 9600L;
  short sb39rxch, sb39txch;
  if (on) {
    DBG(flogf("Opening the SBE port 9600\n");)
    PIOSet(SBEPWR); // turn on SBE serial term power
    sb39rxch = TPUChanFromPin(SBERX);
    sb39txch = TPUChanFromPin(SBETX);

    // Define SBE TD tuporst
    Delayms(100);
    sbePort = TUOpen(sb39rxch, sb39txch, baud, 0);
    if (sbePort == NULL) {
      flogf("\n!!! Error opening SBE channel...");
    } else {
      dev[SBE].port = sbePort;
    }
    return sbePort;
  } else { // if (!on) {
    TUClose(dev[SBE].port);
    dev[SBE].port=NULL;
    PIOClear(SBEPWR); // SBE TD
    return NULL;
  }
} /*OpenSbePt(bool on) */


/*************************************************************************
* OpenIridPt(bool on)
* If on=true, open the com.
* If on=false, close the com.
* IRQ3
**************************************************************************/
TUPort* OpenIridPt(bool on) {
  TUPort *IridPt=NULL;
  long baud = IRID_BAUD;
  short iridrxch, iridtxch;
  if (on) {
    DBG(flogf("Opening the high speed IRID/GPS port\n");)
    PIOSet(A3LAPWR);                   // PWR ON
    iridrxch = TPUChanFromPin(A3LARX); //
    iridtxch = TPUChanFromPin(A3LATX);
    // Important!! This is connected to TXin of the internal RS232 IC
    PIORead(48); 
    PIORead(33);
    PIORead(IRQ3RXX);               // Make IRQ3 read. Not bus.
    Delayms(1000); // wait for IRID/GPS unit to warm up

    // Define dev[SBE].port 
    IridPt = TUOpen(iridrxch, iridtxch, baud, 0);
    Delayms(100);
    TUTxFlush(IridPt);
    TURxFlush(IridPt);
    Delayms(1000);
    if (IridPt == NULL) {
      flogf("\n!!! Error opening IRIDGPS channel...");
    } else {
      dev[IRID].port=IridPt;
    }
    return IridPt;
  } else { // if (!on) {
    TUClose(dev[IRID].port);
    dev[IRID].port=NULL;
    PIOClear(A3LAPWR); // Shut down IRID PWR
    flogf("Close IRIDGPS term\n");
    return NULL;
  }
} /*OpenIridPt(bool on) */

/*************************************************************************
* OpenBuoyPt(bool on) for uMPC. On MPC it is for AMODEM com
* If on=true, open the com.
* If on=false, close the com.
**************************************************************************/
TUPort* OpenBuoyPt(bool on) {
  TUPort *BuoyPt=NULL;
  long baud = BUOY_BAUD;
  short com4rxch, com4txch;
  if (on) {
    DBG(flogf("Opening the buoy COM4 port\n");)
    PIOSet(COM4PWR); // PWR On COM4 device
    com4rxch = TPUChanFromPin(COM4RX);
    com4txch = TPUChanFromPin(COM4TX);
    Delayms(1000);
    PIORead(IRQ5);

    // Define COM4 tuport
    BuoyPt = TUOpen(com4rxch, com4txch, baud, 0);
    Delayms(100);
    TUTxFlush(BuoyPt);
    TURxFlush(BuoyPt);
    Delayms(100);
    if (BuoyPt == NULL) 
      flogf("\n!!! Error opening COM4 port...");
    else
      dev[BUOY].port=BuoyPt;
    return BuoyPt;
  } else { // if (!on) {
    TUClose(dev[BUOY].port);
    dev[BUOY].port=NULL;
    PIOClear(COM4PWR); // Shut down COM4 device
    DBG(flogf("Close COM4 port\n");)
    return NULL;
  }
} /*OpenBuoyPt(bool on) */


/*************************************************************************\
**  static void Irq2ISR(void)
static void Irq2RxISR(void) {
  PinIO(IRQ2);
  RTE();
} //____ Irq2ISR ____//
\*************************************************************************/

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
static void Irq5RxISR(void) {
  PinIO(IRQ5); // 39 IRQ5 (tied to Rx)
  RTE();
} //____ Irq4RxISR() ____//
\******************************************************************************/

/******************************************************************************\
**	ExtPulseRuptHandler		IRQ5 request interrupt
**
IEV_C_FUNCT(CharRuptHandler) {

#pragma unused(ievstack) // implied (IEVStack *ievstack:__a0) parameter
  PinIO(IRQ5);

  // flogf("Interrupted by IRQ5 \n", time_chr);cdrain();coflush();
} //____ ExtFinishPulseRuptHandler() ____//
\******************************************************************************/


/*
 * help() - help message to console
 */
void help() {
  // Identify the progam and build
  char *ProgramDescription = {
      "\n"
      "Serial interface program to control and communiate with SBE and "
      "Iridium/GPS.\n"
      "Buoy is downstream, connecting to com4 at %d BAUD\n"
      " ^A Antenna G|I \n"
      " ^B Binary byte \n"
      " ^C Connect G|I|S \n"
      " ^D powerDown G|I|S|A \n"
      " ^E binary lEngth (2byte short) \n"
      " ^F unused \n"
      " ^G unused \n"
      "On console (com1):\n s=status x=exit *=this message\n"
      };

  printf("\nProgram: %s: 2.1-%f, %s %s \n", __FILE__, (float)VERSION, __DATE__,
         __TIME__);
  printf("Persistor CF%d SN:%ld   BIOS:%d.%02d   PicoDOS:%d.%02d\n", CFX,
         BIOSGVT.CFxSerNum, BIOSGVT.BIOSVersion, BIOSGVT.BIOSRelease,
         BIOSGVT.PICOVersion, BIOSGVT.PICORelease);
  printf(ProgramDescription, BUOY_BAUD);
} // help()

/*
 * init() - initialize hardware, open com ports
 */
void init() {
  // I am here because the main eletronics powered this unit up and *.app
  // program started.
  // Make sure unnecessary devices are powered down
  PIOClear(A3LAPWR);
  PIOClear(SBEPWR);
  PIOClear(COM4PWR);
  PIOClear(PAMPWR);
  PIOClear(ADCPWR);
  CSSetSysAccessSpeeds(nsFlashStd, nsRAMStd, nsCFStd, WTMODE);
  TMGSetSpeed(SYSCLK);

  // Initialize activity logging
  Initflog(LogFile, true);

  // settle antenna
  antennaSwitch('I');
  Delayms(500);
  antennaSwitch('G');

  // Initialize to open TPU UART
  TUInit(calloc, free);

  power('B', true);
  buoy=dev[BUOY].port;
  Delayms(500);
} // init()

/*
 * status(command) - console <- "connected:A3LA A3LA:on SBE:on"
 */
void status(short command) {
  cprintf("connected:%s antenna:%c iridgps:%s sbe:%s command:%d\n",
    dev[devID].name, antSw,
    dev[IRID].power ? "on" : "off",
    dev[SBE].power ? "on" : "off",
    command);
}

/*
 * getByte(tup) - get byte from TU port and log rs232 errors
 */
short getByte(TUPort *tup) {
  // global int rs232errors, char *devName
  short ch;
  ch=TURxGetByte(tup, true); // blocking, best to check queue first
  // high bits means errors, log
  if (ch & 0xFF00) {
    short i;
    for (i=0; i<3; i++)  if (dev[i].port == tup) break;
    flogf("Error code %d on char '%c' from %s\n", ch>>8, ch, dev[devID].name);
    ch&=0x00FF;
  }
  return ch;
} // getByte()

/*
 * antennaSwitch(ch) - change antenna=G|I, else return current state
 */
void antennaSwitch(char c) {
  // global short antSw
  DBG(printf("antennaSwitch %c\n", c);)
  switch(c) {
    case 'G': PIOClear(ANTSW); break;
    case 'I': PIOSet(ANTSW); break;
    default: flogf("antennaSwitch(%c) ?\n", c); return;
  }
  antSw=c;
} // antennaSwitch()

/*
 * char2id('G') - returns id 0-2, no match -1
 */
short char2id(short ch) {
  switch (ch) {
    case 'I': return IRID;
    case 'S': return SBE;
    case 'B': return BUOY;
    default: return -1;
  }
} // char2id()
  

/*
 * connect(I|S) - make a3la or sbe be the upstream device
 */
void connect(char c) {
  // global short devID, TUPort *devPort
  devID=char2id(c);
  // power up if not
  if (! dev[devID].power) 
    power(c, true);
  // for efficiency in char handling
  devPort=dev[devID].port; 
} // connect()

/*
 * power(I|S, on) - power device on/off
 * returns 1 if power unchanged, 0 changed, -1 failed
 */
short power(short c, bool onoff) {
  short id;
  TUPort *r;
  id=char2id(c);
  DBG(printf("dev:%c devid:%d onoff:%d\n", c, id, onoff);)
  if (dev[id].power == onoff) return 1;
  switch (c) {
    case 'I': r=OpenIridPt(onoff); break;
    case 'S': r=OpenSbePt(onoff); break;
    case 'B': r=OpenBuoyPt(onoff); break;
  }
  if (onoff && (r == NULL)) { // fail, on returns tup*
    BIOSResetToPicoDOS();
    // return -1;
  }
  if (!onoff && (id == devID)) { // powering off, check if connected
    devID=BUOY;
    devPort=NULL;
  }
  dev[id].power=onoff;
  dev[id].port=r;  // currently also done in Open*Pt
  return 0;
} // power()
