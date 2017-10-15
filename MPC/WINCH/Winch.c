#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions

#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <fcntl.h>
#include <stat.h>    // PicoDOS POSIX-like File Status Definitions
#include <termios.h> // PicoDOS POSIX-like Terminal I/O Definitions
#include <unistd.h>

#include <MPC_Global.h>
#include <Winch.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <CTD.h>
#include <GPSIRID.h>
#include <PLATFORM.h>
#include <Settings.h>

WINCHParameters NIGK;
TUPort *NIGKPort;
char NextCharacter;

// extern SystemStatus LARA;
bool Surfaced;
extern bool PutInSleepMode;
extern SystemStatus LARA;

WinchCalls WINCH;

/*************************************************************************\
\*************************************************************************/
void WinchConsole() {
  char in;
  cprintf("\n\t|WinchConsole():");
  in = cgetc();

  // Stop
  if (in == 'S')
    Winch_Stop();
  // Descend
  else if (in == 'F')
    Winch_Descend();
  // Ascend
  else if (in == 'R')
    Winch_Ascend();
  // Buoy Status?
  else if (in == 'B')
    Buoy_Status();
  else
    cprintf("Bad Input Character.");

} //_____ Console_Data() _____//

/*************************************************************\
\*************************************************************/
void AModem_Data(void) {
  char *inString; // Array for input command/response
  short inchar;
  char prechar; // Two ints to grab different characters of the input
  int i, decimal = 0, multiplier = 4096; // location within input array
  bool symbol = false; // Symbol if '#' command is present or not
  float cableLength = 0.0;
  char *command;
  char hexCode[] = "0000";
  float depth;
  static int AscentStopTries = 0;

  inString = (char *)calloc(64, sizeof(char));

  for (i = 0; i < 64; i++) { // Scan through input from NIGKPort
    inchar = TURxGetByteWithTimeout(NIGKPort, 500);
    if (inchar == -1) {
      break;
    } else
      inString[i] = (char)inchar;
  }

  // if command from Winch. If stop, why? Update LARA System Statuses
  if (strchr(inString, '#') != NULL) {
    command = strtok(inString, "#");
    flogf("\n%s\t|Command: #%s", Time(NULL), command);
    prechar = command[0];
    symbol = true;
  }
  // Confirming a new winch command//response from command
  else if (strchr(inString, '%') != NULL) {
    command = strtok(inString, "%");
    flogf("\n%s\t|Response: %%%s", Time(NULL), command);
    prechar = command[0];
    symbol = false;

  } else if (strstr(inString, "OK") != NULL) {
    cprintf("OK");
    TURxFlush(NIGKPort);
    TUTxFlush(NIGKPort);
    free(inString);
    return;
  } else
    flogf("\n%s\t|other: %s", Time(NULL), inString);

  if (prechar == 'R') {

    if (LARA.BUOYMODE == 0)
      CTD_Sample(1);
    LARA.BUOYMODE = 1;
    WINCH.ASCENTRCV++;
    // if(!symbol)

  } else if (prechar == 'F') {

    if (LARA.BUOYMODE == 0)
      CTD_Sample(1);
    LARA.BUOYMODE = 2;
    WINCH.DESCENTRCV++;

  } else if (prechar == 'S') {

    if (symbol) {
      WINCH.STOPRCV++;
      flogf("\n%s|Stop Command Received", Time(NULL));

      if (LARA.BUOYMODE == 1) { // Winch Stops buoy from Ascending... Lack of
                                // Tension? Are we close to TDepth? Try Calling?
        depth = LARA.DEPTH - NIGK.TDEPTH;
        flogf("\n\t|AModem_Data() Winch Stopped Buoy");
        if (depth > 5.0 &&
            AscentStopTries <
                2) { // if buoy still too deep. try ascending again
          Winch_Ascend();
          AscentStopTries++;
        } else if (depth < 2.0 ||
                   AscentStopTries >= 1) { // if buoy less than 5 meters to
                                           // target depth OR tried ascending at
                                           // least twice before.
          flogf("\n\t|Buoy close enough to target depth. Try Calling");
          LARA.PHASE++;
          LARA.SURFACED = true;
          AscentStopTries = 0;
        }
      }
    }
    LARA.BUOYMODE = 0;

  }

  else if (prechar == 'B')
    Buoy_Status();

  TURxFlush(NIGKPort);
  free(inString);
  return;

} //____ AModem_Data() ____//
/*************************************************************\
\*************************************************************/
ulong Winch_Ascend(void) {
  DBG(flogf("\n%s|Winch_Ascend():", Time(NULL));)

  TUTxWaitCompletion(NIGKPort);
  TUTxPrintf(NIGKPort, "#R,01,03\n");
  Delayms(25);
  WINCH.ASCENTCALLS++;
  return (time(NULL) + (ulong)NIGK.DELAY);
} //____ Ascend() ____//
/*************************************************************\
\*************************************************************/
ulong Winch_Descend(void) {
  DBG(flogf("\n%s|Winch_Descend():", Time(NULL));)

  TUTxWaitCompletion(NIGKPort);
  TUTxPrintf(NIGKPort, "#F,01,00\n");
  Delayms(25);
  WINCH.DESCENTCALLS++;
  return (time(NULL) + (ulong)NIGK.DELAY); // why return time?
} //____ Descend() ____//
/*************************************************************\
\*************************************************************/
ulong Winch_Stop(void) {
  DBG(flogf("\n%s|Winch_Stop():", Time(NULL));)

  TUTxWaitCompletion(NIGKPort);
  TUTxPrintf(NIGKPort, "#S,01,00\n");
  Delayms(25);
  WINCH.STOPCALLS++;
  return (time(NULL) + (ulong)NIGK.DELAY);
}
/*************************************************************\
** void Buoy(void)
** A function call to the Buoy to get the status of equipment.
** The return call through NIGKPort sends 2 bytes of data as the
** response to the Buoy CommandWinch from the Deck Unit
\*************************************************************/
void Buoy_Status(void) {
  char B_Status[3] = "00"; // Base return call
  DBG(flogf("\n%s|Buoy Status:", Time(NULL));)

  if (LARA.BUOYMODE != 0) // If CTDPort is Active and the Buoy is in motion
    B_Status[1] = '1';
  else if (LARA.BUOYMODE == 0) // If the Buoy is inactive and the CTDPort is off
    B_Status[0] = '1';

  TUTxWaitCompletion(NIGKPort);
  TUTxPrintf(
      NIGKPort, "%%B,01,%c%c\n", B_Status[0],
      B_Status[1]); // Send rest of status via acoustic remote to deck unit
  Delayms(5000);        // Make sure all the Buoy Calls have been received....
  TURxFlush(NIGKPort); // Before clearing the NIGKPort Rx
  WINCH.BUOYRCV++;

} //____ Buoy() ____//
/****************************************************************************\
** void OpenTUPort_NIGK(bool)
\****************************************************************************/
void OpenTUPort_NIGK(bool on) {
  static bool once = true;
  short AModem_RX, AModem_TX;
  flogf("\n\t|%s NIGK Winch TUPort", on ? "Open" : "Close");

  if (on) {
    AModem_RX = TPUChanFromPin(AMODEMRX);
    AModem_TX = TPUChanFromPin(AMODEMTX);

    PIOClear(AMODEMPWR);
    Delayms(250);
    PIORead(48);
    PIOSet(AMODEMPWR); // Powers up the DC-DC for the Acoustic Modem Port
    NIGKPort = TUOpen(AModem_RX, AModem_TX, AMODEMBAUD, 0);
    Delayms(150);
    if (NIGKPort == 0)
      flogf("\n\t|Bad Winch TUPort\n");
    else {
      TUTxFlush(NIGKPort);
      TURxFlush(NIGKPort);
      Delayms(5000); // Wait 5 seconds for NIGKPort to power up
      TUTxPrintf(NIGKPort, "\n");
      Delayms(250);
      TUTxFlush(NIGKPort);
      TURxFlush(NIGKPort);
    }
  } else {
    Delayms(500);
    PIOClear(AMODEMPWR);
    TUClose(NIGKPort);
  }
  return;

} //____ OpenTUPort_NIGK() ____//
/****************************************************************************\
** void GetWinchSettings()
\****************************************************************************/
void GetWinchSettings() {
  char *p;

  p = VEEFetchData(NIGKDELAY_NAME).str;
  NIGK.DELAY = atoi(p ? p : NIGKDELAY_DEFAULT);
  DBG(uprintf("NIGK.DELAY=%u (%s)\n", NIGK.DELAY, p ? "vee" : "def"); cdrain();)

  p = VEEFetchData(NIGKANTENNALENGTH_NAME).str;
  NIGK.ANTLEN = atoi(p ? p : NIGKANTENNALENGTH_DEFAULT);
  DBG(uprintf("NIGK.ANTLEN=%u (%s)\n", NIGK.ANTLEN, p ? "vee" : "def");
      cdrain();)

  p = VEEFetchData(NIGKTARGETDEPTH_NAME).str;
  NIGK.TDEPTH = atoi(p ? p : NIGKTARGETDEPTH_DEFAULT);
  DBG(uprintf("NIGK.TDEPTH=%u (%s)\n", NIGK.TDEPTH, p ? "vee" : "def");
      cdrain();)

  p = VEEFetchData(NIGKRISERATE_NAME).str;
  NIGK.RRATE = atoi(p ? p : NIGKRISERATE_DEFAULT);
  DBG(uprintf("NIGK.RRATE=%u (%s)\n", NIGK.RRATE, p ? "vee" : "def"); cdrain();)

  p = VEEFetchData(NIGKFALLRATE_NAME).str;
  NIGK.FRATE = atoi(p ? p : NIGKFALLRATE_DEFAULT);
  DBG(uprintf("NIGK.FRATE=%u (%s)\n", NIGK.FRATE, p ? "vee" : "def"); cdrain();)

  p = VEEFetchData(NIGKPROFILES_NAME).str;
  NIGK.PROFILES = atoi(p ? p : NIGKPROFILES_DEFAULT);
  DBG(uprintf("NIGK.PROFILES=%u (%s)\n", NIGK.PROFILES, p ? "vee" : "def");
      cdrain();)

  p = VEEFetchData(NIGKRECOVERY_NAME).str;
  NIGK.RECOVERY = atoi(p ? p : NIGKRECOVERY_DEFAULT);
  DBG(uprintf("NIGK.RECOVERY=%u (%s)\n", NIGK.RECOVERY, p ? "vee" : "def");
      cdrain();)
}
/****************************************************************************\
** void Winch_Monitor(int filehandle)
\****************************************************************************/
void Winch_Monitor(int filehandle) {
  // global WriteBuffer
  int byteswritten = 0;
  memset(WriteBuffer, 0, BUFSZ);
  sprintf(
      WriteBuffer, "NIGK "
                   "%s\nProfile:%d\nDelay:%d\nTDepth:%d\nRiseRate:%d\nFallRate:"
                   "%d\nASCENTCALLS:%d, RCV:%d\nDESCENTCALLS:%d, "
                   "RCV%d\nSTOPCALLS:%d, RCV%d\n\0",
      NIGK.RECOVERY ? "RECOVERY" : "ON", NIGK.PROFILES, NIGK.DELAY, NIGK.TDEPTH,
      NIGK.RRATE, NIGK.FRATE, WINCH.ASCENTCALLS, WINCH.ASCENTRCV,
      WINCH.DESCENTCALLS, WINCH.DESCENTRCV, WINCH.STOPCALLS, WINCH.STOPRCV);

  // Maybe include something about calculated velocities and cable length of
  // winch after #of seconds.
  flogf("\n\t|Winch_Monitor():\n%s", WriteBuffer);

  byteswritten = write(filehandle, WriteBuffer, strlen(WriteBuffer));
  DBG(flogf("\nBytesWritten: %d", byteswritten);)
  // free(writebuffer);

  WINCH.ASCENTCALLS = 0;
  WINCH.ASCENTRCV = 0;
  WINCH.DESCENTCALLS = 0;
  WINCH.DESCENTRCV = 0;
  WINCH.STOPCALLS = 0;
  WINCH.STOPRCV = 0;
  WINCH.BUOYRCV = 0;
  WINCH.WINCHCALLS = 0;
}
