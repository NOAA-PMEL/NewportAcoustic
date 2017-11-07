// AntMod.c - for working with antenna.c antmodule
#include <common.h>
#include <AntMod.h>

/*
 * int DevSelect(DEVA);
 * DEVX=0 = off, DEVA=1 = antenna, DEVB=2 = buoy
 * mod: mpc.devID
 */
void DevSelect(int dev) {
  // global devicePort
  // use a stronger tests for open??
  if (dev==mpc.devID) return; // already selected
  DBG2(flogf("\n\t|DevSelect(%d)", dev);)

  switch (dev) {
  // deva devb share the port
  case DEVA: // antenna module dev
    mpc.devID=DEVA;
    if (PIOTestAssertClear(ANTMODPWR)) { // ant module is off, turn it on
      PIOSet(ANTMODPWR);
      Delay_AD_Log(5); // power up delay
    }
    PIOSet(DEVICECOM);
    break;
  case DEVB: // buoy sbe
    mpc.devID=DEVB;
    PIOClear(DEVICECOM); // talk to local port // do not turn off ant module
    break;
  } //switch
  return;
} //DevSelect

/*
 * AntMode(G|I|S)
 * Switch antenna module between SBE39 TD, Iridium and GPS.
 * ^A Antenna G|I * ^B Binary byte * ^C Connect I|S * ^D powerDown I|S
 */
int AntMode(char r) {
  static char ant='-', dev='-';
  char a, d;
  DBG2(flogf("\n\t|AntMode(%c)", r);)
  DevSelect(DEVA);
  // select ant SBE16, switch ant device
  switch (r) {
    case 'G': { a='G'; d='I'; break; }
    case 'I': { a='I'; d='I'; break; }
    case 'S': { a=ant; d='S'; break; }
    default: { // bad case
      flogf("\nError DevSelect(%c): bad choice", r);
      return -1;
    }
  }
  if (d!=dev) {
    // turn on, connect
    TUTxPutByte(devicePort, 3, true);  // ^C connect device
    TUTxPutByte(devicePort, d, true);
    dev=d;
    Delayms(3000); // wait 3 sec for device start
  }
  if (a!=ant) {
    TUTxPutByte(devicePort, 1, true);  // ^A antenna
    TUTxPutByte(devicePort, a, true);  // G I
    ant=a;
    Delayms(1000); // wait 1 sec to settle antenna switch noise
  }
  return 0;
} //AntMode

