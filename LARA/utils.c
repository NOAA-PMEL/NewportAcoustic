// utils.c
// utility routines used by several files
#include <utils.h>

// up to .05 second between chars, normally chars take .001-.016
#define CHARDELAY 50

// DelayTX(40) delay long enough for transmit 40 chars at 2400bd
void DelayTX(int ch) { RTCDelayMicroSeconds((long) ch * 3333L); }

int ReadLine(TUPort *port, char *in) {
  int len;
  for (len=1; len<BUFSZ; len++) {
    in[len] = TURxGetByteWithTimeout(port, CHARDELAY);
    if (in[len]<0) {
      // error, we should not get a timeout
      in[len]=0;
      len = -len;
    if (in[len]=='\n') {
      in[len]=0;
    }
  }
  return (len);
}

/*
 * write out\n - put chars until \0
 */
void WriteLine(TUPort *port, char *out) {
  while (*out) { TUTxPutByte(port, *out++, true); }
  TUTxPutByte(port, '\r', true);
}


/*
 * GetStringWait(port, stringin, 5)
 *  delay up to 5 seconds for first char, return length
 */
int GetStringWait(TUPort *port, int wait, char *in) {
  int len;
  in[0] = TURxGetByteWithTimeout(port, (short) wait*1000);
  TickleSWSR(); // could have been a long wait
  if (in[0]<0) {
    DBG1(flogf("\n\t|GetStringWait() timeout");)
    in[0]=0;
    return (-len);
  }
  for (len=1; len<BUFSZ; len++) {
    in[len] = TURxGetByteWithTimeout(port, CHARDELAY);
    if (in[len]<0) {
      // expect timeout
      in[len]=0;
      break;
    }
  }
  return len;
}

/*
 * GetResponse(port, strIn, 5, strOut)
 * flush, output, readline(echo), GetStrW; return out
 * err: 0:=echo fail
 */
int GetResponse(TUPort *port, char *out, int wait, char *in) {
  int len=0;
  TURxFlush(port);
  // consume echo - up to \n
  if (ReadLine(port, in) < 1) {
    DBG1( flogf( "\nErr: GetResponse() echo timeout %s", in); )
    return 0;
  }
  len = GetStringWait( port, wait, in );
  return len;
