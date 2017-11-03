// utils.c
// utility routines used by several files

// DelayTX(40) delay long enough for transmit 40 chars at 2400bd
void DelayTX(int ch) { RTCDelayMicroSeconds((long) ch * 3333L); }

/*
 * GetStringWait(port, stringin, 5)
 *  delay up to 5 seconds for first char, return length
 */
int GetStringWait(TUPort port, int wait, char *str) {
  // global devicePort
  int len;
  short charDelay=50; // up to .05 second between chars, normally .001

  str[0] = TURxGetByteWithTimeout(devicePort, (short) wait*1000);
  TickleSWSR(); // could have been a long wait
  if (str[0]<0) {
    DBG1(flogf("\n\t|GetStringWait() timeout");)
    str[0]=0;
    return 0;
  }
  for (len=1; len<BUFSZ; len++) {
    str[len] = TURxGetByteWithTimeout(devicePort, charDelay);
    if (str[len]<0) {
      str[len]=0;
      break;
    }
  }
  return len;
}

void GetResponse(TUPort port, char *in, int wait, char *out) {
  // flush, output, readline(echo), GetStrW; return out
  TURxFlush(port);
  // write string - put chars until \0
  while (*in) { TUTxPutByte(port, *in++, true); }
  TUTxPutByte(port, '\r', true);
  // consume echo - 
