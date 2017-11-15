void DelayTX(int ch);
int ReadLine(TUPort *port, char *in);
void WriteLine(TUPort *port, char *out);
int GetStringWait(TUPort port, int wait, char *in);
int GetResponse(TUPort port, char *out, int wait, char *in);
