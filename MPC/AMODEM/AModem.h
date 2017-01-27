// DO WE NEED A PARAMETER STRUCTURE FOR THE USE OF LINKQUEST ACOUSTIC MODEM?
typedef struct {
  short MAXUPL;  // in bytes, 10000? 20000?
  short BLKSIZE; // 256/512/1024/2048/4096
  short OFFSET;  // seconds for which to offset the GPS Rx Time

} AMODEMParameters;

void AModemSend(char *);
void AModemResend();
short AModem_Data();
int AModemStream(int);
void GetAMODEMSettings();
void OpenTUPort_AModem(bool);
void AModem_SetPower(bool);
#define AMODEMBAUD 9600L

// Global Variables defined
extern TUPort *AModemPort;

// if ACOUSTICMODEM defined
#define AMODEMPWR 21
#define AMODEMRX 33
#define AMODEMTX 35

#define MAX_RESENDS 4