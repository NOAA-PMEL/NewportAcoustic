// if ACOUSTICMODEM defined
#define AMODEMPWR 21
#define AMODEMRX 33
#define AMODEMTX 35

// if IRIDGPSMODEM defined
#define IRDGPSPWR 23 // Iridium Power pin (1=ON, 0=OFF)
#define IRDGPSCOM 22
#define IRDGPSRX 32
#define IRDGPSTX 31

#define WISPRONE 29
#define WISPRTWO 30
#define WISPRTHREE 24
#define WISPRFOUR 25

#define WISPR_PWR_ON 37
#define WISPR_PWR_OFF 42

// TUPort Baud Rates
#define BAUD 9600L
#define SGBAUD 4800L
#define NIGKBAUD 4800L
#define AMODEMBAUD 9600L
#define IRIDBAUD 19200L

void OpenTUPort(char *, bool);

// Global Variables defined
extern TUPort *AModemPort;
extern TUPort *PAMPort;
extern TUPort *CTDPort;
extern TUPort *SGPort;
