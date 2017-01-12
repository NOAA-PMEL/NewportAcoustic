/*
Bluetooth Module
*/
#include <PLATFORM.h>


typedef struct 
{
short ON;       //t logging change in battery capacity
}   BluetoothParameters;

#ifdef RAOB
   #define	BTPWR	   23  	//BT Power Pin  (1=ON, 0=OFF)
   #define  BTCOM    22
   #define  BTRX     32
   #define  BTTX     31
   #define  BTBAUD   9600L
#endif

#ifdef RAOT               //Using PAM Port? PAM 4. Requires the ON/OFF to 5
   #define  BTPWR    25    //pin set for transceiver
   #define  BTPWR2   37    //turn on dc-dc.
   #define  BTCOM    24    //pin set
   #define  BTRX     28    //rx
   #define  BTTX     27    //tx
   #define  BTBAUD   9600L //any higher baud?
#endif

#ifdef AUH
   #define	BTPWR	   23  	//BT Power Pin  (1=ON, 0=OFF)
   #define  BTCOM    22
   #define  BTRX     32
   #define  BTTX     31
   #define  BTBAUD   9600L
#endif

extern TUPort *BTPort;

int  Bluetooth_LowPower();
void  Bluetooth_Power(bool);
void  Bluetooth_Test();
void Bluetooth_Interface();
int   Bluetooth_Data();
void Bluetooth_Sleep();
void GetBTSettings();
void BT_INIT();
void Bluetooth_LPWait();


extern bool BT_On;
extern bool BT_LP;