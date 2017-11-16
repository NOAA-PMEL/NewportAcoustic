// A-D SYSTEM CURRENT AND VOLTAGE LOGGING
// Changing parameters here will cause problems to program timing.
#include <ADExamples.h>

#define FCHAN 0   // first channel
#define NCHAN 2   // number of channels, just accumulating current in buffers.
#define PITRATE 1 // 1=51ms cycle, 2=102ms,etc..... @ 100us. 250=25ms.
#define PITPERIOD .051 // represents 51ms

void Setup_Acquisition(ushort);
void AD_Write(ushort *);
void AD_Log(void);
ushort GetSystemTimeInt();
bool AD_Check();
ushort Setup_ADS(bool, long, ushort);
float Power_Monitor(ulong, int, ulong *);
void Delay_AD_Log(short Sec);
float Get_Voltage();
float Voltage_Now();
void Open_Avg_File(long);
ushort Return_ADSTIME();
int Get_ADCounter();
bool ADS_Status();
void GetPowerSettings();
void Reset_ADCounter();
void ADSFileName(long);
