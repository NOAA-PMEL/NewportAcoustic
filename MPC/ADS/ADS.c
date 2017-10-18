/*****************************************************************************************************
ADS Version for Seaglider version 3.0

Updates from ADS.c version 2.0
   -Working to make file open more efficient.
   -Received many errno=0 when fopen return NULL. No idea why. waiting to hear
back from JG 12.2.2015

Updated from ADS.c version 1.5
   -with more efficient storing and calculating of current and voltage.

Version 1.5:
Storing "shorts" in circular array, averaging once every 30 seconds to another
array of floating point values,
after 5 minutes averaging those 20 different 8-byte floating point values and
writing to file.

Version 2.0:
Summing "shorts" for giving sampling interval which lasts
#seconds=(2^#bits*PITRATE*PITINT)
Here we can average a large number of samples at a quick rate due to bit shift
division. At the end of every
average, we write the three different "ushort" values (current, voltage, and
time of sampling) to the power file.
Only when power monitor is called does the power file sum its "ushort" values
and divided again ( non-bit shifted integer division)
before it is converted to a floating point value with "CFxADRawToVolts(...)"
This makes for a very fast and efficient power logging
process.

   A) Need to check which BITSHIFT size leads to overflow.
      1) BITSHIFT of 10 results in 25.6 secon buffers, 11: 51.2, 12: 102.4 etc.
16? Too big?

   B) No required Array allocation or "data2" for knowing power needs to be
averaged.
      2) Just checking to see if "data" is true will spark averaging.

   C) ADSTIME will be saved as a "ushort" and can be divided by "10.0" to
transfer into the real power write time.



****************************************************************************************************/

#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions

#include <ADExamples.h>
#include <ADS.h>
#include <MPC_Global.h>
#include <PLATFORM.h>
#include <Settings.h>
#include <assert.h>
#include <cfxad.h>
#include <ctype.h>
#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <float.h>
#include <float.h>
#include <limits.h>
#include <locale.h>
#include <math.h>
#include <setjmp.h>
#include <signal.h>
#include <stat.h> // PicoDOS POSIX-like File Status Definitions
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h> // PicoDOS POSIX-like Terminal I/O Definitions
#include <time.h>
#include <unistd.h>

void Setup_Acquisition(ushort);
void AD_Write(ushort *);
void AD_Log(void);
ushort GetSystemTimeInt();

IEV_C_PROTO(ADTimingRuptHandler);
IEV_C_PROTO(ADSamplingRuptHandler);

// Total Number of samples per ADSTIME. Defined by BITSHIFT and sampling
// frequency
ushort SAMPLES;
ushort BitShift;

// File to save all values for Power Logging;
CFxAD *ad, adbuf;
bool ADSOn;

// ADSample is ptr returned by CFxADQueueToArray(), alloc'd by magic
short *ADSample;
long TotalPower[2] = {0, 0};

// Time duration of AD Sampling interval in Deciseconds
ushort ADSTIME = 0;

// Parameters Summed for calculation of ADS
long VOLTAGE; // Summation of channel 1 from QSPI sampling function
long CURRENT; // Summation of channel 0 from QSPI...
long Nsamps;  // Incremented upon each sample. Once Nsamps equals SAMPLES
             // (2^BITSHIFT)

ushort minvoltage = 0;
ushort maxcurrent = 0;

float Voltage = 0.0;

bool data;
int ADCounter = 0;
PowerParameters ADS;
static char ADAvgFileName[] = "c:00000000.pwr";

int ADSFileHandle;

ushort Return_ADSTIME() { return ADSTIME; }
bool AD_Check() {
  if (data == true && ADSOn) {
    TickleSWSR();
    AD_Log();
    return true;
  } else
    return false;
}
bool ADS_Status() { return ADSOn; }
int Get_ADCounter() { return ADCounter; }
void Reset_ADCounter() { ADCounter = 0; }
void ADSFileName(long counter) {
  sprintf(&ADAvgFileName[2], "%08ld.pwr", counter);
}
/**********************************************************\
** Void SetUpADS()
** Set up AD to sample voltage and current usage.
** Name the file name with 8-digit numeral as a counter
** No need to calculate current upon Power off
\**********************************************************/
ushort Setup_ADS(bool ads_on, long filecounter, ushort val) {

  ADSOn = ads_on;
  if (ADSOn) {
    BitShift = val;
    Open_Avg_File(filecounter);
    flogf("\n%s|ADS(%s)", Time(NULL), ADAvgFileName);
    Setup_Acquisition(BitShift);

  } else if (!ADSOn) {
    PITSet100usPeriod(PITOff); // Stop sampling
    PITRemoveChore(0);
    Delayms(10);
    data = true;
    ADCounter = 0;
  }
  return ADSTIME;
} //_____ void SetUpADS() _____//
  /*********************************************************\
  ** Void OpenAvgFile()
  **********************************************************/
void Open_Avg_File(long counter) {

  sprintf(&ADAvgFileName[2], "%08ld.pwr", counter);
  Delayms(25);
  ADSFileHandle = creat(ADAvgFileName, 0);
  if (ADSFileHandle <= 0) {
    flogf("\nCouldn't Open %s file, errno%d", ADAvgFileName, errno);
    return;
  }
  if (close(ADSFileHandle) != 0)
    flogf("\nERROR  |Open_Avg_File() %s Close error: %d", ADAvgFileName, errno);

  Delayms(10);

} //_____ void OpenAvgFile() _____//
/************************************************\
** Setup Acquisition
\************************************************/
void Setup_Acquisition(ushort BitShift) {
  // global ADSample
  double vref = VREF;
  bool uni = true; // true for unipolar, false for bipolar
  bool sgl = true; // true for single-ended, false for differential

  PITInit(6);               // PIT Priority Level
  PITSet51msPeriod(PITOff); // Assert Both PIT Timers off at this point.
  PITSet100usPeriod(PITOff);

  Delayms(20);

  TotalPower[0] = 0L;
  TotalPower[1] = 0L;

  // Initialize AD Slot and Lock
  ad = CFxADInit(&adbuf, ADSLOT, ADInitFunction);
  if (!CFxADLock(ad)) {
    flogf("\nCouldn't lock and own A-D with QSPI\n");
    return;
  } // Lock in the ADDataBuf

  // for first dummy reading to set things up
  CFxADSampleBlock(ad, FCHAN, NCHAN, &ADSamplingRuptHandler, uni, sgl, false);

  // Make sure PIT is off
  PITSet51msPeriod(PITOff); // disable timer
  PITRemoveChore(0);        // get rid of all current chores?

  IEVInsertCFunct(&ADTimingRuptHandler, pitVector); // replacement fast routine

  // Current and voltage samples per ADSTIME interval
  SAMPLES = (ushort)pow(2, BitShift);
  ADSTIME = (10 * SAMPLES * (PITRATE * PITPERIOD));

  DBG(flogf("\n\t|Writing every %4.1fSeconds", ADSTIME / 10.0);)
  Delayms(1);

  // Set the Rate and start the PIT
  Nsamps = 0;
  CURRENT = 0;
  VOLTAGE = 0;
  minvoltage = 0;
  maxcurrent = 0;

  PITSet51msPeriod(PITRATE);
  data = false;

} //_____ SetupAcquistion() _____//
/******************************************************************************\
**	AD_Log
** 1)Comes here from Main() if either data or data2 is true
** 2)Checks both booleans and writes correct side of AD Buffer to file (doesn't
acutally
** save file, rather averages each side of buffer and stores one averaged value
for set buffer period)
** 3)Once both data booleans are false set PutInSleepMode=true
\******************************************************************************/
void AD_Log(void) {

  ushort AveragedEnergy[3] = {0, 0, 0};
  float current = 0.0;

  ADCounter++;

  Delayms(5);

  if (data == true) {
    AveragedEnergy[0] = (ushort)(TotalPower[0] >> BitShift); // Voltage
    AveragedEnergy[1] = (ushort)(TotalPower[1] >> BitShift); // Current
    AveragedEnergy[2] = ADSTIME;                             // Time
    data = false;

  } else if (data == true && ADSOn == false) {
    AveragedEnergy[0] = (ushort)(TotalPower[0] >> BitShift);
    AveragedEnergy[1] = (ushort)(TotalPower[1] >> BitShift);
    AveragedEnergy[2] = ADSTIME;
  }

  current = CFxADRawToVolts(ad, AveragedEnergy[0], VREF, true);
  Voltage = CFxADRawToVolts(ad, AveragedEnergy[1], VREF, true) * 100;
  flogf("\n\t|POWER: %5.3fA, %5.2fV", current, Voltage);

  AD_Write(AveragedEnergy);
} //_____ ADLog() _____//
/**********************************************************\
** Get_Voltage()
\**********************************************************/
float Get_Voltage() { return Voltage; } //____ Get_Voltage ____//
/*******************************************************************************\
** Voltage Now()
\*******************************************************************************/
float Voltage_Now() {
  float volts = 0.0;

  QPBRepeatAsync(); // starts the QSPI running with previous parameters

  ADSample = (short *)CFxADQueueToArray(ad, (void *)QRR, 2);
  volts = CFxADRawToVolts(ad, (ushort)ADSample[1], VREF, true) * 100;
  Voltage = volts;
  return volts;

} //____ Voltage_Now() ____//
/**********************************************************\
** AD Write
** Open file of Current averages, go to end of file and grab last averaged
reading.
** This function will increment the variable ADCounter==FWT ~5minutes
**
\**********************************************************/
void AD_Write(ushort *AveragedEnergy) {

  CLK(start_clock = clock();)
  ADSFileHandle = open(ADAvgFileName, O_RDWR | O_BINARY | O_APPEND);
  if (ADSFileHandle <= 0) {
    flogf("\nERROR|AD_Write() %s open fail. errno: %d", ADAvgFileName, errno);
    return;
  }
  // DBG(   else      flogf("\n\t|AD_Write() %s opened", ADAvgFileName);)

  CLK(stop_clock = clock();
      print_clock_cycle_count(start_clock, stop_clock, "AD_Write: open");)

  CLK(start_clock = clock();)

  write(ADSFileHandle, AveragedEnergy, 3 * sizeof(ushort));
  Delayms(25);
  CLK(stop_clock = clock();
      print_clock_cycle_count(start_clock, stop_clock, "AD_Write: write");)

  if (!ADSOn) // SetupAD(false) from power monitor
    return;
  if (close(ADSFileHandle) < 0)
    flogf("\nERROR  |AD_Write() %s Close error: %d", ADAvgFileName, errno);
  // DBG(   else      flogf("\n\t|AD_Write() %s Closed", ADAvgFileName);)
 
  Delayms(10);

} //_____ AD_Write() _____//
/******************************************************************************\
** PowerMonitor
** This function is called when the WriteInterval (WRTINT) is met.
** With a FWT for the ADS of 32seconds and a WRTINT of ~60 minutes (really 64
minutes)
\******************************************************************************/
float Power_Monitor(ulong totaltime, int filehandle, ulong *LoggingTime) {
  struct stat fileinfo;
  ulong DataCount = 0;
  ulong filelength = 0;
  ulong TotalAmp = 0;
  ulong TotalVolts = 0;
  ulong TotalTime = 0;
  ulong calculatedtime = 0L;
  ushort energy[3] = {0, 0, 0};
  float MaxCurrent = 0.0, MinVoltage = 0.0;
  float kjoules;
  float floater = 0.0;
  int byteswritten;
  float voltage = 0.0, amps = 0.0;

  // Normal enterance to Power_Monitor
  if (totaltime != 0) {
    Setup_ADS(false, NULL, NULL);
    if (ADSTIME < 1)
      ADSTIME = 1044;
    ADSTIME = ((10 * totaltime) % ADSTIME); // Last AD Power Buffer size
    AD_Log();
    // opens adsfh
  }
  // Coming in after reboot // Setup_ADS(false), AD_Log also opens .pwr file
  else {
    ADSFileHandle = open(ADAvgFileName, O_RDWR | O_BINARY | O_APPEND);
    ad = CFxADInit(&adbuf, ADSLOT, ADInitFunction);
    if (!CFxADLock(ad)) {
      flogf("\nCouldn't lock and own A-D with QSPI\n");
      // close
      return 0.0;
    }
  }
  // adsfh is open

  flogf("\n\t|POWERMonitor(%s)", ADAvgFileName);

  // Reset Global Write Buffer
  memset(WriteBuffer, 0, BUFSZ);

  // Get file status
  stat(ADAvgFileName, &fileinfo);
  filelength = fileinfo.st_size;

  // if file unwritten to
  if (filelength < 6) {
    if (close(ADSFileHandle) < 0)
      flogf("\nERROR  |PowerMonitor: File Close error: %d", errno);
    DBG(else flogf("\n\t|PowerMonitor: ADSFile Closed");)
    return 0.0;
  }

  if (ADSFileHandle > 0) {
    // we maybe just wrote into file, so seek back to start
    lseek(ADSFileHandle, 0, SEEK_SET);
    // 6 is the number of bytes for the values of current, voltage, time.
    filelength = filelength / 6; 

    // Get the number of times file has been written to
    while (DataCount < filelength) {
      byteswritten = read(ADSFileHandle, energy, 3 * sizeof(ushort));
      TotalAmp += energy[0];
      TotalVolts += energy[1];
      TotalTime += (ulong)energy[2];
      DataCount++;
    }

    if (close(ADSFileHandle) < 0)
      flogf("\nERROR  |PowerMonitor: File Close error: %d", errno);
    DBG(else flogf("\n\t|PowerMonitor: ADSFile Closed");)

    Delayms(25);
    if (DataCount != 0) {
      energy[0] = (ushort)(TotalAmp / DataCount);
      energy[1] = (ushort)(TotalVolts / DataCount);
    }

    amps = CFxADRawToVolts(ad, energy[0], VREF, true);
    Delayms(10);
    voltage = CFxADRawToVolts(ad, energy[1], VREF, true) * 100;
    Delayms(10);
    TotalTime = TotalTime / 10;
    kjoules = (amps * voltage * TotalTime) / 1000.0;
  }
  MaxCurrent = CFxADRawToVolts(ad, maxcurrent, VREF, true);
  MinVoltage = CFxADRawToVolts(ad, minvoltage, VREF, true) * 100;
  *LoggingTime = TotalTime;

  sprintf(WriteBuffer, "\n---POWER---\nTime: %lu\nEnergy:%.2fkJ\nAvg "
                       "Volt:%.2fV\nMin Volt:%.2fV\nMax Current:%.3fA\n",
          TotalTime, kjoules, voltage, MinVoltage, MaxCurrent);

  // Adjust and record Battery Capacity
  if (ADS.BATLOG) {
    flogf("\n\t|Current Bat Cap: %s", ADS.BATCAP);
    floater = atof(ADS.BATCAP);
    floater -= (kjoules * POWERERROR);
    sprintf(ADS.BATCAP, "%7.2f", floater);
    VEEStoreStr(BATTERYCAPACITY_NAME, ADS.BATCAP);
    flogf(", New Bat Cap: %s", ADS.BATCAP);
    sprintf(WriteBuffer, "\n---POWER---\nTime: %lu\nEnergy:%.2fkJ\nAvg "
                         "Volt:%.2fV\nMin Volt:%.2fV\nMax "
                         "Current:%.3fA\nBattery Capacity:%.2fkJ\n",
            TotalTime, kjoules, voltage, MinVoltage, MaxCurrent, floater);
  } else
    sprintf(WriteBuffer, "\n---POWER---\nTime: %lu\nEnergy:%.2fkJ\nAvg "
                         "Volt:%.2fV\nMin Volt:%.2fV\nMax Current:%.3fA\n",
            TotalTime, kjoules, voltage, MinVoltage, MaxCurrent);

  DBG(flogf("\n%s", WriteBuffer); cdrain(); coflush();)
  if (filehandle > 0)
    byteswritten = write(filehandle, WriteBuffer, strlen(WriteBuffer));

  Delayms(150);

  return floater;

} //_____ PowerMonitor() _____//
/******************************************************************************\
**	ADTimingRuptHandler Chore		Initiate conversion
** 1) Makes sure QSM is running and repeats previous synchronization
** 2) While ADRawHead is Less than the total buffer size, take sample and store
in buffer
** 3) If equal to half, make sure on next system wake, to catch and write data
** 4)
\******************************************************************************/
IEV_C_FUNCT(ADTimingRuptHandler) // implied (IEVStack *ievstack:__a0) parameter
{

#pragma unused(ievstack)

  QSMRun();
  QPBRepeatAsync(); // starts the QSPI running with previous parameters

  ADSample = (short *)CFxADQueueToArray(
      ad, (void *)QRR, 2); // QSPI driver function to sample '2' channels from
                           // 'ad' and save into ushort array 'ADSample'

  CURRENT += (ulong)(ADSample[0]); // Here we sum the voltage and current
  VOLTAGE += (ulong)(ADSample[1]);
  Nsamps++;

  if (ADSample[0] > maxcurrent) {
    maxcurrent = ADSample[0];
    minvoltage = ADSample[1];
  }

  if (Nsamps == SAMPLES) {
    TotalPower[0] = CURRENT;
    TotalPower[1] = VOLTAGE;
    VOLTAGE = 0;
    CURRENT = 0;
    data = true;
    Nsamps = 0;
  }

} //____ ADTimingRuptHandler() ____//
/******************************************************************************\
**	ADSamplingRuptHandler		Move raw QPSI data to main buffer
\******************************************************************************/
IEV_C_FUNCT(
    ADSamplingRuptHandler) // implied (IEVStack *ievstack:__a0) parameter
{
#pragma unused(ievstack)

  CPUWriteInterruptMask(SIM_PITR_DEF_IPL);
  QPBClearInterrupt();

} //____ ADSamplingRuptHandler() ____//

/*
**
*/
void GetPowerSettings() {

  char *p;

  //"v" depending on battery type 11.5-14
  p = VEEFetchData(MINSYSVOLT_NAME).str;
  strncpy(ADS.MINVOLT, p ? p : MINSYSVOLT_DEFAULT, sizeof(ADS.MINVOLT));
  //	MPC.MINVOLT =p;
  DBG(uprintf("MINVOLT=%s (%s)\n", ADS.MINVOLT, p ? "vee" : "def"); cdrain();)
  if (MIN_BATTERY_VOLTAGE > atof(ADS.MINVOLT)) {
    sprintf(ADS.MINVOLT, "%5.2f", MIN_BATTERY_VOLTAGE);
    VEEStoreStr(MINSYSVOLT_NAME, ADS.MINVOLT);
  }

  p = VEEFetchData(BATTERYCAPACITY_NAME).str;
  strncpy(ADS.BATCAP, p ? p : BATTERYCAPACITY_DEFAULT, sizeof(ADS.BATCAP));
  DBG(uprintf("BATCAP=%s (%s)\n", ADS.BATCAP, p ? "vee" : "def"); cdrain();)

  //"t" 0 or 1
  p = VEEFetchData(BATTERYLOGGER_NAME).str;
  ADS.BATLOG = atoi(p ? p : BATTERYLOGGER_DEFAULT);
  DBG(uprintf("BATLOG=%u (%s)\n", ADS.BATLOG, p ? "vee" : "def"); cdrain();)
}
