/******************************************************************************\
**	cfxmain.c				Persistor and PicoDOS starter C file  
**	
*****************************************************************************
**	
**	
*****************************************************************************
**	
**	
\******************************************************************************/

#include	<cfxbios.h>		// Persistor BIOS and I/O Definitions
#include	<cfxpico.h>		// Persistor PicoDOS Definitions

#include	<assert.h>
#include	<ctype.h>
#include	<errno.h>
#include	<float.h>
#include	<limits.h>
#include	<locale.h>
#include	<math.h>
#include	<setjmp.h>
#include	<signal.h>
#include	<stdarg.h>
#include	<stddef.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include	<time.h>

#include	<dirent.h>		// PicoDOS POSIX-like Directory Access Defines
#include	<dosdrive.h>	// PicoDOS DOS Drive and Directory Definitions
#include	<fcntl.h>		// PicoDOS POSIX-like File Access Definitions
#include	<stat.h>		// PicoDOS POSIX-like File Status Definitions
#include	<termios.h>		// PicoDOS POSIX-like Terminal I/O Definitions
#include	<unistd.h>		// PicoDOS POSIX-like UNIX Function Definitions


/******************************************************************************\
**	main
\******************************************************************************/
int main(int argc, char **argv)
	{
	short	i;
	short	result = 0;		// no errors so far

	// Identify the progam and build
	printf("\nProgram: %s: %s %s \n", __FILE__, __DATE__, __TIME__);
	// Identify the device and its firmware
	printf("Persistor CF%d SN:%ld   BIOS:%d.%02d   PicoDOS:%d.%02d\n", CFX,
		BIOSGVT.CFxSerNum, BIOSGVT.BIOSVersion, BIOSGVT.BIOSRelease, 
		BIOSGVT.PICOVersion, BIOSGVT.PICORelease);
	// Identify the arguments
	printf("\n%d Arguments:\n", argc);
	for (i = 0; i < argc; i++)
		printf("  argv[%d] = \"%s\"\n", i, argv[i]);
	
	
	
	
	
	
// Various exit strategies
//	BIOSReset();			// full hardware reset
//	BIOSResetToPBM();		// full reset, but stay in Pesistor Boot Monitor
//	BIOSResetToPicoDOS();	// full reset, but jump to 0xE10000 (PicoDOS)
	return result;

	}	//____ main() ____//

