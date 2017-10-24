#include <cfxbios.h> // Persistor BIOS and I/O Definitions
#include <cfxpico.h> // Persistor PicoDOS Definitions
#include <MPC_Global.h>
#include <PLATFORM.h>
#include <ADS.h>
#include <Settings.h>
#include <Winch.h>
#include <GPSIRID.h>

#include <assert.h>
#include <ctype.h>
#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <errno.h>
#include <fcntl.h> // PicoDOS POSIX-like File Access Definitions
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
#include <unistd.h> // PicoDOS POSIX-like UNIX Function Definitions

#define BUFSZ 1024

