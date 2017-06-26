#include <cfxbios.h> 
#include <cfxpico.h> 
#include <assert.h>
#include <ctype.h>
#include <errno.h>

#include <limits.h>
#include <locale.h>
#include <math.h>
#include <setjmp.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <dirent.h>   // PicoDOS POSIX-like Directory Access Defines
#include <dosdrive.h> // PicoDOS DOS Drive and Directory Definitions
#include <fcntl.h>    // PicoDOS POSIX-like File Access Definitions
#include <stat.h>     // PicoDOS POSIX-like File Status Definitions
#include <termios.h>  // PicoDOS POSIX-like Terminal I/O Definitions
#include <unistd.h>   // PicoDOS POSIX-like UNIX Function Definitions

void main() {
  int i;
  char *n = "c:12345678.txt";
  i = open(n, O_APPEND | O_CREAT | O_RDWR);
  if (i <= 0) 
    uprintf("ERROR  | %s open errno: %d", n, errno);
  else
    uprintf("success\n");

  i = open(n, O_APPEND | O_CREAT | O_RDWR);
  if (i <= 0) 
    uprintf("ERROR  | %s open errno: %d", n, errno);
  else
    uprintf("success\n");
}
