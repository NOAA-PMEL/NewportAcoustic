#include <cfxpico.h> // Persistor PicoDOS Definitions

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


char *stringin;
void main() {

  cprintf("hello world\n");

  /*
    strtok(stringin, "# ");
    split_temp = strtok(NULL, " ");
    if (split_temp == NULL) {
      cprintf("\nNo Commas in CTD Data");
    }
    strtok(NULL, " "); // erase cond.
    split_pres = strtok(NULL, " ");
    split_SAL = strtok(NULL, " ");
    split_date = strtok(NULL, ",");

  */

  stringin = (char *)calloc(256, sizeof(char));

  strcpy(stringin,
         "# 20.6538,  0.01145,    0.217,   0.0622, 01 Aug 2016 12:16:50");
  cprintf(".. '%s'\n", stringin);
  cprintf(".. \n");
  cprintf(":: '%s'\n", strtok(stringin, "# "));
  cprintf(":: '%s'\n", strtok(NULL, " "));
  cprintf(":: '%s'\n", strtok(NULL, " "));
  cprintf(":: '%s'\n", strtok(NULL, " "));
  cprintf(":: '%s'\n", strtok(NULL, " "));
  cprintf(":: '%s'\n", strtok(NULL, ","));
  cprintf(".. \n");
}
