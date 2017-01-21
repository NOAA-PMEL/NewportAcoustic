Seaglider Program Version 3.0. June 17th, 2016, Alex Turpin
For use with SG607 at Catalina Island for Selene Fregosi/Dave Mellinger acoustic study 
This program takes use of the MPC Common Program File Version 3.1

This program is built upon the Multiport Controller developed by Haru Matsumoto at Oregon State University.

The sole purpose is to control a passive acoustic monitoring board by the name of WISPR: Wide-band intelligent
signal processor and recorder. The MPC program collects incoming data from the Seaglider's logger port (serial commands can be found in the kongsberg "Logdev User Guide") as if our CF2 microcontroller is acting as a slave device to the master Seaglider.

The file found on the Seaglider that controls the logger is the configuration file: ".cnf" called "pam.cnf"
In this file:

name=PAM
prefix=pa
timeout=2000
baud=4800
warmup=2000
powerup-timeout=2000
voltage=10
current=0.100
cmdprefix=$PA_
prompt=>
datatype=u
start=%i%b%r+%3+%3+%3%X***%r%n
stop=%b%r+%3+%3+%3+++STOP***%r%n
status=%b%r+%3+%3+++%D,%d***%r%n
download=%b%r+%3+%3+++cat,%d,%m***%r%n
script-x=system.cfg
clock-sync=gps2
clock-set="%r+%3+%3+%3++GPS=%{%m/%d/%Y,%H:%M:%S**}%r"

a status is sent from seaglider to CF2 every time a Seaglider sensor reading is taken. For example: depth on the seaglider is sampled every 5 - 15 seconds depending on depth in the water column. This depth info is passed onto the CF2 in the format shown above on the status line. %D represents the depth, and %d represents the dive number. The other symbols can be referenced in the logdev file. 

The Seaglider is considered surfaced when parameters the depth read from the seaglider is less than SEAG.ONDEPTH while descending and greater than SEAG.OFFDEPTH when ascending. While not surfaced, the MPC will control the WISPR board to record acoustic data at a set duty cycle, specific gain, and with a predetermined detection algorithm. 


These VEEPROM parameters can be changed through uploading a new script "script-x" named "system.cfg" to the seaglider when it downloads new flight parameters at the end of every dive. This script, or file, must be in the format:  $$$PAM()***
Where new parameters to be updated are placed inside the parethesis. Example:   $$$PAM(G0E25O50)***
Where G is the gain for the WISPR, E is the SEAG.OFFDEPTH, O is the SEAG.ONDEPTH.
These characters are not case sensitive and include all the follow:
g, d, c, e, o, i, v, s, u 
gain, detection number, duty cycle, off depth, on depth, detection interval, minimum voltage, startups, and maximum upload size respectively. The program has lower and upper bounds for what value can be saved to each VEEPROM parameter. For example: Gain can only be an integer from 0 to 3 inclusive. Whereas v can be a floating point from 10.0 to some upper value. 

These limits are usually defined in the platform.h header file. This file is specific to all MPC related CF2 programs yet retains the same name "platform.h" for all variations of the MPC common program files to make modularity of code easy. 

The SG3.0 MPC program tracks all incoming depth values received from the glider and averages its movement vertically in the water column. Depending on it's vertical velocity (positive for descending and negative for ascending) SG_STATUS changes. 

The value held in SG_STATUS (not a VEEPROM parameter) is dependent upon many variables more than just vertical velocity. It also understands if it's above or below the PAM start depth or nearing the surface. Depending on this status, the glider will decide how it deciphers incoming Seaglider Serial data. For example, to upload a file to the seaglider (which in turn will be uploaded through Iridium to the communications base-station) the glider needs to receive a negative averaged vertical velocity and be less than 20.0meters of depth to prepare the upload file. It then waits to receive the upload string shown above in the pam.cnf file. At this point, the MPC has two chances to pass it's file to the glider.  
  