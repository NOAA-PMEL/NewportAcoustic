Alex Turpin Dec 8 2015
AUH MPC Program Rev 3.0

This new revision takes advantage of the new MPC Common Program files 3.0+
	The change in these core files for the MPC is using the Posix open fuction for file i/o rather than the fopen file pointers. 
		Benefits:
		-Write and read times are an order of magnitude faster
		-Does not crash anymore
		
	