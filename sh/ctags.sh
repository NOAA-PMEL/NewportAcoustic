#!/bin/bash
# make lara tags file

files="MPC/ADS/ADS.c LARA/LARA.c MPC/CTD/CTD.c MPC/GPSIRID/GPSIRID.c MPC/WISPR/WISPR.c MPC/WINCH/Winch.c MPC/MPC_SETTINGS/Settings.c MPC/MPC_Global/MPC.c MPC/CTD/CTD.h MPC/GPSIRID/GPSIRID.h MPC/WISPR/WISPR.h MPC/WINCH/Winch.h MPC/MPC_SETTINGS/Settings.h LARA/PLATFORM.h MPC/MPC_Global/MPC_Global.h MPC/ADS/ADS.h"
ctags -f .tags $files
