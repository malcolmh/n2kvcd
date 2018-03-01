# n2kvcd
Daemon to connect NGT-1/YDNU to SocketCAN

Usage: n2kvcd <path_to_NGT-1/YDNU_device> <name_of_CAN_interface> [D]

e.g. n2kvcd /dev/ttyUSB1 can0

Optional 'D' as third argument to run in foreground rather than as a daemon.

NB: The Yacht Devices YDNU unconditionally forwards all PGNs, whereas the Actisense NGT-1
requires each PGN to be enabled. This can be done using "NMEA Reader" from Actisense - a Windows program (that will run in Wine)

Building & running requires the shared library libmnd to be installed. See: github.com/victronenergy/libmnd

Building: cc -o n2kvcd n2kvcd.c -lmnd