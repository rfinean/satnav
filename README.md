# satnav #
======

## [Satellite navigation](wiki: Satellite_navigation) position-fixing ##

How to fix your position from navigation satellite data.

This program (from the late 80s) pre-dates [GPS](wiki: Global_Positioning_System)
and used the US Navy's [Transit NavSat](wiki: Transit_(satellite)).
Only one satellite would be visible at a time and it transmitted an almanac message
every two minutes at a very precise time, position and radio frequency.
A UHF receiver had to
* demodulate the message data
* measure the Doppler shift for each message
* input this to an MS-DOS PC on a serial port.

This program uses each message it receives to improve its position fix.
To enable initial acquisition of a position fix from just one satellite,
rough values for latitude, longitude, altitude and time are required
and it is assumed that we are moving relatively slowly (compared to the satellites' 27000km/h).

With GPS we now have many more satellites orbiting and usually 3 or more simultaneously visible
so the process of resolving the accuray of a position fix is much quicker and less reliant on
initial assumptions but the basic process of solving multiple simultaneous equations is the same.