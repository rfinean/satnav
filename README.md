#satnav

##[Satellite navigation](http://www.polstargps.com/en/) position-fixing

How to fix your position from navigation satellite data.

This program (from the late 80s) pre-dates [GPS](http://en.wikipedia.org/wiki/Global_Positioning_System)
and used the US Navy's older [Transit NavSat](http://en.wikipedia.org/wiki/Transit_%28satellite%29).
Only one satellite would be visible at a time and it transmitted an almanac message
every two minutes at a very precise time, position and UHF radio frequency.
A UHF receiver had to
* demodulate the message data
* measure the Doppler frequency shift for each message
* input this to an MS-DOS PC on a serial port.

This program uses each message it receives to improve its position fix.
To enable initial acquisition of a position fix from just one satellite,
rough values for latitude, longitude, altitude and time are required
and it is assumed that we are moving very slowly (compared to the satellite's 27000km/h).

With GPS we now have many more satellites orbiting and usually 3 or more simultaneously visible
so the process of resolving the accuray of a position fix is much quicker and less reliant on
initial assumptions.
Still the basic process of solving multiple simultaneous equations is the same 20 years later in GPS
systems like Finean/PolNav, TomTom, Route66, Mio, etc...