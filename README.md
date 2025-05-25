# astroshell
Optimized Arduino Code for controlling of my AstroShell Dome

This arduino code is kind of branched from the producer of the AstroShell Telescope Domes.
As I was not supersatisfied with some aspects I enhanced it to my likings. 
So feel free to use it too. Just one important hint: When my dome was installed the installations team accidentally 
swapped the limit-endpoint-switches so my dome reported open while it was actually closed and vice versa.
Because I didn't wanted to solder all the cablings around I simply asked AstroShell for the code to make the changes there,
thus avoiding re-cabling.

While I was initially just solving my little problem I ran into some further ideas which I implemented into this code now:

1. A enhanced UI webfront-end which works better for iphones as well as tablets (imho)
2. Disabled the cloudsensor evaluation because it simply doesn't make sense at all (imho)
3. Disabled the "vibration motor option" (for whatever it was for). No clue.
4. Added hardcoded ethernet pinging of my lunatic cloudsensor to automatically close the dome in case the clouddetector is not up and running and thus might not be able to detect rain
5. Added a new command URL Command $S to get a quick response if the Dome is open or closed (to use it in other scripts or tools e.g. lunatic cloudwatcher solo)
6. optimized memory usage of arduino
7. disabled Serial debugging Outputs as it's conflicting with Data Input Ports 0,1 from the arduino which are used for the astroshell end-point limit switches.
8. Little stabilty tweaks and checks


The motor engine logic was not touched at all.

If you like to use it, feel free to. But on your own risk of course.
It's always a good idea to try out things while the dome is half open (not fully closed or fully open) so you have a chance you can react in case of....

Cheers
Joerg
