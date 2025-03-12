# Release notes

## revision MB07

### Major improvements

Fixed bugs in rotation of radar view.
Updated ogn.cdb in March, 2025.


## revision MB05

### Major improvements

Switched from SQLITE to CDB format for aircrafts database (only OGN suppported).
Data folder includes ogn.cdb from January 28, 2023.
Fixed bug in voice settings.


## revision MB03

### Major improvements

Corrected handling of this aircraft's time stamp, which affects voice notifications.


## revision MB02

### Major improvements

Corrected Bluetooth code, based on SoftRF v1.2, now works.


## revision MB01b

### Major improvements

#### ESP32

This version, by Moshe Braner, Dec. 22, 2022

Skyview gives voice notifications about nearby traffic.  The original version behaves as follows: as soon as another aircraft (with a compatible device transmitting) gets closer than 10 km from this aircraft, a voice notification announces it's relative position (bearing, distance and altitude).  After that, no further notifications are given about the same aircraft, unless it travels out of range and later comes back.

Thus, you may get an audio notification about an aircraft that is sort-of-close but not a danger yet, then you'll get a notification about some other aircraft that is sort-of-close (even if 2000 feet above you?), and meanwhile the first aircraft may be approaching closer in a dangerous way but you won't hear about it.

Moreover, the voice notifications were only based on PFLAA sentences from FLARM or SoftRF, and not the PFLAU sentences.  But the latter are supposed to be the primary collision warnings, and there is no guarantee that the same aircraft will also be reported in a PFLAA sentence.

I revised the behavior as follows:  Based on the "alarm level" for each reported aircraft, which is included in those "sentences", a voice warning is given about the aircraft that is at the highest alarm level.  (There are 3 levels, predicting a possible collision within 0-8, 9-12, and 13-19 seconds - reporting in Skyview may be a second or two later than when these were computed.)  If there are two aircraft at the same highest level, the closer one is chosen.  Once a warning is given about an aircraft, no further warnings are given at the same alarm level.  But if the aircraft's alarm level increases, or if it first decreases and then increases to what it was before, then another warning is given about the same aircraft.

These voice warnings are given based on both PFLAU and PFLAA sentences.

As a secondary feature, only when there are no aircraft around that are considered a collision danger (to avoid distraction), voice traffic advisories are given about other aircraft when they first get closer than 6 km (about 4 miles) from this aircraft.  After that, no further notifications are given about the same aircraft, unless its signal is lost and regained, or if it goes outside of that threshold and then returns.  Also, if that aircraft is later deemed to be a collision danger, the warning messages are given as explained above.

In addition, I made the following further changes:
* Collision danger voice warnings are short, e.g., "danger, 3 o'clock, high".
* Traffic advisories are longer, e.g., "traffic, 3 o'clock, 2 miles, 500 feet above".
* Collision danger voice warnings are given in a female voice speaking rapidly.
* Traffic advisories are given in a male voice speaking more slowly.
* Reduced the volume of the advisories 6dB to make it about level with the female voice.
* The settings now only allow voice on or off, no choice of voices - see above.


## revision 0.12

### Major improvements

#### ESP32

- rebuilt in conjunction with most recent [Arduino Core 1.0.5](https://github.com/espressif/arduino-esp32/releases/tag/1.0.5). More stable and less bugs in:
    - Wi-Fi
    - Bluetooth SPP
    - Bluetooth LE

### Known issues and limitations

- same as in 0.11

### Flashing instructions

&nbsp;&nbsp;&nbsp;&nbsp;**Regular:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'[Web Update](https://github.com/lyusupov/SoftRF/wiki/Firmware-update-%28Web-method%29#esp32)' method should work just fine when you are updating from **Regular** 0.11.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In case of troubles - use generic ('cable') method instead. Follow **Step 1** and **Step 2** of this [**Quick start**](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start) guidance.

## revision 0.11

### Major improvements

#### Common

- improved traffic acceptor logic
- improved voice alerts logic
- fix for remote (>20km) traffic processing
- work around of an issue when ownship (pressure) altitude is not reported in GDL90 input data

#### ESP32

- confirmed to work nicely with version 2.8 of TTGO T5S
- sends proper NMEA sentences to put connected (by wires) SoftRF Dongle/Uni/Mini into sleep or wake it up
- battery current consumption in 'sleep' mode has been reduced 10-100 times down to: uSD in - 0.2 mA, uSD out - 0.1 mA
- better response on buttons press (except when a voice alert is in progress)
- improved buffering and response time on input data flow
- Bluetooth LE connection method works better now

### Known issues and limitations

- same as in 0.10

### Flashing instructions

&nbsp;&nbsp;&nbsp;&nbsp;**Regular:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'[Web Update](https://github.com/lyusupov/SoftRF/wiki/Firmware-update-%28Web-method%29#esp32)' method should work just fine when you are updating from **Regular** 0.10.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;In case of troubles - use generic ('cable') method instead. Follow **Step 1** and **Step 2** of this [**Quick start**](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start) guidance.

## revision 0.10

### New features

- Bluetooth SPP and LE connection methods. Mostly applicable for a 'SoftRF on ESP32' partner - other devices may or may not work ;
- serial data input over EZ built-in micro-USB port ;
- one more radar view's zoom level ;
- 'Wi-Fi off' timer option ;
- traffic filter (by altitude) ;
- team member's aircraft (if any) is depicted in a bit different manner ;
- dual boot option ( demo application is [iArradio](https://github.com/TioRuben/iArradio) ).

### Known issues and limitations

- same as in 0.9, plus
- first Bluetooth SPP connection may cause restart of a partner SoftRF data source device ;
- due to high RAM memory usage, Bluetooth SPP may coexist with either voice or aircraft's data option but not both ;
- only Bluetooth 'Simple Pairing' (no key) method is currently supported.

### Flashing instructions

&nbsp;&nbsp;&nbsp;&nbsp;**Regular:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;'Web Update' UI feature does not work in previous 0.9 revision. Use generic ('cable') method instead.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Follow **Step 1** and **Step 2** of this [**Quick start**](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Quick-start) guidance.

&nbsp;&nbsp;&nbsp;&nbsp;**Dual boot:**<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Follow [these instructions](https://github.com/lyusupov/SoftRF/wiki/SkyView.-Dual-boot).<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;**NOTICE!**: [**iArradio**](https://github.com/TioRuben/iArradio) firmware that comes with it is a DEMO. I will reject your every claim or question.<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;If you need a custom weather, playlist or timezone - build your own firmware file by yourself.<br>

## revision 0.9

### Known issues and limitations

- maximum number of tracked objects is limited to 9 ;
- very short keypress may not work. Make a more distinct one ;
- keypress may not work when audio is playing ;
- view mode change may cause 2-3 seconds of '**NO DATA**' or '**NO FIX**' warning ;
- 'anti-ghosting' full screen refresh may cause 2-3 seconds of '**NO DATA**' or '**NO FIX**' warning ;
- '**NO DATA**' or '**NO FIX**' warning may appear right after voice traffic alert message ;
- **VOICE2** and **VOICE3** may have some WAV files missing ;
- Wi-Fi re-connect may fail sometimes. Reset of SoftRF server and/or SkyView client does typically help ;
- does not work with Stratux yet due to issues with DHCP leases and GDL90 data (over-)flow.

## revision 0.8

Very first deployment of SkyView's firmware binaries.
