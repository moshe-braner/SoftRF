# SoftRF

DIY, multifunctional, compatible, sub-1 GHz ISM band radio based proximity awareness system for general aviation.
<br>
[User Guide](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/SoftRF_MB_user_guide.txt)
<br>

<p><img src="https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/documentation/T-Beam_MB149_.jpg"></p>

### Latest major additions:

* vMB204: allows FANET+PAW+FLR+ADSL multiprotocol mode, revised relaying again
* vMB203: supports FLARM and FANET messaging (PFLAM)
* vMB202: using RadioLib, now supports the Sensecap T1000-E and Thinknode M3 (& M1)
* vMB179: added FANET id_method, auto-region by default, buzz at first GNSS fix
* vMB174: dual-protocol FANET (or PAW) plus FLARM (or ADS-L) modes
* vMB172: dual-protocol FLARM/ADS-L reception, some relaying in ADS-L protocol
* vMB171: revised relaying of ADS-B traffic, including "relay only" mode 
* vMB159: capability to periodically transmit in an alternate protocol
* vMB155: collect statistics on reception range by relative direction 
* vMB153: record compressed flight logs in flash memory (T-Beam & T-Echo)
* vMB152: settings stored in an editable text file
* vMB138: supports using add-on GNSS modules, on the T-Beam
* vMB130: supports the GNS5892R ADS-B receiver module, on the T-Beam
* vMB120: supports the latest 2024 radio protocol
* vMB110: added second serial port and data bridging (only on T-Beam)

### Beyond Lyusupov's version:

* Collision prediction for circling aircraft
* Optional multi-protocol operation
* Can set aircraft ID for self, ID to ignore, and ID to follow
* Support 3-level collision alarms via buzzer
* Can configure two simultaneous NMEA output destinations
* Settable baud rate for serial output
* Estimates wind while circling, uses for collision prediction
* Corrected frequency hopping and time slots
* Airborne devices can relay radio packets from landed aircraft
* Can adjust SoftRF settings within T-Echo (without an app)

### And on the T-Beam specifically: 

* Louder buzzer via 2-pin differential drive, or external
* Collision-danger traffic VOICE warnings!
* Includes strobe-control logic
* Option to connect to ambient WiFi network instead of creating one
* Option to send data as TCP client instead of TCP server
* Specify server's IP address for TCP client, and choice of 2 ports
* Ability to use WiFi TCP & UDP NMEA outputs simultaneously
* These WiFi options allow wireless output to XCvario
* Detect and use OLED display on either I2C port
* Winch mode ("aircraft type") with variable reported altitude

### What is here:

Source code, and compiled binaries for [ESP32 (T-Beam)](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SoftRF) and [nRF52 (T-Echo, M1, M3 & T1000-E)](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/nRF52840/SoftRF/MassStorage) (only).  Note: a new T1000-E requires [downgrading the bootloader](https://raw.githubusercontent.com/moshe-braner/SoftRF/refs/heads/master/software/firmware/documentation/SoftRF_MB_user_guide.txt) before loading SoftRF.
<br>
<br>

[Documentation files](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/documentation).
<br>
<br>

Modified version of [SkyView](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SkyView)
<br>
[SkyStrobe](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32/SkyStrobe) - a controller for a visibility strobe (and more)
<br>
<br>

For discussions join the [SoftRF Community](https://groups.google.com/g/softrf_community).
<br>
<br>

For additional info see also [Lyusupov's repository](https://github.com/lyusupov/SoftRF).


