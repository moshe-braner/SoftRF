# First-time firmware installation procedure

- [NodeMCU](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/binaries/README.md#nodemcu) (ESP8266)
- [ESP32](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/binaries/README.md#esp32)
- [S76G](https://github.com/moshe-braner/SoftRF/wiki/AcSiP-S7xG-flashing-instructions#s76g) (STM32)
- [CC1352R](https://github.com/moshe-braner/SoftRF/wiki/Uni-Edition.-Firmware-maintenance-procedures#initial-installation)
- [CubeCell](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/binaries/README.md#cubecell) (ASR650x)
- [nRF52840](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/binaries/README.md#nrf52840)
- [LPC4320](https://github.com/moshe-braner/SoftRF/blob/master/software/firmware/binaries/README.md#lpc4320)

## NodeMCU

### Select NodeMCU COM port
![](https://github.com/moshe-braner/SoftRF/blob/master/documents/images/NodeMCU-Flasher-1.GIF)



### Select firmware file
![](https://github.com/moshe-braner/SoftRF/blob/master/documents/images/NodeMCU-Flasher-2.GIF)



### Start flashing cycle
![](https://github.com/moshe-braner/SoftRF/blob/master/documents/images/NodeMCU-Flasher-3.GIF)



### Wait for completion
![](https://github.com/moshe-braner/SoftRF/blob/master/documents/images/NodeMCU-Flasher-4.GIF)

## ESP32

1. Take ESP32 flash download tool from this location: https://www.espressif.com/en/support/download/other-tools <br>
You might also need to install a [driver for the CP210X USB to UART bridge from Silicon Labs](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers) prior to first use of the ESP32 tool ;
2. Download appropriate version of the SoftRF firmware from [this location](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ESP32) and unzip the archive ;

3. Download the additional needed files from [this location](https://github.com/moshe-braner/SoftRF/tree/master/software/app/Flashing) and unzip the archive ;

4. Select COM port, enter partition files and addresses, select options ;<br>

   Here is an example:<br>

![](https://github.com/moshe-braner/SoftRF/raw/master/documents/images/ESP32-Flasher-1.JPG)



5. Press **START** button and wait for completion.

For some boards you may need to push **BOOT** button in order to activate flash download mode.<br>
"Stock" modules may also require to apply full flash memory erase (use **ERASE** UI "button") prior to first flashing with SoftRF's firmware.

<br>

## CubeCell

1. Take **CubeCellflash** for Windows <sub>_(Linux and MacOS variants are also available there)_</sub> download tool from this location: https://resource.heltec.cn/download/<br>
You might also need to install a [driver for the CP210X USB to UART bridge from Silicon Labs](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers) prior to first use of the CubeCellflash tool ;
2. Download appropriate version of the SoftRF firmware from [this location](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/ASR650x) and unzip the archive ;

![](https://github.com/moshe-braner/SoftRF/blob/master/documents/images/Mini-1.jpg)

3. Use Windows command line tool to execute firmware flashing procedure as follows:

![](https://github.com/moshe-braner/SoftRF/blob/master/documents/images/Mini-2.jpg)

<br>

## nRF52840

The board typically comes with factory pre-installed [Adafruit_nRF52_Bootloader](https://github.com/adafruit/Adafruit_nRF52_Bootloader).<br>
The Bootloader is capable to self-program an application firmware into the device. In order to simplify the firmware transfer, the bootloader emulates a "USB Mass Storage" interface.

1. Download appropriate version of the SoftRF firmware from [this location](https://github.com/moshe-braner/SoftRF/tree/master/software/firmware/binaries/nRF52840/SoftRF/MassStorage) ;

2. Connect the SoftRF device to your PC by means of a USB cable (Type-A <-> Type-C) ;

3. Enter "DFU mode", using the procedure that fits your device:
    * on the T-Echo or M1, double click (two clicks within 0.5 seconds) the RESET button
    * on the M3, press and hold the button for close to 20 seconds
    * on the T1000-E, flip the magnetic USB connector twice holding the button pressed
         (see short video here: https://github.com/lyusupov/SoftRF/wiki/Card-Edition.-Quick-start )
    * or, for any of these devices, if version MB202 or later is already installed:
          connect from https://ogn.helioho.st/mysoftrf/ and click "update firmware"
         (or send it the NMEA sentence $PSRFC,DFU*2F)
   A virtual storage device labeled TECHOBOOT, T1000-E or ThinknodeM1/3 will then appear in your "File manager".

4. Drag the downloaded firmware file by your pointing device (mouse, trackball,...) , then drop it into the virtual disk that opened. Wait until the file transfer is complete, the virtual disk will disappear as the device reboots.

<br>

<img src="https://github.com/moshe-braner/SoftRF/raw/master/documents/images/Badge-2.jpg" height="302" width="800">

<br>

## LPC4320

For Linux and Mac OS X users, you will need a few tools installed on your computer before you begin:

* [dfu-util](http://dfu-util.sourceforge.net/) 0.8 or newer - Used to load and run the stock HackRF firmware from RAM. dfu-util 0.8 is recommended, as it is the most extensively tested with the HackRF hardware and build software.
* [hackrf](https://github.com/greatscottgadgets/hackrf) - All you need is the host tools, specifically, hackrf_spiflash.

### Backup of factory firmware

```
$ hackrf_spiflash -v -r HackRF_factory_firmware.bin
Reading 256 bytes from 0x000000.
Reading 256 bytes from 0x000100.

< ... skipped ... >

$ ls -la HackRF_factory_firmware.bin
-rw-r--r-- 1 pi pi 1048576 Nov  4 10:18 HackRF_factory_firmware.bin
```

### Flashing

Plug the HackRF into USB power while holding down the DFU button (the button closer to the antenna jack). Release the DFU button. Then run:

```
$ dfu-util -D hackrf_one_usb.dfu --reset
$ hackrf_spiflash -v -w SoftRF-firmware-v1.0-LPC43.bin
```

### Restoring HackRF firmware

Plug the HackRF into USB power while holding down the DFU button (the button closer to the antenna jack). Release the DFU button. Then run:

```
$ dfu-util -D hackrf_one_usb.dfu --reset
$ hackrf_spiflash -v -w HackRF_factory_firmware.bin
```
