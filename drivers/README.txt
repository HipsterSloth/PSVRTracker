These drivers were generated using the libusbk's "USB Inf Creator/Installer" Wizard

Project Page: https://sourceforge.net/projects/libusbk/
Download: https://svwh.dl.sourceforge.net/project/libusbk/libusbK-beta/3.0.5.16/libusbK-3.0.5.16-bin.7z

This wizard looks for connected USB interfaces and then allows the user to create a simple installer for them.
The user created installer can register one of the following generic USB drivers for the USB interface:

* WinUSB
* libusbK
* libusb0

For the purposes of PSVRService we use the WinUSB driver for devices we care about, though in theory we could use LibUSB0 as well.

We need drivers for the following USB interfaces:

* PSVR USB Interface 4: 
 - Used to read sensor data from the PSVR headset

* PSVR USB Interface 5: 
 - Used to send commands to the PSVR headset

* PS4 Camera Boot Device: 
 - A simple usb device that firmware is send to on start of PSVR Service
 - Once the firware is loaded on the PS4 Camera a Windows Media Framework compatible "USB Camera-OV580" will appear
 - If the camera becomes unplugged, PSVR Service will need to resend the firmware to the PS4 Camera Boot Device
