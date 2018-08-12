/*
 This file is largely reproduced from PS3EYEDriver(https://github.com/inspirit/PS3EYEDriver).
 We adapted the library to work with a shared USBManager in the service.
 The license for PS3EYEDriver is reproduced below:

 <license>
 License information for PS3EYEDriver
 ------------------------------------
 
 The license of the PS3EYEDriver is MIT (for newly-written code) and GPLv2 for
 all code derived from the Linux Kernel Driver (ov534) sources.
 
 In https://github.com/inspirit/PS3EYEDriver/pull/3, Eugene Zatepyakin writes:
 
 "all of my code is MIT licensed and to tell the truth i didnt check Linux
 p3eye version license. as far as i know it was contributed to Linux by some
 devs who decided to do it on their own..."
 
 The code is based on the Linux driver for the PSEye, which can be found here:
 
 http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/drivers/media/usb/gspca/ov534.c
  
 ov534-ov7xxx gspca driver
 
 Copyright (C) 2008 Antonio Ospite <ospite@studenti.unina.it>
 Copyright (C) 2008 Jim Paris <jim@jtan.com>
 Copyright (C) 2009 Jean-Francois Moine http://moinejf.free.fr
 
 Based on a prototype written by Mark Ferrell <majortrips@gmail.com>
 USB protocol reverse engineered by Jim Paris <jim@jtan.com>
 https://jim.sh/svn/jim/devl/playstation/ps3/eye/test/
 
 PS3 Eye camera enhanced by Richard Kaswy http://kaswy.free.fr
 PS3 Eye camera - brightness, contrast, awb, agc, aec controls
                  added by Max Thrun <bear24rw@gmail.com>
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 </license>
*/

#ifndef PS3EYE_LIBUSB_CAPTURE_H
#define PS3EYE_LIBUSB_CAPTURE_H

//-- includes -----
#include "USBApiInterface.h"
#include "PS3EyeConfig.h"

//-- constants -----
#define INVALID_DEVICE_FORMAT_INDEX			-1
#define UNSPECIFIED_CAMERA_WIDTH			0xFFFFFFFF
#define UNSPECIFIED_CAMERA_HEIGHT			0xFFFFFFFF
#define UNSPECIFIED_CAMERA_FPS				0xFFFFFFFF

enum ePS3EyeVideoMode
{
	PS3EyeVideoMode_640x480_83fps,
	PS3EyeVideoMode_640x480_75fps,
	PS3EyeVideoMode_640x480_60fps,
	PS3EyeVideoMode_640x480_50fps,
	PS3EyeVideoMode_640x480_40fps,
	PS3EyeVideoMode_640x480_30fps,
	PS3EyeVideoMode_640x480_25fps,
	PS3EyeVideoMode_640x480_20fps,
	PS3EyeVideoMode_640x480_15fps,
	PS3EyeVideoMode_640x480_10fps,
	PS3EyeVideoMode_640x480_8fps,
	PS3EyeVideoMode_640x480_5fps,
	PS3EyeVideoMode_640x480_3fps,
	PS3EyeVideoMode_640x480_2fps,

	PS3EyeVideoMode_320x240_290fps,
	PS3EyeVideoMode_320x240_205fps,
	PS3EyeVideoMode_320x240_187fps,
	PS3EyeVideoMode_320x240_150fps,
	PS3EyeVideoMode_320x240_137fps,
	PS3EyeVideoMode_320x240_125fps,
	PS3EyeVideoMode_320x240_100fps,
	PS3EyeVideoMode_320x240_90fps,
	PS3EyeVideoMode_320x240_75fps,
	PS3EyeVideoMode_320x240_60fps,
	PS3EyeVideoMode_320x240_50fps,
	PS3EyeVideoMode_320x240_40fps,
	PS3EyeVideoMode_320x240_37fps,
	PS3EyeVideoMode_320x240_30fps,
	PS3EyeVideoMode_320x240_17fps,
	PS3EyeVideoMode_320x240_15fps,
	PS3EyeVideoMode_320x240_12fps,
	PS3EyeVideoMode_320x240_10fps,
	PS3EyeVideoMode_320x240_7fps,
	PS3EyeVideoMode_320x240_5fps,
	PS3EyeVideoMode_320x240_3fps,
	PS3EyeVideoMode_320x240_2fps,

	PS3EyeVideoMode_COUNT,
	PS3EyeVideoMode_INVALID= 255
};

//-- definitions -----
struct PS3EyeProperties
{
    bool autogain;
    unsigned char gain; // 0 <-> 63
    unsigned char exposure; // 0 <-> 255
    unsigned char sharpness; // 0 <-> 63
    unsigned char hue; // 0 <-> 255
    bool awb;
    unsigned char brightness; // 0 <-> 255
    unsigned char contrast; // 0 <-> 255
    unsigned char blueBalance; // 0 <-> 255
    unsigned char redBalance; // 0 <-> 255
    unsigned char greenBalance; // 0 <-> 255
    bool flip_h;
    bool flip_v;

    unsigned int frame_width;
    unsigned int frame_height;
    unsigned short frame_rate;

    PS3EyeProperties();
};

class PS3EyeVideoDevice
{
public:
    PS3EyeVideoDevice(USBDeviceEnumerator* enumerator);
    virtual ~PS3EyeVideoDevice();

	static ePS3EyeVideoMode findBestVideoMode(unsigned int w, unsigned int h, unsigned int frameRate);

    bool open(ePS3EyeVideoMode desiredVideoMode, PS3EyeTrackerConfig &cfg, class ITrackerListener *trackerListener);
    void close();

	inline const PSVRVideoPropertyConstraint *getVideoPropertyConstraints() const { return m_videoPropertyConstraints; }

	bool getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const;
	void setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value);
	int getVideoProperty(const PSVRVideoPropertyType property_type) const;

    // Camera Control Setters
    // Used in when we don't care about an async task result
    void setAutogain(bool val);
    void setAutoWhiteBalance(bool val);
    void setGain(unsigned char val);
    void setExposure(unsigned char val);
    void setSharpness(unsigned char val);
    void setContrast(unsigned char val);
    void setBrightness(unsigned char val);
    void setHue(unsigned char val);
    void setRedBalance(unsigned char val);
    void setGreenBalance(unsigned char val);
    void setBlueBalance(unsigned char val);
    void setFlip(bool horizontal, bool vertical);

    // Camera Control Accessors
    inline bool getAutogain() const { return m_properties.autogain; }
    inline bool getAutoWhiteBalance() const { return m_properties.awb; }
    inline unsigned char getGain() const { return m_properties.gain; }
    inline unsigned char getExposure() const { return m_properties.exposure; }
    inline unsigned char getSharpness() const { return m_properties.sharpness; }
    inline unsigned char getContrast() const { return m_properties.contrast; }
    inline unsigned char getBrightness() const { return m_properties.brightness; }
    inline unsigned char getHue() const { return m_properties.hue; }
    inline unsigned char getRedBalance() const { return m_properties.redBalance; }
    inline unsigned char getBlueBalance() const { return m_properties.blueBalance; }
    inline unsigned char getGreenBalance() const { return m_properties.greenBalance; }
    inline bool getFlipH() const { return m_properties.flip_h; }
    inline bool getFlipV() const { return m_properties.flip_v; }

    // Camera Property Accessors
    inline bool isStreaming() const { return m_is_streaming; }
    inline unsigned int getWidth() const { return m_properties.frame_width; }
    inline unsigned int getHeight() const { return m_properties.frame_height; }
    inline unsigned short getFrameRate() const { return m_properties.frame_rate; }

    // Get the USB Bus/Port path that this camera is connected to
    inline t_usb_device_handle getUSBDeviceHandle() const { return m_usb_device_handle; }
    bool getUSBPortPath(char *out_identifier, size_t max_identifier_length) const;

private:
    PS3EyeVideoDevice(const PS3EyeVideoDevice&);
    void operator=(const PS3EyeVideoDevice&);

    PS3EyeProperties m_properties;
	PSVRVideoPropertyConstraint m_videoPropertyConstraints[PSVRVideoProperty_COUNT];

    bool m_is_streaming;
    double m_last_qued_frame_time;

    // usb stuff
    t_usb_device_handle m_usb_device_handle;
    class PS3EyeVideoPacketProcessor *m_video_packet_processor;
};

#endif