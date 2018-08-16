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

//-- includes -----
#include "PS3EyeVideo.h"
#include "DeviceInterface.h"
#include "Logger.h"
#include "Utility.h"
#include "USBDeviceManager.h"
#include "WorkerThread.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <iomanip>
#include <mutex>
#include <string>


//-- constants -----
#define TRANSFER_SIZE		16384
#define NUM_TRANSFERS		8

#define OV534_REG_ADDRESS	0xf1	/* sensor address */
#define OV534_REG_SUBADDR	0xf2
#define OV534_REG_WRITE		0xf3
#define OV534_REG_READ		0xf4
#define OV534_REG_OPERATION	0xf5
#define OV534_REG_STATUS	0xf6

#define OV534_OP_WRITE_3	0x37
#define OV534_OP_WRITE_2	0x33
#define OV534_OP_READ_2		0xf9

#define CTRL_TIMEOUT 500
#define VGA	 0
#define QVGA 1

/* Values for bmHeaderInfo (Video and Still Image Payload Headers, 2.4.3.3) */
#define UVC_STREAM_EOH	(1 << 7)
#define UVC_STREAM_ERR	(1 << 6)
#define UVC_STREAM_STI	(1 << 5)
#define UVC_STREAM_RES	(1 << 4)
#define UVC_STREAM_SCR	(1 << 3)
#define UVC_STREAM_PTS	(1 << 2)
#define UVC_STREAM_EOF	(1 << 1)
#define UVC_STREAM_FID	(1 << 0)

/* packet types when moving from iso buf to frame buf */
enum gspca_packet_type {
    DISCARD_PACKET,
    FIRST_PACKET,
    INTER_PACKET,
    LAST_PACKET
};

//-- typedefs -----
// From: libusb.h
/* stdint.h is not available on older MSVC */
#if defined(_MSC_VER) && (_MSC_VER < 1600) && (!defined(_STDINT)) && (!defined(_STDINT_H))
    typedef unsigned __int8   uint8_t;
    typedef unsigned __int16  uint16_t;
    typedef unsigned __int32  uint32_t;
#else
    #include <stdint.h>
#endif

//-- data -----
struct PS3EyeVideoModeInfo 
{
	uint16_t width;
	uint16_t height;
	uint16_t fps;
	uint8_t r11;
	uint8_t r0d;
	uint8_t re5;
};

static const struct PS3EyeVideoModeInfo k_supported_video_modes[PS3EyeVideoMode_COUNT] = { 
	/* 640x480 */
    {640, 480, 83, 0x01, 0xc1, 0x02}, /* 83 FPS: video is partly corrupt */
    {640, 480, 75, 0x01, 0x81, 0x02}, /* 75 FPS or below: video is valid */
    {640, 480, 60, 0x00, 0x41, 0x04},
    {640, 480, 50, 0x01, 0x41, 0x02},
    {640, 480, 40, 0x02, 0xc1, 0x04},
    {640, 480, 30, 0x04, 0x81, 0x02},
    {640, 480, 25, 0x00, 0x01, 0x02},
    {640, 480, 20, 0x04, 0x41, 0x02},
    {640, 480, 15, 0x09, 0x81, 0x02},
    {640, 480, 10, 0x09, 0x41, 0x02},
    {640, 480, 8, 0x02, 0x01, 0x02},
    {640, 480, 5, 0x04, 0x01, 0x02},
    {640, 480, 3, 0x06, 0x01, 0x02},
    {640, 480, 2, 0x09, 0x01, 0x02},
	/* 320x240 */
    {320, 240, 290, 0x00, 0xc1, 0x04},
    {320, 240, 205, 0x01, 0xc1, 0x02}, /* 205 FPS or above: video is partly corrupt */
    {320, 240, 187, 0x01, 0x81, 0x02}, /* 187 FPS or below: video is valid */
    {320, 240, 150, 0x00, 0x41, 0x04},
    {320, 240, 137, 0x02, 0xc1, 0x02},
    {320, 240, 125, 0x01, 0x41, 0x02},
    {320, 240, 100, 0x02, 0xc1, 0x04},
    {320, 240, 90, 0x03, 0x81, 0x02},
    {320, 240, 75, 0x04, 0x81, 0x02},
    {320, 240, 60, 0x04, 0xc1, 0x04},
    {320, 240, 50, 0x04, 0x41, 0x02},
    {320, 240, 40, 0x06, 0x81, 0x03},
    {320, 240, 37, 0x00, 0x01, 0x04},
    {320, 240, 30, 0x04, 0x41, 0x04},
    {320, 240, 17, 0x18, 0xc1, 0x02},
    {320, 240, 15, 0x18, 0x81, 0x02},
    {320, 240, 12, 0x02, 0x01, 0x04},
    {320, 240, 10, 0x18, 0x41, 0x02},
    {320, 240, 7, 0x04, 0x01, 0x04},
    {320, 240, 5, 0x06, 0x01, 0x04},
    {320, 240, 3, 0x09, 0x01, 0x04},
    {320, 240, 2, 0x18, 0x01, 0x02},
};

static const uint8_t ov534_reg_initdata[][2] = {
	{ 0xe7, 0x3a },

	{ OV534_REG_ADDRESS, 0x42 }, /* select OV772x sensor */

	{ 0x92, 0x01 },
	{ 0x93, 0x18 },
	{ 0x94, 0x10 },
	{ 0x95, 0x10 },
	{ 0xE2, 0x00 },
	{ 0xE7, 0x3E },
	
	{ 0x96, 0x00 },
	{ 0x97, 0x20 },
	{ 0x97, 0x20 },
	{ 0x97, 0x20 },
	{ 0x97, 0x0A },
	{ 0x97, 0x3F },
	{ 0x97, 0x4A },
	{ 0x97, 0x20 },
	{ 0x97, 0x15 },
	{ 0x97, 0x0B },

	{ 0x8E, 0x40 },
	{ 0x1F, 0x81 },
	{ 0xC0, 0x50 },
	{ 0xC1, 0x3C },
	{ 0xC2, 0x01 },
	{ 0xC3, 0x01 },
	{ 0x50, 0x89 },
	{ 0x88, 0x08 },
	{ 0x8D, 0x00 },
	{ 0x8E, 0x00 },

	{ 0x1C, 0x00 },		/* video data start (V_FMT) */

	{ 0x1D, 0x00 },		/* RAW8 mode */
	{ 0x1D, 0x02 },		/* payload size 0x0200 * 4 = 2048 bytes */
	{ 0x1D, 0x00 },		/* payload size */

	{ 0x1D, 0x01 },		/* frame size = 0x012C00 * 4 = 307200 bytes (640 * 480 @ 8bpp) */
	{ 0x1D, 0x2C },		/* frame size */
	{ 0x1D, 0x00 },		/* frame size */

	{ 0x1C, 0x0A },		/* video data start (V_CNTL0) */
	{ 0x1D, 0x08 },		/* turn on UVC header */
	{ 0x1D, 0x0E },

	{ 0x34, 0x05 },
	{ 0xE3, 0x04 },
	{ 0x89, 0x00 },
	{ 0x76, 0x00 },
	{ 0xE7, 0x2E },
	{ 0x31, 0xF9 },
	{ 0x25, 0x42 },
	{ 0x21, 0xF0 },
	{ 0xE5, 0x04 }	
};

static const uint8_t ov772x_reg_initdata[][2] = {
	{ 0x12, 0x80 },		/* reset */
	{ 0x3D, 0x00 },

	{ 0x12, 0x01 },		/* Processed Bayer RAW (8bit) */

	{ 0x11, 0x01 },
	{ 0x14, 0x40 },
	{ 0x15, 0x00 },
	{ 0x63, 0xAA },		// AWB	
	{ 0x64, 0x87 },
	{ 0x66, 0x00 },
	{ 0x67, 0x02 },
	{ 0x17, 0x26 },
	{ 0x18, 0xA0 },
	{ 0x19, 0x07 },
	{ 0x1A, 0xF0 },
	{ 0x29, 0xA0 },
	{ 0x2A, 0x00 },
	{ 0x2C, 0xF0 },
	{ 0x20, 0x10 },
	{ 0x4E, 0x0F },
	{ 0x3E, 0xF3 },
	{ 0x0D, 0x41 },
	{ 0x32, 0x00 },
	{ 0x13, 0xF0 },		// COM8  - jfrancois 0xf0	orig x0f7
	{ 0x22, 0x7F },
	{ 0x23, 0x03 },
	{ 0x24, 0x40 },
	{ 0x25, 0x30 },
	{ 0x26, 0xA1 },
	{ 0x2A, 0x00 },
	{ 0x2B, 0x00 },
	{ 0x13, 0xF7 },
	{ 0x0C, 0xC0 },

	{ 0x11, 0x00 },
	{ 0x0D, 0x41 },

 	{ 0x8E, 0x00 },		// De-noise threshold - jfrancois 0x00 - orig 0x04

};

static const uint8_t bridge_start_vga[][2] = {
	{0x1c, 0x00},
	{0x1d, 0x00},
	{0x1d, 0x02},
	{0x1d, 0x00},
	{0x1d, 0x01},	/* frame size = 0x012C00 * 4 = 307200 bytes (640 * 480 @ 8bpp) */
	{0x1d, 0x2C},	/* frame size */
	{0x1d, 0x00},	/* frame size */
	{0xc0, 0x50},
	{0xc1, 0x3c},
};
static const uint8_t sensor_start_vga[][2] = {
	{0x12, 0x01},
	{0x17, 0x26},
	{0x18, 0xa0},
	{0x19, 0x07},
	{0x1a, 0xf0},
	{0x29, 0xa0},
	{0x2c, 0xf0},
	{0x65, 0x20},
};
static const uint8_t bridge_start_qvga[][2] = {
	{0x1c, 0x00},
	{0x1d, 0x00},
	{0x1d, 0x02},
	{0x1d, 0x00},	
	{0x1d, 0x00},	/* frame size = 0x004B00 * 4 = 76800 bytes (320 * 240 @ 8bpp) */
	{0x1d, 0x4b},	/* frame size */
	{0x1d, 0x00},	/* frame size */
	{0xc0, 0x28},
	{0xc1, 0x1e},
};
static const uint8_t sensor_start_qvga[][2] = {
	{0x12, 0x41},
	{0x17, 0x3f},
	{0x18, 0x50},
	{0x19, 0x03},
	{0x1a, 0x78},
	{0x29, 0x50},
	{0x2c, 0x78},
	{0x65, 0x2f},
};

//-- private methods -----
static void init_camera(t_usb_device_handle device_handle, const PS3EyeVideoModeInfo &video_mode);

static void set_autogain(t_usb_device_handle device_handle, bool bAutoGain, uint8_t gain, uint8_t exposure);
static void set_auto_white_balance(t_usb_device_handle device_handle, bool bAutoWhiteBalance);
static void set_gain(t_usb_device_handle handle, unsigned char val);
static void set_exposure(t_usb_device_handle handle, unsigned char val);
static void set_sharpness(t_usb_device_handle device_handle,unsigned char val);
static void set_contrast(t_usb_device_handle device_handle, unsigned char val);
static void set_brightness(t_usb_device_handle device_handle, unsigned char val);
static void set_hue(t_usb_device_handle device_handle, unsigned char val);
static void set_red_balance(t_usb_device_handle device_handle, unsigned char val);
static void set_green_balance(t_usb_device_handle device_handle, unsigned char val);
static void set_blue_balance(t_usb_device_handle device_handle, unsigned char val);
static void set_flip(t_usb_device_handle device_handle, bool horizontal, bool vertical);
static void set_frame_rate(t_usb_device_handle device_handle, const PS3EyeVideoModeInfo &video_mode);

static void sccb_reg_write(t_usb_device_handle device_handle, uint8_t reg, uint8_t val);
static void sccb_reg_write_array(t_usb_device_handle device_handle, const uint8_t (*sequence)[2], int sequenceLength);
static uint8_t sccb_reg_read(t_usb_device_handle device_handle, uint16_t reg);
static bool sccb_check_status(t_usb_device_handle device_handle);

static void ov534_reg_write(t_usb_device_handle device_handle, uint16_t reg, uint8_t val);
static void ov534_reg_write_array(t_usb_device_handle device_handle, const uint8_t (*sequence)[2], int sequenceLength);
static uint8_t ov534_reg_read(t_usb_device_handle device_handle, uint16_t reg);
static void ov534_set_led(t_usb_device_handle device_handle, bool bLedOn);

static void log_usb_result_code(const char *function_name, eUSBResultCode result_code);

//-- PS3EyeVideoFrameProcessor -----
class PS3EyeFrameProcessorThread : public WorkerThread
{
public:
	PS3EyeFrameProcessorThread(const PS3EyeVideoModeInfo &video_mode, ITrackerListener *trackerListener)
		: WorkerThread(std::string("PS3EyeVideoFrameProcessor"))
		, m_maxCompressedFrameCount(2)
		, m_compressedFrameWriteIndex(0)
		, m_compressedFrameReadIndex(0)
		, m_compressedFrameCount(0)
		, m_compressedFramesBuffer(nullptr)
		, m_uncompressedFrameBuffer(nullptr)
		, m_compressedFrameSizeBytes(video_mode.width*video_mode.height) // Bayer Buffer = 1 byte per pixel
		, m_uncompressedFrameSizeBytes(3*video_mode.width*video_mode.height) // RBG Buffer = 3 bytes per pixel
		, m_trackerListener(trackerListener)
	{
        if (m_compressedFrameSizeBytes > 0)
        {
            m_compressedFramesBuffer = new uint8_t[m_compressedFrameSizeBytes*m_maxCompressedFrameCount];
            m_uncompressedFrameBuffer = new uint8_t[m_uncompressedFrameSizeBytes];
            memset(m_compressedFramesBuffer, 0, m_compressedFrameSizeBytes);
            memset(m_uncompressedFrameBuffer, 0, m_uncompressedFrameSizeBytes);
        }
	}

    virtual ~PS3EyeFrameProcessorThread()
    {
        if (m_compressedFramesBuffer != nullptr)
        {
            delete[] m_compressedFramesBuffer;
            m_compressedFramesBuffer= nullptr;
        } 

        if (m_uncompressedFrameBuffer != nullptr)
        {
            delete[] m_uncompressedFrameBuffer;
            m_uncompressedFrameBuffer= nullptr;
        } 
    }

	uint32_t getCompressedFrameSizeBytes() const 
	{
		return m_compressedFrameSizeBytes;
	}

	uint8_t* getCompressedFrameBufferStart()
	{
		return m_compressedFramesBuffer;
	}

	uint8_t* enqueueCompressedFrame()
	{
		uint8_t* new_frame = NULL;

		std::lock_guard<std::mutex> lock(mutex);

		// Unlike traditional producer/consumer, we don't block the producer if the buffer is full (ie. the consumer is not reading data fast enough).
		// Instead, if the buffer is full, we simply return the current frame pointer, causing the producer to overwrite the previous frame.
		// This allows performance to degrade gracefully: if the consumer is not fast enough (< Camera FPS), it will miss frames, but if it is fast enough (>= Camera FPS), it will see everything.
		//
		// Note that because the the producer is writing directly to the ring buffer, we can only ever be a maximum of num_frames-1 ahead of the consumer, 
		// otherwise the producer could overwrite the frame the consumer is currently reading (in case of a slow consumer)
		if (m_compressedFrameCount >= m_maxCompressedFrameCount - 1)
		{
			return m_compressedFramesBuffer + m_compressedFrameWriteIndex*m_compressedFrameSizeBytes;
		}

		// Note: we don't need to copy any data to the buffer since the USB packets are directly written to the frame buffer.
		// We just need to update head and available count to signal to the consumer that a new frame is available
		m_compressedFrameWriteIndex = (m_compressedFrameWriteIndex + 1) % m_maxCompressedFrameCount;
		m_compressedFrameCount++;

		// Determine the next frame pointer that the producer should write to
		new_frame = m_compressedFramesBuffer + m_compressedFrameWriteIndex*m_compressedFrameSizeBytes;

		// Signal consumer that data became available
		empty_condition.notify_one();

		return new_frame;
	}

protected:
	// Called in a loop by the parent WorkerThread class
	virtual bool doWork() override
	{
		// Blocks until the next frame is available
		dequeAndDecompressNextFrame();

		// Send the uncompressed frame off to the tracker for processing
		m_trackerListener->notifyVideoFrameReceived(m_uncompressedFrameBuffer);

		return true;
	}

	void dequeAndDecompressNextFrame()
	{
		std::unique_lock<std::mutex> lock(mutex);

		// If there is no data in the buffer, wait until data becomes available
		empty_condition.wait(lock, [this] () { return m_compressedFrameCount != 0; });

		// Copy from internal buffer
		uint8_t* source = m_compressedFramesBuffer + m_compressedFrameReadIndex*m_compressedFrameSizeBytes;

		// Convert the bayer encoded frame to RBG
		debayer(m_frameWidth, m_frameHeight, source, m_uncompressedFrameBuffer, false);

		// Update tail and available count
		m_compressedFrameReadIndex = (m_compressedFrameReadIndex + 1) % m_maxCompressedFrameCount;
		m_compressedFrameCount--;
	}

	void debayer(int frame_width, int frame_height, const uint8_t* inBayer, uint8_t* outBuffer, bool inBGR)
	{
		// PSMove output is in the following Bayer format (GRBG):
		//
		// G R G R G R
		// B G B G B G
		// G R G R G R
		// B G B G B G
		//
		// This is the normal Bayer pattern shifted left one place.

		int				num_output_channels	    = 3;
		int				source_stride			= frame_width;
		const uint8_t*	source_row				= inBayer;												// Start at first bayer pixel
		int				dest_stride				= frame_width * num_output_channels;
		uint8_t*		dest_row				= outBuffer + dest_stride + num_output_channels + 1; 	// We start outputting at the second pixel of the second row's G component
		int				swap_br					= inBGR ? 1 : -1;

		// Fill rows 1 to height-1 of the destination buffer. First and last row are filled separately (they are copied from the second row and second-to-last rows respectively)
		for (int y = 0; y < frame_height-1; source_row += source_stride, dest_row += dest_stride, ++y)
		{
			const uint8_t* source		= source_row;
			const uint8_t* source_end	= source + (source_stride-2);								// -2 to deal with the fact that we're starting at the second pixel of the row and should end at the second-to-last pixel of the row (first and last are filled separately)
			uint8_t* dest				= dest_row;		

			// Row starting with Green
			if (y % 2 == 0)
			{
				// Fill first pixel (green)
				dest[-1*swap_br]	= (source[source_stride] + source[source_stride + 2] + 1) >> 1;
				dest[0]				= source[source_stride + 1];
				dest[1*swap_br]		= (source[1] + source[source_stride * 2 + 1] + 1) >> 1;		

				source++;
				dest += num_output_channels;

				// Fill remaining pixel
				for (; source <= source_end - 2; source += 2, dest += num_output_channels * 2)
				{
					// Blue pixel
					uint8_t* cur_pixel	= dest;
					cur_pixel[-1*swap_br]	= source[source_stride + 1];
					cur_pixel[0]			= (source[1] + source[source_stride] + source[source_stride + 2] + source[source_stride * 2 + 1] + 2) >> 2;
					cur_pixel[1*swap_br]	= (source[0] + source[2] + source[source_stride * 2] + source[source_stride * 2 + 2] + 2) >> 2;				

					//  Green pixel
					uint8_t* next_pixel		= cur_pixel+num_output_channels;
					next_pixel[-1*swap_br]	= (source[source_stride + 1] + source[source_stride + 3] + 1) >> 1;					
					next_pixel[0]			= source[source_stride + 2];
					next_pixel[1*swap_br]	= (source[2] + source[source_stride * 2 + 2] + 1) >> 1;
				}
			}
			else
			{
				for (; source <= source_end - 2; source += 2, dest += num_output_channels * 2)
				{
					// Red pixel
					uint8_t* cur_pixel	= dest;
					cur_pixel[-1*swap_br]	= (source[0] + source[2] + source[source_stride * 2] + source[source_stride * 2 + 2] + 2) >> 2;;
					cur_pixel[0]			= (source[1] + source[source_stride] + source[source_stride + 2] + source[source_stride * 2 + 1] + 2) >> 2;;
					cur_pixel[1*swap_br]	= source[source_stride + 1];

					// Green pixel
					uint8_t* next_pixel		= cur_pixel+num_output_channels;
					next_pixel[-1*swap_br]	= (source[2] + source[source_stride * 2 + 2] + 1) >> 1;
					next_pixel[0]			= source[source_stride + 2];
					next_pixel[1*swap_br]	= (source[source_stride + 1] + source[source_stride + 3] + 1) >> 1;
				}
			}

			if (source < source_end)
			{
				dest[-1*swap_br]	= source[source_stride + 1];
				dest[0]				= (source[1] + source[source_stride] + source[source_stride + 2] + source[source_stride * 2 + 1] + 2) >> 2;			
				dest[1*swap_br]		= (source[0] + source[2] + source[source_stride * 2] + source[source_stride * 2 + 2] + 2) >> 2;;			

				source++;
				dest += num_output_channels;
			}

			// Fill first pixel of row (copy second pixel)
			uint8_t* first_pixel		= dest_row-num_output_channels;
			first_pixel[-1*swap_br]		= dest_row[-1*swap_br];
			first_pixel[0]				= dest_row[0];
			first_pixel[1*swap_br]		= dest_row[1*swap_br];
		
 			// Fill last pixel of row (copy second-to-last pixel). Note: dest row starts at the *second* pixel of the row, so dest_row + (width-2) * num_output_channels puts us at the last pixel of the row
			uint8_t* last_pixel				= dest_row + (frame_width - 2)*num_output_channels;
			uint8_t* second_to_last_pixel	= last_pixel - num_output_channels;
			
			last_pixel[-1*swap_br]			= second_to_last_pixel[-1*swap_br];
			last_pixel[0]					= second_to_last_pixel[0];
			last_pixel[1*swap_br]			= second_to_last_pixel[1*swap_br];
		}

		// Fill first & last row
		for (int i = 0; i < dest_stride; i++)
		{
			outBuffer[i]									= outBuffer[i + dest_stride];
			outBuffer[i + (frame_height - 1)*dest_stride]	= outBuffer[i + (frame_height - 2)*dest_stride];
		}
	}

protected:
	// Queue State
	uint32_t m_maxCompressedFrameCount;
	uint32_t m_compressedFrameWriteIndex;
	uint32_t m_compressedFrameReadIndex;
	uint32_t m_compressedFrameCount;
	std::mutex mutex;
	std::condition_variable empty_condition;

	// Buffer State
	int m_frameWidth;
	int m_frameHeight;
	uint8_t *m_compressedFramesBuffer;
	uint8_t *m_uncompressedFrameBuffer;
    uint32_t m_compressedFrameSizeBytes;
	uint32_t m_uncompressedFrameSizeBytes;

	// External Processing
	ITrackerListener *m_trackerListener;
};

//-- PS3EyeUSBPacketProcessor -----
// Decodes incoming "USB Video Class" (UVC) packets from the USBDeviceManager worker thread
class PS3EyeUSBPacketProcessor
{
public:
    PS3EyeUSBPacketProcessor(
		const PS3EyeVideoModeInfo &video_mode,
		ITrackerListener *tracker_listener)
        : m_last_packet_type(DISCARD_PACKET)
        , m_lastPresentationTimestamp(0)
        , m_lastFieldID(0)
        , m_currentFrameStart(nullptr)
        , m_currentFrameBytesWritten(0)
		, m_frameProcessorThread(new PS3EyeFrameProcessorThread(video_mode, tracker_listener))
    {	
        // Point the write pointer at the start of the bayer buffer
        m_currentFrameStart= m_frameProcessorThread->getCompressedFrameBufferStart();
    }

	bool startUSBBulkTransfer(t_usb_device_handle usb_device_handle)
	{
		bool bStartedStream= false;

		// Send a request to the USBDeviceManager to start a bulk transfer stream for video frames.
		// This will spin up a thread in the USB manager for reading the incoming USB packets.
		// The usbBulkTransferCallback_workerThread() callback will be executed on this thread.
		USBTransferRequest request;
		memset(&request, 0, sizeof(USBTransferRequest));
		request.request_type= eUSBTransferRequestType::_USBRequestType_BulkTransfer;
		request.payload.start_bulk_transfer_bundle.usb_device_handle= usb_device_handle;
		request.payload.start_bulk_transfer_bundle.bAutoResubmit= true;
		request.payload.start_bulk_transfer_bundle.in_flight_transfer_packet_count= NUM_TRANSFERS;
		request.payload.start_bulk_transfer_bundle.transfer_packet_size= TRANSFER_SIZE;
		request.payload.start_bulk_transfer_bundle.on_data_callback= usbBulkTransferCallback_usbThread;
		request.payload.start_bulk_transfer_bundle.transfer_callback_userdata= this;

		USBTransferResult result= usb_device_submit_transfer_request_blocking(request);
		log_usb_result_code("v", result.payload.bulk_transfer_bundle.result_code); 
		if (result.result_type == eUSBTransferRequestType::_USBRequestType_StartBulkTransferBundle)
		{
			// Start decompressing the incoming video frames
			m_frameProcessorThread->startThread();
			bStartedStream= true;
		}

		return bStartedStream;
	}

	void stopUSBBulkTransfer(t_usb_device_handle usb_device_handle)
	{
		// Send a request to the USBDeviceManager to stop the bulk transfer stream for video frames
		USBTransferRequest request;
		memset(&request, 0, sizeof(USBTransferRequest));
		request.request_type= eUSBTransferRequestType::_USBRequestType_CancelBulkTransferBundle;
		request.payload.cancel_bulk_transfer_bundle.usb_device_handle= usb_device_handle;

		// This will block until the processing USB packet processing thread exits
		USBTransferResult result= usb_device_submit_transfer_request_blocking(request);
		assert(result.result_type == eUSBTransferRequestType::_USBRequestType_CancelBulkTransferBundle);
		log_usb_result_code("stopUSBBulkTransfer", result.payload.bulk_transfer_bundle.result_code);

		// Block until the frame processor halts itself
		m_frameProcessorThread->stopThread();
	}

    virtual ~PS3EyeUSBPacketProcessor()
    {
		delete m_frameProcessorThread;
    }

    static void usbBulkTransferCallback_usbThread(unsigned char *packet_data, int packet_length, void *userdata)
    {
        PS3EyeUSBPacketProcessor *processor= reinterpret_cast<PS3EyeUSBPacketProcessor *>(userdata);

        processor->packetScan_usbThread(packet_data, packet_length);
    }

protected:
    void packetScan_usbThread(unsigned char *data, int len)
    {
        uint32_t presentationTimeStamp; // a.k.a. "PTS"
        uint16_t fieldID; // a.k.a. "FID"
        int remaining_len = len;
        int payload_len;

        payload_len = 2048; // bulk type
        do 
		{
            len = std::min(remaining_len, payload_len);

            // Payloads are prefixed with a "USB Video Class" (UVC) style header. 
            // We consider a frame to start when the "Field ID Bit" (FID) toggles 
            // or the "Presentation Time Stamp" (PTS) changes.
            // A frame ends when EOF is set, and we've received
            // the correct number of bytes.

            // Verify UVC header.  Header length is always 12 
            if (data[0] != 12 || len < 12) 
            {
                PSVR_MT_LOG_DEBUG("packetScan_usbThread") << "bad header";
                goto discard;
            }

            // Check errors
            if (data[1] & UVC_STREAM_ERR) 
            {
                PSVR_MT_LOG_DEBUG("packetScan_usbThread") << "payload error";
                goto discard;
            }

            // Extract PTS and FID
            if (!(data[1] & UVC_STREAM_PTS))
            {
                PSVR_MT_LOG_DEBUG("packetScan_usbThread") << "PTS not present";
                goto discard;
            }

            presentationTimeStamp = (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2];
            fieldID = (data[1] & UVC_STREAM_FID) ? 1 : 0;

            // If PTS or FID has changed, start a new frame.
            if (presentationTimeStamp != m_lastPresentationTimestamp || fieldID != m_lastFieldID) 
            {
                if (m_last_packet_type == INTER_PACKET)
                {
                    // The last frame was incomplete, so don't keep it or we will glitch
                    frameAddData_usbThread(DISCARD_PACKET, NULL, 0);
                }

                m_lastPresentationTimestamp = presentationTimeStamp;
                m_lastFieldID = fieldID;
                frameAddData_usbThread(FIRST_PACKET, data + 12, len - 12);
            } 
            // If this packet is marked as EOF, end the frame
            else if (data[1] & UVC_STREAM_EOF) 
            {
                m_lastPresentationTimestamp = 0;
                if(m_currentFrameBytesWritten + len - 12 != m_frameProcessorThread->getCompressedFrameSizeBytes())
                {
                    goto discard;
                }
                frameAddData_usbThread(LAST_PACKET, data + 12, len - 12);
            }
            else 
            {
                // Add the data from this payload
                frameAddData_usbThread(INTER_PACKET, data + 12, len - 12);
            }


            // Done this payload
            goto scan_next;

        discard:
            // Discard data until a new frame starts.
            frameAddData_usbThread(DISCARD_PACKET, NULL, 0);

        scan_next:
            remaining_len -= len;
            data += len;
        } while (remaining_len > 0);
    }

    void frameAddData_usbThread(enum gspca_packet_type packet_type, const uint8_t *data, int len)
    {
        if (packet_type == FIRST_PACKET) 
        {
            m_currentFrameBytesWritten = 0;
        } 
        else
        {
            switch(m_last_packet_type)  // ignore warning.
            {
            case DISCARD_PACKET:
                if (packet_type == LAST_PACKET) 
                {
                    m_last_packet_type = packet_type;
                    m_currentFrameBytesWritten = 0;
                }
                return;
            case LAST_PACKET:
                return;
            default:
                break;
            }
        }

        /* append the packet to the frame buffer */
        if (len > 0)
        {
            if(m_currentFrameBytesWritten + len > m_frameProcessorThread->getCompressedFrameSizeBytes())
            {
                packet_type = DISCARD_PACKET;
                m_currentFrameBytesWritten = 0;
            }
            else
            {
                memcpy(m_currentFrameStart+m_currentFrameBytesWritten, data, len);
                m_currentFrameBytesWritten += len;
            }
        }

        m_last_packet_type = packet_type;

        if (packet_type == LAST_PACKET)
        {
            m_currentFrameBytesWritten = 0;
            m_currentFrameStart = m_frameProcessorThread->enqueueCompressedFrame();
        }
    }

private:
	// USB Packet Processing State
    enum gspca_packet_type m_last_packet_type;
    uint32_t m_lastPresentationTimestamp;
    uint16_t m_lastFieldID;
    uint8_t* m_currentFrameStart;
    uint32_t m_currentFrameBytesWritten;
    
	// Frame Decompression Thread
	PS3EyeFrameProcessorThread *m_frameProcessorThread;
};

//-- PS3EyeVideoDevice -----
PS3EyeProperties::PS3EyeProperties()
    : autogain(false)
    , gain(20)
    , exposure(120)
    , sharpness(0)
    , hue(143)
    , awb(false)
    , brightness(20)
    , contrast(37)
    , blueBalance(128)
    , redBalance(128)
    , greenBalance(128)
    , flip_h(false)
    , flip_v(false)
    , frame_width(0)
    , frame_height(0)
    , frame_rate(0)
{
}

PS3EyeVideoDevice::PS3EyeVideoDevice(USBDeviceEnumerator* enumerator)
    : m_properties()
    , m_is_streaming(false)
    , m_last_qued_frame_time(0.0)
    , m_usb_device_handle(usb_device_open(enumerator))
    , m_video_packet_processor(nullptr)
{
	memset(&m_properties, 0, sizeof(PS3EyeProperties));
}

PS3EyeVideoDevice::~PS3EyeVideoDevice()
{
    close();

    if(m_usb_device_handle != k_invalid_usb_device_handle)
    {
		usb_device_close(m_usb_device_handle);
		m_usb_device_handle= k_invalid_usb_device_handle;
    }
}

ePS3EyeVideoMode PS3EyeVideoDevice::findBestVideoMode(
	unsigned int w,
	unsigned int h,
	unsigned int frameRate)
{
	ePS3EyeVideoMode result_id= PS3EyeVideoMode_INVALID;

	for (int attempt = 0; attempt < 2; ++attempt)
	{
		for (int format_index= 0; format_index < PS3EyeVideoMode_COUNT; ++format_index)
		{
			const PS3EyeVideoModeInfo &info= k_supported_video_modes[format_index];

			if ((w == UNSPECIFIED_CAMERA_WIDTH || info.width == w) && 
				(h == UNSPECIFIED_CAMERA_HEIGHT || info.height == h) && 
				(frameRate == UNSPECIFIED_CAMERA_FPS || info.fps == frameRate))
			{
				result_id= static_cast<ePS3EyeVideoMode>(format_index);
				break;
			}
		}

		if (result_id != PS3EyeVideoMode_INVALID)
		{
			break;
		}
		else if (attempt == 0)
		{
			// Fallback to no FPS restriction on second pass
			frameRate= UNSPECIFIED_CAMERA_FPS;
		}
	}

	return result_id;
}

bool PS3EyeVideoDevice::open(
	ePS3EyeVideoMode desired_video_mode, 
	PS3EyeTrackerConfig &cfg, 
	ITrackerListener *tracker_listener)
{
	// USB device should be opened in the constructor
    if (m_usb_device_handle == k_invalid_usb_device_handle)
        return false;

	// Bailed if the camera stream is already started
    if (m_is_streaming)
		return true;

	assert(desired_video_mode >= 0 && desired_video_mode < PS3EyeVideoMode_COUNT);
	const PS3EyeVideoModeInfo &video_mode= k_supported_video_modes[desired_video_mode];

    // Initialize the camera
	init_camera(m_usb_device_handle, video_mode);
   
    if (video_mode.width == 320) // 320x240
        ov534_reg_write_array(m_usb_device_handle, bridge_start_qvga, ARRAY_SIZE(bridge_start_qvga));
    else // 640x480 
        ov534_reg_write_array(m_usb_device_handle, bridge_start_vga, ARRAY_SIZE(bridge_start_vga));

    if (video_mode.width == 320) // 320x240
        sccb_reg_write_array(m_usb_device_handle, sensor_start_qvga, ARRAY_SIZE(sensor_start_qvga));
    else // 640x480
        sccb_reg_write_array(m_usb_device_handle, sensor_start_vga, ARRAY_SIZE(sensor_start_vga));

	// Set the desired frame rate
    set_frame_rate(m_usb_device_handle, video_mode);

	// Remember the video frame properties
    m_properties.frame_width = video_mode.width;
    m_properties.frame_height = video_mode.height;
    m_properties.frame_rate = video_mode.fps;

	// Cache the property constraints for the current video format
	for (int prop_index = 0; prop_index < PSVRVideoProperty_COUNT; ++prop_index)
	{
		getVideoPropertyConstraint((PSVRVideoPropertyType)prop_index, m_videoPropertyConstraints[prop_index]);
	}

	// Apply video property settings stored in config onto the camera
	for (int prop_index = 0; prop_index < PSVRVideoProperty_COUNT; ++prop_index)
	{
		const PSVRVideoPropertyType prop_type = (PSVRVideoPropertyType)prop_index;
		const PSVRVideoPropertyConstraint &constraint= m_videoPropertyConstraints[prop_index];

		if (constraint.is_supported)
		{
			// Use the properties from the config if we used this video mode previously
			if (desired_video_mode == cfg.ps3eye_video_mode_index)
			{
				int currentValue= getVideoProperty(prop_type);
				int desiredValue= cfg.video_properties[prop_index];

				if (desiredValue != currentValue)
				{
					// Use the desired value if it is in-range
					if (desiredValue >= constraint.min_value &&
						desiredValue <= constraint.max_value)
					{
						setVideoProperty(prop_type, desiredValue);
					}
					// Otherwise update the config to use the current value
					else
					{
						cfg.video_properties[prop_index]= currentValue;
					}
				}
			}
			// Otherwise use the current value for the property
			// and update the config to match
			else
			{
				int currentValue= getVideoProperty(prop_type);

				if (currentValue >= constraint.min_value &&
					currentValue <= constraint.max_value)
				{
					cfg.video_properties[prop_index]= currentValue;
				}
				else
				{
					// If the current value is somehow out-of-range
					// fallback to the default value
					setVideoProperty(prop_type, constraint.default_value);
					cfg.video_properties[prop_index]= constraint.default_value;
				}
			}
		}
	}

	// Remember which video mode was last successfully opened
	cfg.ps3eye_video_mode_index= desired_video_mode;

	// Flip the image horizontally
	set_flip(m_usb_device_handle, true, false);

	// Turn on the "recording" LED
    ov534_set_led(m_usb_device_handle, true);

	// Tell the camera to start the video stream
    ov534_reg_write(m_usb_device_handle, 0xe0, 0x00);

    // Start the USB video packet bulk transfers and the frame processor thread
    m_video_packet_processor= new PS3EyeUSBPacketProcessor(video_mode, tracker_listener);
	m_is_streaming= m_video_packet_processor->startUSBBulkTransfer(m_usb_device_handle);

	return m_is_streaming;	
}

void PS3EyeVideoDevice::close()
{
    if(!m_is_streaming) 
		return;

	if (m_usb_device_handle != k_invalid_usb_device_handle)
	{
		// Tell the camera to stop the video stream
		ov534_reg_write(m_usb_device_handle, 0xe0, 0x09); 

		// Turn off the "recording" LED light
		ov534_set_led(m_usb_device_handle, false);
	}

	// Stop the USB video packet bulk transfers and the frame processor thread
    if (m_video_packet_processor != nullptr)
    {
		m_video_packet_processor->stopUSBBulkTransfer(m_usb_device_handle);
        delete m_video_packet_processor;
        m_video_packet_processor= nullptr;
    }

    m_is_streaming = false;
}

inline PSVRVideoPropertyConstraint create_property_constraint(
	int min_value,
	int max_value,
	int stepping_delta,
	int default_value,
	bool is_automatic,
	bool is_supported)
{
	PSVRVideoPropertyConstraint constraint;

	constraint.min_value= min_value;
	constraint.max_value= max_value;
	constraint.stepping_delta= stepping_delta;
	constraint.default_value= default_value;
	constraint.is_automatic= is_automatic;
	constraint.is_supported= is_supported;

	return constraint;
}

bool PS3EyeVideoDevice::getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const
{
	bool bSuccess= true;

	switch (property_type)
	{
    case PSVRVideoProperty_Brightness:
		outConstraint= create_property_constraint(0, 255, 1, 20, false, true);
		break;
	case PSVRVideoProperty_Contrast:
		outConstraint= create_property_constraint(0, 255, 1, 37, false, true);
		break;
	case PSVRVideoProperty_Hue:
		outConstraint= create_property_constraint(0, 255, 1, 143, false, true);
		break;
	case PSVRVideoProperty_Saturation:
		outConstraint= create_property_constraint(0, 0, 0, 0, false, false);
		break;
	case PSVRVideoProperty_Sharpness:
		outConstraint= create_property_constraint(0, 63, 1, 0, false, false);
		break;
	case PSVRVideoProperty_Gamma:
		outConstraint= create_property_constraint(0, 0, 0, 0, false, false);
		break;
	case PSVRVideoProperty_WhiteBalance:
		outConstraint= create_property_constraint(0, 1, 1, 0, false, true);
		break;
	case PSVRVideoProperty_RedBalance:
		outConstraint= create_property_constraint(0, 255, 1, 128, true, true);
		break;
	case PSVRVideoProperty_GreenBalance:
		outConstraint= create_property_constraint(0, 255, 1, 128, true, true);
		break;
	case PSVRVideoProperty_BlueBalance:
		outConstraint= create_property_constraint(0, 255, 1, 128, true, true);
		break;
	case PSVRVideoProperty_Gain:
		outConstraint= create_property_constraint(0, 63, 1, 20, true, true);
		break;
	case PSVRVideoProperty_Pan:
		outConstraint= create_property_constraint(0, 0, 0, 0, false, false);
		break;
	case PSVRVideoProperty_Tilt:
		outConstraint= create_property_constraint(0, 0, 0, 0, false, false);
		break;
	case PSVRVideoProperty_Roll:
		outConstraint= create_property_constraint(0, 0, 0, 0, false, false);
		break;
	case PSVRVideoProperty_Zoom:
		outConstraint= create_property_constraint(0, 0, 0, 0, false, false);
		break;
	case PSVRVideoProperty_Exposure:
		outConstraint= create_property_constraint(0, 255, 1, 120, false, true);
		break;
	case PSVRVideoProperty_Iris:
		outConstraint= create_property_constraint(0, 0, 0, 0, false, false);
		break;
	case PSVRVideoProperty_Focus:
		outConstraint= create_property_constraint(0, 0, 0, 0, false, false);
		break;
	default:
		bSuccess= false;
		break;
	}
	
	return bSuccess;
}

void PS3EyeVideoDevice::setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value)
{
	switch (property_type)
	{
    case PSVRVideoProperty_Brightness:
		setBrightness((unsigned char)desired_value);
		break;
	case PSVRVideoProperty_Contrast:
		setContrast((unsigned char)desired_value);
		break;
	case PSVRVideoProperty_Hue:
		setHue((unsigned char)desired_value);
		break;
	case PSVRVideoProperty_Saturation:
	case PSVRVideoProperty_Sharpness:
	case PSVRVideoProperty_Gamma:
		// not supported
		break;
	case PSVRVideoProperty_WhiteBalance:
		setAutoWhiteBalance(desired_value == 1);
		break;
	case PSVRVideoProperty_RedBalance:
		setRedBalance((unsigned char)desired_value);
		break;
	case PSVRVideoProperty_GreenBalance:
		setGreenBalance((unsigned char)desired_value);
		break;
	case PSVRVideoProperty_BlueBalance:
		setBlueBalance((unsigned char)desired_value);
		break;
	case PSVRVideoProperty_Gain:
		setGain((unsigned char)desired_value);
		break;
	case PSVRVideoProperty_Pan:
	case PSVRVideoProperty_Tilt:
	case PSVRVideoProperty_Roll:
	case PSVRVideoProperty_Zoom:
		// not supported
		break;
	case PSVRVideoProperty_Exposure:
		setExposure((unsigned char)desired_value);
		break;
	case PSVRVideoProperty_Iris:
	case PSVRVideoProperty_Focus:
		// not supported
		break;
	}
}

int PS3EyeVideoDevice::getVideoProperty(const PSVRVideoPropertyType property_type) const
{
	int value= 0;

	switch (property_type)
	{
    case PSVRVideoProperty_Brightness:
		value= getBrightness();
		break;
	case PSVRVideoProperty_Contrast:
		value= getContrast();
		break;
	case PSVRVideoProperty_Hue:
		value= getHue();
		break;
	case PSVRVideoProperty_Saturation:
	case PSVRVideoProperty_Sharpness:
	case PSVRVideoProperty_Gamma:
		// not supported
		break;
	case PSVRVideoProperty_WhiteBalance:
		value= getAutoWhiteBalance() ? 1 : 0;
		break;
	case PSVRVideoProperty_RedBalance:
		value= getRedBalance();
		break;
	case PSVRVideoProperty_GreenBalance:
		value= getGreenBalance();
		break;
	case PSVRVideoProperty_BlueBalance:
		value= getBlueBalance();
		break;
	case PSVRVideoProperty_Gain:
		value= getGain();
		break;
	case PSVRVideoProperty_Pan:
	case PSVRVideoProperty_Tilt:
	case PSVRVideoProperty_Roll:
	case PSVRVideoProperty_Zoom:
		// not supported
		break;
	case PSVRVideoProperty_Exposure:
		value= getExposure();
		break;
	case PSVRVideoProperty_Iris:
	case PSVRVideoProperty_Focus:
		// not supported
		break;
	}

	return value;
}

void PS3EyeVideoDevice::setAutogain(bool bAutoGain)
{
    // Cache the new auto-gain flag (the camera will converge to this)
    m_properties.autogain = bAutoGain;

    // Add an async task to set the autogain on the camera
    set_autogain(m_usb_device_handle, bAutoGain, m_properties.gain, m_properties.exposure);
}

void PS3EyeVideoDevice::setAutoWhiteBalance(bool val)
{
    // Cache the new AWB value (the camera will converge to this)
    m_properties.awb = val;

    // Add an async task to set the awb on the camera
    set_auto_white_balance(m_usb_device_handle, val);
}

void PS3EyeVideoDevice::setGain(unsigned char val)
{
    // Cache the new gain value (the camera will converge to this)
    m_properties.gain = val;

    // Add an async task to set the gain on the camera
    set_gain(m_usb_device_handle, val);
}

void PS3EyeVideoDevice::setExposure(unsigned char val)
{
    // Cache the new exposure value (the camera will converge to this)
    m_properties.exposure = val;

    // Add an async task to set the exposure on the camera
    set_exposure(m_usb_device_handle, val);
}


void PS3EyeVideoDevice::setSharpness(unsigned char val)
{
    // Cache the new sharpness value (the camera will converge to this)
    m_properties.sharpness = val;

    // Add an async task to set the sharpness on the camera
    set_sharpness(m_usb_device_handle, val);
}

void PS3EyeVideoDevice::setContrast(unsigned char val)
{
    // Cache the new contrast value (the camera will converge to this)
    m_properties.contrast = val;

    // Add an async task to set the sharpness on the camera
    set_contrast(m_usb_device_handle, val);
}

void PS3EyeVideoDevice::setBrightness(unsigned char val)
{
    // Cache the new brightness value (the camera will converge to this)
    m_properties.brightness = val;

    // Add an async task to set the sharpness on the camera
    set_brightness(m_usb_device_handle, val);
}

void PS3EyeVideoDevice::setHue(unsigned char val)
{
    // Cache the new hue value (the camera will converge to this)
    m_properties.hue = val;

    // Add an async task to set the sharpness on the camera
    set_hue(m_usb_device_handle, val);
}

void PS3EyeVideoDevice::setRedBalance(unsigned char val)
{
    // Cache the new red balance value (the camera will converge to this)
    m_properties.redBalance = val;

    // Add an async task to set the red balance on the camera
    set_red_balance(m_usb_device_handle, val);
}

void PS3EyeVideoDevice::setGreenBalance(unsigned char val)
{
    // Cache the new green balance value (the camera will converge to this)
    m_properties.greenBalance = val;

    // Add an async task to set the green balance on the camera
    set_green_balance(m_usb_device_handle, val);
}

void PS3EyeVideoDevice::setBlueBalance(unsigned char val)
{
    // Cache the new blue balance value (the camera will converge to this)
    m_properties.blueBalance = val;

    // Add an async task to set the red balance on the camera
    set_blue_balance(m_usb_device_handle, val);
}

void PS3EyeVideoDevice::setFlip(bool horizontal, bool vertical)
{
    // Cache the new camera flip flags (the camera will converge to this)
    m_properties.flip_h= horizontal;
    m_properties.flip_v= vertical;

    // Add an async task to set the red balance on the camera
    set_flip(m_usb_device_handle, horizontal, vertical);
}

bool PS3EyeVideoDevice::getUSBPortPath(char *out_identifier, size_t max_identifier_length) const
{
	return usb_device_get_port_path(m_usb_device_handle, out_identifier, max_identifier_length);
}

//-- private helpers ----
static void init_camera(
    t_usb_device_handle device_handle,
    const PS3EyeVideoModeInfo &video_mode)
{
    uint8_t sensor_id= 0;

	// Set the desired frame rate
	set_frame_rate(device_handle, video_mode);

    // reset bridge
    ov534_reg_write(device_handle, 0xe7, 0x3a);
    ov534_reg_write(device_handle, 0xe0, 0x08);
    Utility::sleep_ms(100);

    // initialize the sensor address
    ov534_reg_write(device_handle, OV534_REG_ADDRESS, 0x42);

	// reset sensor
    sccb_reg_write(device_handle, 0x12, 0x80);
    Utility::sleep_ms(10);

    // probe the sensor
	sccb_reg_read(device_handle, 0x0a);
	sensor_id = sccb_reg_read(device_handle, 0x0a) << 8;
	sccb_reg_read(device_handle, 0x0b);
	sensor_id |= sccb_reg_read(device_handle, 0x0b);
	PSVR_LOG_INFO("init_camera") <<  "PS3EYE Sensor ID: "
		<< std::hex << std::setfill('0') << std::setw(2) << sensor_id;

    // initialize 
    ov534_reg_write_array(device_handle, ov534_reg_initdata, ARRAY_SIZE(ov534_reg_initdata));
    ov534_set_led(device_handle, true);
    sccb_reg_write_array(device_handle, ov772x_reg_initdata, ARRAY_SIZE(ov772x_reg_initdata));
    ov534_reg_write(device_handle, 0xe0, 0x09);
    ov534_set_led(device_handle, false);
}

static void set_autogain(
    t_usb_device_handle device_handle, 
    bool bAutoGain, 
    uint8_t gain,
    uint8_t exposure)
{
    if (bAutoGain) 
    {
        sccb_reg_write(device_handle, 0x13, 0xf7); //AGC,AEC,AWB ON
        uint8_t read_reg_0x64_result= sccb_reg_read(device_handle, 0x64);
        sccb_reg_write(device_handle, 0x64, read_reg_0x64_result | 0x03);
    }
    else 
    {
        sccb_reg_write(device_handle, 0x13, 0xf0); //AGC,AEC,AWB OFF
        uint8_t read_reg_0x64_result= sccb_reg_read(device_handle, 0x64);
        sccb_reg_write(device_handle, 0x64, read_reg_0x64_result & 0xFC);
        set_gain(device_handle, gain);
        set_exposure(device_handle, exposure);
    }
}

static void set_auto_white_balance(
    t_usb_device_handle device_handle, 
    bool bAutoWhiteBalance)
{
    if (bAutoWhiteBalance)
    {
        sccb_reg_write(device_handle, 0x63, 0xe0); //AWB ON
    }
    else
    {
        sccb_reg_write(device_handle, 0x63, 0xAA); //AWB OFF
    }
}

static void set_gain(
    t_usb_device_handle device_handle, 
    unsigned char val)
{
    switch (val & 0x30)
    {
    case 0x00:
        val &= 0x0F;
        break;
    case 0x10:
        val &= 0x0F;
        val |= 0x30;
        break;
    case 0x20:
        val &= 0x0F;
        val |= 0x70;
        break;
    case 0x30:
        val &= 0x0F;
        val |= 0xF0;
        break;
    }

    sccb_reg_write(device_handle, 0x00, val);
}

static void set_exposure(
    t_usb_device_handle device_handle, 
    unsigned char val)
{
    sccb_reg_write(device_handle, 0x08, val >> 7);
    sccb_reg_write(device_handle, 0x10, val << 1);
}

static void set_sharpness(
    t_usb_device_handle device_handle,
    unsigned char val)
{
    sccb_reg_write(device_handle, 0x91, val);
    sccb_reg_write(device_handle, 0x8E, val);
}

static void set_contrast(
    t_usb_device_handle device_handle, 
    unsigned char val)
{
    sccb_reg_write(device_handle, 0x9C, val);
}

static void set_brightness(
    t_usb_device_handle device_handle, 
    unsigned char val)
{
    sccb_reg_write(device_handle, 0x9B, val);
}

static void set_hue(
    t_usb_device_handle device_handle,
    unsigned char val)
{
    sccb_reg_write(device_handle, 0x01, val);
}

static void set_red_balance(
    t_usb_device_handle device_handle,
    unsigned char val)
{
    sccb_reg_write(device_handle, 0x43, val);
}

static void set_green_balance(
    t_usb_device_handle device_handle,
    unsigned char val)
{
    sccb_reg_write(device_handle, 0x44, val);
}

static void set_blue_balance(
    t_usb_device_handle device_handle,
    unsigned char val)
{
    sccb_reg_write(device_handle, 0x42, val);
}

static void set_flip(
    t_usb_device_handle device_handle, 
    bool horizontal, 
    bool vertical)
{
	uint8_t read_reg_0x0C_result= sccb_reg_read(device_handle, 0x0C) & ~0xC0;
    uint8_t val= read_reg_0x0C_result;
    
	if (!horizontal) val |= 0x40;
    if (!vertical) val |= 0x80;

    sccb_reg_write(device_handle, 0x0C, val);
}

static void set_frame_rate(
    t_usb_device_handle device_handle, 
    const PS3EyeVideoModeInfo &video_mode)
{
    sccb_reg_write(device_handle, 0x11, video_mode.r11);
    sccb_reg_write(device_handle, 0x0d, video_mode.r0d);
    ov534_reg_write(device_handle, 0xe5, video_mode.re5);
}

static void sccb_reg_write(
    t_usb_device_handle device_handle, 
    uint8_t reg, 
    uint8_t val)
{
	ov534_reg_write(device_handle, OV534_REG_SUBADDR, reg);
	ov534_reg_write(device_handle, OV534_REG_WRITE, val);
	ov534_reg_write(device_handle, OV534_REG_OPERATION, OV534_OP_WRITE_3);
	
	if (!sccb_check_status(device_handle))
	{
		PSVR_LOG_WARNING("sccb_reg_write") << "failed";
	}
}

static void sccb_reg_write_array(
    t_usb_device_handle device_handle, 
    const uint8_t (*sequence)[2], 
    int sequenceLength)
{
    const uint8_t (*data)[2]= sequence;
    int len= sequenceLength;

	while (--len >= 0) 
	{
		if ((*data)[0] != 0xff) 
		{
			sccb_reg_write(device_handle, (*data)[0], (*data)[1]);
		}
		else 
		{
			sccb_reg_read(device_handle, (*data)[1]);
			sccb_reg_write(device_handle, 0xff, 0x00);
		}

		data++;
	}
}

static uint8_t sccb_reg_read(
    t_usb_device_handle device_handle, 
    uint16_t reg)
{
    ov534_reg_write(device_handle, OV534_REG_SUBADDR, (uint8_t)reg);
    ov534_reg_write(device_handle, OV534_REG_OPERATION, OV534_OP_WRITE_2);
    if (!sccb_check_status(device_handle))
    {
		PSVR_LOG_WARNING("sccb_reg_read") << "failed 1st write";
        return 0;
    }

    ov534_reg_write(device_handle, OV534_REG_OPERATION, OV534_OP_READ_2);
    if (!sccb_check_status(device_handle))
    {
		PSVR_LOG_WARNING("sccb_reg_read") << "failed 2nd write";
        return 0;
    }

    return ov534_reg_read(device_handle, OV534_REG_READ);
}

static bool sccb_check_status(t_usb_device_handle device_handle)
{
	for (int i = 0; i < 5; i++) 
	{
		uint8_t data = ov534_reg_read(device_handle, OV534_REG_STATUS);

		switch (data) 
		{
		case 0x00:
			return true;
		case 0x04:
			return false;
		case 0x03:
			break;
		default:
            PSVR_LOG_WARNING("sccb_check_status") << "unknown sccb status 0x"
                << std::hex << std::setfill('0') << std::setw(2) << data;
		}
	}

	return false;
}

static void ov534_reg_write(
    t_usb_device_handle device_handle, 
    uint16_t reg, 
    uint8_t val)
{
    USBTransferRequest request;
    memset(&request, 0, sizeof(USBTransferRequest));
    request.request_type= eUSBTransferRequestType::_USBRequestType_ControlTransfer;
    request.payload.control_transfer.usb_device_handle= device_handle;
    request.payload.control_transfer.bmRequestType = 
        USB_ENDPOINT_OUT | USB_REQUEST_TYPE_VENDOR | USB_RECIPIENT_DEVICE;
    request.payload.control_transfer.bRequest= 0x01;
    request.payload.control_transfer.wValue= 0x00;
    request.payload.control_transfer.wIndex = reg;
    request.payload.control_transfer.data[0]= val;
    request.payload.control_transfer.wLength= 1;
    request.payload.control_transfer.timeout= 500;

    // Submit the async USB control transfer request...
	USBTransferResult result= usb_device_submit_transfer_request_blocking(request);
    assert(result.result_type == eUSBTransferResultType::_USBResultType_ControlTransfer);

    //... whose result we get notified of here
    if (result.payload.control_transfer.result_code != eUSBResultCode::_USBResultCode_Completed)
    {
        // Tell the callback the transfer failed :(
        // The callback parameter value will be the error code
        log_usb_result_code("ov534_reg_write", result.payload.control_transfer.result_code);
    }
}

static void ov534_reg_write_array(
    t_usb_device_handle device_handle,
    const uint8_t (*sequence)[2], 
    int sequenceLength)
{
    const uint8_t (*data)[2]= sequence;
    int len= sequenceLength;

	while (--len >= 0) 
	{
		ov534_reg_write(device_handle, (*data)[0], (*data)[1]);
		data++;
	}
}

static uint8_t ov534_reg_read(
    t_usb_device_handle device_handle, 
    uint16_t reg)
{
    USBTransferRequest request;
    memset(&request, 0, sizeof(USBTransferRequest));
    request.request_type = eUSBTransferRequestType::_USBRequestType_ControlTransfer;
    request.payload.control_transfer.usb_device_handle= device_handle;
    request.payload.control_transfer.bmRequestType =
        USB_ENDPOINT_IN | USB_REQUEST_TYPE_VENDOR | USB_RECIPIENT_DEVICE;
    request.payload.control_transfer.bRequest = 0x01;
    request.payload.control_transfer.wValue = 0x00;
    request.payload.control_transfer.wIndex = reg;
    request.payload.control_transfer.wLength = 1;
    request.payload.control_transfer.timeout = 500;

    // Submit the async USB control transfer request...
	USBTransferResult result= usb_device_submit_transfer_request_blocking(request);
    assert(result.result_type == eUSBTransferResultType::_USBResultType_ControlTransfer);

    //... whose result we get notified of here
    if (result.payload.control_transfer.result_code == eUSBResultCode::_USBResultCode_Completed)
    {
        assert(result.payload.control_transfer.dataLength == 1);

        // Tell the callback the transfer completed ok :)
        // The callback parameter value will value we read from the camera register.
        return static_cast<uint8_t>(result.payload.control_transfer.data[0]);
    }
    else
    {
        log_usb_result_code("ov534_reg_read", result.payload.control_transfer.result_code);

        return 0;
    }
}

/* Two bits control LED: 0x21 bit 7 and 0x23 bit 7.
 * (direction and output)? */
static void ov534_set_led(
    t_usb_device_handle device_handle,
    bool bLedOn)
{
	uint8_t read_reg_result;

    // Change register value 0x21
	read_reg_result= ov534_reg_read(device_handle, 0x21); 
	read_reg_result|= 0x80;
    ov534_reg_write(device_handle, 0x21, read_reg_result);

    // Change register value 0x23
    read_reg_result= ov534_reg_read(device_handle, 0x23);
    if (bLedOn)
        read_reg_result|= 0x80;
    else
        read_reg_result&= ~0x80;

    ov534_reg_write(device_handle, 0x23, read_reg_result);

    if (!bLedOn)
    {
        read_reg_result= ov534_reg_read(device_handle, 0x21);
		read_reg_result&= ~0x80;
        ov534_reg_write(device_handle, 0x21, read_reg_result);
    }
}

static void log_usb_result_code(const char *function_name, eUSBResultCode result_code)
{
    switch (result_code)
    {
    // Success Codes
    case eUSBResultCode::_USBResultCode_Started:
        PSVR_LOG_INFO(function_name) << "request started";
        break;
    case eUSBResultCode::_USBResultCode_Canceled:
        PSVR_LOG_INFO(function_name) << "request canceled";
        break;
    case eUSBResultCode::_USBResultCode_Completed:
        PSVR_LOG_INFO(function_name) << "request completed";
        break;

    // Failure Codes
    case eUSBResultCode::_USBResultCode_GeneralError:
        PSVR_LOG_INFO(function_name) << "request failed: general request error";
        break;
    case eUSBResultCode::_USBResultCode_BadHandle:
        PSVR_LOG_INFO(function_name) << "request failed: bad USB device handle";
        break;
    case eUSBResultCode::_USBResultCode_NoMemory:
        PSVR_LOG_INFO(function_name) << "request failed: no memory";
        break;
    case eUSBResultCode::_USBResultCode_SubmitFailed:
        PSVR_LOG_INFO(function_name) << "request failed: submit failed";
        break;
    case eUSBResultCode::_USBResultCode_DeviceNotOpen:
        PSVR_LOG_INFO(function_name) << "request failed: device not open";
        break;
    case eUSBResultCode::_USBResultCode_TransferNotActive:
        PSVR_LOG_INFO(function_name) << "request failed: transfer not active";
        break;
    case eUSBResultCode::_USBResultCode_TransferAlreadyStarted:
        PSVR_LOG_INFO(function_name) << "request failed: transfer already started";
        break;
    case eUSBResultCode::_USBResultCode_Overflow:
        PSVR_LOG_INFO(function_name) << "request failed: transfer overflow";
        break;
    case eUSBResultCode::_USBResultCode_Pipe:
        PSVR_LOG_INFO(function_name) << "request failed: transfer pipe error";
        break;
    case eUSBResultCode::_USBResultCode_TimedOut:
        PSVR_LOG_INFO(function_name) << "request failed: transfer timed out";
        break;
    };
}