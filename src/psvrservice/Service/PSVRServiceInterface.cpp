//-- includes -----
#include "PSVRClient.h"
#include "Logger.h"
#include <memory>

//-- implementation -----
SharedVideoFrameBuffer::SharedVideoFrameBuffer()
	: m_buffer_name()
	, m_width(-1)
	, m_height(-1)
	, m_stride(-1)
	, m_buffer(nullptr)
    , m_section_count(0)
	, m_frame_index(-1)
{}

SharedVideoFrameBuffer::~SharedVideoFrameBuffer()
{
	dispose();
}

bool SharedVideoFrameBuffer::initialize(const char *buffer_name, int width, int height, int stride, int section_count)
{
	bool bSuccess = false;

	if (m_buffer == nullptr)
	{			
		PSVR_LOG_INFO("SharedVideoFrameBuffer::initialize()") << "Allocating video frame buffer: " << m_buffer_name;

		size_t buffer_size= computeVideoBufferSize(section_count, stride, height);
		m_buffer= new unsigned char[buffer_size];
		std::memset(m_buffer, 0, buffer_size);			
		
		m_buffer_name = buffer_name;
	   
		m_width = width;
		m_height = height;
		m_stride = stride;
        m_section_count = section_count;
		m_frame_index = 0;

		bSuccess = true;
	}

	return bSuccess;
}

void SharedVideoFrameBuffer::dispose()
{
	if (m_buffer != nullptr)
	{
		PSVR_LOG_INFO("SharedVideoFrameBuffer::dispose()") << "Deallocating video frame buffer: " << m_buffer_name;
		
		delete[] m_buffer;
		m_buffer = nullptr;
		
		m_buffer_name= "";
		m_width= -1;
		m_height= -1;
		m_stride= -1;
        m_section_count= 0;
		m_frame_index= -1;
	}
}

void SharedVideoFrameBuffer::writeVideoFrame(PSVRVideoFrameSection section, const unsigned char *buffer)
{
	size_t buffer_size = computeVideoBufferSize(1, m_stride, m_height);
    size_t buffer_offset = computeVideoBufferSize(section, m_stride, m_height);

	++m_frame_index;
	std::memcpy(m_buffer+buffer_offset, buffer, buffer_size);
}

const unsigned char *SharedVideoFrameBuffer::getBuffer(PSVRVideoFrameSection section) const
{
	return m_buffer;
}

unsigned char *SharedVideoFrameBuffer::getBufferMutable(PSVRVideoFrameSection section)
{
	return m_buffer;
}

size_t SharedVideoFrameBuffer::computeVideoBufferSize(int section_count, int stride, int height)
{
	return section_count*stride*height;
}