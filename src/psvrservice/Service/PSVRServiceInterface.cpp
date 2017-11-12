//-- includes -----
#include "PSVRClient.h"
#include "Logger.h"
#include <memory>

//-- implementation -----
SharedVideoFrameBuffer::SharedVideoFrameBuffer()
	: m_shared_memory_name()
	, m_width(-1)
	, m_height(-1)
	, m_stride(-1)
	, m_buffer(nullptr)
	, m_frame_index(-1)
{}

SharedVideoFrameBuffer::~SharedVideoFrameBuffer()
{
	dispose();
}

bool SharedVideoFrameBuffer::initialize(const char *buffer_name, int width, int height, int stride)
{
	bool bSuccess = false;

	if (m_buffer == nullptr)
	{			
		PSVR_LOG_INFO("SharedVideoFrameBuffer::initialize()") << "Allocating video frame buffer: " << buffer_name;

		size_t buffer_size= computeVideoBufferSize(stride, height);
		m_buffer= new unsigned char[buffer_size];
		std::memset(m_buffer, 0, buffer_size);			
		
		m_shared_memory_name = buffer_name;
	   
		m_width = width;
		m_height = height;
		m_stride = stride;
		m_frame_index = 0;

		bSuccess = true;
	}

	return bSuccess;
}

void SharedVideoFrameBuffer::dispose()
{
	if (m_buffer != nullptr)
	{
		PSVR_LOG_INFO("SharedVideoFrameBuffer::dispose()") << "Deallocating video frame buffer: " << buffer_name;
		
		delete[] m_buffer;
		m_buffer = nullptr;
		
		m_shared_memory_name= "";
		m_width= -1;
		m_height= -1;
		m_stride= -1;
		m_frame_index= -1;
	}
}

void SharedVideoFrameBuffer::writeVideoFrame(const unsigned char *buffer)
{
	size_t buffer_size = computeVideoBufferSize(m_stride, m_height);

	++m_frame_index;
	std::memcpy(m_buffer, buffer, buffer_size);
}

const unsigned char *SharedVideoFrameBuffer::getBuffer() const
{
	return m_buffer;
}

unsigned char *SharedVideoFrameBuffer::getBufferMutable()
{
	return m_buffer;
}

size_t SharedVideoFrameBuffer::computeVideoBufferSize(int stride, int height)
{
	return stride*height;
}