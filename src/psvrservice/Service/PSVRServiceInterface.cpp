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
    , m_section_count(0)
	, m_frame_index(-1)
{
	m_buffer[0]= nullptr;
	m_buffer[1]= nullptr;
}

SharedVideoFrameBuffer::~SharedVideoFrameBuffer()
{
	dispose();
}

bool SharedVideoFrameBuffer::initialize(const char *buffer_name, int width, int height, int stride, int section_count)
{
	bool bSuccess = false;

	if (m_buffer[0] == nullptr && m_buffer[1] == nullptr)
	{			
		PSVR_LOG_INFO("SharedVideoFrameBuffer::initialize()") << "Allocating video frame buffer: " << m_buffer_name;

		size_t buffer_size= computeVideoBufferSize(section_count, stride, height);
		m_buffer[0]= new unsigned char[buffer_size];
		m_buffer[1]= new unsigned char[buffer_size];
		std::memset(m_buffer[0], 0, buffer_size);
		std::memset(m_buffer[1], 0, buffer_size);
		
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
	if (m_buffer[0] != nullptr && m_buffer[1] != nullptr)
	{
		PSVR_LOG_INFO("SharedVideoFrameBuffer::dispose()") << "Deallocating video frame buffer: " << m_buffer_name;
		
		delete[] m_buffer[0];
		delete[] m_buffer[1];
		m_buffer[0] = nullptr;
		m_buffer[1] = nullptr;
		
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
	const size_t buffer_size = computeVideoBufferSize(1, m_stride, m_height);
    const size_t buffer_offset = computeVideoBufferSize(section, m_stride, m_height);
	const int write_buffer_index= m_frame_index % 2;

	std::memcpy(m_buffer[write_buffer_index]+buffer_offset, buffer, buffer_size);
}

void SharedVideoFrameBuffer::finalizeVideoFrameWrite()
{
	++m_frame_index;
}

const unsigned char *SharedVideoFrameBuffer::getBuffer(PSVRVideoFrameSection section) const
{
	const int read_buffer_index= (m_frame_index + 1) % 2;

	return m_buffer[read_buffer_index] + computeVideoBufferSize(static_cast<int>(section), m_stride, m_height);
}

unsigned char *SharedVideoFrameBuffer::getBufferMutable(PSVRVideoFrameSection section)
{
	return const_cast<unsigned char *>(getBuffer(section));
}

size_t SharedVideoFrameBuffer::computeVideoBufferSize(int section_count, int stride, int height)
{
	return section_count*stride*height;
}