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
	, m_write_thread_frame_index(-1)
	, m_read_thread_frame_index(-1)
{
	m_write_thread_buffer= nullptr;
	m_read_thread_buffer= nullptr;
}

SharedVideoFrameBuffer::~SharedVideoFrameBuffer()
{
	dispose();
}

bool SharedVideoFrameBuffer::initialize(const char *buffer_name, int width, int height, int stride, int section_count)
{
	bool bSuccess = false;

	if (m_write_thread_buffer == nullptr)
	{			
		PSVR_LOG_INFO("SharedVideoFrameBuffer::initialize()") << "Allocating video frame buffer: " << m_buffer_name;

		size_t buffer_size= computeVideoBufferSize(section_count, stride, height);
		m_write_thread_buffer= new unsigned char[buffer_size];
		m_read_thread_buffer= new unsigned char[buffer_size];
		std::memset(m_write_thread_buffer, 0, buffer_size);
		std::memset(m_read_thread_buffer, 0, buffer_size);
		
		m_buffer_name = buffer_name;
	   
		m_width = width;
		m_height = height;
		m_stride = stride;
        m_section_count = section_count;
		m_write_thread_frame_index = -1;
		m_read_thread_frame_index= -1;

		bSuccess = true;
	}

	return bSuccess;
}

void SharedVideoFrameBuffer::dispose()
{
	if (m_write_thread_buffer != nullptr && m_read_thread_buffer != nullptr)
	{
		PSVR_LOG_INFO("SharedVideoFrameBuffer::dispose()") << "Deallocating video frame buffer: " << m_buffer_name;
		
		delete[] m_write_thread_buffer;
		delete[] m_read_thread_buffer;
		m_write_thread_buffer = nullptr;
		
		m_buffer_name= "";
		m_width= -1;
		m_height= -1;
		m_stride= -1;
        m_section_count= 0;
		m_write_thread_frame_index= -1;
		m_read_thread_frame_index= -1;
	}
}

void SharedVideoFrameBuffer::writeMonoVideoFrame(const unsigned char *buffer)
{
	std::lock_guard<std::mutex> lock(m_buffer_mutex);

	const size_t buffer_size = computeVideoBufferSize(1, m_stride, m_height);
	const size_t primary_buffer_offset = computeVideoBufferSize(PSVRVideoFrameSection_Primary, m_stride, m_height);

	std::memcpy(m_write_thread_buffer+primary_buffer_offset, buffer, buffer_size);

	++m_write_thread_frame_index;
}

void SharedVideoFrameBuffer::writeStereoVideoFrame(const unsigned char *left_buffer, const unsigned char *right_buffer)
{
	std::lock_guard<std::mutex> lock(m_buffer_mutex);

	const size_t buffer_size = computeVideoBufferSize(1, m_stride, m_height);
    const size_t left_buffer_offset = computeVideoBufferSize(PSVRVideoFrameSection_Left, m_stride, m_height);
	const size_t right_buffer_offset = computeVideoBufferSize(PSVRVideoFrameSection_Right, m_stride, m_height);

	std::memcpy(m_write_thread_buffer+left_buffer_offset, left_buffer, buffer_size);
	std::memcpy(m_write_thread_buffer+right_buffer_offset, right_buffer, buffer_size);

	++m_write_thread_frame_index;
}

const unsigned char *SharedVideoFrameBuffer::fetchBufferSection(PSVRVideoFrameSection section)
{
	// Copy the buffer from the write thread if we don't have the latest buffer already
	const int last_write_index= m_write_thread_frame_index.load();
	if (last_write_index != m_read_thread_frame_index)
	{
		std::lock_guard<std::mutex> lock(m_buffer_mutex);
		size_t buffer_size= computeVideoBufferSize(m_section_count, m_stride, m_height);

		std::memcpy(m_read_thread_buffer, m_write_thread_buffer, buffer_size);
		m_read_thread_frame_index= last_write_index;
	}

	return m_read_thread_buffer + computeVideoBufferSize(static_cast<int>(section), m_stride, m_height);
}

size_t SharedVideoFrameBuffer::computeVideoBufferSize(int section_count, int stride, int height)
{
	return section_count*stride*height;
}