//-- includes -----
#include "Logger.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <ostream>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': localtime
#endif

//-- globals -----
PSVRLogSeverityLevel g_min_log_level= PSVRLogSeverityLevel_info;
std::ostream *g_console_stream= nullptr;
std::ostream *g_file_stream = nullptr;
std::mutex *g_logger_mutex = nullptr;

//-- public implementation -----
void log_init(PSVRLogSeverityLevel log_level, const std::string &log_filename)
{
	log_dispose();

    g_min_log_level= log_level;

	g_console_stream = new std::ostream(std::cout.rdbuf());
	if (log_filename.length() > 0)
	{
		g_file_stream = new std::ofstream(log_filename, std::ofstream::out);
	}
	g_logger_mutex = new std::mutex();
}

void log_dispose()
{
	if (g_console_stream != nullptr)
	{
		g_console_stream->flush();
		delete g_console_stream;
		g_console_stream = nullptr;
	}

	if (g_file_stream != nullptr)
	{
		g_console_stream->flush();
		delete g_file_stream;
		g_file_stream = nullptr;
	}

	if (g_logger_mutex != nullptr)
	{
		delete g_logger_mutex;
		g_logger_mutex = nullptr;
	}
}

bool log_can_emit_level(PSVRLogSeverityLevel level)
{
    return (level >= g_min_log_level);
}

std::string log_get_timestamp_prefix()
{
    auto now = std::chrono::system_clock::now();
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds);
    time_t in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << "[" << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S") << "." << milliseconds.count() << "]: ";

    return ss.str();
}

//-- member functions -----
LoggerStream::LoggerStream(bool bEmit) :
	m_bEmitLine(bEmit)
{
}

LoggerStream::~LoggerStream()
{
	write_line();
}

void LoggerStream::write_line()
{
	if (m_bEmitLine)
	{
		const std::string line = m_lineBuffer.str();

		if (g_console_stream != nullptr)
		{
			*g_console_stream << line << std::endl;
		}

		if (g_file_stream != nullptr)
		{
			*g_file_stream << line << std::endl;
		}
	}
}

ThreadSafeLoggerStream::ThreadSafeLoggerStream(bool bEmit) :
	LoggerStream(bEmit)
{
}

void ThreadSafeLoggerStream::write_line()
{
	std::lock_guard<std::mutex> lock(*g_logger_mutex);

	LoggerStream::write_line();
}