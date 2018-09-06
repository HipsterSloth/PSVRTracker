#ifndef UTILITY_H
#define UTILITY_H

#include "stdlib.h" // size_t
#include <string>
#include <vector>

//-- macros -----
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(_A) (sizeof(_A) / sizeof((_A)[0]))
#endif

//-- utility methods -----
namespace Utility
{
    template <typename t_index>
    inline bool is_index_valid(const t_index index, const t_index count)
    {
        return index >= 0 && index < count;
    }

	template <class T> void SafeRelease(T **ppT)
	{
		if (*ppT)
		{
			(*ppT)->Release();
			*ppT = nullptr;
		}
	}

	template <class T> void SafeReleaseAllCount(T **ppT)
	{
		if (*ppT)
		{
			unsigned long e = (*ppT)->Release();

			while (e)
			{
				e = (*ppT)->Release();
			}

			*ppT = nullptr;
		}
	}

    unsigned char int32_to_int8_verify(int value);
    bool convert_wcs_to_mbs(const wchar_t *wc_string, char *out_mb_string, const size_t mb_buffer_size);
	bool convert_mbs_to_wcs(const char *mb_string, wchar_t *out_wc_string, const size_t wc_buffer_size);

    /// Formats a string into the given target buffer
    /// \param buffer The target buffer to write in to
    /// \param buffer_size The max number of bytes that can be written to the buffer
    /// \param format The formatting string that will be written to the buffer
    /// \return The number of characters successfully written
    int format_string(char *buffer, size_t buffer_size, const char *format, ...);

    /// Converts the given bluetooth string into standarized format. Ex) "XX-XX-XX-XX-XX-XX" to "xx:xx:xx:xx:xx:xx"
    /// \param addr The given bluetooth address we want to convers
    /// \param bLowercase true if we want the result to be in lower case or not
    /// \param separator The separator character to use (typically ':')
    /// \param result The buffer for the resulting string
    /// \param result_max_size The capacity of the result buffer. Must be at least 17 bytes.
    /// \return false if the result buffer is too small or there was a parsing error
    bool bluetooth_cstr_address_normalize(
        const char *addr, bool bLowercase, char separator, 
        char *result, size_t result_max_size);

    /// Converts the given 6-octet raw byte bluetooth address to a std::string, with ':' as the octet separator
    /// \param addr_buff An array whose first 6 bytes contain the 6 octets of a bluetooth address (reverse order)
    /// \return The bluetooth address as a string
    std::string bluetooth_byte_addr_to_string(const unsigned char* addr_buff);

    /// Converts the given string containing an address of the form "%x:%x:%x:%x:%x:%x" into a 6-octet byte array
    /// \param addr The source bluetooth string
    /// \param addr_buff The target object buffer
    /// \param addr_buf_size The size of the target buffer
    /// \return true of the string could be parse and the target array could hold the octets
    bool bluetooth_string_address_to_bytes(const std::string &addr, unsigned char *addr_buff, const int addr_buf_size);

    /// Sets the name of the current thread
    void set_current_thread_name(const char* thread_name);

    /// Sleeps the current thread for the given number of milliseconds
    void sleep_ms(int milliseconds);	

	/// Get the current working directory
	std::string get_current_directory();

	/// Get the location of resource files 
	std::string get_resource_directory();

    /// Get the "home" location where config files can be stored
    std::string get_home_directory();

    /// Attempts to create a directory at the given path. Returns 0 on success
    bool create_directory(const std::string &path);

	/// Attempts to build a list of all of the files in a directory
	bool fetch_filenames_in_directory(std::string path, std::vector<std::string> &out_filenames);

    /// Returns true if a file exists
    bool file_exists(const std::string& filename);
};

#endif // UTILITY_H