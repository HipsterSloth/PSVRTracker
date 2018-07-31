#ifndef PSVR_CONFIG_H
#define PSVR_CONFIG_H

//-- includes -----
#include "ClientColor_CAPI.h"
#include "PSVRClient_CAPI.h"
#include <string>

#ifdef _MSC_VER
    #pragma warning (push)
    #pragma warning (disable: 4996) // This function or variable may be unsafe
    #pragma warning (disable: 4244) // 'return': conversion from 'const int64_t' to 'float', possible loss of data
    #pragma warning (disable: 4715) // configuru::Config::operator[]': not all control paths return a value
#endif
#include <configuru.hpp>
#ifdef _MSC_VER
    #pragma warning (pop)
#endif

//-- constants -----
extern const PSVR_HSVColorRange *k_default_color_presets;

//-- definitions -----
/*
Note that PSVRConfig is an abstract class because it has 2 pure virtual functions.
Child classes must add public member variables that store the config data,
as well as implement writeToJSON and readFromJSON that use pt[key]= value and
pt.get_or<type>(), respectively, to convert between member variables and the
property tree. See tests/test_config.cpp for an example.
*/
class PSVRConfig {
public:
    PSVRConfig(const std::string &fnamebase = std::string("PSVRConfig"));
    void save();
	void save(const std::string &path);
    bool load();
	bool load(const std::string &path);
    
    std::string ConfigFileBase;

    virtual const configuru::Config writeToJSON() = 0;  // Implement by each device class' own Config
    virtual void readFromJSON(const configuru::Config &pt) = 0;  // Implement by each device class' own Config
    
	static void writeMonoTrackerIntrinsics(
		configuru::Config &pt,
		const PSVRMonoTrackerIntrinsics &tracker_intrinsics);
	static void readMonoTrackerIntrinsics(
		const configuru::Config &pt,
		PSVRMonoTrackerIntrinsics &tracker_intrinsics);

	static void writeStereoTrackerIntrinsics(
		configuru::Config &pt,
		const PSVRStereoTrackerIntrinsics &tracker_intrinsics);
	static void readStereoTrackerIntrinsics(
		const configuru::Config &pt,
		PSVRStereoTrackerIntrinsics &tracker_intrinsics);

    static void writeDistortionCoefficients(
        configuru::Config &pt,
        const char *coefficients_name,
        const PSVRDistortionCoefficients *coefficients);
    static void readDistortionCoefficients(
        const configuru::Config &pt,
        const char *coefficients_name,
        PSVRDistortionCoefficients *outCoefficients,
        const PSVRDistortionCoefficients *defaultCoefficients);

    static void writeMatrix3d(
        configuru::Config &pt,
        const char *matrix_name,
        const PSVRMatrix3d &matrix);
    static void readMatrix3d(
        const configuru::Config &pt,
        const char *matrix_name,
        PSVRMatrix3d &outMatrix);

    static void writeMatrix34d(
        configuru::Config &pt,
        const char *matrix_name,
        const PSVRMatrix34d &matrix);
    static void readMatrix34d(
        const configuru::Config &pt,
        const char *matrix_name,
        PSVRMatrix34d &outMatrix);

    static void writeMatrix4d(
        configuru::Config &pt,
        const char *matrix_name,
        const PSVRMatrix4d &matrix);
    static void readMatrix4d(
        const configuru::Config &pt,
        const char *matrix_name,
        PSVRMatrix4d &outMatrix);

    static void writeVector3d(
        configuru::Config &pt,
        const char *vector_name,
        const PSVRVector3d &vector);
    static void readVector3d(
        const configuru::Config &pt,
        const char *vector_name,
        PSVRVector3d &outVector);

    static void writeColorPreset(
        configuru::Config &pt,
        const char *profile_name,
        const char *color_name,
        const PSVR_HSVColorRange *colorPreset);
    static void readColorPreset(
        const configuru::Config &pt,
        const char *profile_name,
        const char *color_name,
        PSVR_HSVColorRange *outColorPreset,
        const PSVR_HSVColorRange *defaultPreset);

	static void writeColorPropertyPresetTable(
		const PSVR_HSVColorRangeTable *table,
		configuru::Config &pt);
	static void readColorPropertyPresetTable(
		const configuru::Config &pt,
		PSVR_HSVColorRangeTable *table);

	static void writeTrackingColor(configuru::Config &pt, int tracking_color_id);
	static int readTrackingColor(const configuru::Config &pt);

private:
    const std::string getConfigPath();
};

#endif // PSVR_CONFIG_H
