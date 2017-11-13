#ifndef CLIENT_COLOR_H
#define CLIENT_COLOR_H

//-- includes -----
/** 
\addtogroup PSVRClient_CAPI 
@{ 
*/

/// The available tracking color types
typedef enum
{
    PSVRTrackingColorType_INVALID=-1,

    PSVRTrackingColorType_Magenta,    ///< R:0xFF, G:0x00, B:0xFF
    PSVRTrackingColorType_Cyan,       ///< R:0x00, G:0xFF, B:0xFF
    PSVRTrackingColorType_Yellow,     ///< R:0xFF, G:0xFF, B:0x00
    PSVRTrackingColorType_Red,        //</ R:0xFF, G:0x00, B:0x00
    PSVRTrackingColorType_Green,      ///< R:0x00, G:0xFF, B:0x00
    PSVRTrackingColorType_Blue,       ///< R:0x00, G:0x00, B:0xFF
	
	PSVRTrackingColorType_MaxColorTypes
} PSVRTrackingColorType;

/// A 1D range of values.
typedef struct
{
    float center, range;
} PSVRRangef;

/// A range of colors in the Hue-Saturation-Value color space
typedef struct
{
    PSVRRangef hue_range;
	PSVRRangef saturation_range;
	PSVRRangef value_range;
} PSVR_HSVColorRange;

/// A table of color filters for each tracking color
typedef struct
{
	char table_name[64];
	PSVR_HSVColorRange color_presets[PSVRTrackingColorType_MaxColorTypes];	
} PSVR_HSVColorRangeTable;

/** 
@} 
*/ 

#endif // CLIENT_COLOR_H
