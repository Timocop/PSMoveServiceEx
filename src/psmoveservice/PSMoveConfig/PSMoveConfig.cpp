#include "PSMoveConfig.h"
#include "ServerLog.h"
#include "DeviceInterface.h"
#include "ServerUtility.h"
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <iostream>

// Format: {hue center, hue range}, {sat center, sat range}, {val center, val range}
// All hue angles are 60 degrees apart to maximize hue separation for 6 max tracked colors.
// Hue angle reference: http://i.imgur.com/PKjgfFXm.jpg 
// Hue angles divide by 2 for opencv which remaps hue range to [0,180]
const CommonHSVColorRange g_default_color_presets[] = {
    { { 300 / 2, 10 },	{ 255, 32 },	{ 255, 32 } }, // Magenta
    { { 180 / 2, 10 },	{ 255, 32 },	{ 255, 32 } }, // Cyan
    { { 60 / 2, 10 },	{ 255, 32 },	{ 255, 32 } }, // Yellow
    { { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Red
    { { 120 / 2, 10 },	{ 255, 32 },	{ 255, 32 } }, // Green
	{ { 240 / 2, 10 },	{ 255, 32 },	{ 255, 32 } }, // Blue

	// Custom colors. Mostly for virtual devices because it turns of PSmove LEDs.
	{ { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Custom0
	{ { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Custom1
	{ { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Custom2
	{ { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Custom3
	{ { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Custom4
	{ { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Custom5
	{ { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Custom6
	{ { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Custom7
	{ { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Custom8
	{ { 0, 10 },		{ 255, 32 },	{ 255, 32 } }, // Custom9
};
const CommonHSVColorRange *k_default_color_presets = g_default_color_presets;

PSMoveConfig::PSMoveConfig(const std::string &fnamebase)
	: ConfigFileBase(fnamebase)
	, bHasLoaded(false)
{
}

const std::string
PSMoveConfig::getConfigPath()
{
    const char *homedir;
#ifdef _WIN32
    size_t homedir_buffer_req_size;
    char homedir_buffer[512];
    getenv_s(&homedir_buffer_req_size, homedir_buffer, "APPDATA");
    assert(homedir_buffer_req_size <= sizeof(homedir_buffer));
    homedir= homedir_buffer;
#else
    homedir = getenv("HOME");
    // if run as root, use system-wide data directory
    if (geteuid() == 0) {
        homedir = "/etc/psmoveservice";
    }
#endif
    
    boost::filesystem::path configpath(homedir);
    configpath /= "PSMoveService";
    boost::filesystem::create_directory(configpath);
    configpath /= ConfigFileBase + ".json";
    std::cout << "Config file name: " << configpath << std::endl;
    return configpath.string();
}

void
PSMoveConfig::save()
{
	if (!bHasLoaded)
		return;

    boost::property_tree::write_json(getConfigPath(), config2ptree());
}

bool
PSMoveConfig::load()
{
    bool bLoadedOk = false;
    boost::property_tree::ptree pt;
    std::string configPath = getConfigPath();

    if ( boost::filesystem::exists( configPath ) )
    {
        boost::property_tree::read_json(configPath, pt);
        ptree2config(pt);
        bLoadedOk = true;

		if (!bHasLoaded)
		{
			// Open the file
			std::ifstream file(configPath);
			if (file.is_open()) {
				// Read the file content into a stringstream
				std::ostringstream configStream;
				configStream << file.rdbuf();

				// Close the file
				file.close();
				
				SERVER_LOG_FILE_INFO(configPath.c_str()) << configStream.str();
			}
		}
    }

	bHasLoaded = true;

    return bLoadedOk;
}

void
PSMoveConfig::writeColorPropertyPresetTable(
	const CommonHSVColorRangeTable *table,
    boost::property_tree::ptree &pt)
{
	const char *profile_name= table->table_name.c_str();

    writeColorPreset(pt, profile_name, "magenta", &table->color_presets[eCommonTrackingColorID::Magenta]);
    writeColorPreset(pt, profile_name, "cyan", &table->color_presets[eCommonTrackingColorID::Cyan]);
    writeColorPreset(pt, profile_name, "yellow", &table->color_presets[eCommonTrackingColorID::Yellow]);
    writeColorPreset(pt, profile_name, "red", &table->color_presets[eCommonTrackingColorID::Red]);
    writeColorPreset(pt, profile_name, "green", &table->color_presets[eCommonTrackingColorID::Green]);
	writeColorPreset(pt, profile_name, "blue", &table->color_presets[eCommonTrackingColorID::Blue]);

	writeColorPreset(pt, profile_name, "custom0", &table->color_presets[eCommonTrackingColorID::Custom0]);
	writeColorPreset(pt, profile_name, "custom1", &table->color_presets[eCommonTrackingColorID::Custom1]);
	writeColorPreset(pt, profile_name, "custom2", &table->color_presets[eCommonTrackingColorID::Custom2]);
	writeColorPreset(pt, profile_name, "custom3", &table->color_presets[eCommonTrackingColorID::Custom3]);
	writeColorPreset(pt, profile_name, "custom4", &table->color_presets[eCommonTrackingColorID::Custom4]);
	writeColorPreset(pt, profile_name, "custom5", &table->color_presets[eCommonTrackingColorID::Custom5]);
	writeColorPreset(pt, profile_name, "custom6", &table->color_presets[eCommonTrackingColorID::Custom6]);
	writeColorPreset(pt, profile_name, "custom7", &table->color_presets[eCommonTrackingColorID::Custom7]);
	writeColorPreset(pt, profile_name, "custom8", &table->color_presets[eCommonTrackingColorID::Custom8]);
	writeColorPreset(pt, profile_name, "custom9", &table->color_presets[eCommonTrackingColorID::Custom9]);
}

void
PSMoveConfig::readColorPropertyPresetTable(
	const boost::property_tree::ptree &pt,
	CommonHSVColorRangeTable *table)
{
	const char *profile_name= table->table_name.c_str();

    readColorPreset(pt, profile_name, "magenta", &table->color_presets[eCommonTrackingColorID::Magenta], &k_default_color_presets[eCommonTrackingColorID::Magenta]);
    readColorPreset(pt, profile_name, "cyan", &table->color_presets[eCommonTrackingColorID::Cyan], &k_default_color_presets[eCommonTrackingColorID::Cyan]);
    readColorPreset(pt, profile_name, "yellow", &table->color_presets[eCommonTrackingColorID::Yellow], &k_default_color_presets[eCommonTrackingColorID::Yellow]);
    readColorPreset(pt, profile_name, "red", &table->color_presets[eCommonTrackingColorID::Red], &k_default_color_presets[eCommonTrackingColorID::Red]);
    readColorPreset(pt, profile_name, "green", &table->color_presets[eCommonTrackingColorID::Green], &k_default_color_presets[eCommonTrackingColorID::Green]);
	readColorPreset(pt, profile_name, "blue", &table->color_presets[eCommonTrackingColorID::Blue], &k_default_color_presets[eCommonTrackingColorID::Blue]);

	readColorPreset(pt, profile_name, "custom0", &table->color_presets[eCommonTrackingColorID::Custom0], &k_default_color_presets[eCommonTrackingColorID::Custom0]);
	readColorPreset(pt, profile_name, "custom1", &table->color_presets[eCommonTrackingColorID::Custom1], &k_default_color_presets[eCommonTrackingColorID::Custom1]);
	readColorPreset(pt, profile_name, "custom2", &table->color_presets[eCommonTrackingColorID::Custom2], &k_default_color_presets[eCommonTrackingColorID::Custom2]);
	readColorPreset(pt, profile_name, "custom3", &table->color_presets[eCommonTrackingColorID::Custom3], &k_default_color_presets[eCommonTrackingColorID::Custom3]);
	readColorPreset(pt, profile_name, "custom4", &table->color_presets[eCommonTrackingColorID::Custom4], &k_default_color_presets[eCommonTrackingColorID::Custom4]);
	readColorPreset(pt, profile_name, "custom5", &table->color_presets[eCommonTrackingColorID::Custom5], &k_default_color_presets[eCommonTrackingColorID::Custom5]);
	readColorPreset(pt, profile_name, "custom6", &table->color_presets[eCommonTrackingColorID::Custom6], &k_default_color_presets[eCommonTrackingColorID::Custom6]);
	readColorPreset(pt, profile_name, "custom7", &table->color_presets[eCommonTrackingColorID::Custom7], &k_default_color_presets[eCommonTrackingColorID::Custom7]);
	readColorPreset(pt, profile_name, "custom8", &table->color_presets[eCommonTrackingColorID::Custom8], &k_default_color_presets[eCommonTrackingColorID::Custom8]);
	readColorPreset(pt, profile_name, "custom9", &table->color_presets[eCommonTrackingColorID::Custom9], &k_default_color_presets[eCommonTrackingColorID::Custom9]);
}

void
PSMoveConfig::writeTrackingColor(
	boost::property_tree::ptree &pt,
	int tracking_color_id)
{
	switch (tracking_color_id)
	{
	case eCommonTrackingColorID::INVALID_COLOR:
		pt.put("tracking_color", "invalid");
		break;
	case eCommonTrackingColorID::Magenta:
		pt.put("tracking_color", "magenta");
		break;
	case eCommonTrackingColorID::Cyan:
		pt.put("tracking_color", "cyan");
		break;
	case eCommonTrackingColorID::Yellow:
		pt.put("tracking_color", "yellow");
		break;
	case eCommonTrackingColorID::Red:
		pt.put("tracking_color", "red");
		break;
	case eCommonTrackingColorID::Green:
		pt.put("tracking_color", "green");
		break;
	case eCommonTrackingColorID::Blue:
		pt.put("tracking_color", "blue");
		break;
	case eCommonTrackingColorID::Custom0:
		pt.put("tracking_color", "custom0");
		break;
	case eCommonTrackingColorID::Custom1:
		pt.put("tracking_color", "custom1");
		break;
	case eCommonTrackingColorID::Custom2:
		pt.put("tracking_color", "custom2");
		break;
	case eCommonTrackingColorID::Custom3:
		pt.put("tracking_color", "custom3");
		break;
	case eCommonTrackingColorID::Custom4:
		pt.put("tracking_color", "custom4");
		break;
	case eCommonTrackingColorID::Custom5:
		pt.put("tracking_color", "custom5");
		break;
	case eCommonTrackingColorID::Custom6:
		pt.put("tracking_color", "custom6");
		break;
	case eCommonTrackingColorID::Custom7:
		pt.put("tracking_color", "custom7");
		break;
	case eCommonTrackingColorID::Custom8:
		pt.put("tracking_color", "custom8");
		break;
	case eCommonTrackingColorID::Custom9:
		pt.put("tracking_color", "custom9");
		break;
	default:
		assert(false && "unreachable");
	}
}

int 
PSMoveConfig::readTrackingColor(
	const boost::property_tree::ptree &pt)
{
	std::string tracking_color_string = pt.get<std::string>("tracking_color", "invalid");
	int tracking_color_id = eCommonTrackingColorID::INVALID_COLOR;

	if (tracking_color_string == "magenta")
	{
		tracking_color_id = eCommonTrackingColorID::Magenta;
	}
	else if (tracking_color_string == "cyan")
	{
		tracking_color_id = eCommonTrackingColorID::Cyan;
	}
	else if (tracking_color_string == "yellow")
	{
		tracking_color_id = eCommonTrackingColorID::Yellow;
	}
	else if (tracking_color_string == "red")
	{
		tracking_color_id = eCommonTrackingColorID::Red;
	}
	else if (tracking_color_string == "green")
	{
		tracking_color_id = eCommonTrackingColorID::Green;
	}
	else if (tracking_color_string == "blue")
	{
		tracking_color_id = eCommonTrackingColorID::Blue;
	}
	else if (tracking_color_string == "custom0")
	{
		tracking_color_id = eCommonTrackingColorID::Custom0;
	}
	else if (tracking_color_string == "custom1")
	{
		tracking_color_id = eCommonTrackingColorID::Custom1;
	}
	else if (tracking_color_string == "custom2")
	{
		tracking_color_id = eCommonTrackingColorID::Custom2;
	}
	else if (tracking_color_string == "custom3")
	{
		tracking_color_id = eCommonTrackingColorID::Custom3;
	}
	else if (tracking_color_string == "custom4")
	{
		tracking_color_id = eCommonTrackingColorID::Custom4;
	}
	else if (tracking_color_string == "custom5")
	{
		tracking_color_id = eCommonTrackingColorID::Custom5;
	}
	else if (tracking_color_string == "custom6")
	{
		tracking_color_id = eCommonTrackingColorID::Custom6;
	}
	else if (tracking_color_string == "custom7")
	{
		tracking_color_id = eCommonTrackingColorID::Custom7;
	}
	else if (tracking_color_string == "custom8")
	{
		tracking_color_id = eCommonTrackingColorID::Custom8;
	}
	else if (tracking_color_string == "custom9")
	{
		tracking_color_id = eCommonTrackingColorID::Custom9;
	}

	return tracking_color_id;
}

static void
writeColorPropertyPreset(
    boost::property_tree::ptree &pt,
    const char *profile_name,
    const char *color_name,
    const char *property_name,
    float value)
{
    char full_property_name[256];

    if (profile_name != nullptr && profile_name[0] != '\0')
    {
        ServerUtility::format_string(full_property_name, sizeof(full_property_name), "%s.color_preset.%s.%s", 
            profile_name, color_name, property_name);
    }
    else
    {
        ServerUtility::format_string(full_property_name, sizeof(full_property_name), "color_preset.%s.%s", 
            color_name, property_name);
    }
    pt.put(full_property_name, value);
}

void
PSMoveConfig::writeColorPreset(
    boost::property_tree::ptree &pt,
    const char *profile_name,
    const char *color_name,
    const CommonHSVColorRange *colorPreset)
{
    writeColorPropertyPreset(pt, profile_name, color_name, "hue_center", colorPreset->hue_range.center);
    writeColorPropertyPreset(pt, profile_name, color_name, "hue_range", colorPreset->hue_range.range);
    writeColorPropertyPreset(pt, profile_name, color_name, "saturation_center", colorPreset->saturation_range.center);
    writeColorPropertyPreset(pt, profile_name, color_name, "saturation_range", colorPreset->saturation_range.range);
    writeColorPropertyPreset(pt, profile_name, color_name, "value_center", colorPreset->value_range.center);
    writeColorPropertyPreset(pt, profile_name, color_name, "value_range", colorPreset->value_range.range);
}

static void
readColorPropertyPreset(
    const boost::property_tree::ptree &pt,
    const char *profile_name,
    const char *color_name,
    const char *property_name,
    float &out_value,
    const float default_value)
{
    char full_property_name[256];

    if (profile_name != nullptr && profile_name[0] != '\0')
    {
        ServerUtility::format_string(full_property_name, sizeof(full_property_name), "%s.color_preset.%s.%s", 
            profile_name, color_name, property_name);
    }
    else
    {
        ServerUtility::format_string(full_property_name, sizeof(full_property_name), "color_preset.%s.%s",
            color_name, property_name);
    }
    out_value = pt.get<float>(full_property_name, default_value);
}

void
PSMoveConfig::readColorPreset(
    const boost::property_tree::ptree &pt,
    const char *profile_name,
    const char *color_name,
    CommonHSVColorRange *outColorPreset,
    const CommonHSVColorRange *defaultPreset)
{
    readColorPropertyPreset(pt, profile_name, color_name, "hue_center", outColorPreset->hue_range.center, defaultPreset->hue_range.center);
    readColorPropertyPreset(pt, profile_name, color_name, "hue_range", outColorPreset->hue_range.range, defaultPreset->hue_range.range);
    readColorPropertyPreset(pt, profile_name, color_name, "saturation_center", outColorPreset->saturation_range.center, defaultPreset->saturation_range.center);
    readColorPropertyPreset(pt, profile_name, color_name, "saturation_range", outColorPreset->saturation_range.range, defaultPreset->saturation_range.range);
    readColorPropertyPreset(pt, profile_name, color_name, "value_center", outColorPreset->value_range.center, defaultPreset->value_range.center);
    readColorPropertyPreset(pt, profile_name, color_name, "value_range", outColorPreset->value_range.range, defaultPreset->value_range.range);
}