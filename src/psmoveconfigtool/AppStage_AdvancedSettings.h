#ifndef APP_STAGE_ADVANCED_SETTINGS_H
#define APP_STAGE_ADVANCED_SETTINGS_H

//-- includes -----
#include "AppStage.h"
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <array>
#include <chrono>
#include <map>
#include <utility> // std::pair

class PSMoveConfig {
public:
	PSMoveConfig(const std::string &fnamebase = std::string("PSMoveConfig"));
	void save();
	bool load();

	std::string ConfigFileBase;
	std::map<std::string, std::string> cfg_map;

	virtual const boost::property_tree::ptree config2ptree() = 0;  // Implement by each device class' own Config
	virtual void ptree2config(const boost::property_tree::ptree &pt) = 0;  // Implement by each device class' own Config

	void map_flatten(const boost::property_tree::ptree& pt, std::string key);

private:
	const std::string getConfigPath();
};

//-- definitions -----
class TrackerConfig : public PSMoveConfig
{
public:
	TrackerConfig(const std::string &fnamebase = "TrackerManagerConfig")
		: PSMoveConfig(fnamebase)
	{
		isLoaded = false;

		virtual_tracker_count = 0;
		ignore_pose_from_one_tracker = true;
		tracker_sync_mode = 0;
		optical_tracking_timeout = 100;
		thread_sleep_ms = 1;
		use_bgr_to_hsv_lookup_table = true;
		exclude_opposed_cameras = false;
		min_valid_projection_area = 6;
		occluded_area_on_loss_size = 4.f;
		occluded_area_ignore_num_trackers = 0;
		occluded_area_regain_projection_size = 32.f;
		average_position_cache_enabled = true;
		average_position_cache_cell_size = 5.f;
		average_position_cache_avg_size = 30.f;
		average_position_cache_limit = 1000.f;
		projection_collision_avoid = true;
		projection_collision_offset = 5.f;
		min_points_in_contour = 4;
		max_tracker_position_deviation = 15.0f;
		disable_roi = false;
		autoscale_roi = true;
		optimized_roi = true;
		roi_size = 32;
		roi_search_size = 164;
		roi_edge_offset = 4;
		position_interpolation = true;
		angular_interpolation = true;
		thread_maximum_framrate = 200;
		global_forward_degrees = 270.f; // Down -Z by default
	};

	virtual const boost::property_tree::ptree config2ptree();
	virtual void ptree2config(const boost::property_tree::ptree &pt);

	bool isLoaded;

	int virtual_tracker_count;
	bool ignore_pose_from_one_tracker;
	int tracker_sync_mode;
	int optical_tracking_timeout;
	int thread_sleep_ms;
	bool use_bgr_to_hsv_lookup_table;
	bool exclude_opposed_cameras;
	float min_valid_projection_area;
	float occluded_area_on_loss_size;
	int occluded_area_ignore_num_trackers;
	float occluded_area_regain_projection_size;
	bool average_position_cache_enabled;
	float average_position_cache_cell_size;
	float average_position_cache_avg_size;
	float average_position_cache_limit;
	bool projection_collision_avoid;
	float projection_collision_offset;
	int min_points_in_contour;
	float max_tracker_position_deviation;
	bool disable_roi;
	bool autoscale_roi;
	bool optimized_roi;
	int roi_size;
	int roi_search_size;
	int roi_edge_offset;
	bool position_interpolation;
	bool angular_interpolation;
	int thread_maximum_framrate;
	float global_forward_degrees;
};

class ControllerConfig : public PSMoveConfig
{
public:
	ControllerConfig(const std::string &fnamebase = "ControllerManagerConfig")
		: PSMoveConfig(fnamebase)
	{
		isLoaded = false;

		virtual_controller_count = 0;
	};

	virtual const boost::property_tree::ptree config2ptree();
	virtual void ptree2config(const boost::property_tree::ptree &pt);

	bool isLoaded;

	int virtual_controller_count;
};

class HMDConfig : public PSMoveConfig
{
public:
	HMDConfig(const std::string &fnamebase = "HMDManagerConfig")
		: PSMoveConfig(fnamebase)
	{
		isLoaded = false;

		virtual_hmd_count = 0;
		enable_morpheus = true;
	};

	virtual const boost::property_tree::ptree config2ptree();
	virtual void ptree2config(const boost::property_tree::ptree &pt);

	bool isLoaded;

	int virtual_hmd_count;
	bool enable_morpheus;
};

class DeviceConfig : public PSMoveConfig
{
public:
	DeviceConfig(const std::string &fnamebase = "DeviceManagerConfig")
		: PSMoveConfig(fnamebase)
	{
		isLoaded = false;

		controller_reconnect_interval = 1000;
		tracker_reconnect_interval = 10000;
		hmd_reconnect_interval = 10000;
		gamepad_api_enabled = true;
		gamepad_api_xinput_only = true;
		platform_api_enabled = true;
	};

	virtual const boost::property_tree::ptree config2ptree();
	virtual void ptree2config(const boost::property_tree::ptree &pt);

	bool isLoaded;

	int controller_reconnect_interval;
	int tracker_reconnect_interval;
	int hmd_reconnect_interval;
	bool gamepad_api_enabled;
	bool gamepad_api_xinput_only;
	bool platform_api_enabled;
};



class AppStage_AdvancedSettings : public AppStage
{
public:    
	AppStage_AdvancedSettings(class App *app);

    virtual bool init(int argc, char** argv) override;
    virtual void enter() override;
    virtual void exit() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    enum eMainMenuState
    {
        inactive,
        idle,
    };
    eMainMenuState m_menuState;

	TrackerConfig cfg_tracker;
	ControllerConfig cfg_controller;
	HMDConfig cfg_hmd;
	DeviceConfig cfg_device;
private:
	void RestartService();

	int m_tabSelectedTab;
};

#endif // APP_STAGE_ADVANCED_SETTINGS_H