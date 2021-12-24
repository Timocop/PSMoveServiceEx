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
		controller_position_smoothing = 0.f;
		controller_position_prediction = 0.f;
		controller_position_prediction_history = 5;
		ignore_pose_from_one_tracker = true;
		optical_tracking_timeout = 100;
		tracker_sleep_ms = 1;
		use_bgr_to_hsv_lookup_table = true;
		exclude_opposed_cameras = false;
		min_valid_projection_area = 6;
		occluded_area_on_loss_size = 4.f;
		occluded_area_ignore_trackers = 0;
		occluded_area_regain_projection_size = 32.f;
		min_points_in_contour = 4;
		max_tracker_position_deviation = 12.0f;
		disable_roi = false;
		optimized_roi = true;
		roi_edge_offset = 4;
		global_forward_degrees = 270.f; // Down -Z by default
	};

	virtual const boost::property_tree::ptree config2ptree();
	virtual void ptree2config(const boost::property_tree::ptree &pt);

	bool isLoaded;

	int virtual_tracker_count;
	float controller_position_smoothing;
	float controller_position_prediction;
	int controller_position_prediction_history;
	bool ignore_pose_from_one_tracker;
	int optical_tracking_timeout;
	int tracker_sleep_ms;
	bool use_bgr_to_hsv_lookup_table;
	bool exclude_opposed_cameras;
	float min_valid_projection_area;
	float occluded_area_on_loss_size;
	int occluded_area_ignore_trackers;
	float occluded_area_regain_projection_size;
	int min_points_in_contour;
	float max_tracker_position_deviation;
	bool disable_roi;
	bool optimized_roi;
	int roi_edge_offset;
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
	};

	virtual const boost::property_tree::ptree config2ptree();
	virtual void ptree2config(const boost::property_tree::ptree &pt);

	bool isLoaded;

	int virtual_hmd_count;
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
private:
	void RestartService();
};

#endif // APP_STAGE_ADVANCED_SETTINGS_H