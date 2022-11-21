#ifndef TRACKER_MANAGER_H
#define TRACKER_MANAGER_H

//-- includes -----
#include <memory>
#include <deque>
#include "DeviceTypeManager.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include "PSMoveConfig.h"
#include "MathUtility.h"
#include "MathEigen.h"
#include <vector>

//-- typedefs -----

class ServerTrackerView;
typedef std::shared_ptr<ServerTrackerView> ServerTrackerViewPtr;

//-- definitions -----
struct TrackerProfile
{
	float frame_width;
	//float frame_height;
	float frame_rate;
	float exposure;
    float gain;
    CommonHSVColorRangeTable color_preset_table;

    inline void clear()
    {
		frame_width = 0.f;
		// frame_height = 0.f;
		frame_rate = 0.f;
		exposure = 0.f;
        gain = 0;
		color_preset_table.table_name.clear();
        for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
        {
            color_preset_table.color_presets[preset_index].clear();
        }
    }
};

class TrackerManagerConfig : public PSMoveConfig
{
public:
    static const int CONFIG_VERSION;

    TrackerManagerConfig(const std::string &fnamebase = "TrackerManagerConfig");

    virtual const boost::property_tree::ptree config2ptree();
    virtual void ptree2config(const boost::property_tree::ptree &pt);

	int virtual_tracker_count;
	float controller_position_smoothing;
	float controller_position_prediction;
	int controller_position_prediction_history;
	bool ignore_pose_from_one_tracker;
    long version;
    int optical_tracking_timeout;
	int thread_sleep_ms;
	bool use_bgr_to_hsv_lookup_table;
	bool exclude_opposed_cameras;
	float min_valid_projection_area;
	float occluded_area_on_loss_size;
	int occluded_area_ignore_num_trackers;
	float occluded_area_regain_projection_size;
	bool projection_collision_avoid;
	float projection_collision_offset;
	bool average_position_cache_enabled;
	float average_position_cache_cell_size;
	float average_position_cache_avg_size;
	int min_points_in_contour;
	float max_tracker_position_deviation;
	bool disable_roi;
	bool optimized_roi;
	int roi_edge_offset;
    TrackerProfile default_tracker_profile;
	float global_forward_degrees;

	float playspace_orientation_yaw;
	float playspace_position_x;
	float playspace_position_y;
	float playspace_position_z;

	CommonDeviceVector get_global_forward_axis() const;
	CommonDeviceVector get_global_backward_axis() const;
	CommonDeviceVector get_global_right_axis() const;
	CommonDeviceVector get_global_left_axis() const;
	CommonDeviceVector get_global_up_axis() const;
	CommonDeviceVector get_global_down_axis() const;
};

class TrackerManager : public DeviceTypeManager
{
public:
    TrackerManager();

    bool startup() override;
	void poll_devices();

    void closeAllTrackers();

    static const int k_max_devices = PSMOVESERVICE_MAX_TRACKER_COUNT;
    int getMaxDevices() const override
    {
        return TrackerManager::k_max_devices;
    }

    ServerTrackerViewPtr getTrackerViewPtr(int device_id) const;

    inline void saveDefaultTrackerProfile(const TrackerProfile *profile)
    {
        cfg.default_tracker_profile = *profile;
        cfg.save();
    }

    inline const TrackerProfile *getDefaultTrackerProfile() const
    {
        return &cfg.default_tracker_profile; 
    }

    inline const TrackerManagerConfig& getConfig() const
    {
        return cfg;
    }
	inline TrackerManagerConfig *getConfigMutable()
	{
		return &cfg;
	}

	inline static bool trackersSynced()
	{
		return m_trackersSynced;
	}

	inline static void setTrackerReady(int deviceId)
	{
		m_isTrackerReady[deviceId] = true;
	}

	inline static bool isTrackerPollAllowed()
	{
		return m_isTrackerPollAllowed;
	}

    eCommonTrackingColorID allocateTrackingColorID();
    bool claimTrackingColorID(const class ServerControllerView *controller_view, eCommonTrackingColorID color_id);
    bool claimTrackingColorID(const class ServerHMDView *hmd_view, eCommonTrackingColorID color_id);
    void freeTrackingColorID(eCommonTrackingColorID color_id);
	void applyPlayspaceOffsets(Eigen::Vector3f &poseVec, Eigen::Quaternionf &postQuat);
	void applyPlayspaceOffsets(Eigen::Vector3f &poseVec, Eigen::Quaternionf &postQuat, bool move_pos, bool rotate_pos, bool rotate_ang);

protected:
    bool can_update_connected_devices() override;
    void mark_tracker_list_dirty();

    DeviceEnumerator *allocate_device_enumerator() override;
    void free_device_enumerator(DeviceEnumerator *) override;
    ServerDeviceView *allocate_device_view(int device_id) override;
	int getListUpdatedResponseType() override;

private:
    std::deque<eCommonTrackingColorID> m_available_color_ids;
    TrackerManagerConfig cfg;
    bool m_tracker_list_dirty;

	static bool m_trackersSynced;
	static bool m_isTrackerReady[TrackerManager::k_max_devices];
	static bool m_isTrackerPollAllowed;
};

#endif // TRACKER_MANAGER_H