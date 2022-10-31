//-- includes -----
#include "TrackerManager.h"
#include "TrackerDeviceEnumerator.h"
#include "VirtualTrackerEnumerator.h"
#include "ControllerManager.h"
#include "DeviceManager.h"
#include "HMDManager.h"
#include "ServerLog.h"
#include "ServerControllerView.h"
#include "ServerHMDView.h"
#include "ServerTrackerView.h"
#include "ServerDeviceView.h"
#include "MathUtility.h"
#include "PSMoveProtocol.pb.h"

//-- constants -----

//-- Tracker Manager Config -----
const int TrackerManagerConfig::CONFIG_VERSION = 3;

TrackerManagerConfig::TrackerManagerConfig(const std::string &fnamebase)
    : PSMoveConfig(fnamebase)
{
	virtual_tracker_count = 0;
	controller_position_smoothing = 0.f;
	controller_position_prediction = 0.0f;
	controller_position_prediction_history = 5;
	ignore_pose_from_one_tracker = true;
	optical_tracking_timeout= 100;
	thread_sleep_ms = 1;
	use_bgr_to_hsv_lookup_table = true;
	exclude_opposed_cameras = false;
	min_valid_projection_area = 6;
	occluded_area_on_loss_size = 4.f;
	occluded_area_ignore_trackers = 0;
	projection_collision_avoid = true;
	projection_collision_offset = 5.0f;
	average_position_cache_enabled = false;
	average_position_cache_cell_size = 15.f;
	average_position_cache_avg_size = 30.f;
	min_points_in_contour = 4;
	max_tracker_position_deviation = 15.0f;
	disable_roi = false;
	optimized_roi = true;
	roi_edge_offset = 4;
	default_tracker_profile.frame_width = 640;
	//default_tracker_profile.frame_height = 480;
	default_tracker_profile.frame_rate = 30;
	default_tracker_profile.exposure = 32;
	default_tracker_profile.gain = 32;
	default_tracker_profile.color_preset_table.table_name= "default_tracker_profile";
	global_forward_degrees = 270.f; // Down -Z by default

	playspace_orientation_yaw = 0.f;
	playspace_position_x = 0.f;
	playspace_position_y = 0.f;
	playspace_position_z = 0.f;

	for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
	{
		default_tracker_profile.color_preset_table.color_presets[preset_index] = k_default_color_presets[preset_index];
	}
};

const boost::property_tree::ptree
TrackerManagerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("version", TrackerManagerConfig::CONFIG_VERSION);

	pt.put("virtual_tracker_count", virtual_tracker_count);
	pt.put("controller_position_smoothing", controller_position_smoothing);
	pt.put("controller_position_prediction", controller_position_prediction);
	pt.put("controller_position_prediction_history", controller_position_prediction_history);
	pt.put("ignore_pose_from_one_tracker", ignore_pose_from_one_tracker);
    pt.put("optical_tracking_timeout", optical_tracking_timeout);
	pt.put("use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table);
	pt.put("thread_sleep_ms", thread_sleep_ms);

	pt.put("excluded_opposed_cameras", exclude_opposed_cameras);

	pt.put("min_valid_projection_area", min_valid_projection_area);
	pt.put("occluded_area_on_loss_size", occluded_area_on_loss_size);
	pt.put("occluded_area_ignore_trackers", occluded_area_ignore_trackers);
	pt.put("occluded_area_regain_projection_size", occluded_area_regain_projection_size);
	pt.put("projection_collision_avoid", projection_collision_avoid);
	pt.put("projection_collision_offset", projection_collision_offset);
	pt.put("average_position_cache_enabled", average_position_cache_enabled);
	pt.put("average_position_cache_cell_size", average_position_cache_cell_size);
	pt.put("average_position_cache_avg_size", average_position_cache_avg_size);
	pt.put("min_points_in_contour", min_points_in_contour);
	pt.put("max_tracker_position_deviation", max_tracker_position_deviation);

	pt.put("disable_roi", disable_roi);
	pt.put("optimized_roi", optimized_roi);
	pt.put("roi_edge_offset", roi_edge_offset);

	pt.put("default_tracker_profile.frame_width", default_tracker_profile.frame_width);
	//pt.put("default_tracker_profile.frame_height", default_tracker_profile.frame_height);
	pt.put("default_tracker_profile.frame_rate", default_tracker_profile.frame_rate);
    pt.put("default_tracker_profile.exposure", default_tracker_profile.exposure);
    pt.put("default_tracker_profile.gain", default_tracker_profile.gain);

	pt.put("global_forward_degrees", global_forward_degrees);

	pt.put("playspace_orientation_yaw", playspace_orientation_yaw);
	pt.put("playspace_position_x", playspace_position_x);
	pt.put("playspace_position_y", playspace_position_y);
	pt.put("playspace_position_z", playspace_position_z);

	writeColorPropertyPresetTable(&default_tracker_profile.color_preset_table, pt);

    return pt;
}

void
TrackerManagerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    version = pt.get<int>("version", 0);

    if (version == TrackerManagerConfig::CONFIG_VERSION)
    {
		virtual_tracker_count = pt.get<int>("virtual_tracker_count", virtual_tracker_count);
		controller_position_smoothing = pt.get<float>("controller_position_smoothing", controller_position_smoothing);
		controller_position_prediction = pt.get<float>("controller_position_prediction", controller_position_prediction);
		controller_position_prediction_history = pt.get<int>("controller_position_prediction_history", controller_position_prediction_history);
		ignore_pose_from_one_tracker = pt.get<bool>("ignore_pose_from_one_tracker", ignore_pose_from_one_tracker);
        optical_tracking_timeout= pt.get<int>("optical_tracking_timeout", optical_tracking_timeout);
		use_bgr_to_hsv_lookup_table = pt.get<bool>("use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table);
		thread_sleep_ms = pt.get<int>("thread_sleep_ms", thread_sleep_ms);

		exclude_opposed_cameras = pt.get<bool>("excluded_opposed_cameras", exclude_opposed_cameras);

		min_valid_projection_area = pt.get<float>("min_valid_projection_area", min_valid_projection_area);
		occluded_area_on_loss_size = pt.get<float>("occluded_area_on_loss_size", occluded_area_on_loss_size);
		occluded_area_ignore_trackers = pt.get<int>("occluded_area_ignore_trackers", occluded_area_ignore_trackers);
		occluded_area_regain_projection_size = pt.get<float>("occluded_area_regain_projection_size", occluded_area_regain_projection_size);
		projection_collision_avoid = pt.get<bool>("projection_collision_avoid", projection_collision_avoid);
		projection_collision_offset = pt.get<float>("projection_collision_offset", projection_collision_offset);
		average_position_cache_enabled = pt.get<bool>("average_position_cache_enabled", average_position_cache_enabled);
		average_position_cache_cell_size = pt.get<float>("average_position_cache_cell_size", average_position_cache_cell_size);
		average_position_cache_avg_size = pt.get<float>("average_position_cache_avg_size", average_position_cache_avg_size);
		min_points_in_contour = pt.get<int>("min_points_in_contour", min_points_in_contour);
		max_tracker_position_deviation = pt.get<float>("max_tracker_position_deviation", max_tracker_position_deviation);

		disable_roi = pt.get<bool>("disable_roi", disable_roi);
		optimized_roi = pt.get<bool>("optimized_roi", optimized_roi);
		roi_edge_offset = pt.get<int>("roi_edge_offset", roi_edge_offset);

		default_tracker_profile.frame_width = pt.get<float>("default_tracker_profile.frame_width", 640);
		//default_tracker_profile.frame_height = pt.get<float>("default_tracker_profile.frame_height", 480);
		default_tracker_profile.frame_rate = pt.get<float>("default_tracker_profile.frame_rate", 30);
        default_tracker_profile.exposure = pt.get<float>("default_tracker_profile.exposure", 32);
        default_tracker_profile.gain = pt.get<float>("default_tracker_profile.gain", 32);

		global_forward_degrees = pt.get<float>("global_forward_degrees", global_forward_degrees);

		playspace_orientation_yaw = pt.get<float>("playspace_orientation_yaw", playspace_orientation_yaw);
		playspace_position_x = pt.get<float>("playspace_position_x", playspace_position_x);
		playspace_position_y = pt.get<float>("playspace_position_y", playspace_position_y);
		playspace_position_z = pt.get<float>("playspace_position_z", playspace_position_z);

		readColorPropertyPresetTable(pt, &default_tracker_profile.color_preset_table);
    }
    else
    {
        SERVER_LOG_WARNING("TrackerManagerConfig") <<
            "Config version " << version << " does not match expected version " <<
            TrackerManagerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

CommonDeviceVector 
TrackerManagerConfig::get_global_forward_axis() const
{
	return CommonDeviceVector::create(cosf(global_forward_degrees*k_degrees_to_radians), 0.f, sinf(global_forward_degrees*k_degrees_to_radians));
}

CommonDeviceVector 
TrackerManagerConfig::get_global_backward_axis() const
{
	return CommonDeviceVector::create(-cosf(global_forward_degrees*k_degrees_to_radians), 0.f, -sinf(global_forward_degrees*k_degrees_to_radians));
}

CommonDeviceVector
TrackerManagerConfig::get_global_right_axis() const
{
	return CommonDeviceVector::create(-sinf(global_forward_degrees*k_degrees_to_radians), 0.f, cosf(global_forward_degrees*k_degrees_to_radians));
}

CommonDeviceVector
TrackerManagerConfig::get_global_left_axis() const
{
	return CommonDeviceVector::create(sinf(global_forward_degrees*k_degrees_to_radians), 0.f, -cosf(global_forward_degrees*k_degrees_to_radians));
}

CommonDeviceVector 
TrackerManagerConfig::get_global_up_axis() const
{
	return CommonDeviceVector::create(0.f, 1.f, 0.f);
}

CommonDeviceVector 
TrackerManagerConfig::get_global_down_axis() const
{
	return CommonDeviceVector::create(0.f, -1.f, 0.f);
}

//-- Tracker Manager -----
bool TrackerManager::m_trackersSynced = true;
bool TrackerManager::m_isTrackerReady[TrackerManager::k_max_devices];
bool TrackerManager::m_isTrackerPollAllowed = false;

TrackerManager::TrackerManager()
    : DeviceTypeManager(10000, 13)
    , m_tracker_list_dirty(false)
{
}

bool 
TrackerManager::startup()
{
    bool bSuccess = DeviceTypeManager::startup();

    if (bSuccess)
    {
		// Load any config from disk
		cfg.load();

        // Save back out the config in case there were updated defaults
        cfg.save();

		// Copy the virtual tracker count into the Virtual tracker enumerator's static variable.
		// This breaks the dependency between the Tracker Manager and the enumerator.
		VirtualTrackerEnumerator::virtual_tracker_count = cfg.virtual_tracker_count;

        // Refresh the tracker list
        mark_tracker_list_dirty();

        // Put all of the available tracking colors in the queue
        for (int color_index = 0; color_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++color_index)
        {
            m_available_color_ids.push_back(static_cast<eCommonTrackingColorID>(color_index));
        }
    }

    return bSuccess;
}

void TrackerManager::poll_devices()
{
	m_trackersSynced = false;
	m_isTrackerPollAllowed = true;
	for (int i = 0; i < getMaxDevices(); i++)
	{
		const ServerTrackerViewPtr tracker = getTrackerViewPtr(i);
		if (tracker->getIsOpen())
		{
			if (!m_isTrackerReady[tracker->getDeviceID()])
			{
				m_isTrackerPollAllowed = false;
				break;
			}
		}
	}

	DeviceTypeManager::poll_devices();

	if (m_isTrackerPollAllowed)
	{
		for (int i = 0; i < getMaxDevices(); i++)
		{
			m_isTrackerReady[i] = false;
		}
		m_isTrackerPollAllowed = false;
		m_trackersSynced = true;
	}
}

void
TrackerManager::closeAllTrackers()
{
    for (int tracker_id = 0; tracker_id < k_max_devices; ++tracker_id)
    {
        ServerTrackerViewPtr tracker_view = getTrackerViewPtr(tracker_id);

        if (tracker_view->getIsOpen())
        {
            tracker_view->close();
        }
    }

    // Refresh the tracker list once we're allowed to
    mark_tracker_list_dirty();

    // Tell any clients that the tracker list changed
    send_device_list_changed_notification();
}

bool
TrackerManager::can_update_connected_devices()
{
    return m_tracker_list_dirty && DeviceTypeManager::can_update_connected_devices();
}

void 
TrackerManager::mark_tracker_list_dirty()
{
    m_tracker_list_dirty= true;
}

DeviceEnumerator *
TrackerManager::allocate_device_enumerator()
{
    return new TrackerDeviceEnumerator(TrackerDeviceEnumerator::CommunicationType_ALL);
}

void
TrackerManager::free_device_enumerator(DeviceEnumerator *enumerator)
{
    delete static_cast<TrackerDeviceEnumerator *>(enumerator);

    // Tracker list is no longer dirty after we have iterated through the list of cameras
    m_tracker_list_dirty = false;
}

ServerDeviceView *
TrackerManager::allocate_device_view(int device_id)
{
    return new ServerTrackerView(device_id);
}

ServerTrackerViewPtr
TrackerManager::getTrackerViewPtr(int device_id) const
{
    assert(m_deviceViews != nullptr);

    return std::static_pointer_cast<ServerTrackerView>(m_deviceViews[device_id]);
}

int TrackerManager::getListUpdatedResponseType()
{
	return PSMoveProtocol::Response_ResponseType_TRACKER_LIST_UPDATED;
}

eCommonTrackingColorID 
TrackerManager::allocateTrackingColorID()
{
    assert(m_available_color_ids.size() > 0);
    eCommonTrackingColorID tracking_color = m_available_color_ids.front();

    m_available_color_ids.pop_front();

    return tracking_color;
}

bool 
TrackerManager::claimTrackingColorID(const ServerControllerView *claiming_controller_view, eCommonTrackingColorID color_id)
{
    bool bColorWasInUse = false;
    bool bSuccess= true;

    // If any other controller has this tracking color, make them pick a new color (if possible)
    HMDManager *hmdManager= DeviceManager::getInstance()->m_hmd_manager;
    for (int device_id = 0; device_id < hmdManager->getMaxDevices(); ++device_id)
    {
        ServerHMDViewPtr hmd_view = hmdManager->getHMDViewPtr(device_id);

        if (hmd_view->getIsOpen())
        {
            if (hmd_view->getTrackingColorID() == color_id)
            {
                eCommonTrackingColorID newTrackingColor= allocateTrackingColorID();

                if (!hmd_view->setTrackingColorID(newTrackingColor))
                {
                    freeTrackingColorID(newTrackingColor);
                    bSuccess= false;
                }

                bColorWasInUse = true;
                break;
            }
        }
    }

    // If any other controller has this tracking color, make them pick a new color
    if (!bColorWasInUse)
    {
        // If any other controller has this tracking color, make them pick a new color
        ControllerManager *controllerManager= DeviceManager::getInstance()->m_controller_manager;
        for (int device_id = 0; device_id < controllerManager->getMaxDevices(); ++device_id)
        {
            ServerControllerViewPtr controller_view = controllerManager->getControllerViewPtr(device_id);

            if (controller_view->getIsOpen() && controller_view.get() != claiming_controller_view)
            {
                if (controller_view->getTrackingColorID() == color_id)
                {
                    controller_view->setTrackingColorID(allocateTrackingColorID());
                    bColorWasInUse = true;
                    break;
                }
            }
        }
    }

    // If the color was not in use, remove it from the color queue
    if (!bColorWasInUse)
    {
        for (auto iter = m_available_color_ids.begin(); iter != m_available_color_ids.end(); ++iter)
        {
            if (*iter == color_id)
            {
                m_available_color_ids.erase(iter);
                break;
            }
        }
    }

    return bSuccess;
}

bool 
TrackerManager::claimTrackingColorID(const ServerHMDView *claiming_hmd_view, eCommonTrackingColorID color_id)
{
    bool bColorWasInUse = false;
    bool bSuccess= true;

    // If any other controller has this tracking color, make them pick a new color (if possible)
    HMDManager *hmdManager= DeviceManager::getInstance()->m_hmd_manager;
    for (int device_id = 0; device_id < hmdManager->getMaxDevices(); ++device_id)
    {
        ServerHMDViewPtr hmd_view = hmdManager->getHMDViewPtr(device_id);

        if (hmd_view->getIsOpen() && hmd_view.get() != claiming_hmd_view)
        {
            if (hmd_view->getTrackingColorID() == color_id)
            {
                eCommonTrackingColorID newTrackingColor= allocateTrackingColorID();

                if (!hmd_view->setTrackingColorID(newTrackingColor))
                {
                    freeTrackingColorID(newTrackingColor);
                    bSuccess= false;
                }

                bColorWasInUse = true;
                break;
            }
        }
    }

    // If any other controller has this tracking color, make them pick a new color
    if (!bColorWasInUse)
    {
        // If any other controller has this tracking color, make them pick a new color
        ControllerManager *controllerManager= DeviceManager::getInstance()->m_controller_manager;
        for (int device_id = 0; device_id < controllerManager->getMaxDevices(); ++device_id)
        {
            ServerControllerViewPtr controller_view = controllerManager->getControllerViewPtr(device_id);

            if (controller_view->getIsOpen())
            {
                if (controller_view->getTrackingColorID() == color_id)
                {
                    controller_view->setTrackingColorID(allocateTrackingColorID());
                    bColorWasInUse = true;
                    break;
                }
            }
        }
    }

    // If the color was not in use, remove it from the color queue
    if (!bColorWasInUse)
    {
        for (auto iter = m_available_color_ids.begin(); iter != m_available_color_ids.end(); ++iter)
        {
            if (*iter == color_id)
            {
                m_available_color_ids.erase(iter);
                break;
            }
        }
    }

    return bSuccess;
}

void 
TrackerManager::freeTrackingColorID(eCommonTrackingColorID color_id)
{
    assert(std::find(m_available_color_ids.begin(), m_available_color_ids.end(), color_id) == m_available_color_ids.end());
    m_available_color_ids.push_back(color_id);
}

void
TrackerManager::applyPlayspaceOffsets(Eigen::Vector3f &poseVec, Eigen::Quaternionf &postQuat)
{
	applyPlayspaceOffsets(poseVec, postQuat, true, true, true);
}

void
TrackerManager::applyPlayspaceOffsets(Eigen::Vector3f &poseVec, Eigen::Quaternionf &postQuat, bool move_pos, bool rotate_pos, bool rotate_ang)
{
	const TrackerManagerConfig &cfg = DeviceManager::getInstance()->m_tracker_manager->getConfig();

	// Move by Axis
	if (move_pos)
		poseVec += Eigen::Vector3f(cfg.playspace_position_x, cfg.playspace_position_y, cfg.playspace_position_z);

	// Rotate by Axis
	const Eigen::Quaternionf offset_yaw = eigen_quaternion_angle_axis(cfg.playspace_orientation_yaw * k_degrees_to_radians, Eigen::Vector3f::UnitY());

	if (rotate_pos)
		poseVec = eigen_vector3f_clockwise_rotate(offset_yaw, poseVec);

	// Rotate Orientation
	if (rotate_ang)
		postQuat = offset_yaw.inverse() * postQuat;
}