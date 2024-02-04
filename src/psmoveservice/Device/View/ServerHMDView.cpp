//-- includes -----
#include "DeviceManager.h"
#include "ServerHMDView.h"
#include "MathAlignment.h"
#include "MorpheusHMD.h"
#include "VirtualHMD.h"
#include "CompoundPoseFilter.h"
#include "PoseFilterInterface.h"
#include "PSMoveProtocol.pb.h"
#include "ServerLog.h"
#include "ServerRequestHandler.h"
#include "ServerTrackerView.h"
#include "TrackerManager.h"
#include "HMDManager.h"
#include "ControllerManager.h"
#include "ServerControllerView.h"

#include "opencv2/opencv.hpp"

//-- private methods -----
static void init_filters_for_morpheus_hmd(
	const MorpheusHMD *morpheusHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);
static void init_filters_for_virtual_hmd(
	const VirtualHMD *virtualHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);
static IPoseFilter *pose_filter_factory(
	const CommonDeviceState::eDeviceType deviceType,
	const std::string &position_filter_type, const std::string &orientation_filter_type,
	const PoseFilterConstants &constants);
static void post_imu_filter_packets_for_morpheus(
	const MorpheusHMD *hmd,
	const MorpheusHMDState *hmdState,
	const t_high_resolution_timepoint now,
	const t_high_resolution_duration duration_since_last_update,
	t_hmd_pose_sensor_queue *pose_filter_queue);
static void post_imu_filter_packets_for_virtual(
	const VirtualHMD *hmd,
	const VirtualHMDState *hmdState,
	const t_high_resolution_timepoint now,
	const t_high_resolution_duration duration_since_last_update,
	t_hmd_pose_sensor_queue *pose_filter_queue);
static void post_optical_filter_packet_for_morpheus_hmd(
	const MorpheusHMD * hmd,
	const t_high_resolution_timepoint now, 
	const HMDOpticalPoseEstimation * pose_estimation,
	t_hmd_pose_optical_queue * pose_filter_queue);
static void post_optical_filter_packet_for_virtual_hmd(
	const VirtualHMD * hmd, 
	const t_high_resolution_timepoint now, 
	const HMDOpticalPoseEstimation * pose_estimation,
	t_hmd_pose_optical_queue * pose_filter_queue);
static void generate_morpheus_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view, const HMDStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame);
static void generate_virtual_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view, const HMDStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame);

static Eigen::Vector3f CommonDevicePosition_to_EigenVector3f(const CommonDevicePosition &p);
static Eigen::Vector3f CommonDeviceVector_to_EigenVector3f(const CommonDeviceVector &v);
static Eigen::Quaternionf CommonDeviceQuaternion_to_EigenQuaternionf(const CommonDeviceQuaternion &q);

static CommonDevicePosition EigenVector3f_to_CommonDevicePosition(const Eigen::Vector3f &p);
static CommonDeviceQuaternion EigenQuaternionf_to_CommonDeviceQuaternion(const Eigen::Quaternionf &q);

static void computeSpherePoseForHmdFromSingleTracker(
    const ServerHMDView *hmdView,
    const ServerTrackerViewPtr tracker,
    HMDOpticalPoseEstimation *tracker_pose_estimation,
    HMDOpticalPoseEstimation *multicam_pose_estimation);
static void computePointCloudPoseForHmdFromSingleTracker(
    const ServerHMDView *hmdView,
    const ServerTrackerViewPtr tracker,
    HMDOpticalPoseEstimation *tracker_pose_estimation,
    HMDOpticalPoseEstimation *multicam_pose_estimation);
static void computeSpherePoseForHmdFromMultipleTrackers(
    const ServerHMDView *controllerView,
    const TrackerManager* tracker_manager,
    const int *valid_projection_tracker_ids,
    const int projections_found,
    HMDOpticalPoseEstimation *tracker_pose_estimations,
    HMDOpticalPoseEstimation *multicam_pose_estimation);
static void computePointCloudPoseForHmdFromMultipleTrackers(
    const ServerHMDView *controllerView,
    const TrackerManager* tracker_manager,
    const int *valid_projection_tracker_ids,
    const int projections_found,
    HMDOpticalPoseEstimation *tracker_pose_estimations,
    HMDOpticalPoseEstimation *multicam_pose_estimation);

//-- public implementation -----
ServerHMDView::ServerHMDView(const int device_id)
	: ServerDeviceView(device_id)
	, m_tracking_listener_count(0)
	, m_tracking_enabled(false)
	, m_roi_disable_count(0)
	, m_device(nullptr)
	, m_tracker_pose_estimations(nullptr)
	, m_multicam_pose_estimation(nullptr)
	, m_pose_filter(nullptr)
	, m_pose_filter_space(nullptr)
	, m_lastPollSeqNumProcessed(-1)
{
}

ServerHMDView::~ServerHMDView()
{
}

bool ServerHMDView::allocate_device_interface(const class DeviceEnumerator *enumerator)
{
    switch (enumerator->get_device_type())
    {
    case CommonDeviceState::Morpheus:
        {
            m_device = new MorpheusHMD();
			m_device->setHmdListener(this); // Listen for IMU packets

			m_pose_filter = nullptr; // no pose filter until the device is opened

			m_tracker_pose_estimations = new HMDOpticalPoseEstimation[TrackerManager::k_max_devices];
			for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
			{
				m_tracker_pose_estimations[tracker_index].clear();
			}

			m_multicam_pose_estimation = new HMDOpticalPoseEstimation();
			m_multicam_pose_estimation->clear();
        } break;
    case CommonDeviceState::VirtualHMD:
        {
            m_device = new VirtualHMD();
			m_device->setHmdListener(this); // Listen for IMU packets

			m_pose_filter = nullptr; // no pose filter until the device is opened

			m_tracker_pose_estimations = new HMDOpticalPoseEstimation[TrackerManager::k_max_devices];
			for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
			{
				m_tracker_pose_estimations[tracker_index].clear();
			}

			m_multicam_pose_estimation = new HMDOpticalPoseEstimation();
			m_multicam_pose_estimation->clear();
        } break;
    default:
        break;
    }

    return m_device != nullptr;
}

void ServerHMDView::free_device_interface()
{
	if (m_multicam_pose_estimation != nullptr)
	{
		delete m_multicam_pose_estimation;
		m_multicam_pose_estimation = nullptr;
	}

	if (m_tracker_pose_estimations != nullptr)
	{
		delete[] m_tracker_pose_estimations;
		m_tracker_pose_estimations = nullptr;
	}

	if (m_pose_filter_space != nullptr)
	{
		delete m_pose_filter_space;
		m_pose_filter_space = nullptr;
	}

	if (m_pose_filter != nullptr)
	{
		delete m_pose_filter;
		m_pose_filter = nullptr;
	}

    if (m_device != nullptr)
    {
        delete m_device;
        m_device = nullptr;
    }
}

bool ServerHMDView::open(const class DeviceEnumerator *enumerator)
{
    // Attempt to open the controller
    bool bSuccess = ServerDeviceView::open(enumerator);

    // Setup the orientation filter based on the controller configuration
    if (bSuccess)
    {
        IDeviceInterface *device = getDevice();
        bool bAllocateTrackingColor = false;

		switch (device->getDeviceType())
		{
		case CommonDeviceState::Morpheus:
        case CommonDeviceState::VirtualHMD:
			{
				// Create a pose filter based on the HMD type
				resetPoseFilter();
				m_multicam_pose_estimation->clear();
                bAllocateTrackingColor= true;
			} break;
		default:
			break;
		}

        // If needed for this kind of HMD, assign a tracking color id
        if (bAllocateTrackingColor)
        {
            eCommonTrackingColorID tracking_color_id;
            assert(m_device != nullptr);

            // If this device already has a valid tracked color assigned, 
            // claim it from the pool (or another controller/hmd that had it previously)
            if (m_device->getTrackingColorID(tracking_color_id) && tracking_color_id != eCommonTrackingColorID::INVALID_COLOR)
            {
                DeviceManager::getInstance()->m_tracker_manager->claimTrackingColorID(this, tracking_color_id);
            }
            else
            {
                // Allocate a color from the list of remaining available color ids
                eCommonTrackingColorID allocatedColorID= DeviceManager::getInstance()->m_tracker_manager->allocateTrackingColorID();

                // Attempt to assign the tracking color id to the controller
                if (!m_device->setTrackingColorID(allocatedColorID))
                {
                    // If the device can't be assigned a tracking color, release the color back to the pool
                    DeviceManager::getInstance()->m_tracker_manager->freeTrackingColorID(allocatedColorID);
                }
            }
        }

        // Reset the poll sequence number high water mark
        m_lastPollSeqNumProcessed = -1;
    }

    return bSuccess;
}

void ServerHMDView::close()
{
    set_tracking_enabled_internal(false);

    eCommonTrackingColorID tracking_color_id= eCommonTrackingColorID::INVALID_COLOR;
    if (m_device != nullptr && m_device->getTrackingColorID(tracking_color_id))
    {
        if (tracking_color_id != eCommonTrackingColorID::INVALID_COLOR)
        {
            DeviceManager::getInstance()->m_tracker_manager->freeTrackingColorID(tracking_color_id);
        }
    }

    ServerDeviceView::close();
}

void ServerHMDView::resetPoseFilter()
{
	assert(m_device != nullptr);

	if (m_pose_filter != nullptr)
	{
		delete m_pose_filter;
		m_pose_filter = nullptr;
	}

	if (m_pose_filter_space != nullptr)
	{
		delete m_pose_filter_space;
		m_pose_filter_space = nullptr;
	}

	switch (m_device->getDeviceType())
	{
	case CommonDeviceState::Morpheus:
		{
			const MorpheusHMD *morpheusHMD = this->castCheckedConst<MorpheusHMD>();

			init_filters_for_morpheus_hmd(morpheusHMD, &m_pose_filter_space, &m_pose_filter);
		} break;
	case CommonDeviceState::VirtualHMD:
		{
			const VirtualHMD *virtualHMD = this->castCheckedConst<VirtualHMD>();

			init_filters_for_virtual_hmd(virtualHMD, &m_pose_filter_space, &m_pose_filter);
		} break;
	default:
		break;
	}
}

void ServerHMDView::updateOpticalPoseEstimation(TrackerManager* tracker_manager)
{
    const std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
	const TrackerManagerConfig &trackerMgrConfig = DeviceManager::getInstance()->m_tracker_manager->getConfig();
	ControllerManager *m_controllerManager = DeviceManager::getInstance()->m_controller_manager;

    // TODO: Probably need to first update IMU state to get velocity.
    // If velocity is too high, don't bother getting a new position.
    // Though it may be enough to just use the camera ROI as the limit.
    
    if (getIsTrackingEnabled())
    {
        int valid_projection_tracker_ids[TrackerManager::k_max_devices];
        int projections_found = 0;

		int available_trackers = 0;

		static bool occluded_tracker_ids[TrackerManager::k_max_devices][HMDManager::k_max_devices];
		static float occluded_projection_tracker_ids[TrackerManager::k_max_devices][HMDManager::k_max_devices][2];

        CommonDeviceTrackingShape trackingShape;
        m_device->getTrackingShape(trackingShape);
        assert(trackingShape.shape_type != eCommonTrackingShapeType::INVALID_SHAPE);

		struct projectionInfo
		{
			int tracker_id;
			float screen_area;
		};
		std::vector<projectionInfo> sorted_projections;

		for (int tracker_id = 0; tracker_id < tracker_manager->getMaxDevices(); ++tracker_id)
		{
			const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
			if (tracker->getIsOpen())
			{
				HMDOpticalPoseEstimation &trackerPoseEstimateRef = m_tracker_pose_estimations[tracker_id];

				projectionInfo info;
				info.tracker_id = tracker_id;
				info.screen_area = trackerPoseEstimateRef.projection.screen_area;
				sorted_projections.push_back(info);
			}
		}

		// Sort by biggest projector.
		// Go through all trackers and sort them by biggest projector to make tracking quality better.
		// The bigger projections should be closer to trackers and smaller far away.
		std::sort(
			sorted_projections.begin(), sorted_projections.end(),
			[](const projectionInfo & a, const projectionInfo & b) -> bool
		{
			return a.screen_area > b.screen_area;
		});


		// Find the projection of the controller from the perspective of each tracker.
		// In the case of sphere projections, go ahead and compute the tracker relative position as well.
		for (int list_index = 0; list_index < sorted_projections.size(); ++list_index)
		{
			int tracker_id = sorted_projections[list_index].tracker_id;
			int hmd_id = this->getDeviceID();

            ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
            HMDOpticalPoseEstimation &trackerPoseEstimateRef = m_tracker_pose_estimations[tracker_id];

            const bool bWasTracking= trackerPoseEstimateRef.bCurrentlyTracking;

            // Assume we're going to lose tracking this frame
			bool bCurrentlyTracking = false;
			bool bOccluded = false;
			bool bBlacklisted = false;
			bool bOutOfBounds = false;

            if (tracker->getIsOpen())
            {
				available_trackers++;

                // See how long it's been since we got a new video frame
                const std::chrono::time_point<std::chrono::high_resolution_clock> now= 
                    std::chrono::high_resolution_clock::now();
                const std::chrono::duration<float, std::milli> timeSinceNewDataMillis= 
                    now - tracker->getLastNewDataTimestamp();
                const float timeoutMilli= 
                    static_cast<float>(trackerMgrConfig.optical_tracking_timeout);

                // Can't compute tracking on video data that's too old
                if (timeSinceNewDataMillis.count() < timeoutMilli)
                {
                    // Initially the newTrackerPoseEstimate is a copy of the existing pose
                    bool bIsVisibleThisUpdate= false;

					// If a new video frame is available this tick, 
					// attempt to update the tracking location
					if (tracker->getHasUnpublishedState())
					{
						// Create a copy of the pose estimate state so that in event of a 
						// failure part way through computing the projection we don't
						// set partially valid state
						HMDOpticalPoseEstimation newTrackerPoseEstimate = trackerPoseEstimateRef;

						if (tracker->computeProjectionForHMD(
							this,
							&trackingShape,
							&newTrackerPoseEstimate))
						{
							bIsVisibleThisUpdate = true;

							// Actually apply the pose estimate state
							trackerPoseEstimateRef = newTrackerPoseEstimate;
							trackerPoseEstimateRef.last_visible_timestamp = now;
						}
					}


					bool bIsOccluded = false;
					bool bIsBlacklisted = false;

					//Only available
					if (trackerPoseEstimateRef.projection.shape_type == eCommonTrackingProjectionType::ProjectionType_Ellipse)
					{
						//Create an occlusion area at the last seen valid tracked projection.
						//If the projection center is near the occluded area it will not mark the projection as valid.
						//This will remove jitter when the shape of the controllers is partially visible to the trackers.
						if (!getIsROIDisabled())
						{
							if (trackerMgrConfig.occluded_area_on_loss_size >= 0.01)
							{
								if (!occluded_tracker_ids[tracker_id][hmd_id])
								{
									if (bWasTracking || bIsVisibleThisUpdate)
									{
										occluded_tracker_ids[tracker_id][hmd_id] = false;
										occluded_projection_tracker_ids[tracker_id][hmd_id][0] = trackerPoseEstimateRef.projection.shape.ellipse.center.x;
										occluded_projection_tracker_ids[tracker_id][hmd_id][1] = trackerPoseEstimateRef.projection.shape.ellipse.center.y;
									}
									else
									{
										occluded_tracker_ids[tracker_id][hmd_id] = true;
									}
								}

								if (occluded_tracker_ids[tracker_id][hmd_id])
								{
									if (bWasTracking || bIsVisibleThisUpdate)
									{
										bool bInArea = (abs(trackerPoseEstimateRef.projection.shape.ellipse.center.x - occluded_projection_tracker_ids[tracker_id][hmd_id][0])
											< trackerMgrConfig.occluded_area_on_loss_size
											&& abs(trackerPoseEstimateRef.projection.shape.ellipse.center.y - occluded_projection_tracker_ids[tracker_id][hmd_id][1])
											< trackerMgrConfig.occluded_area_on_loss_size);

										bool bRegain = (fmaxf(trackerPoseEstimateRef.projection.screen_area, trackerMgrConfig.min_valid_projection_area)
											> trackerMgrConfig.occluded_area_regain_projection_size);

										if (bInArea && !bRegain)
										{
											bIsOccluded = true;

											trackerPoseEstimateRef.occlusionAreaSize = trackerMgrConfig.occluded_area_on_loss_size;
											trackerPoseEstimateRef.occlusionAreaPos.x = occluded_projection_tracker_ids[tracker_id][hmd_id][0];
											trackerPoseEstimateRef.occlusionAreaPos.y = occluded_projection_tracker_ids[tracker_id][hmd_id][1];
										}
										else
										{
											occluded_tracker_ids[tracker_id][hmd_id] = false;
										}
									}
									else
									{
										// Recenter to last visible projection origin
										occluded_projection_tracker_ids[tracker_id][hmd_id][0] = trackerPoseEstimateRef.projection.shape.ellipse.center.x;
										occluded_projection_tracker_ids[tracker_id][hmd_id][1] = trackerPoseEstimateRef.projection.shape.ellipse.center.y;
									}
								}
							}
						}

						if (bWasTracking || bIsVisibleThisUpdate)
						{
							// Avoid other device projections
							if (trackerMgrConfig.projection_collision_avoid)
							{
								for (int i = 0; i < ControllerManager::k_max_devices; ++i)
								{
									ServerControllerViewPtr controllerView = m_controllerManager->getControllerViewPtr(i);
									if (!controllerView || !controllerView->getIsOpen())
										continue;

									const ControllerOpticalPoseEstimation *poseEst = controllerView->getTrackerPoseEstimate(tracker_id);
									if (!poseEst)
										continue;
									
									if (poseEst->projection.shape_type != eCommonTrackingProjectionType::ProjectionType_Ellipse)
										continue;

									float other_x = poseEst->projection.shape.ellipse.center.x;
									float other_y = poseEst->projection.shape.ellipse.center.y;
									float other_area = poseEst->projection.screen_area;

									float x = trackerPoseEstimateRef.projection.shape.ellipse.center.x - trackerPoseEstimateRef.projection.shape.ellipse.half_x_extent - trackerMgrConfig.projection_collision_offset;
									float y = trackerPoseEstimateRef.projection.shape.ellipse.center.y - trackerPoseEstimateRef.projection.shape.ellipse.half_y_extent - trackerMgrConfig.projection_collision_offset;
									float w = (trackerPoseEstimateRef.projection.shape.ellipse.half_x_extent * 2) + (trackerMgrConfig.projection_collision_offset * 2);
									float h = (trackerPoseEstimateRef.projection.shape.ellipse.half_y_extent * 2) + (trackerMgrConfig.projection_collision_offset * 2);
									float area = trackerPoseEstimateRef.projection.screen_area;

									// $TODO Should we just ignore both?
									if (area > other_area)
										continue;

									bool bInArea = (other_x >= x)
										&& (other_y >= y)
										&& (other_x < x + w)
										&& (other_y < y + h);

									// Blacklist if the projection are is already used by another.
									// Avoiding color collisions between controllers.
									if (bInArea)
									{
										trackerPoseEstimateRef.blacklistedAreaRec.x = x;
										trackerPoseEstimateRef.blacklistedAreaRec.y = y;
										trackerPoseEstimateRef.blacklistedAreaRec.w = w;
										trackerPoseEstimateRef.blacklistedAreaRec.h = h;

										bIsBlacklisted = true;
										break;
									}
								}
							}
						}
					}

					if (bIsBlacklisted)
					{
						bBlacklisted = true;
					}
					else
					{
						bBlacklisted = false;

						// Ignore projections that are occluded BUT always pass atleast 2 biggest projected trackers.
						if (!bIsOccluded || projections_found < trackerMgrConfig.occluded_area_ignore_num_trackers)
						{
							bOccluded = false;

							// If the projection isn't too old (or updated this tick), 
							// say we have a valid tracked location
							if ((bWasTracking && !tracker->getHasUnpublishedState()) || bIsVisibleThisUpdate)
							{
								// If this tracker has a valid projection for the controller
								// add it to the tracker id list
								valid_projection_tracker_ids[projections_found] = tracker_id;
								++projections_found;

								// Flag this pose estimate as invalid
								bCurrentlyTracking = true;
							}
						}
						else
						{
							bOccluded = true;
						}
					}
                }
            }

            // Keep track of the last time the position estimate was updated
            trackerPoseEstimateRef.last_update_timestamp = now;
            trackerPoseEstimateRef.bValidTimestamps = true;
            trackerPoseEstimateRef.bCurrentlyTracking = bCurrentlyTracking;
			trackerPoseEstimateRef.bIsOccluded = bOccluded;
			trackerPoseEstimateRef.bIsBlacklisted = bBlacklisted;
        }

        // How we compute the final world pose estimate varies based on
        // * Number of trackers that currently have a valid projections of the controller
        // * The kind of projection shape (psmove sphere or ds4 lightbar)
        if (projections_found > 1)
        {
            // If multiple trackers can see the controller, 
            // triangulate all pairs of projections and average the results
            switch (trackingShape.shape_type)
            {
            case eCommonTrackingShapeType::Sphere:
                computeSpherePoseForHmdFromMultipleTrackers(
                    this,
                    tracker_manager,
                    valid_projection_tracker_ids,
                    projections_found,
                    m_tracker_pose_estimations,
                    m_multicam_pose_estimation);
                break;
            case eCommonTrackingShapeType::PointCloud:
                computePointCloudPoseForHmdFromMultipleTrackers(
                    this,
                    tracker_manager,
                    valid_projection_tracker_ids,
                    projections_found,
                    m_tracker_pose_estimations,
                    m_multicam_pose_estimation);
                break;
            default:
                assert(false && "unreachable");
            }
        }
        else if (projections_found == 1 && (available_trackers == 1 || !trackerMgrConfig.ignore_pose_from_one_tracker))
        {
            const int tracker_id = valid_projection_tracker_ids[0];
            const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);

            // If only one tracker can see the controller, 
            // then use the tracker to derive a world space location
            switch (trackingShape.shape_type)
            {
            case eCommonTrackingShapeType::Sphere:
                computeSpherePoseForHmdFromSingleTracker(
                    this,
                    tracker,
                    &m_tracker_pose_estimations[tracker_id],
                    m_multicam_pose_estimation);
                break;
            case eCommonTrackingShapeType::PointCloud:
                computePointCloudPoseForHmdFromSingleTracker(
                    this,
                    tracker,
                    &m_tracker_pose_estimations[tracker_id],
                    m_multicam_pose_estimation);
                break;
            default:
                assert(false && "unreachable");
            }
        }
        // If no trackers can see the controller, maintain the last known position and time it was seen
        else
        {
            m_multicam_pose_estimation->bCurrentlyTracking= false;
        }

        // Update the position estimation timestamps
        if (m_multicam_pose_estimation->bCurrentlyTracking)
        {
            m_multicam_pose_estimation->last_visible_timestamp = now;
        }
        m_multicam_pose_estimation->last_update_timestamp = now;
        m_multicam_pose_estimation->bValidTimestamps = true;
    }

	// Update the filter if we have a valid optically tracked pose
	// TODO: These packets will eventually get posted from the notifyTrackerDataReceived()
	// callback function which will be called by camera processing threads as new video
	// frames are received.
	if (m_multicam_pose_estimation->bCurrentlyTracking)
	{
		switch (getHMDDeviceType())
		{
		case CommonDeviceState::Morpheus:
		{
			const MorpheusHMD *hmd = this->castCheckedConst<MorpheusHMD>();

			post_optical_filter_packet_for_morpheus_hmd(
				hmd,
				now,
				m_multicam_pose_estimation,
				&m_PoseSensorOpticalPacketQueue);
		} break;
		case CommonDeviceState::VirtualHMD:
		{
			const VirtualHMD *hmd = this->castCheckedConst<VirtualHMD>();

			post_optical_filter_packet_for_virtual_hmd(
				hmd,
				now,
				m_multicam_pose_estimation,
				&m_PoseSensorOpticalPacketQueue);
		} break;
		default:
			assert(0 && "Unhandled HMD Type");
		}
	}
}

void ServerHMDView::updateStateAndPredict()
{
	std::vector<PoseSensorPacket> timeSortedPackets;

	// Drain the packet queues filled by the threads
	PoseSensorPacket packet;
	while (m_PoseSensorIMUPacketQueue.try_dequeue(packet))
	{
		timeSortedPackets.push_back(packet);
	}

	//TODO: m_PoseSensorOpticalPacketQueue is currently getting filled on the main thread by
	// updateOpticalPoseEstimation() when triangulating the optical pose estimates.
	// Eventually this work will move to it's own camera processing thread 
	// this line will read from a lock-less queue just like the IMU packet queue.
	while (m_PoseSensorOpticalPacketQueue.size() > 0)
	{
		timeSortedPackets.push_back(m_PoseSensorOpticalPacketQueue.front());
		m_PoseSensorOpticalPacketQueue.pop_front();
	}

	// Sort the packets in order of ascending time
	if (timeSortedPackets.size() > 1)
	{
		std::sort(
			timeSortedPackets.begin(), timeSortedPackets.end(),
			[](const PoseSensorPacket & a, const PoseSensorPacket & b) -> bool
		{
			return a.timestamp < b.timestamp;
		});

		t_high_resolution_duration duration =
			timeSortedPackets[timeSortedPackets.size() - 1].timestamp - timeSortedPackets[0].timestamp;
		std::chrono::duration<float, std::milli> milli_duration = duration;

		const size_t k_max_process_count = 1000;
		if (timeSortedPackets.size() > k_max_process_count)
		{
			const size_t excess = timeSortedPackets.size() - k_max_process_count;

			SERVER_LOG_WARNING("updatePoseFilter()") << "Incoming packet count: " << timeSortedPackets.size() << " (" << milli_duration.count() << "ms)" << ", trimming: " << excess;
			timeSortedPackets.erase(timeSortedPackets.begin(), timeSortedPackets.begin() + excess);
		}
		else
		{
			//SERVER_LOG_DEBUG("updatePoseFilter()") << "Incoming packet count: " << timeSortedPackets.size() << " (" << milli_duration.count() << "ms)";
		}
	}

	// Process the sensor packets from oldest to newest
	for (const PoseSensorPacket &sensorPacket : timeSortedPackets)
	{
		PoseFilterPacket filter_packet;
		filter_packet.clear();

		filter_packet.hmdDeviceId = this->getDeviceID();
		filter_packet.isCurrentlyTracking = this->getIsCurrentlyTracking();
		filter_packet.isSynced = DeviceManager::getInstance()->m_tracker_manager->trackersSynced();

		// Create a filter input packet from the sensor data 
		// and the filter's previous orientation and position
		m_pose_filter_space->createFilterPacket(
			sensorPacket,
			m_pose_filter,
			filter_packet);

		// Process the filter packet
		m_pose_filter->update(sensorPacket.timestamp, filter_packet);

		// Flag the state as unpublished, which will trigger an update to the client
		markStateAsUnpublished();
	}
}

CommonDevicePose
ServerHMDView::getFilteredPose(float time, float ang_time) const
{
	CommonDevicePose pose;

	pose.clear();

	if (m_pose_filter != nullptr)
	{
		switch (m_device->getDeviceType())
		{
		case CommonDeviceState::Morpheus:
		{
			const MorpheusHMD *hmd = this->castCheckedConst<MorpheusHMD>();
			const CommonDevicePosition offset_position = hmd->getConfig()->offset_position;
			const CommonDevicePosition offset_orientation = hmd->getConfig()->offset_orientation;
			const CommonDevicePosition offset_world_orientation = hmd->getConfig()->offset_world_orientation;
			const CommonDevicePosition offset_scale = hmd->getConfig()->offset_scale;

			const Eigen::Quaternionf orientation = m_pose_filter->getOrientation(
				ang_time,
				offset_orientation.x,
				offset_orientation.y,
				offset_orientation.z,
				offset_world_orientation.x,
				offset_world_orientation.y,
				offset_world_orientation.z
			);
			const Eigen::Vector3f position_cm = m_pose_filter->getPositionCm(time);

			// Playspace
			{
				const TrackerManagerConfig &cfg = DeviceManager::getInstance()->m_tracker_manager->getConfig();

				Eigen::Vector3f poseVec = Eigen::Vector3f(position_cm.x(), position_cm.y(), position_cm.z());
				Eigen::Quaternionf postQuat = Eigen::Quaternionf(orientation.w(), orientation.x(), orientation.y(), orientation.z());

				DeviceManager::getInstance()->m_tracker_manager->applyPlayspaceOffsets(poseVec, postQuat, true, true, true, true);

				pose.PositionCm.x = (poseVec.x() + clampf(offset_position.x, -(1 << 16), (1 << 16))) * clampf(offset_scale.x, 0.01f, 100.f);
				pose.PositionCm.y = (poseVec.y() + clampf(offset_position.y, -(1 << 16), (1 << 16))) * clampf(offset_scale.y, 0.01f, 100.f);
				pose.PositionCm.z = (poseVec.z() + clampf(offset_position.z, -(1 << 16), (1 << 16))) * clampf(offset_scale.z, 0.01f, 100.f);

				// Rotate Orientation
				pose.Orientation.w = postQuat.w();
				pose.Orientation.x = postQuat.x();
				pose.Orientation.y = postQuat.y();
				pose.Orientation.z = postQuat.z();
			}
			break;
		}
		case CommonDeviceState::VirtualHMD:
		{
			const VirtualHMD *virt = this->castCheckedConst<VirtualHMD>();
			const CommonDevicePosition offset_position = virt->getConfig()->offset_position;
			const CommonDevicePosition offset_orientation = virt->getConfig()->offset_orientation;
			const CommonDevicePosition offset_world_orientation = virt->getConfig()->offset_world_orientation;
			const CommonDevicePosition offset_scale = virt->getConfig()->offset_scale;

			const Eigen::Quaternionf orientation = m_pose_filter->getOrientation(
				ang_time,
				offset_orientation.x,
				offset_orientation.y,
				offset_orientation.z,
				offset_world_orientation.x,
				offset_world_orientation.y,
				offset_world_orientation.z
			);
			const Eigen::Vector3f position_cm = m_pose_filter->getPositionCm(time);

			// Playspace
			{
				const TrackerManagerConfig &cfg = DeviceManager::getInstance()->m_tracker_manager->getConfig();

				Eigen::Vector3f poseVec = Eigen::Vector3f(position_cm.x(), position_cm.y(), position_cm.z());
				Eigen::Quaternionf postQuat = Eigen::Quaternionf(orientation.w(), orientation.x(), orientation.y(), orientation.z());

				DeviceManager::getInstance()->m_tracker_manager->applyPlayspaceOffsets(poseVec, postQuat, true, true, true, true);

				pose.PositionCm.x = (poseVec.x() + clampf(offset_position.x, -(1 << 16), (1 << 16))) * clampf(offset_scale.x, 0.01f, 100.f);
				pose.PositionCm.y = (poseVec.y() + clampf(offset_position.y, -(1 << 16), (1 << 16))) * clampf(offset_scale.y, 0.01f, 100.f);
				pose.PositionCm.z = (poseVec.z() + clampf(offset_position.z, -(1 << 16), (1 << 16))) * clampf(offset_scale.z, 0.01f, 100.f);

				// Rotate Orientation
				pose.Orientation.w = postQuat.w();
				pose.Orientation.x = postQuat.x();
				pose.Orientation.y = postQuat.y();
				pose.Orientation.z = postQuat.z();
			}
			break;
		}
		default:
			const Eigen::Quaternionf orientation = m_pose_filter->getOrientation(ang_time);
			const Eigen::Vector3f position_cm = m_pose_filter->getPositionCm(time);

			// Playspace
			{
				const TrackerManagerConfig &cfg = DeviceManager::getInstance()->m_tracker_manager->getConfig();

				Eigen::Vector3f poseVec = Eigen::Vector3f(position_cm.x(), position_cm.y(), position_cm.z());
				Eigen::Quaternionf postQuat = Eigen::Quaternionf(orientation.w(), orientation.x(), orientation.y(), orientation.z());

				DeviceManager::getInstance()->m_tracker_manager->applyPlayspaceOffsets(poseVec, postQuat, true, true, true, true);

				pose.PositionCm.x = poseVec.x();
				pose.PositionCm.y = poseVec.y();
				pose.PositionCm.z = poseVec.z();

				// Rotate Orientation
				pose.Orientation.w = postQuat.w();
				pose.Orientation.x = postQuat.x();
				pose.Orientation.y = postQuat.y();
				pose.Orientation.z = postQuat.z();
			}
		}
	}

	return pose;
}

CommonDevicePhysics
ServerHMDView::getFilteredPhysics() const
{
	CommonDevicePhysics physics;

	if (m_pose_filter != nullptr)
	{
		Eigen::Vector3f first_derivative = m_pose_filter->getAngularVelocityRadPerSec();
		Eigen::Vector3f second_derivative = m_pose_filter->getAngularAccelerationRadPerSecSqr();
		Eigen::Vector3f velocity(m_pose_filter->getVelocityCmPerSec());
		Eigen::Vector3f acceleration(m_pose_filter->getAccelerationCmPerSecSqr());

		Eigen::Quaternionf nullQuat = Eigen::Quaternionf::Identity();
		DeviceManager::getInstance()->m_tracker_manager->applyPlayspaceOffsets(first_derivative, nullQuat, false, true, false, true);
		DeviceManager::getInstance()->m_tracker_manager->applyPlayspaceOffsets(second_derivative, nullQuat, false, true, false, true);
		DeviceManager::getInstance()->m_tracker_manager->applyPlayspaceOffsets(velocity, nullQuat, false, true, false, true);
		DeviceManager::getInstance()->m_tracker_manager->applyPlayspaceOffsets(acceleration, nullQuat, false, true, false, true);

		physics.AngularVelocityRadPerSec.i = first_derivative.x();
		physics.AngularVelocityRadPerSec.j = first_derivative.y();
		physics.AngularVelocityRadPerSec.k = first_derivative.z();

		physics.AngularAccelerationRadPerSecSqr.i = second_derivative.x();
		physics.AngularAccelerationRadPerSecSqr.j = second_derivative.y();
		physics.AngularAccelerationRadPerSecSqr.k = second_derivative.z();

		physics.VelocityCmPerSec.i = velocity.x();
		physics.VelocityCmPerSec.j = velocity.y();
		physics.VelocityCmPerSec.k = velocity.z();

		physics.AccelerationCmPerSecSqr.i = acceleration.x();
		physics.AccelerationCmPerSecSqr.j = acceleration.y();
		physics.AccelerationCmPerSecSqr.k = acceleration.z();
	}

	return physics;
}

// Returns the full usb device path for the controller
std::string
ServerHMDView::getUSBDevicePath() const
{
    return m_device->getUSBDevicePath();
}

// Returns the "controller_" + serial number for the controller
std::string
ServerHMDView::getConfigIdentifier() const
{
	std::string	identifier = "";

	if (m_device != nullptr)
	{
		if (m_device->getDeviceType() == CommonDeviceState::Morpheus)
		{
			identifier = "hmd_morpheus";
		}
		else if (m_device->getDeviceType() == CommonDeviceState::VirtualHMD)
		{
            // THe "USB device path" for a Virtual HMD is actually just the Virtual HMD identifier, i.e. "VirtualHMD_0"
			identifier = "hmd_"+m_device->getUSBDevicePath();
		}
		else
		{
			identifier = "hmd_unknown";
		}
	}

	return identifier;
}

CommonDeviceState::eDeviceType
ServerHMDView::getHMDDeviceType() const
{
    return m_device->getDeviceType();
}

// Fetch the controller state at the given sample index.
// A lookBack of 0 corresponds to the most recent data.
const struct CommonHMDState * ServerHMDView::getState(
    int lookBack) const
{
    const struct CommonDeviceState *device_state = m_device->getState(lookBack);
    assert(device_state == nullptr ||
        ((int)device_state->DeviceType >= (int)CommonDeviceState::HeadMountedDisplay &&
        device_state->DeviceType < CommonDeviceState::SUPPORTED_HMD_TYPE_COUNT));

    return static_cast<const CommonHMDState *>(device_state);
}

void ServerHMDView::startTracking()
{
	if (!m_tracking_enabled)
	{
		set_tracking_enabled_internal(true);
	}

	++m_tracking_listener_count;
}

void ServerHMDView::stopTracking()
{
	assert(m_tracking_listener_count > 0);
	--m_tracking_listener_count;

	if (m_tracking_listener_count <= 0 && m_tracking_enabled)
	{
		set_tracking_enabled_internal(false);
	}
}

void ServerHMDView::notifySensorDataReceived(const CommonDeviceState * sensor_state)
{
	// Compute the time in seconds since the last update
	const t_high_resolution_timepoint now = std::chrono::high_resolution_clock::now();
	t_high_resolution_duration durationSinceLastUpdate = t_high_resolution_duration::zero();

	if (m_bIsLastSensorDataTimestampValid)
	{
		durationSinceLastUpdate = now - m_lastSensorDataTimestamp;
	}
	m_lastSensorDataTimestamp = now;
	m_bIsLastSensorDataTimestampValid = true;

	// Apply device specific filtering
	switch (sensor_state->DeviceType)
	{
	case CommonDeviceState::Morpheus:
	{
		const MorpheusHMD *hmd = this->castCheckedConst<MorpheusHMD>();
		const MorpheusHMDState *hmdState =
			static_cast<const MorpheusHMDState *>(sensor_state);

		// Only update the position filter when tracking is enabled
		post_imu_filter_packets_for_morpheus(
			hmd, hmdState,
			now, durationSinceLastUpdate,
			&m_PoseSensorIMUPacketQueue);
	} break;
	case CommonDeviceState::VirtualHMD:
	{
		const VirtualHMD *hmd = this->castCheckedConst<VirtualHMD>();
		const VirtualHMDState *hmdState =
			static_cast<const VirtualHMDState *>(sensor_state);

		// Only update the position filter when tracking is enabled
		post_imu_filter_packets_for_virtual(
			hmd, hmdState,
			now, durationSinceLastUpdate,
			&m_PoseSensorIMUPacketQueue);
	} break;
	default:
		assert(0 && "Unhandled Controller Type");
	}

	// Consider this HMD state sequence num processed
	m_lastPollSeqNumProcessed = sensor_state->PollSequenceNumber;
}

void ServerHMDView::set_tracking_enabled_internal(bool bEnabled)
{
	if (m_tracking_enabled != bEnabled)
	{
		switch (getHMDDeviceType())
		{
		case CommonHMDState::Morpheus:
			castChecked<MorpheusHMD>()->setTrackingEnabled(bEnabled);
			break;
		case CommonHMDState::VirtualHMD:
			castChecked<VirtualHMD>()->setTrackingEnabled(bEnabled);
			break;
		default:
			assert(0 && "unreachable");
		}

		m_tracking_enabled = bEnabled;
	}
}

// Get the tracking shape for the controller
bool ServerHMDView::getTrackingShape(CommonDeviceTrackingShape &trackingShape) const
{
	m_device->getTrackingShape(trackingShape);

	return trackingShape.shape_type != eCommonTrackingShapeType::INVALID_SHAPE;
}

eCommonTrackingColorID ServerHMDView::getTrackingColorID() const
{
	eCommonTrackingColorID tracking_color_id = eCommonTrackingColorID::INVALID_COLOR;

	if (m_device != nullptr)
	{
		m_device->getTrackingColorID(tracking_color_id);
	}

	return tracking_color_id;
}

bool ServerHMDView::setTrackingColorID(eCommonTrackingColorID colorID)
{
    bool bSuccess= true;

    if (colorID != getTrackingColorID())
    {
        if (m_device != nullptr)
        {
            bSuccess= m_device->setTrackingColorID(colorID);

            if (bSuccess && getIsTrackingEnabled())
            {
                set_tracking_enabled_internal(false);
                set_tracking_enabled_internal(true);
            }
        }
        else
        {
            bSuccess= false;
        }
    }

    return bSuccess;
}

float ServerHMDView::getROIPredictionTime() const
{
	return m_device->getPredictionTime();
}

void ServerHMDView::publish_device_data_frame()
{
    // Tell the server request handler we want to send out HMD updates.
    // This will call generate_hmd_data_frame_for_stream for each listening connection.
    ServerRequestHandler::get_instance()->publish_hmd_data_frame(
        this, &ServerHMDView::generate_hmd_data_frame_for_stream);
}

void ServerHMDView::generate_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const struct HMDStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame)
{
    PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket *hmd_data_frame =
        data_frame->mutable_hmd_data_packet();

    hmd_data_frame->set_hmd_id(hmd_view->getDeviceID());
    hmd_data_frame->set_sequence_num(hmd_view->m_sequence_number);
    hmd_data_frame->set_isconnected(hmd_view->getDevice()->getIsOpen());

    switch (hmd_view->getHMDDeviceType())
    {
    case CommonHMDState::Morpheus:
        {
            generate_morpheus_hmd_data_frame_for_stream(hmd_view, stream_info, data_frame);
        } break;
    case CommonHMDState::VirtualHMD:
        {
            generate_virtual_hmd_data_frame_for_stream(hmd_view, stream_info, data_frame);
        } break;
    default:
        assert(0 && "Unhandled HMD type");
    }

    data_frame->set_device_category(PSMoveProtocol::DeviceOutputDataFrame::HMD);
}

static void
init_filters_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD,
	PoseFilterSpace **out_pose_filter_space,
	IPoseFilter **out_pose_filter)
{
    const MorpheusHMDConfig *hmd_config = morpheusHMD->getConfig();

	// Setup the space the pose filter operates in
	PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
	pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f));
	pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f::Zero());
	pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
	pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

	CommonDeviceVector accel_var = hmd_config->get_calibrated_accelerometer_variance();
	CommonDeviceVector gyro_var = hmd_config->get_calibrated_gyro_variance();
	CommonDeviceVector gyro_drift = hmd_config->get_calibrated_gyro_drift();

	// Copy the pose filter constants from the controller config
	PoseFilterConstants constants;
	constants.clear();

	constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.orientation_constants.accelerometer_variance = Eigen::Vector3f(accel_var.i, accel_var.j, accel_var.k);
	constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
	constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
	constants.orientation_constants.gyro_drift = Eigen::Vector3f(gyro_drift.i, gyro_drift.j, gyro_drift.k);
	constants.orientation_constants.gyro_variance = Eigen::Vector3f(gyro_var.i, gyro_var.j, gyro_var.k);
	constants.orientation_constants.mean_update_time_delta = hmd_config->mean_update_time_delta;
	constants.orientation_constants.orientation_variance_curve.A = hmd_config->orientation_variance;
	constants.orientation_constants.orientation_variance_curve.B = 0.f;
	constants.orientation_constants.orientation_variance_curve.MaxValue = 1.f;
	constants.orientation_constants.magnetometer_variance = Eigen::Vector3f::Zero();
	constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero();

	constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.position_constants.accelerometer_variance = Eigen::Vector3f(accel_var.i, accel_var.j, accel_var.k);
	constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
	constants.position_constants.accelerometer_noise_radius = 0.f; // TODO
	constants.position_constants.max_velocity = hmd_config->max_velocity;
	constants.position_constants.mean_update_time_delta = hmd_config->mean_update_time_delta;
	constants.position_constants.position_variance_curve.A = hmd_config->position_variance_exp_fit_a;
	constants.position_constants.position_variance_curve.B = hmd_config->position_variance_exp_fit_b;
	constants.position_constants.position_variance_curve.MaxValue = 1.f;

	*out_pose_filter_space = pose_filter_space;
	*out_pose_filter = pose_filter_factory(
		CommonDeviceState::eDeviceType::PSMove,
		hmd_config->position_filter_type,
		hmd_config->orientation_filter_type,
		constants);
}

static void init_filters_for_virtual_hmd(
    const VirtualHMD *virtualHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter)
{
    const VirtualHMDConfig *hmd_config = virtualHMD->getConfig();

	// Setup the space the pose filter operates in
	PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
	pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f));
	pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f::Zero());
	pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
	pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

	// Copy the pose filter constants from the controller config
	PoseFilterConstants constants;
	constants.clear();

	constants.orientation_constants.gravity_calibration_direction = Eigen::Vector3f::Zero();
	constants.orientation_constants.accelerometer_variance = Eigen::Vector3f::Zero();
	constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
	constants.orientation_constants.magnetometer_calibration_direction = Eigen::Vector3f::Zero();
	constants.orientation_constants.gyro_drift = Eigen::Vector3f::Zero();
	constants.orientation_constants.gyro_variance = Eigen::Vector3f::Zero();
	constants.orientation_constants.mean_update_time_delta = 0.f;
	constants.orientation_constants.orientation_variance_curve.A = 0.f;
	constants.orientation_constants.orientation_variance_curve.B = 0.f;
	constants.orientation_constants.orientation_variance_curve.MaxValue = 0.f;
	constants.orientation_constants.magnetometer_variance = Eigen::Vector3f::Zero();
	constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero();

	constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
	constants.position_constants.accelerometer_variance = Eigen::Vector3f::Zero();
	constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
	constants.position_constants.accelerometer_noise_radius = 0.f; // TODO
	constants.position_constants.max_velocity = hmd_config->max_velocity;
	constants.position_constants.mean_update_time_delta = hmd_config->mean_update_time_delta;
	constants.position_constants.position_variance_curve.A = hmd_config->position_variance_exp_fit_a;
	constants.position_constants.position_variance_curve.B = hmd_config->position_variance_exp_fit_b;
	constants.position_constants.position_variance_curve.MaxValue = 1.f;

	*out_pose_filter_space = pose_filter_space;
	*out_pose_filter = pose_filter_factory(
		CommonDeviceState::eDeviceType::VirtualHMD,
		hmd_config->position_filter_type,
		"",
		constants);
}

static IPoseFilter *
pose_filter_factory(
	const CommonDeviceState::eDeviceType deviceType,
	const std::string &position_filter_type,
	const std::string &orientation_filter_type,
	const PoseFilterConstants &constants)
{
	static IPoseFilter *filter = nullptr;

	// Convert the position filter type string into an enum
	PositionFilterType position_filter_enum = PositionFilterTypeNone;
	if (position_filter_type == "PassThru")
	{
		position_filter_enum = PositionFilterTypePassThru;
	}
	else if (position_filter_type == "LowPassOptical")
	{
		position_filter_enum = PositionFilterTypeLowPassOptical;
	}
	else if (position_filter_type == "LowPassIMU")
	{
		position_filter_enum = PositionFilterTypeLowPassIMU;
	}
	else if (position_filter_type == "LowPassExponential")
	{
		position_filter_enum = PositionFilterTypeLowPassExponential;
	}
	else if (position_filter_type == "ComplimentaryOpticalIMU")
	{
		position_filter_enum = PositionFilterTypeComplimentaryOpticalIMU;
	}
	else if (position_filter_type == "PositionKalman")
	{
		position_filter_enum = PositionFilterTypeKalman;
	}
	else
	{
		SERVER_LOG_INFO("pose_filter_factory()") <<
			"Unknown position filter type: " << position_filter_type << ". Using default.";

		// fallback to a default based on hmd type
		switch (deviceType)
		{
		case CommonDeviceState::Morpheus:
        case CommonDeviceState::VirtualHMD:
			position_filter_enum = PositionFilterTypeKalman;
			break;
		default:
			assert(0 && "unreachable");
		}
	}

	// Convert the orientation filter type string into an enum
	OrientationFilterType orientation_filter_enum = OrientationFilterTypeNone;
	if (orientation_filter_type == "")
	{
		orientation_filter_enum = OrientationFilterTypeNone;
	}
	else if (orientation_filter_type == "PassThru")
	{
		orientation_filter_enum = OrientationFilterTypePassThru;
	}
	else if (orientation_filter_type == "MadgwickARG")
	{
		orientation_filter_enum = OrientationFilterTypeMadgwickARG;
	}
	else if (orientation_filter_type == "ComplementaryOpticalARG")
	{
		orientation_filter_enum = OrientationFilterTypeComplementaryOpticalARG;
	}
	else
	{
		SERVER_LOG_INFO("pose_filter_factory()") <<
			"Unknown orientation filter type: " << orientation_filter_type << ". Using default.";

		// fallback to a default based on controller type
		switch (deviceType)
		{
		case CommonDeviceState::Morpheus:
			orientation_filter_enum = OrientationFilterTypeMadgwickARG;
			break;
        case CommonDeviceState::VirtualHMD:
            orientation_filter_enum = OrientationFilterTypeNone;
            break;
		default:
			assert(0 && "unreachable");
		}
	}

	CompoundPoseFilter *compound_pose_filter = new CompoundPoseFilter();
	compound_pose_filter->init(deviceType, orientation_filter_enum, position_filter_enum, constants);
	filter = compound_pose_filter;

	assert(filter != nullptr);

	return filter;
}


static void post_imu_filter_packets_for_morpheus(
	const MorpheusHMD *hmd,
	const MorpheusHMDState *hmdState,
	const t_high_resolution_timepoint now,
	const t_high_resolution_duration duration_since_last_update,
	t_hmd_pose_sensor_queue *pose_filter_queue)
{
	const MorpheusHMDConfig *config = hmd->getConfig();

	PoseSensorPacket sensor_packet;

	sensor_packet.clear();

	// Don't bother with the earlier frame if this is the very first IMU packet 
	// (since we have no previous timestamp to use)
	int start_frame_index = 0;
	if (duration_since_last_update == t_high_resolution_duration::zero())
	{
		start_frame_index = 1;
	}

	const t_high_resolution_timepoint prev_timestamp = now - (duration_since_last_update / 2);
	t_high_resolution_timepoint timestamps[2] = { prev_timestamp, now };

	// Each state update contains two readings (one earlier and one later) of accelerometer and gyro data
	for (int frame = start_frame_index; frame < 2; ++frame)
	{
		sensor_packet.timestamp = timestamps[frame];

		sensor_packet.raw_imu_accelerometer = {
			hmdState->SensorFrames[frame].RawAccel.i,
			hmdState->SensorFrames[frame].RawAccel.j,
			hmdState->SensorFrames[frame].RawAccel.k };
		sensor_packet.imu_accelerometer_g_units =
			Eigen::Vector3f(
				hmdState->SensorFrames[frame].CalibratedAccel.i,
				hmdState->SensorFrames[frame].CalibratedAccel.j,
				hmdState->SensorFrames[frame].CalibratedAccel.k);
		sensor_packet.has_accelerometer_measurement = true;

		sensor_packet.raw_imu_gyroscope = {
			hmdState->SensorFrames[frame].RawGyro.i,
			hmdState->SensorFrames[frame].RawGyro.j,
			hmdState->SensorFrames[frame].RawGyro.k };
		sensor_packet.imu_gyroscope_rad_per_sec =
			Eigen::Vector3f(
				hmdState->SensorFrames[frame].CalibratedGyro.i,
				hmdState->SensorFrames[frame].CalibratedGyro.j,
				hmdState->SensorFrames[frame].CalibratedGyro.k);
		sensor_packet.has_gyroscope_measurement = true;

		pose_filter_queue->enqueue(sensor_packet);
	}
}


static void post_imu_filter_packets_for_virtual(
	const VirtualHMD *hmd,
	const VirtualHMDState *hmdState,
	const t_high_resolution_timepoint now,
	const t_high_resolution_duration duration_since_last_update,
	t_hmd_pose_sensor_queue *pose_filter_queue)
{
	const VirtualHMDConfig *config = hmd->getConfig();

	PoseSensorPacket sensor_packet;

	sensor_packet.clear();

	sensor_packet.timestamp = now;

	pose_filter_queue->enqueue(sensor_packet);
}

static void post_optical_filter_packet_for_morpheus_hmd(
	const MorpheusHMD *hmd,
	const t_high_resolution_timepoint now,
	const HMDOpticalPoseEstimation *pose_estimation,
	t_hmd_pose_optical_queue *pose_filter_queue)
{
	const MorpheusHMDConfig *config = hmd->getConfig();
	PoseSensorPacket sensor_packet;

	sensor_packet.clear();
	sensor_packet.timestamp = now;

	if (pose_estimation->bOrientationValid)
	{
		sensor_packet.optical_orientation =
			Eigen::Quaternionf(
				pose_estimation->orientation.w,
				pose_estimation->orientation.x,
				pose_estimation->orientation.y,
				pose_estimation->orientation.z);
	}

	// HMD does have an optical position
	if (pose_estimation->bCurrentlyTracking)
	{
		sensor_packet.optical_position_cm =
			Eigen::Vector3f(
				pose_estimation->position_cm.x,
				pose_estimation->position_cm.y,
				pose_estimation->position_cm.z);
		sensor_packet.tracking_projection_area_px_sqr = pose_estimation->projection.screen_area;
	}

	pose_filter_queue->push_back(sensor_packet);
}


static void post_optical_filter_packet_for_virtual_hmd(
	const VirtualHMD *hmd,
	const t_high_resolution_timepoint now,
	const HMDOpticalPoseEstimation *pose_estimation,
	t_hmd_pose_optical_queue *pose_filter_queue)
{
	const VirtualHMDConfig *config = hmd->getConfig();
	PoseSensorPacket sensor_packet;

	sensor_packet.clear();
	sensor_packet.timestamp = now;

	// HMD does have an optical position
	if (pose_estimation->bCurrentlyTracking)
	{
		sensor_packet.optical_position_cm =
			Eigen::Vector3f(
				pose_estimation->position_cm.x,
				pose_estimation->position_cm.y,
				pose_estimation->position_cm.z);
		sensor_packet.tracking_projection_area_px_sqr = pose_estimation->projection.screen_area;
	}

	pose_filter_queue->push_back(sensor_packet);
}

static void generate_morpheus_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const HMDStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame)
{
    const MorpheusHMD *morpheus_hmd = hmd_view->castCheckedConst<MorpheusHMD>();
    const MorpheusHMDConfig *morpheus_config = morpheus_hmd->getConfig();
	const IPoseFilter *pose_filter = hmd_view->getPoseFilter();
    const CommonHMDState *hmd_state = hmd_view->getState();
    const CommonDevicePose hmd_pose = hmd_view->getFilteredPose(morpheus_config->prediction_time, morpheus_config->ang_prediction_time);

    PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket *hmd_data_frame = data_frame->mutable_hmd_data_packet();

    if (hmd_state != nullptr)
    {
        assert(hmd_state->DeviceType == CommonDeviceState::Morpheus);
        const MorpheusHMDState * morpheus_hmd_state = static_cast<const MorpheusHMDState *>(hmd_state);

        PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket_MorpheusState* morpheus_data_frame = 
            hmd_data_frame->mutable_morpheus_state();

		morpheus_data_frame->set_iscurrentlytracking(hmd_view->getIsCurrentlyTracking());
		morpheus_data_frame->set_istrackingenabled(hmd_view->getIsTrackingEnabled());
		morpheus_data_frame->set_isorientationvalid(pose_filter->getIsStateValid());
		morpheus_data_frame->set_ispositionvalid(pose_filter->getIsStateValid());

		morpheus_data_frame->mutable_orientation()->set_w(hmd_pose.Orientation.w);
		morpheus_data_frame->mutable_orientation()->set_x(hmd_pose.Orientation.x);
		morpheus_data_frame->mutable_orientation()->set_y(hmd_pose.Orientation.y);
		morpheus_data_frame->mutable_orientation()->set_z(hmd_pose.Orientation.z);

		if (stream_info->include_position_data)
		{
			morpheus_data_frame->mutable_position_cm()->set_x(hmd_pose.PositionCm.x);
			morpheus_data_frame->mutable_position_cm()->set_y(hmd_pose.PositionCm.y);
			morpheus_data_frame->mutable_position_cm()->set_z(hmd_pose.PositionCm.z);
		}
		else
		{
			morpheus_data_frame->mutable_position_cm()->set_x(0);
			morpheus_data_frame->mutable_position_cm()->set_y(0);
			morpheus_data_frame->mutable_position_cm()->set_z(0);
		}

		// If requested, get the raw sensor data for the hmd
		if (stream_info->include_physics_data)
		{
			const CommonDevicePhysics hmd_physics = hmd_view->getFilteredPhysics();
			auto *physics_data = morpheus_data_frame->mutable_physics_data();

			physics_data->mutable_velocity_cm_per_sec()->set_i(hmd_physics.VelocityCmPerSec.i);
			physics_data->mutable_velocity_cm_per_sec()->set_j(hmd_physics.VelocityCmPerSec.j);
			physics_data->mutable_velocity_cm_per_sec()->set_k(hmd_physics.VelocityCmPerSec.k);

			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_i(hmd_physics.AccelerationCmPerSecSqr.i);
			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_j(hmd_physics.AccelerationCmPerSecSqr.j);
			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_k(hmd_physics.AccelerationCmPerSecSqr.k);

			physics_data->mutable_angular_velocity_rad_per_sec()->set_i(hmd_physics.AngularVelocityRadPerSec.i);
			physics_data->mutable_angular_velocity_rad_per_sec()->set_j(hmd_physics.AngularVelocityRadPerSec.j);
			physics_data->mutable_angular_velocity_rad_per_sec()->set_k(hmd_physics.AngularVelocityRadPerSec.k);

			physics_data->mutable_angular_acceleration_rad_per_sec_sqr()->set_i(hmd_physics.AngularAccelerationRadPerSecSqr.i);
			physics_data->mutable_angular_acceleration_rad_per_sec_sqr()->set_j(hmd_physics.AngularAccelerationRadPerSecSqr.j);
			physics_data->mutable_angular_acceleration_rad_per_sec_sqr()->set_k(hmd_physics.AngularAccelerationRadPerSecSqr.k);
		}

        // If requested, get the raw sensor data for the hmd
        if (stream_info->include_raw_sensor_data)
        {
            PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket_MorpheusState_RawSensorData *raw_sensor_data =
                morpheus_data_frame->mutable_raw_sensor_data();

			// Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
			// Take the most recent frame: [ax1, ay1, az1]
			raw_sensor_data->mutable_accelerometer()->set_i(morpheus_hmd_state->SensorFrames[1].RawAccel.i);
			raw_sensor_data->mutable_accelerometer()->set_j(morpheus_hmd_state->SensorFrames[1].RawAccel.j);
			raw_sensor_data->mutable_accelerometer()->set_k(morpheus_hmd_state->SensorFrames[1].RawAccel.k);

			// Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
			// Take the most recent frame: [wx1, wy1, wz1]
			raw_sensor_data->mutable_gyroscope()->set_i(morpheus_hmd_state->SensorFrames[1].RawGyro.i);
			raw_sensor_data->mutable_gyroscope()->set_j(morpheus_hmd_state->SensorFrames[1].RawGyro.j);
			raw_sensor_data->mutable_gyroscope()->set_k(morpheus_hmd_state->SensorFrames[1].RawGyro.k);
        }

		// If requested, get the raw sensor data for the hmd
		if (stream_info->include_calibrated_sensor_data)
		{
			PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket_MorpheusState_CalibratedSensorData *calibrated_sensor_data =
				morpheus_data_frame->mutable_calibrated_sensor_data();

			// Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
			// Take the most recent frame: [ax1, ay1, az1]
			calibrated_sensor_data->mutable_accelerometer()->set_i(morpheus_hmd_state->SensorFrames[1].CalibratedAccel.i);
			calibrated_sensor_data->mutable_accelerometer()->set_j(morpheus_hmd_state->SensorFrames[1].CalibratedAccel.j);
			calibrated_sensor_data->mutable_accelerometer()->set_k(morpheus_hmd_state->SensorFrames[1].CalibratedAccel.k);

			// Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
			// Take the most recent frame: [wx1, wy1, wz1]
			calibrated_sensor_data->mutable_gyroscope()->set_i(morpheus_hmd_state->SensorFrames[1].CalibratedGyro.i);
			calibrated_sensor_data->mutable_gyroscope()->set_j(morpheus_hmd_state->SensorFrames[1].CalibratedGyro.j);
			calibrated_sensor_data->mutable_gyroscope()->set_k(morpheus_hmd_state->SensorFrames[1].CalibratedGyro.k);
		}

		// If requested, get the raw tracker data for the controller
		if (stream_info->include_raw_tracker_data)
		{
			auto *raw_tracker_data = morpheus_data_frame->mutable_raw_tracker_data();
            int selectedTrackerId= stream_info->selected_tracker_index;
            unsigned int validTrackerBitmask= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
			    const HMDOpticalPoseEstimation *positionEstimate = hmd_view->getTrackerPoseEstimate(trackerId);

                if (positionEstimate != nullptr && positionEstimate->bCurrentlyTracking)
                {
                    validTrackerBitmask |= (1 << trackerId);

                    if (trackerId == selectedTrackerId)
                    {
				        const CommonDevicePosition &trackerRelativePosition = positionEstimate->position_cm;
				        const ServerTrackerViewPtr tracker_view = DeviceManager::getInstance()->getTrackerViewPtr(trackerId);

				        // Project the 3d camera position back onto the tracker screen
				        {
					        const CommonDeviceScreenLocation trackerScreenLocation =
						        tracker_view->projectTrackerRelativePosition(&trackerRelativePosition);
					        PSMoveProtocol::Pixel *pixel = raw_tracker_data->mutable_screen_location();

					        pixel->set_x(trackerScreenLocation.x);
					        pixel->set_y(trackerScreenLocation.y);
				        }

				        // Add the tracker relative 3d position
				        {
					        PSMoveProtocol::Position *position = raw_tracker_data->mutable_relative_position_cm();

					        position->set_x(trackerRelativePosition.x);
					        position->set_y(trackerRelativePosition.y);
					        position->set_z(trackerRelativePosition.z);
				        }

				        // Add the tracker relative projection shapes
				        {
					        const CommonDeviceTrackingProjection &trackerRelativeProjection =
						        positionEstimate->projection;

					        assert(trackerRelativeProjection.shape_type == eCommonTrackingProjectionType::ProjectionType_Points);
					        PSMoveProtocol::Polygon *polygon = raw_tracker_data->mutable_projected_point_cloud();

					        for (int vert_index = 0; vert_index < trackerRelativeProjection.shape.points.point_count; ++vert_index)
					        {
						        PSMoveProtocol::Pixel *pixel = polygon->add_vertices();

						        pixel->set_x(trackerRelativeProjection.shape.points.point[vert_index].x);
						        pixel->set_y(trackerRelativeProjection.shape.points.point[vert_index].y);
					        }
				        }

				        raw_tracker_data->set_tracker_id(trackerId);
                    }
                }
            }
            raw_tracker_data->set_valid_tracker_bitmask(validTrackerBitmask);
		}
    }

    hmd_data_frame->set_hmd_type(PSMoveProtocol::Morpheus);
}

static void generate_virtual_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const HMDStreamInfo *stream_info,
    DeviceOutputDataFramePtr &data_frame)
{
    const VirtualHMD *virtual_hmd = hmd_view->castCheckedConst<VirtualHMD>();
    const VirtualHMDConfig *virtual_hmd_config = virtual_hmd->getConfig();
	const IPoseFilter *pose_filter = hmd_view->getPoseFilter();
    const CommonHMDState *hmd_state = hmd_view->getState();
    const CommonDevicePose hmd_pose = hmd_view->getFilteredPose(virtual_hmd_config->prediction_time, virtual_hmd_config->ang_prediction_time);

    PSMoveProtocol::DeviceOutputDataFrame_HMDDataPacket *hmd_data_frame = data_frame->mutable_hmd_data_packet();

    if (hmd_state != nullptr)
    {
        assert(hmd_state->DeviceType == CommonDeviceState::VirtualHMD);
        const VirtualHMDState * virtual_hmd_state = static_cast<const VirtualHMDState *>(hmd_state);

        auto * virtual_hmd_data_frame = hmd_data_frame->mutable_virtual_hmd_state();

		virtual_hmd_data_frame->set_iscurrentlytracking(hmd_view->getIsCurrentlyTracking());
		virtual_hmd_data_frame->set_istrackingenabled(hmd_view->getIsTrackingEnabled());
		virtual_hmd_data_frame->set_ispositionvalid(pose_filter->getIsStateValid());

		if (stream_info->include_position_data)
		{
			virtual_hmd_data_frame->mutable_position_cm()->set_x(hmd_pose.PositionCm.x);
			virtual_hmd_data_frame->mutable_position_cm()->set_y(hmd_pose.PositionCm.y);
			virtual_hmd_data_frame->mutable_position_cm()->set_z(hmd_pose.PositionCm.z);
		}
		else
		{
			virtual_hmd_data_frame->mutable_position_cm()->set_x(0);
			virtual_hmd_data_frame->mutable_position_cm()->set_y(0);
			virtual_hmd_data_frame->mutable_position_cm()->set_z(0);
		}

		// If requested, get the raw sensor data for the hmd
		if (stream_info->include_physics_data)
		{
			const CommonDevicePhysics hmd_physics = hmd_view->getFilteredPhysics();
			auto *physics_data = virtual_hmd_data_frame->mutable_physics_data();

			physics_data->mutable_velocity_cm_per_sec()->set_i(hmd_physics.VelocityCmPerSec.i);
			physics_data->mutable_velocity_cm_per_sec()->set_j(hmd_physics.VelocityCmPerSec.j);
			physics_data->mutable_velocity_cm_per_sec()->set_k(hmd_physics.VelocityCmPerSec.k);

			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_i(hmd_physics.AccelerationCmPerSecSqr.i);
			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_j(hmd_physics.AccelerationCmPerSecSqr.j);
			physics_data->mutable_acceleration_cm_per_sec_sqr()->set_k(hmd_physics.AccelerationCmPerSecSqr.k);
		}

		// If requested, get the raw tracker data for the controller
		if (stream_info->include_raw_tracker_data)
		{
			auto *raw_tracker_data = virtual_hmd_data_frame->mutable_raw_tracker_data();
			int selectedTrackerId= stream_info->selected_tracker_index;
            unsigned int validTrackerBitmask= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
			    const HMDOpticalPoseEstimation *positionEstimate = hmd_view->getTrackerPoseEstimate(trackerId);

                if (positionEstimate != nullptr && positionEstimate->bCurrentlyTracking)
                {
                    validTrackerBitmask |= (1 << trackerId);

                    if (trackerId == selectedTrackerId)
                    {
				        const CommonDevicePosition &trackerRelativePosition = positionEstimate->position_cm;
				        const ServerTrackerViewPtr tracker_view = DeviceManager::getInstance()->getTrackerViewPtr(selectedTrackerId);

				        // Project the 3d camera position back onto the tracker screen
				        {
					        const CommonDeviceScreenLocation trackerScreenLocation =
						        tracker_view->projectTrackerRelativePosition(&trackerRelativePosition);
					        PSMoveProtocol::Pixel *pixel = raw_tracker_data->mutable_screen_location();

					        pixel->set_x(trackerScreenLocation.x);
					        pixel->set_y(trackerScreenLocation.y);
				        }

				        // Add the tracker relative 3d position
				        {
					        PSMoveProtocol::Position *position = raw_tracker_data->mutable_relative_position_cm();

					        position->set_x(trackerRelativePosition.x);
					        position->set_y(trackerRelativePosition.y);
					        position->set_z(trackerRelativePosition.z);
				        }

				        // Add the tracker relative projection shapes
				        {
					        const CommonDeviceTrackingProjection &trackerRelativeProjection =
						        positionEstimate->projection;

					        assert(trackerRelativeProjection.shape_type == eCommonTrackingProjectionType::ProjectionType_Ellipse);
					        PSMoveProtocol::Ellipse *ellipse = raw_tracker_data->mutable_projected_sphere();

                            ellipse->mutable_center()->set_x(trackerRelativeProjection.shape.ellipse.center.x);
                            ellipse->mutable_center()->set_y(trackerRelativeProjection.shape.ellipse.center.y);
                            ellipse->set_half_x_extent(trackerRelativeProjection.shape.ellipse.half_x_extent);
                            ellipse->set_half_y_extent(trackerRelativeProjection.shape.ellipse.half_y_extent);
                            ellipse->set_angle(trackerRelativeProjection.shape.ellipse.angle);
				        }

				        raw_tracker_data->set_tracker_id(selectedTrackerId);
                    }
                }
            }
            raw_tracker_data->set_valid_tracker_bitmask(validTrackerBitmask);
		}
    }

    hmd_data_frame->set_hmd_type(PSMoveProtocol::VirtualHMD);
}

static Eigen::Vector3f CommonDevicePosition_to_EigenVector3f(const CommonDevicePosition &p)
{
    return Eigen::Vector3f(p.x, p.y, p.z);
}

static Eigen::Vector3f CommonDeviceVector_to_EigenVector3f(const CommonDeviceVector &v)
{
    return Eigen::Vector3f(v.i, v.j, v.k);
}

static Eigen::Quaternionf CommonDeviceQuaternion_to_EigenQuaternionf(const CommonDeviceQuaternion &q)
{
    return Eigen::Quaternionf(q.w, q.x, q.y, q.z);
}

static CommonDevicePosition EigenVector3f_to_CommonDevicePosition(const Eigen::Vector3f &p)
{
    CommonDevicePosition result;

    result.x = p.x();
    result.y = p.y();
    result.z = p.z();

    return result;
}

static CommonDeviceQuaternion EigenQuaternionf_to_CommonDeviceQuaternion(const Eigen::Quaternionf &q)
{
    CommonDeviceQuaternion result;

    result.w = q.w();
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();

    return result;
}

static void computeSpherePoseForHmdFromSingleTracker(
    const ServerHMDView *hmdView,
    const ServerTrackerViewPtr tracker,
    HMDOpticalPoseEstimation *tracker_pose_estimation,
    HMDOpticalPoseEstimation *multicam_pose_estimation)
{
    // No orientation for the sphere projection
    multicam_pose_estimation->orientation.clear();
    multicam_pose_estimation->bOrientationValid = false;

    // For the sphere projection, the tracker relative position has already been computed
    // Put the tracker relative position into world space
    multicam_pose_estimation->position_cm = tracker->computeWorldPosition(&tracker_pose_estimation->position_cm);
    multicam_pose_estimation->bCurrentlyTracking = true;

    // Copy over the screen projection area
    multicam_pose_estimation->projection.screen_area = tracker_pose_estimation->projection.screen_area;
}

static void computePointCloudPoseForHmdFromSingleTracker(
    const ServerHMDView *hmdView,
    const ServerTrackerViewPtr tracker,
    HMDOpticalPoseEstimation *tracker_pose_estimation,
    HMDOpticalPoseEstimation *multicam_pose_estimation)
{
    // No orientation for the sphere projection
    multicam_pose_estimation->orientation= tracker->computeWorldOrientation(&tracker_pose_estimation->orientation);
    multicam_pose_estimation->bOrientationValid = tracker_pose_estimation->bOrientationValid;

    // For the sphere projection, the tracker relative position has already been computed
    // Put the tracker relative position into world space
    multicam_pose_estimation->position_cm = tracker->computeWorldPosition(&tracker_pose_estimation->position_cm);
    multicam_pose_estimation->bCurrentlyTracking = true;

    // Copy over the screen projection area
    multicam_pose_estimation->projection.screen_area = tracker_pose_estimation->projection.screen_area;
}

static void computeSpherePoseForHmdFromMultipleTrackers(
    const ServerHMDView *hmdView,
    const TrackerManager* tracker_manager,
    const int *valid_projection_tracker_ids,
    const int projections_found,
    HMDOpticalPoseEstimation *tracker_pose_estimations,
    HMDOpticalPoseEstimation *multicam_pose_estimation)
{
	int available_trackers = 0;

	for (int tracker_id = 0; tracker_id < tracker_manager->getMaxDevices(); ++tracker_id)
	{
		ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);

		if (tracker->getIsOpen())
		{
			available_trackers++;
		}
	}

	struct positionOffsetCaching
	{
		bool isValid;
		CommonDevicePosition local_position;
		CommonDevicePosition world_avg_position;
		CommonDevicePosition new_local_position;
		int paired_trackers;
	};
	struct orgPositionOffsetCaching
	{
		int tracker_1_id;
		int tracker_2_id;
		int index;
	};
	static std::vector<positionOffsetCaching> globalPositionOffsetCaching[TrackerManager::k_max_devices][TrackerManager::k_max_devices];
	static int globalPositionOffsetCachingCount[TrackerManager::k_max_devices][TrackerManager::k_max_devices];
	std::vector<orgPositionOffsetCaching> newPositionOffsetCaching;

	const TrackerManagerConfig &cfg = tracker_manager->getConfig();
	float screen_area_sum = 0;

	// Compute triangulations amongst all pairs of projections
	int pair_count = 0;


	struct projectionInfo
	{
		int index;
		int tracker_id;
		CommonDeviceScreenLocation position2d_list;
		float screen_area;
	};
	std::vector<projectionInfo> sorted_projections;

	// Project the tracker relative 3d tracking position back on to the tracker camera plane
	// and sum up the total controller projection area across all trackers
	for (int list_index = 0; list_index < projections_found; ++list_index)
	{
		const int tracker_id = valid_projection_tracker_ids[list_index];
		const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
		const HMDOpticalPoseEstimation &poseEstimate = tracker_pose_estimations[tracker_id];

		projectionInfo info;
		info.index = list_index;
		info.tracker_id = tracker_id;
		info.position2d_list = tracker->projectTrackerRelativePosition(&poseEstimate.position_cm);
		info.screen_area = tracker_pose_estimations[tracker_id].projection.screen_area;
		sorted_projections.push_back(info);

		screen_area_sum += poseEstimate.projection.screen_area;
	}

	CommonDevicePosition average_world_position = { 0.f, 0.f, 0.f };
	CommonDevicePosition unfiltered_average_world_position = { 0.f, 0.f, 0.f };
    for (int list_index = 0; list_index < projections_found; ++list_index)
    {
		int bad_deviations = 0;

		const int tracker_id = sorted_projections[list_index].tracker_id;
		const CommonDeviceScreenLocation &screen_location = sorted_projections[list_index].position2d_list;
		const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);

        for (int other_list_index = list_index + 1; other_list_index < projections_found; ++other_list_index)
        {
			if (list_index == other_list_index)
				continue;

			const int other_tracker_id = sorted_projections[other_list_index].tracker_id;
			const CommonDeviceScreenLocation &other_screen_location = sorted_projections[other_list_index].position2d_list;
			const ServerTrackerViewPtr other_tracker = tracker_manager->getTrackerViewPtr(other_tracker_id);

            // if trackers are on opposite sides
            if (cfg.exclude_opposed_cameras)
            {
                if ((tracker->getTrackerPose().PositionCm.x > 0) == (other_tracker->getTrackerPose().PositionCm.x < 0) &&
                    (tracker->getTrackerPose().PositionCm.z > 0) == (other_tracker->getTrackerPose().PositionCm.z < 0))
                {
                    continue;
                }
            }

            // Using the screen locations on two different trackers we can triangulate a world position
            CommonDevicePosition world_position =
                ServerTrackerView::triangulateWorldPosition(
                    tracker.get(), &screen_location,
                    other_tracker.get(), &other_screen_location);

			bool add_pair = false;

			// Check how much the trangulation deviates from other trackers.
			// Ignore its position if it deviates too much and renew its ROI.
			if (pair_count > 0 && cfg.max_tracker_position_deviation > 0.01f)
			{
				const float N = static_cast<float>(pair_count);

				const float distance = sqrtf(
					pow(abs((average_world_position.x / N) - world_position.x), 2) +
					pow(abs((average_world_position.y / N) - world_position.y), 2) +
					pow(abs((average_world_position.z / N) - world_position.z), 2)
				);

				if (distance < cfg.max_tracker_position_deviation)
				{
					add_pair = true;
				}
				else
				{
					++bad_deviations;
				}
			}
			else
			{
				add_pair = true;
			}

			if (add_pair)
			{
				// Do some runtime position caching to make transitions between cameras smoother.
				// Give each camera an offset from the total average.
				if (cfg.average_position_cache_enabled)
				{
					++pair_count;

					unfiltered_average_world_position.x += world_position.x;
					unfiltered_average_world_position.y += world_position.y;
					unfiltered_average_world_position.z += world_position.z;

					float cell_size = fmax(1.f, cfg.average_position_cache_cell_size);
					float avg_size = fmax(0.f, cfg.average_position_cache_avg_size);
					int avg_limit = (int)fmax(1.f, cfg.average_position_cache_limit);

					// Find the closest cache item
					int cacheIndex = -1;
					float lastCacheDistance = cell_size;
					for (int j = globalPositionOffsetCaching[tracker_id][other_tracker_id].size() - 1; j >= 0; --j)
					{
						if (!globalPositionOffsetCaching[tracker_id][other_tracker_id][j].isValid)
							continue;

						const float distance = sqrtf(
							pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.x - world_position.x), 2) +
							pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.y - world_position.y), 2) +
							pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.z - world_position.z), 2)
						);

						if (lastCacheDistance == -1.f || distance < lastCacheDistance)
						{
							cacheIndex = j;
							lastCacheDistance = distance;
						}
					}

					// Cycle buffer if limit is reached
					int cycleCacheIndex = -1;
					if (cacheIndex == -1 && globalPositionOffsetCaching[tracker_id][other_tracker_id].size() >= avg_limit)
					{
						if (globalPositionOffsetCachingCount[tracker_id][other_tracker_id] >= avg_limit)
							globalPositionOffsetCachingCount[tracker_id][other_tracker_id] = 0;

						cycleCacheIndex = globalPositionOffsetCachingCount[tracker_id][other_tracker_id];
					}

					if (cycleCacheIndex > -1)
					{
						orgPositionOffsetCaching orgCache;
						orgCache.tracker_1_id = tracker_id;
						orgCache.tracker_2_id = other_tracker_id;
						orgCache.index = cycleCacheIndex;
						newPositionOffsetCaching.push_back(orgCache);

						positionOffsetCaching *cache = &globalPositionOffsetCaching[tracker_id][other_tracker_id][cycleCacheIndex];
						cache->isValid = false;
						cache->local_position.x = world_position.x;
						cache->local_position.y = world_position.y;
						cache->local_position.z = world_position.z;
						cache->world_avg_position.x = 0.f;
						cache->world_avg_position.y = 0.f;
						cache->world_avg_position.z = 0.f;
						cache->new_local_position.x = world_position.x;
						cache->new_local_position.y = world_position.y;
						cache->new_local_position.z = world_position.z;
						cache->paired_trackers = 0;

						globalPositionOffsetCachingCount[tracker_id][other_tracker_id]++;
					}
					else if (cacheIndex == -1)
					{
						orgPositionOffsetCaching orgCache;
						orgCache.tracker_1_id = tracker_id;
						orgCache.tracker_2_id = other_tracker_id;
						orgCache.index = globalPositionOffsetCaching[tracker_id][other_tracker_id].size();
						newPositionOffsetCaching.push_back(orgCache);

						positionOffsetCaching cache;
						cache.isValid = false;
						cache.local_position.x = world_position.x;
						cache.local_position.y = world_position.y;
						cache.local_position.z = world_position.z;
						cache.world_avg_position.x = 0.f;
						cache.world_avg_position.y = 0.f;
						cache.world_avg_position.z = 0.f;
						cache.new_local_position.x = world_position.x;
						cache.new_local_position.y = world_position.y;
						cache.new_local_position.z = world_position.z;
						cache.paired_trackers = 0;

						globalPositionOffsetCaching[tracker_id][other_tracker_id].push_back(cache);
						globalPositionOffsetCachingCount[tracker_id][other_tracker_id]++;
					}
					else
					{
						orgPositionOffsetCaching orgCache;
						orgCache.tracker_1_id = tracker_id;
						orgCache.tracker_2_id = other_tracker_id;
						orgCache.index = cacheIndex;
						newPositionOffsetCaching.push_back(orgCache);

						positionOffsetCaching *cache = &globalPositionOffsetCaching[tracker_id][other_tracker_id][cacheIndex];

						cache->new_local_position.x = world_position.x;
						cache->new_local_position.y = world_position.y;
						cache->new_local_position.z = world_position.z;
					}

					if (avg_size >= cell_size)
					{
						// Get average from offsets to make transitions between samples smoother
						CommonDevicePosition average_new_position = { 0.f, 0.f, 0.f };
						int average_new_position_pair = 0;

						for (int j = globalPositionOffsetCaching[tracker_id][other_tracker_id].size() - 1; j >= 0; --j)
						{
							if (!globalPositionOffsetCaching[tracker_id][other_tracker_id][j].isValid)
								continue;

							const float distance = sqrtf(
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.x - world_position.x), 2) +
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.y - world_position.y), 2) +
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.z - world_position.z), 2)
							);

							if (distance < avg_size)
							{
								average_new_position.x += world_position.x + (
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].world_avg_position.x -
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.x
									);
								average_new_position.y += world_position.y + (
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].world_avg_position.y -
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.y
									);
								average_new_position.z += world_position.z + (
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].world_avg_position.z -
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.z
									);
								++average_new_position_pair;
							}
						}

						if (average_new_position_pair == 0)
						{
							average_world_position.x += world_position.x;
							average_world_position.y += world_position.y;
							average_world_position.z += world_position.z;
						}
						else
						{
							const float N = static_cast<float>(average_new_position_pair);
							average_world_position.x += (average_new_position.x / N);
							average_world_position.y += (average_new_position.y / N);
							average_world_position.z += (average_new_position.z / N);
						}
					}
					else
					{
						// Find nearest average position regardless of distance
						int nearCacheIndex = -1;
						float lastCacheDistance = -1.f;
						for (int j = globalPositionOffsetCaching[tracker_id][other_tracker_id].size() - 1; j >= 0; --j)
						{
							if (!globalPositionOffsetCaching[tracker_id][other_tracker_id][j].isValid)
								continue;

							float distance = sqrtf(
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.x - world_position.x), 2) +
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.y - world_position.y), 2) +
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.z - world_position.z), 2)
							);

							if (lastCacheDistance == -1.f || distance < lastCacheDistance)
							{
								nearCacheIndex = j;
								lastCacheDistance = distance;
							}
						}

						if (nearCacheIndex != -1)
						{
							average_world_position.x += world_position.x + (
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].world_avg_position.x -
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].local_position.x
								);
							average_world_position.y += world_position.y + (
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].world_avg_position.y -
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].local_position.y
								);
							average_world_position.z += world_position.z + (
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].world_avg_position.z -
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].local_position.z
								);
						}
						else
						{
							average_world_position.x += world_position.x;
							average_world_position.y += world_position.y;
							average_world_position.z += world_position.z;
						}
					}
				}
				else
				{
					++pair_count;

					average_world_position.x += world_position.x;
					average_world_position.y += world_position.y;
					average_world_position.z += world_position.z;

					unfiltered_average_world_position.x += world_position.x;
					unfiltered_average_world_position.y += world_position.y;
					unfiltered_average_world_position.z += world_position.z;
				}
			}
        }

		// What happend to that trackers projection? Its probably stuck somewhere on some color noise.
		// Enforce new ROI on this tracker to make it unstuck.
		if (bad_deviations >= projections_found - 1)
		{
			tracker_pose_estimations[tracker_id].bEnforceNewROI = true;
		}
    }

    if (pair_count == 0 
		&& projections_found > 0
		&& (available_trackers == 1 || !cfg.ignore_pose_from_one_tracker))
    {
        // Position not triangulated from opposed camera, estimate from one tracker only.
        computeSpherePoseForHmdFromSingleTracker(
            hmdView,
            tracker_manager->getTrackerViewPtr(0),
            &tracker_pose_estimations[0],
            multicam_pose_estimation);		
    }
    else if(pair_count > 0)
    {
        // Compute the average position
        const float N = static_cast<float>(pair_count);

		average_world_position.x /= N;
		average_world_position.y /= N;
		average_world_position.z /= N;

		unfiltered_average_world_position.x /= N;
		unfiltered_average_world_position.y /= N;
		unfiltered_average_world_position.z /= N;

		if (cfg.average_position_cache_enabled)
		{
			// Renew cached average world position when we have more samples
			for (int j = newPositionOffsetCaching.size() - 1; j >= 0; --j)
			{
				const orgPositionOffsetCaching orgCache = newPositionOffsetCaching[j];
				positionOffsetCaching *cache = &globalPositionOffsetCaching[orgCache.tracker_1_id][orgCache.tracker_2_id][orgCache.index];

				// Only update when theres new pairs.
				if ((cache->paired_trackers & ((1 << orgCache.tracker_1_id) | (1 << (orgCache.tracker_2_id + PSMOVESERVICE_MAX_TRACKER_COUNT)))) > 0)
					continue;
				if ((cache->paired_trackers & ((1 << orgCache.tracker_2_id) | (1 << (orgCache.tracker_1_id + PSMOVESERVICE_MAX_TRACKER_COUNT)))) > 0)
					continue;


				cache->world_avg_position.x = unfiltered_average_world_position.x;
				cache->world_avg_position.y = unfiltered_average_world_position.y;
				cache->world_avg_position.z = unfiltered_average_world_position.z;
				cache->local_position.x = cache->new_local_position.x;
				cache->local_position.y = cache->new_local_position.y;
				cache->local_position.z = cache->new_local_position.z;
				cache->paired_trackers |= (1 << orgCache.tracker_1_id) | (1 << (orgCache.tracker_2_id + PSMOVESERVICE_MAX_TRACKER_COUNT));
				cache->isValid = true;
			}
		}

		//printf("Cache Size: %d\n", globalPositionOffsetCaching[0][1].size());
		//printf("Cache Total Size: %d/%d\n", globalPositionOffsetCachingCount[0][1], (int)fmax(1.f, cfg.average_position_cache_limit));

        // Store the averaged tracking position
		multicam_pose_estimation->position_cm = average_world_position;
        multicam_pose_estimation->bCurrentlyTracking = true;
    }

    // No orientation for the sphere projection
    multicam_pose_estimation->orientation.clear();
    multicam_pose_estimation->bOrientationValid = false;

    // Compute the average projection area.
    // This is proportional to our position tracking quality.
    multicam_pose_estimation->projection.screen_area =
        screen_area_sum / static_cast<float>(projections_found);
}

static void computePointCloudPoseForHmdFromMultipleTrackers(
    const ServerHMDView *hmdView,
    const TrackerManager* tracker_manager,
    const int *valid_projection_tracker_ids,
    const int projections_found,
    HMDOpticalPoseEstimation *tracker_pose_estimations,
    HMDOpticalPoseEstimation *multicam_pose_estimation)
{
	int available_trackers = 0;

	for (int tracker_id = 0; tracker_id < tracker_manager->getMaxDevices(); ++tracker_id)
	{
		ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);

		if (tracker->getIsOpen())
		{
			available_trackers++;
		}
	}

	struct positionOffsetCaching
	{
		bool isValid;
		CommonDevicePosition local_position;
		CommonDevicePosition world_avg_position;
		CommonDevicePosition new_local_position;
		int paired_trackers;
	};
	struct orgPositionOffsetCaching
	{
		int tracker_1_id;
		int tracker_2_id;
		int index;
	};
	static std::vector<positionOffsetCaching> globalPositionOffsetCaching[TrackerManager::k_max_devices][TrackerManager::k_max_devices];
	static int globalPositionOffsetCachingCount[TrackerManager::k_max_devices][TrackerManager::k_max_devices];
	std::vector<orgPositionOffsetCaching> newPositionOffsetCaching;

	const TrackerManagerConfig &cfg = tracker_manager->getConfig();
	float screen_area_sum = 0;

	// Compute triangulations amongst all pairs of projections
	int pair_count = 0;


	struct projectionInfo
	{
		int index;
		int tracker_id;
		CommonDeviceScreenLocation position2d_list;
		float screen_area;
	};
	std::vector<projectionInfo> sorted_projections;

	// Project the tracker relative 3d tracking position back on to the tracker camera plane
	// and sum up the total controller projection area across all trackers
	for (int list_index = 0; list_index < projections_found; ++list_index)
	{
		const int tracker_id = valid_projection_tracker_ids[list_index];
		const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
		const HMDOpticalPoseEstimation &poseEstimate = tracker_pose_estimations[tracker_id];

		projectionInfo info;
		info.index = list_index;
		info.tracker_id = tracker_id;
		info.position2d_list = tracker->projectTrackerRelativePosition(&poseEstimate.position_cm);
		info.screen_area = tracker_pose_estimations[tracker_id].projection.screen_area;
		sorted_projections.push_back(info);

		screen_area_sum += poseEstimate.projection.screen_area;
	}


    // Compute triangulations amongst all pairs of projections
	CommonDevicePosition average_world_position = { 0.f, 0.f, 0.f };
	CommonDevicePosition unfiltered_average_world_position = { 0.f, 0.f, 0.f };
    for (int list_index = 0; list_index < projections_found; ++list_index)
    {
		int bad_deviations = 0;

		const int tracker_id = sorted_projections[list_index].tracker_id;
		const CommonDeviceScreenLocation &screen_location = sorted_projections[list_index].position2d_list;
		const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);

        for (int other_list_index = list_index + 1; other_list_index < projections_found; ++other_list_index)
        {
			if (list_index == other_list_index)
				continue;

			const int other_tracker_id = sorted_projections[other_list_index].tracker_id;
			const CommonDeviceScreenLocation &other_screen_location = sorted_projections[other_list_index].position2d_list;
			const ServerTrackerViewPtr other_tracker = tracker_manager->getTrackerViewPtr(other_tracker_id);

            // if trackers are on opposite sides
            if (cfg.exclude_opposed_cameras)
            {
                if ((tracker->getTrackerPose().PositionCm.x > 0) == (other_tracker->getTrackerPose().PositionCm.x < 0) &&
                    (tracker->getTrackerPose().PositionCm.z > 0) == (other_tracker->getTrackerPose().PositionCm.z < 0))
                {
                    continue;
                }
            }

            // Using the screen locations on two different trackers we can triangulate a world position
            CommonDevicePosition world_position =
                ServerTrackerView::triangulateWorldPosition(
                    tracker.get(), &screen_location,
                    other_tracker.get(), &other_screen_location);

			bool add_pair = false;

			// Check how much the trangulation deviates from other trackers.
			// Ignore its position if it deviates too much and renew its ROI.
			if (pair_count > 0 && cfg.max_tracker_position_deviation > 0.01f)
			{
				const float N = static_cast<float>(pair_count);

				const float distance = sqrtf(
					pow(abs((average_world_position.x / N) - world_position.x), 2) +
					pow(abs((average_world_position.y / N) - world_position.y), 2) +
					pow(abs((average_world_position.z / N) - world_position.z), 2)
				);

				if (distance < cfg.max_tracker_position_deviation)
				{
					add_pair = true;
				}
				else
				{
					++bad_deviations;
				}
			}
			else
			{
				add_pair = true;
			}

			if (add_pair)
			{
				// Do some runtime position caching to make transitions between cameras smoother.
				// Give each camera an offset from the total average.
				if (cfg.average_position_cache_enabled)
				{
					++pair_count;

					unfiltered_average_world_position.x += world_position.x;
					unfiltered_average_world_position.y += world_position.y;
					unfiltered_average_world_position.z += world_position.z;

					float cell_size = fmax(1.f, cfg.average_position_cache_cell_size);
					float avg_size = fmax(0.f, cfg.average_position_cache_avg_size);
					int avg_limit = (int)fmax(1.f, cfg.average_position_cache_limit);

					// Find the closest cache item
					int cacheIndex = -1;
					float lastCacheDistance = cell_size;
					for (int j = globalPositionOffsetCaching[tracker_id][other_tracker_id].size() - 1; j >= 0; --j)
					{
						if (!globalPositionOffsetCaching[tracker_id][other_tracker_id][j].isValid)
							continue;

						const float distance = sqrtf(
							pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.x - world_position.x), 2) +
							pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.y - world_position.y), 2) +
							pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.z - world_position.z), 2)
						);

						if (lastCacheDistance == -1.f || distance < lastCacheDistance)
						{
							cacheIndex = j;
							lastCacheDistance = distance;
						}
					}

					// Cycle buffer if limit is reached
					int cycleCacheIndex = -1;
					if (cacheIndex == -1 && globalPositionOffsetCaching[tracker_id][other_tracker_id].size() >= avg_limit)
					{
						if (globalPositionOffsetCachingCount[tracker_id][other_tracker_id] >= avg_limit)
							globalPositionOffsetCachingCount[tracker_id][other_tracker_id] = 0;

						cycleCacheIndex = globalPositionOffsetCachingCount[tracker_id][other_tracker_id];
					}

					if (cycleCacheIndex > -1)
					{
						orgPositionOffsetCaching orgCache;
						orgCache.tracker_1_id = tracker_id;
						orgCache.tracker_2_id = other_tracker_id;
						orgCache.index = cycleCacheIndex;
						newPositionOffsetCaching.push_back(orgCache);

						positionOffsetCaching *cache = &globalPositionOffsetCaching[tracker_id][other_tracker_id][cycleCacheIndex];
						cache->isValid = false;
						cache->local_position.x = world_position.x;
						cache->local_position.y = world_position.y;
						cache->local_position.z = world_position.z;
						cache->world_avg_position.x = 0.f;
						cache->world_avg_position.y = 0.f;
						cache->world_avg_position.z = 0.f;
						cache->new_local_position.x = world_position.x;
						cache->new_local_position.y = world_position.y;
						cache->new_local_position.z = world_position.z;
						cache->paired_trackers = 0;

						globalPositionOffsetCachingCount[tracker_id][other_tracker_id]++;
					}
					else if (cacheIndex == -1)
					{
						orgPositionOffsetCaching orgCache;
						orgCache.tracker_1_id = tracker_id;
						orgCache.tracker_2_id = other_tracker_id;
						orgCache.index = globalPositionOffsetCaching[tracker_id][other_tracker_id].size();
						newPositionOffsetCaching.push_back(orgCache);

						positionOffsetCaching cache;
						cache.isValid = false;
						cache.local_position.x = world_position.x;
						cache.local_position.y = world_position.y;
						cache.local_position.z = world_position.z;
						cache.world_avg_position.x = 0.f;
						cache.world_avg_position.y = 0.f;
						cache.world_avg_position.z = 0.f;
						cache.new_local_position.x = world_position.x;
						cache.new_local_position.y = world_position.y;
						cache.new_local_position.z = world_position.z;
						cache.paired_trackers = 0;

						globalPositionOffsetCaching[tracker_id][other_tracker_id].push_back(cache);
						globalPositionOffsetCachingCount[tracker_id][other_tracker_id]++;
					}
					else
					{
						orgPositionOffsetCaching orgCache;
						orgCache.tracker_1_id = tracker_id;
						orgCache.tracker_2_id = other_tracker_id;
						orgCache.index = cacheIndex;
						newPositionOffsetCaching.push_back(orgCache);

						positionOffsetCaching *cache = &globalPositionOffsetCaching[tracker_id][other_tracker_id][cacheIndex];

						cache->new_local_position.x = world_position.x;
						cache->new_local_position.y = world_position.y;
						cache->new_local_position.z = world_position.z;
					}

					if (avg_size >= cell_size)
					{
						// Get average from offsets to make transitions between samples smoother
						CommonDevicePosition average_new_position = { 0.f, 0.f, 0.f };
						int average_new_position_pair = 0;

						for (int j = globalPositionOffsetCaching[tracker_id][other_tracker_id].size() - 1; j >= 0; --j)
						{
							if (!globalPositionOffsetCaching[tracker_id][other_tracker_id][j].isValid)
								continue;

							const float distance = sqrtf(
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.x - world_position.x), 2) +
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.y - world_position.y), 2) +
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.z - world_position.z), 2)
							);

							if (distance < avg_size)
							{
								average_new_position.x += world_position.x + (
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].world_avg_position.x -
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.x
									);
								average_new_position.y += world_position.y + (
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].world_avg_position.y -
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.y
									);
								average_new_position.z += world_position.z + (
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].world_avg_position.z -
									globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.z
									);
								++average_new_position_pair;
							}
						}

						if (average_new_position_pair == 0)
						{
							average_world_position.x += world_position.x;
							average_world_position.y += world_position.y;
							average_world_position.z += world_position.z;
						}
						else
						{
							const float N = static_cast<float>(average_new_position_pair);
							average_world_position.x += (average_new_position.x / N);
							average_world_position.y += (average_new_position.y / N);
							average_world_position.z += (average_new_position.z / N);
						}
					}
					else
					{
						// Find nearest average position regardless of distance
						int nearCacheIndex = -1;
						float lastCacheDistance = -1.f;
						for (int j = globalPositionOffsetCaching[tracker_id][other_tracker_id].size() - 1; j >= 0; --j)
						{
							if (!globalPositionOffsetCaching[tracker_id][other_tracker_id][j].isValid)
								continue;

							float distance = sqrtf(
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.x - world_position.x), 2) +
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.y - world_position.y), 2) +
								pow(abs(globalPositionOffsetCaching[tracker_id][other_tracker_id][j].local_position.z - world_position.z), 2)
							);

							if (lastCacheDistance == -1.f || distance < lastCacheDistance)
							{
								nearCacheIndex = j;
								lastCacheDistance = distance;
							}
						}

						if (nearCacheIndex != -1)
						{
							average_world_position.x += world_position.x + (
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].world_avg_position.x -
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].local_position.x
								);
							average_world_position.y += world_position.y + (
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].world_avg_position.y -
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].local_position.y
								);
							average_world_position.z += world_position.z + (
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].world_avg_position.z -
								globalPositionOffsetCaching[tracker_id][other_tracker_id][nearCacheIndex].local_position.z
								);
						}
						else
						{
							average_world_position.x += world_position.x;
							average_world_position.y += world_position.y;
							average_world_position.z += world_position.z;
						}
					}
				}
				else
				{
					++pair_count;

					average_world_position.x += world_position.x;
					average_world_position.y += world_position.y;
					average_world_position.z += world_position.z;

					unfiltered_average_world_position.x += world_position.x;
					unfiltered_average_world_position.y += world_position.y;
					unfiltered_average_world_position.z += world_position.z;
				}
			}
        }

		// What happend to that trackers projection? Its probably stuck somewhere on some color noise.
		// Enforce new ROI on this tracker to make it unstuck.
		if (bad_deviations >= projections_found - 1)
		{
			tracker_pose_estimations[tracker_id].bEnforceNewROI = true;
		}
    }

    if (pair_count == 0 
		&& projections_found > 0
		&& (available_trackers == 1 || !cfg.ignore_pose_from_one_tracker))
    {
        // Position not triangulated from opposed camera, estimate from one tracker only.
        computePointCloudPoseForHmdFromSingleTracker(
            hmdView,
            tracker_manager->getTrackerViewPtr(0),
            &tracker_pose_estimations[0],
            multicam_pose_estimation);		
    }
    else if(pair_count > 0)
    {
        // Compute the average position
        const float N = static_cast<float>(pair_count);

		average_world_position.x /= N;
		average_world_position.y /= N;
		average_world_position.z /= N;

		unfiltered_average_world_position.x /= N;
		unfiltered_average_world_position.y /= N;
		unfiltered_average_world_position.z /= N;

		if (cfg.average_position_cache_enabled)
		{
			// Renew cached average world position when we have more samples
			for (int j = newPositionOffsetCaching.size() - 1; j >= 0; --j)
			{
				const orgPositionOffsetCaching orgCache = newPositionOffsetCaching[j];
				positionOffsetCaching *cache = &globalPositionOffsetCaching[orgCache.tracker_1_id][orgCache.tracker_2_id][orgCache.index];

				// Only update when theres new pairs.
				if ((cache->paired_trackers & ((1 << orgCache.tracker_1_id) | (1 << (orgCache.tracker_2_id + PSMOVESERVICE_MAX_TRACKER_COUNT)))) > 0)
					continue;
				if ((cache->paired_trackers & ((1 << orgCache.tracker_2_id) | (1 << (orgCache.tracker_1_id + PSMOVESERVICE_MAX_TRACKER_COUNT)))) > 0)
					continue;


				cache->world_avg_position.x = unfiltered_average_world_position.x;
				cache->world_avg_position.y = unfiltered_average_world_position.y;
				cache->world_avg_position.z = unfiltered_average_world_position.z;
				cache->local_position.x = cache->new_local_position.x;
				cache->local_position.y = cache->new_local_position.y;
				cache->local_position.z = cache->new_local_position.z;
				cache->paired_trackers |= (1 << orgCache.tracker_1_id) | (1 << (orgCache.tracker_2_id + PSMOVESERVICE_MAX_TRACKER_COUNT));
				cache->isValid = true;
			}
		}

		//printf("Cache Size: %d\n", globalPositionOffsetCaching[0][1].size());
		//printf("Cache Total Size: %d/%d\n", globalPositionOffsetCachingCount[0][1], (int)fmax(1.f, cfg.average_position_cache_limit));

        // Store the averaged tracking position
        multicam_pose_estimation->position_cm = average_world_position;
        multicam_pose_estimation->bCurrentlyTracking = true;
    }

    // No orientation for the sphere projection
    multicam_pose_estimation->orientation.clear();
    multicam_pose_estimation->bOrientationValid = false;

    // Compute the average projection area.
    // This is proportional to our position tracking quality.
    multicam_pose_estimation->projection.screen_area =
        screen_area_sum / static_cast<float>(projections_found);
}