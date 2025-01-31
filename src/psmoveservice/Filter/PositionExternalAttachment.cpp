// -- includes -----
#include "PositionExternalAttachment.h"
#include "DeviceManager.h"
#include "ControllerManager.h"
#include "ServerControllerView.h"
#include "MathAlignment.h"
#include "MathEigen.h"
#include "ServerLog.h"
#include "PSMoveController.h"
#include "VirtualController.h"

#include <chrono>
#include <numeric>

//-- constants -----

// -- private definitions -----

// -- private definitions -----
struct PositionInterpolationState
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		/// Is the current state valid
		bool bIsValid;
	t_high_resolution_timepoint current_timestamp;

	Eigen::Vector3f last_position_meters; // meters
	Eigen::Vector3f last_velocity_meters; // meters
	t_high_resolution_timepoint last_timestamp;
	Eigen::Vector3f previous_position_meters; // meters
	Eigen::Vector3f previous_velocity_meters; // meters
	t_high_resolution_timepoint previous_timestamp;

	void reset()
	{
		bIsValid = false;
		current_timestamp = t_high_resolution_timepoint();
		last_position_meters = Eigen::Vector3f::Zero();
		last_velocity_meters = Eigen::Vector3f::Zero();
		last_timestamp = t_high_resolution_timepoint();
		previous_position_meters = Eigen::Vector3f::Zero();
		previous_velocity_meters = Eigen::Vector3f::Zero();
		previous_timestamp = t_high_resolution_timepoint();
	}

	Eigen::Vector3f getInterpolatedPosition(const Eigen::Vector3f &position_meters)
	{
		if (!bIsValid)
		{
			return position_meters;
		}

		const t_high_resolution_duration_milli previous_time_delta = last_timestamp - previous_timestamp;
		const t_high_resolution_duration_milli last_time_delta = current_timestamp - last_timestamp;
		const float previous_time_delta_milli = previous_time_delta.count();
		const float last_time_delta_milli = last_time_delta.count();

		if ((previous_time_delta_milli / 1000.f) < k_min_time_delta_seconds || (previous_time_delta_milli / 1000.f) > k_max_time_delta_seconds)
		{
			return position_meters;
		}

		float t = clampf01(last_time_delta_milli / previous_time_delta_milli);

		Eigen::Vector3f lerped_pos = Eigen::Vector3f(
			lerpf(previous_position_meters.x(), position_meters.x(), t),
			lerpf(previous_position_meters.y(), position_meters.y(), t),
			lerpf(previous_position_meters.z(), position_meters.z(), t));

		return lerped_pos;
	}

	Eigen::Vector3f getInterpolatedVelocity(const Eigen::Vector3f &velocity_meters)
	{
		if (!bIsValid)
		{
			return velocity_meters;
		}

		const t_high_resolution_duration_milli previous_time_delta = last_timestamp - previous_timestamp;
		const t_high_resolution_duration_milli last_time_delta = current_timestamp - last_timestamp;
		const float previous_time_delta_milli = previous_time_delta.count();
		const float last_time_delta_milli = last_time_delta.count();

		if ((previous_time_delta_milli / 1000.f) < k_min_time_delta_seconds || (previous_time_delta_milli / 1000.f) > k_max_time_delta_seconds)
		{
			return velocity_meters;
		}

		float t = clampf01(last_time_delta_milli / previous_time_delta_milli);

		Eigen::Vector3f lerped_vel = Eigen::Vector3f(
			lerpf(previous_velocity_meters.x(), velocity_meters.x(), t),
			lerpf(previous_velocity_meters.y(), velocity_meters.y(), t),
			lerpf(previous_velocity_meters.z(), velocity_meters.z(), t));

		return lerped_vel;
	}

	void apply_interpolation_state(
		const Eigen::Vector3f &new_position_m,
		const t_high_resolution_timepoint timestamp)
	{
		apply_interpolation_state(new_position_m, Eigen::Vector3f::Zero(), timestamp);
	}

	void apply_interpolation_state(
		const Eigen::Vector3f &new_position_m,
		const Eigen::Vector3f &new_velocity_m_per_sec,
		const t_high_resolution_timepoint timestamp)
	{
		if (bIsValid)
		{
			previous_timestamp = last_timestamp;
		}
		else
		{
			previous_timestamp = timestamp;
		}

		if (eigen_vector3f_is_valid(new_position_m))
		{
			previous_position_meters = last_position_meters;
			last_position_meters = new_position_m;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "Position is NaN!";
		}

		if (eigen_vector3f_is_valid(new_velocity_m_per_sec))
		{
			previous_velocity_meters = last_velocity_meters;
			last_velocity_meters = new_velocity_m_per_sec;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "Velocity is NaN!";
		}

		last_timestamp = timestamp;

		// state is valid now that we have had an update
		bIsValid = true;
	}

	void apply_timestamp_state(const t_high_resolution_timepoint timestamp)
	{
		current_timestamp = timestamp;
	}
};

struct ExternalAttachmentPositionFilterState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Is the current state valid
    bool bIsValid;

    /// Current Sensor State of the filter
    Eigen::Vector3f accelerometer_g_units; // (g-unit)
    Eigen::Vector3f accelerometer_derivative_g_per_sec; // rate of change of the accelerometer in g-unit/s

    /// Current Physics State of the filter
    Eigen::Vector3f position_meters; // meters
    Eigen::Vector3f velocity_m_per_sec; // meters/s
    Eigen::Vector3f acceleration_m_per_sec_sqr; // accelerometer minus gravity in meters/s^2

    /// Position that's considered the origin position 
    Eigen::Vector3f origin_position; // meters

	t_high_resolution_timepoint last_optical_timestamp;
	t_high_resolution_timepoint last_imu_timestamp;

	float getOpticalTime(const t_high_resolution_timepoint timestamp)
	{
		// Compute the time since the last packet
		float time_delta_seconds;
		const t_high_resolution_duration_milli time_delta = timestamp - last_optical_timestamp;
		const float time_delta_milli = time_delta.count();

		// convert delta to seconds clamp time delta between 2500hz and 30hz
		time_delta_seconds = clampf(time_delta_milli / 1000.f, k_min_time_delta_seconds, k_max_time_delta_seconds);

		return time_delta_seconds;
	}

	float getImuTime(const t_high_resolution_timepoint timestamp)
	{
		// Compute the time since the last packet
		float time_delta_seconds;
		const t_high_resolution_duration_milli time_delta = timestamp  - last_imu_timestamp;
		const float time_delta_milli = time_delta.count();

		// convert delta to seconds clamp time delta between 2500hz and 30hz
		time_delta_seconds = clampf(time_delta_milli / 1000.f, k_min_time_delta_seconds, k_max_time_delta_seconds);

		return time_delta_seconds;
	}

    void reset()
    {
        bIsValid = false;
        position_meters = Eigen::Vector3f::Zero();
        velocity_m_per_sec = Eigen::Vector3f::Zero();
        acceleration_m_per_sec_sqr = Eigen::Vector3f::Zero();
        accelerometer_g_units = Eigen::Vector3f::Zero();
        accelerometer_derivative_g_per_sec = Eigen::Vector3f::Zero();
        origin_position = Eigen::Vector3f::Zero();
		last_optical_timestamp = t_high_resolution_timepoint();
		last_imu_timestamp = t_high_resolution_timepoint();
    }

	void apply_imu_state(
		const Eigen::Vector3f &new_position_m,
		const Eigen::Vector3f &new_velocity_m_per_sec,
		const Eigen::Vector3f &new_acceleration_m_per_sec_sqr,
		const Eigen::Vector3f &new_accelerometer_g_units,
		const Eigen::Vector3f &new_accelerometer_derivative_g_per_sec,
		const t_high_resolution_timepoint timestamp)
	{
		if (eigen_vector3f_is_valid(new_position_m))
		{
			position_meters = new_position_m;
		}
		else
		{
			SERVER_LOG_WARNING("PositionExternalAttachmentFilter") << "Position is NaN!";
		}

		if (eigen_vector3f_is_valid(new_velocity_m_per_sec))
		{
			velocity_m_per_sec = new_velocity_m_per_sec;
		}
		else
		{
			SERVER_LOG_WARNING("PositionExternalAttachmentFilter") << "Velocity is NaN!";
		}

		if (eigen_vector3f_is_valid(new_acceleration_m_per_sec_sqr))
		{
			acceleration_m_per_sec_sqr = new_acceleration_m_per_sec_sqr;
		}
		else
		{
			SERVER_LOG_WARNING("PositionExternalAttachmentFilter") << "Acceleration is NaN!";
		}

		if (eigen_vector3f_is_valid(new_accelerometer_g_units))
		{
			accelerometer_g_units = new_accelerometer_g_units;
		}
		else
		{
			SERVER_LOG_WARNING("PositionExternalAttachmentFilter") << "Accelerometer is NaN!";
		}

		if (eigen_vector3f_is_valid(new_accelerometer_derivative_g_per_sec))
		{
			accelerometer_derivative_g_per_sec = new_accelerometer_derivative_g_per_sec;
		}
		else
		{
			SERVER_LOG_WARNING("PositionExternalAttachmentFilter") << "AccelerometerDerivative is NaN!";
		}

		last_imu_timestamp = timestamp;

        // state is valid now that we have had an update
        bIsValid= true;
	}

	void apply_optical_state(
		const Eigen::Vector3f &new_position_m,
		const t_high_resolution_timepoint timestamp)
	{
		apply_optical_state(new_position_m, Eigen::Vector3f::Zero(), timestamp);
	}

	void apply_optical_state(
		const Eigen::Vector3f &new_position_m,
		const Eigen::Vector3f &new_velocity_m_per_sec,
		const t_high_resolution_timepoint timestamp)
	{
		if (eigen_vector3f_is_valid(new_position_m))
		{
			position_meters = new_position_m;
		}
		else
		{
			SERVER_LOG_WARNING("PositionExternalAttachmentFilter") << "Position is NaN!";
		}

		if (eigen_vector3f_is_valid(new_velocity_m_per_sec))
		{
			velocity_m_per_sec = new_velocity_m_per_sec;
		}
		else
		{
			SERVER_LOG_WARNING("PositionExternalAttachmentFilter") << "Velocity is NaN!";
		}

		last_optical_timestamp = timestamp;

        // state is valid now that we have had an update
        bIsValid= true;
	}
};

// -- private methods -----
static void rotate_vector_by_quaternion(const Eigen::Vector3f &v, const Eigen::Quaternionf &q, Eigen::Vector3f &vprime);
static Eigen::Vector3f threshold_vector3f(const Eigen::Vector3f &vector, const float min_length);
static Eigen::Vector3f clamp_vector3f(const Eigen::Vector3f &vector, const float max_length);
static Eigen::Vector3f lowpass_filter_vector3f(
    const float alpha,
    const Eigen::Vector3f &old_filtered_vector,
    const Eigen::Vector3f &new_vector);
static Eigen::Vector3f lowpass_filter_vector3f(
    const float delta_time,
    const float cutoff_frequency,
    const Eigen::Vector3f &old_filtered_vector,
    const Eigen::Vector3f &new_vector);
static Eigen::Vector3f lowpass_prediction_vector3f(
	const float delta_time,
	const Eigen::Vector3f &old_filtered_vector,
	const Eigen::Vector3f &new_vector);

// -- public interface -----
//-- Orientation Filter -----
PositionExternalAttachmentFilter::PositionExternalAttachmentFilter() :
	m_state(new ExternalAttachmentPositionFilterState),
	m_interpolationState(new PositionInterpolationState),
	attachmentPipe(INVALID_HANDLE_VALUE)
{
#ifdef WIN32
	localeInvariant = _create_locale(LC_NUMERIC, "C");
#endif

    memset(&m_constants, 0, sizeof(PositionFilterConstants));
    resetState();
}

PositionExternalAttachmentFilter::~PositionExternalAttachmentFilter()
{
#ifdef WIN32
	if (localeInvariant != NULL)
	{
		_free_locale(localeInvariant);
		localeInvariant = NULL;
	}

	if (attachmentPipe != INVALID_HANDLE_VALUE)
	{
		DisconnectNamedPipe(attachmentPipe);
		CloseHandle(attachmentPipe);

		attachmentPipe = INVALID_HANDLE_VALUE;
	}
#endif
    delete m_state;
	delete m_interpolationState;
}

bool PositionExternalAttachmentFilter::getIsStateValid() const
{
    return m_state->bIsValid;
}

double PositionExternalAttachmentFilter::getTimeInSeconds() const
{
    return m_state->getOpticalTime(std::chrono::high_resolution_clock::now());
}

void PositionExternalAttachmentFilter::resetState()
{
    m_state->reset();
}

void PositionExternalAttachmentFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
}

bool PositionExternalAttachmentFilter::init(const PositionFilterConstants &constants)
{
    resetState();
    m_constants= constants;

    return true;
}

bool PositionExternalAttachmentFilter::init(const PositionFilterConstants &constants, const Eigen::Vector3f &initial_position)
{
	resetState();
	m_constants = constants;
	m_state->position_meters = initial_position;
	m_state->bIsValid = true;
	m_interpolationState->last_position_meters = initial_position;
	m_interpolationState->bIsValid = true;

	return true;
}

Eigen::Vector3f PositionExternalAttachmentFilter::getPositionCm(float time) const
{
	Eigen::Vector3f result = Eigen::Vector3f::Zero();

	if (m_state->bIsValid)
	{
		Eigen::Vector3f position_meters = m_state->position_meters;
		Eigen::Vector3f velocity_meters = m_state->velocity_m_per_sec;

		if (m_interpolationState->bIsValid)
		{
			bool positionInterpolation = true;

#if !defined(IS_TESTING_KALMAN)
			positionInterpolation = DeviceManager::getInstance()->m_tracker_manager->getConfig().position_interpolation;
#endif

			if (positionInterpolation)
			{
				position_meters = m_interpolationState->getInterpolatedPosition(position_meters);
				velocity_meters = m_interpolationState->getInterpolatedVelocity(velocity_meters);
			}
		}

		Eigen::Vector3f predicted_position =
			is_nearly_zero(time)
			? position_meters
			: position_meters + velocity_meters * time;

		result = predicted_position - m_state->origin_position;
	}

	return result * k_meters_to_centimeters;
}

Eigen::Vector3f PositionExternalAttachmentFilter::getVelocityCmPerSec() const
{
    Eigen::Vector3f result= m_state->velocity_m_per_sec * k_meters_to_centimeters;

    return (m_state->bIsValid) ? result : Eigen::Vector3f::Zero();
}

Eigen::Vector3f PositionExternalAttachmentFilter::getAccelerationCmPerSecSqr() const
{
    Eigen::Vector3f result= m_state->acceleration_m_per_sec_sqr * k_meters_to_centimeters;

    return (m_state->bIsValid) ? result : Eigen::Vector3f::Zero();
}

// -- Position Filters ----
// -- PositionFilterPassThru --
void PositionFilterExternalAttachment::update(
	const t_high_resolution_timepoint timestamp,
	const PoseFilterPacket &packet)
{
#ifdef WIN32
	if (packet.controllerDeviceId < 0)
	{
		return;
	}

	ControllerManager *controllerManager = DeviceManager::getInstance()->m_controller_manager;
	ServerControllerViewPtr current_controller_view = controllerManager->getControllerViewPtr(packet.controllerDeviceId);
	if (!current_controller_view ||
		!current_controller_view->getIsOpen())
	{
		return;
	}

	m_interpolationState->apply_timestamp_state(timestamp);

	std::string pipeName = "\\\\.\\pipe\\PSMoveSerivceEx\\AttachPSmoveStream_";

	char indexStr[20];
	pipeName.append(itoa(packet.controllerDeviceId, indexStr, 10));

	if (attachmentPipe == INVALID_HANDLE_VALUE)
	{
		attachmentPipe = CreateNamedPipe(
			pipeName.c_str(),
			PIPE_ACCESS_INBOUND,
			PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_NOWAIT,
			1,
			sizeof(pipeBuffer),
			sizeof(pipeBuffer),
			NMPWAIT_USE_DEFAULT_WAIT,
			NULL
		);

		if (attachmentPipe != INVALID_HANDLE_VALUE)
		{
			if (showMessage)
				SERVER_LOG_INFO("OrientationFilterExternal::update") << pipeName.c_str() << " pipe created.";

			showMessage = false;
			return;
		}
		else
		{
			if (showMessage)
				SERVER_LOG_ERROR("OrientationFilterExternal::update") << pipeName.c_str() << " pipe failed!, GLE=" << GetLastError();

			showMessage = false;
			return;
		}
	}


	if (attachmentPipe == INVALID_HANDLE_VALUE)
		return;

	BOOL connected = ConnectNamedPipe(attachmentPipe, NULL);
	if (connected)
		SERVER_LOG_INFO("OrientationFilterExternal::update") << "ConnectNamedPipe success, index " << packet.controllerDeviceId;

	if (!connected)
		connected = (GetLastError() == ERROR_PIPE_CONNECTED);

	if (!connected)
	{
		if (GetLastError() != ERROR_PIPE_LISTENING) {
			if (showMessage)
				SERVER_LOG_ERROR("OrientationFilterExternal::update") << "ConnectNamedPipe failed, index " << packet.controllerDeviceId << ", GLE=" << GetLastError() << ".";

			showMessage = false;
			DisconnectNamedPipe(attachmentPipe);
		}
	}

	if (!connected)
		return;

	DWORD dwRead;
	BOOL success = ReadFile(attachmentPipe, pipeBuffer, sizeof(pipeBuffer), &dwRead, NULL);
	if (!success)
	{
		switch (GetLastError())
		{
		case ERROR_BROKEN_PIPE:
		{
			if (showMessage)
				SERVER_LOG_ERROR("OrientationFilterExternal::update") << "Client disconnected, index " << packet.controllerDeviceId << ".";

			showMessage = false;
			break;
		}
		case ERROR_PIPE_LISTENING:
		{
			break;
		}
		case ERROR_NO_DATA:
		{
			break;
		}
		default:
		{
			if (showMessage)
				SERVER_LOG_ERROR("OrientationFilterExternal::update") << "ReadFile failed, index " << packet.controllerDeviceId << ", GLE=" << GetLastError() << ".";

			showMessage = false;
			break;
		}
		}

		return;
	}

	showMessage = true;

	const char* p = pipeBuffer;
	std::vector<std::string> vector;

	while (*p) {
		vector.push_back(std::string(p));
		p += vector.back().size() + 1;
	}

	if (vector.size() != 15)
	{
		return;
	}

	int targetId = (int)atof(vector[0].c_str());
	if (targetId < 0 || targetId >= controllerManager->getMaxDevices())
	{
		return;
	}

	// Disallow self-parenting.
	if (targetId == packet.controllerDeviceId)
	{
		return;
	}

	Eigen::Vector3f joint_offset;
	joint_offset[0] = (float)_atof_l(vector[1].c_str(), localeInvariant);
	joint_offset[1] = (float)_atof_l(vector[2].c_str(), localeInvariant);
	joint_offset[2] = (float)_atof_l(vector[3].c_str(), localeInvariant);

	Eigen::Vector3f tracker_offset;
	tracker_offset[0] = (float)_atof_l(vector[4].c_str(), localeInvariant);
	tracker_offset[1] = (float)_atof_l(vector[5].c_str(), localeInvariant);
	tracker_offset[2] = (float)_atof_l(vector[6].c_str(), localeInvariant);

	Eigen::Quaternionf joint_yaw_offset;
	joint_yaw_offset.x() = (float)_atof_l(vector[7].c_str(), localeInvariant);
	joint_yaw_offset.y() = (float)_atof_l(vector[8].c_str(), localeInvariant);
	joint_yaw_offset.z() = (float)_atof_l(vector[9].c_str(), localeInvariant);
	joint_yaw_offset.w() = (float)_atof_l(vector[10].c_str(), localeInvariant);

	Eigen::Quaternionf tracker_yaw_offset;
	tracker_yaw_offset.x() = (float)_atof_l(vector[11].c_str(), localeInvariant);
	tracker_yaw_offset.y() = (float)_atof_l(vector[12].c_str(), localeInvariant);
	tracker_yaw_offset.z() = (float)_atof_l(vector[13].c_str(), localeInvariant);
	tracker_yaw_offset.w() = (float)_atof_l(vector[14].c_str(), localeInvariant);

	if (eigen_vector3f_is_valid(joint_offset) && 
		eigen_vector3f_is_valid(tracker_offset) && 
		eigen_quaternion_is_valid(joint_yaw_offset) &&
		eigen_quaternion_is_valid(tracker_yaw_offset))
	{
		ServerControllerViewPtr parent_controller_view = controllerManager->getControllerViewPtr(targetId);

		if (!parent_controller_view ||
			!parent_controller_view->getIsOpen())
		{
			return;
		}

		IControllerInterface *target_controller = parent_controller_view->castChecked<IControllerInterface>();
		switch (target_controller->getDeviceType())
		{
		case CommonDeviceState::eDeviceType::PSMove:
		{
			PSMoveController *current_psmove = parent_controller_view->castChecked<PSMoveController>();
			const PSMoveControllerConfig &cfg = *current_psmove->getConfig();
			const CommonDevicePosition offset_world_orientation = cfg.offset_world_orientation;

			const Eigen::Quaternionf offset_x_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.x * k_degrees_to_radians, Eigen::Vector3f::UnitX()));
			const Eigen::Quaternionf offset_y_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.y * k_degrees_to_radians, Eigen::Vector3f::UnitY()));
			const Eigen::Quaternionf offset_z_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.z * k_degrees_to_radians, Eigen::Vector3f::UnitZ()));
			const Eigen::Quaternionf offset_quat = offset_y_quat * offset_x_quat * offset_z_quat;

			joint_yaw_offset = offset_quat * joint_yaw_offset;
			break;
		}
		case CommonDeviceState::eDeviceType::VirtualController:
		{
			VirtualController *current_virt = parent_controller_view->castChecked<VirtualController>();
			const VirtualControllerConfig &cfg = *current_virt->getConfig();
			const CommonDevicePosition offset_world_orientation = cfg.offset_world_orientation;

			const Eigen::Quaternionf offset_x_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.x * k_degrees_to_radians, Eigen::Vector3f::UnitX()));
			const Eigen::Quaternionf offset_y_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.y * k_degrees_to_radians, Eigen::Vector3f::UnitY()));
			const Eigen::Quaternionf offset_z_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.z * k_degrees_to_radians, Eigen::Vector3f::UnitZ()));
			const Eigen::Quaternionf offset_quat = offset_y_quat * offset_x_quat * offset_z_quat;

			joint_yaw_offset = offset_quat * joint_yaw_offset;
			break;
		}
		case CommonDeviceState::eDeviceType::PSDualShock4:
		{
			break;
		}
		}

		IControllerInterface *current_controller = current_controller_view->castChecked<IControllerInterface>();
		switch (current_controller->getDeviceType())
		{
		case CommonDeviceState::eDeviceType::PSMove:
		{
			PSMoveController *current_psmove = current_controller_view->castChecked<PSMoveController>();
			const PSMoveControllerConfig &cfg = *current_psmove->getConfig();
			const CommonDevicePosition offset_world_orientation = cfg.offset_world_orientation;

			const Eigen::Quaternionf offset_x_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.x * k_degrees_to_radians, Eigen::Vector3f::UnitX()));
			const Eigen::Quaternionf offset_y_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.y * k_degrees_to_radians, Eigen::Vector3f::UnitY()));
			const Eigen::Quaternionf offset_z_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.z * k_degrees_to_radians, Eigen::Vector3f::UnitZ()));
			const Eigen::Quaternionf offset_quat = offset_y_quat * offset_x_quat * offset_z_quat;

			tracker_yaw_offset = offset_quat * tracker_yaw_offset;
			break;
		}
		case CommonDeviceState::eDeviceType::VirtualController:
		{
			VirtualController *current_virt = current_controller_view->castChecked<VirtualController>();
			const VirtualControllerConfig &cfg = *current_virt->getConfig();
			const CommonDevicePosition offset_world_orientation = cfg.offset_world_orientation;

			const Eigen::Quaternionf offset_x_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.x * k_degrees_to_radians, Eigen::Vector3f::UnitX()));
			const Eigen::Quaternionf offset_y_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.y * k_degrees_to_radians, Eigen::Vector3f::UnitY()));
			const Eigen::Quaternionf offset_z_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_orientation.z * k_degrees_to_radians, Eigen::Vector3f::UnitZ()));
			const Eigen::Quaternionf offset_quat = offset_y_quat * offset_x_quat * offset_z_quat;

			tracker_yaw_offset = offset_quat * tracker_yaw_offset;
			break;
		}
		case CommonDeviceState::eDeviceType::PSDualShock4:
		{
			break;
		}
		}

		Eigen::Vector3f joint_position = parent_controller_view->getPoseFilter()->getPositionCm(current_controller->getPredictionTime());
		Eigen::Quaternionf joint_orientation = parent_controller_view->getPoseFilter()->getOrientation(target_controller->getOrientationPredictionTime());

		Eigen::Vector3f prime = Eigen::Vector3f::Zero();

		rotate_vector_by_quaternion(joint_offset, joint_orientation, prime);
		rotate_vector_by_quaternion(prime, joint_yaw_offset, prime);

		Eigen::Vector3f tracker_position = joint_position + prime;
		Eigen::Quaternionf tracker_orientation = current_controller_view->getPoseFilter()->getOrientation(current_controller->getOrientationPredictionTime());

		rotate_vector_by_quaternion(tracker_offset, tracker_orientation, prime);
		rotate_vector_by_quaternion(prime, tracker_yaw_offset, prime);

		Eigen::Vector3f new_position_meters = (tracker_position + prime) * k_centimeters_to_meters;

		m_state->apply_optical_state(new_position_meters, timestamp);
		m_interpolationState->apply_interpolation_state(new_position_meters, timestamp);
	}

#endif
}


static void rotate_vector_by_quaternion(const Eigen::Vector3f &v, const Eigen::Quaternionf &q, Eigen::Vector3f &vprime)
{
	// Extract the vector part of the quaternion
	Eigen::Vector3f u(q.x(), q.y(), q.z());
	
	// Extract the scalar part of the quaternion
	float s = q.w();
	
	// Do the math
	vprime = 2.0f * u.dot(v) * u
		+ (s*s - u.dot(u)) * v
		+ 2.0f * s * u.cross(v);
}

//-- helper functions ---
static Eigen::Vector3f threshold_vector3f(const Eigen::Vector3f &vector, const float min_length)
{
    const float length= vector.norm();
    const Eigen::Vector3f result= (length > min_length) ? vector : Eigen::Vector3f::Zero();

    return result;
}

static Eigen::Vector3f clamp_vector3f(const Eigen::Vector3f &vector, const float max_length)
{
    const float length= vector.norm();
    const Eigen::Vector3f result= 
        (length > max_length && length > k_real_epsilon) ? vector * (max_length / length) : vector;

    return result;
}

static Eigen::Vector3f lowpass_filter_vector3f(
    const float alpha,
    const Eigen::Vector3f &old_filtered_vector,
    const Eigen::Vector3f &new_vector)
{
    const Eigen::Vector3f filtered_vector= alpha*new_vector + (1.f - alpha)*old_filtered_vector;

    return filtered_vector;
}

static Eigen::Vector3f lowpass_filter_vector3f(
    const float delta_time,
    const float cutoff_frequency,
    const Eigen::Vector3f &old_filtered_vector,
    const Eigen::Vector3f &new_vector)
{
    // https://en.wikipedia.org/wiki/Low-pass_filter
    const float RC= 1.f/(k_real_two_pi*cutoff_frequency);
    const float alpha= clampf01(delta_time / (RC + delta_time));
    const Eigen::Vector3f filtered_vector= lowpass_filter_vector3f(alpha, old_filtered_vector, new_vector);

    return filtered_vector;
}
