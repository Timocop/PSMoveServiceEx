// -- includes -----
#include "ExternalOrientationFilter.h"
#include "MathAlignment.h"
#include "ServerLog.h"
#include <deque>
#include <map>
#include <vector>

#define k_lowpass_velocity_smoothing_factor 0.25f

#define k_adaptive_prediction_cutoff 0.25f

// -- private definitions -----
struct OrientationInterpolationState
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		/// Is the current state valid
		bool bIsValid;
	t_high_resolution_timepoint current_timestamp;

	Eigen::Quaternionf last_orientation;
	Eigen::Vector3f last_velocity;
	Eigen::Vector3f last_acceleration;
	t_high_resolution_timepoint last_timestamp;
	Eigen::Quaternionf previous_orientation;
	Eigen::Vector3f previous_velocity;
	Eigen::Vector3f previous_acceleration;
	t_high_resolution_timepoint previous_timestamp;

	void reset()
	{
		bIsValid = false;
		current_timestamp = t_high_resolution_timepoint();
		last_orientation = Eigen::Quaternionf::Identity();
		last_velocity = Eigen::Vector3f::Zero();
		last_acceleration = Eigen::Vector3f::Zero();
		last_timestamp = t_high_resolution_timepoint();
		previous_orientation = Eigen::Quaternionf::Identity();
		previous_velocity = Eigen::Vector3f::Zero();
		previous_acceleration = Eigen::Vector3f::Zero();
		previous_timestamp = t_high_resolution_timepoint();
	}

	Eigen::Quaternionf getInterpolatedOrientation(const Eigen::Quaternionf &orientation)
	{
		if (!bIsValid)
		{
			return orientation;
		}

		const t_high_resolution_duration_milli previous_time_delta = last_timestamp - previous_timestamp;
		const t_high_resolution_duration_milli last_time_delta = current_timestamp - last_timestamp;
		const float previous_time_delta_milli = previous_time_delta.count();
		const float last_time_delta_milli = last_time_delta.count();

		if ((previous_time_delta_milli / 1000.f) < k_min_time_delta_seconds || (previous_time_delta_milli / 1000.f) > k_max_time_delta_seconds)
		{
			return orientation;
		}

		float t = clampf01(last_time_delta_milli / previous_time_delta_milli);

		Eigen::Quaternionf lerped_ang = previous_orientation.slerp(t, orientation);

		return lerped_ang;
	}

	Eigen::Vector3f getInterpolatedVelocity(const Eigen::Vector3f &velocity)
	{
		if (!bIsValid)
		{
			return velocity;
		}

		const t_high_resolution_duration_milli previous_time_delta = last_timestamp - previous_timestamp;
		const t_high_resolution_duration_milli last_time_delta = current_timestamp - last_timestamp;
		const float previous_time_delta_milli = previous_time_delta.count();
		const float last_time_delta_milli = last_time_delta.count();

		if ((previous_time_delta_milli / 1000.f) < k_min_time_delta_seconds || (previous_time_delta_milli / 1000.f) > k_max_time_delta_seconds)
		{
			return velocity;
		}

		float t = clampf01(last_time_delta_milli / previous_time_delta_milli);

		Eigen::Vector3f lerped_vel = Eigen::Vector3f(
			lerpf(previous_velocity.x(), velocity.x(), t),
			lerpf(previous_velocity.y(), velocity.y(), t),
			lerpf(previous_velocity.z(), velocity.z(), t));

		return lerped_vel;
	}

	void apply_interpolation_state(
		const Eigen::Quaternionf &new_orientation,
		const t_high_resolution_timepoint timestamp)
	{
		apply_interpolation_state(new_orientation, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), timestamp);
	}

	void apply_interpolation_state(
		const Eigen::Quaternionf &new_orientation,
		const Eigen::Vector3f &new_angular_velocity,
		const Eigen::Vector3f &new_angular_acceleration,
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

		if (eigen_quaternion_is_valid(new_orientation))
		{
			previous_orientation = last_orientation;
			last_orientation = new_orientation;
		}
		else
		{
			SERVER_LOG_WARNING("OrientationFilter") << "Orientation is NaN!";
		}

		if (eigen_vector3f_is_valid(new_angular_velocity))
		{
			previous_velocity = last_velocity;
			last_velocity = new_angular_velocity;
		}
		else
		{
			SERVER_LOG_WARNING("OrientationFilter") << "Angular Velocity is NaN!";
		}

		if (eigen_vector3f_is_valid(new_angular_acceleration))
		{
			previous_acceleration = last_acceleration;
			last_acceleration = new_angular_acceleration;
		}
		else
		{
			SERVER_LOG_WARNING("OrientationFilter") << "Angular Acceleration is NaN!";
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

struct ExternalOrientationFilterState
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		/// Is the current fusion state valid
		bool bIsValid;

	/* Physics State */
	Eigen::Quaternionf orientation;
	Eigen::Vector3f angular_velocity;
	Eigen::Vector3f angular_acceleration;

	t_high_resolution_timepoint last_optical_timestamp;
	t_high_resolution_timepoint last_imu_timestamp;

	/* Quaternion measured when controller points towards camera */
	Eigen::Quaternionf reset_orientation;

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
		const t_high_resolution_duration_milli time_delta = timestamp - last_imu_timestamp;
		const float time_delta_milli = time_delta.count();

		// convert delta to seconds clamp time delta between 2500hz and 30hz
		time_delta_seconds = clampf(time_delta_milli / 1000.f, k_min_time_delta_seconds, k_max_time_delta_seconds);

		return time_delta_seconds;
	}

	void reset()
	{
		bIsValid = false;
		orientation = Eigen::Quaternionf::Identity();
		angular_velocity = Eigen::Vector3f::Zero();
		angular_acceleration = Eigen::Vector3f::Zero();
		reset_orientation = Eigen::Quaternionf::Identity();
		last_optical_timestamp = t_high_resolution_timepoint();
		last_imu_timestamp = t_high_resolution_timepoint();
	}

	void apply_imu_state(
		const Eigen::Quaternionf &new_orientation,
		const Eigen::Vector3f &new_angular_velocity,
		const Eigen::Vector3f &new_angular_acceleration,
		const t_high_resolution_timepoint timestamp)
	{
		if (eigen_quaternion_is_valid(new_orientation))
		{
			orientation = new_orientation;
		}
		else
		{
			SERVER_LOG_WARNING("OrientationFilter") << "Orientation is NaN!";
		}

		if (eigen_vector3f_is_valid(new_angular_velocity))
		{
			angular_velocity = new_angular_velocity;
		}
		else
		{
			SERVER_LOG_WARNING("OrientationFilter") << "Angular Velocity is NaN!";
		}

		if (eigen_vector3f_is_valid(new_angular_acceleration))
		{
			angular_acceleration = new_angular_acceleration;
		}
		else
		{
			SERVER_LOG_WARNING("OrientationFilter") << "Angular Acceleration is NaN!";
		}

		last_imu_timestamp = timestamp;

		// state is valid now that we have had an update
		bIsValid = true;
	}

	void apply_optical_state(
		const Eigen::Quaternionf &new_orientation,
		const t_high_resolution_timepoint timestamp)
	{
		if (eigen_quaternion_is_valid(new_orientation))
		{
			orientation = new_orientation;
		}
		else
		{
			SERVER_LOG_WARNING("OrientationFilter") << "Orientation is NaN!";
		}

		last_optical_timestamp = timestamp;

		// state is valid now that we have had an update
		bIsValid = true;
	}
};

// -- private methods -----
static Eigen::Vector3f lowpass_filter_vector3f(
	const float alpha,
	const Eigen::Vector3f &old_filtered_vector,
	const Eigen::Vector3f &new_vector);

// -- public interface -----
//-- Orientation Filter --
ExternalOrientationFilter::ExternalOrientationFilter() :
	m_state(new ExternalOrientationFilterState),
	m_interpolationState(new OrientationInterpolationState),
	orientationPipe(INVALID_HANDLE_VALUE),
	showMessage(true)
{
#ifdef WIN32
	localeInvariant = _create_locale(LC_NUMERIC, "C");
#endif

	memset(&m_constants, 0, sizeof(OrientationFilterConstants));
	resetState();
}

ExternalOrientationFilter::~ExternalOrientationFilter()
{
#ifdef WIN32
	if (localeInvariant != NULL)
	{
		_free_locale(localeInvariant);
		localeInvariant = NULL;
	}

	if (orientationPipe != INVALID_HANDLE_VALUE)
	{
		DisconnectNamedPipe(orientationPipe);
		CloseHandle(orientationPipe);

		orientationPipe = INVALID_HANDLE_VALUE;
	}
#endif
	delete m_state;
	delete m_interpolationState;
}

bool ExternalOrientationFilter::getIsStateValid() const
{
	return m_state->bIsValid;
}

double ExternalOrientationFilter::getTimeInSeconds() const
{
	return m_state->getImuTime(std::chrono::high_resolution_clock::now());
}

void ExternalOrientationFilter::resetState()
{
	m_state->reset();
	m_interpolationState->reset();
}

void ExternalOrientationFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
	Eigen::Quaternionf q_inverse = m_state->orientation.conjugate();

	eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
	m_state->reset_orientation = q_pose*q_inverse;
}

bool ExternalOrientationFilter::init(const OrientationFilterConstants &constants)
{
	resetState();
	m_constants = constants;

	return true;
}

bool ExternalOrientationFilter::init(const OrientationFilterConstants &constants, const Eigen::Quaternionf &initial_orientation)
{
	resetState();
	m_constants = constants;
	m_state->orientation = initial_orientation;
	m_state->bIsValid = true;
	m_interpolationState->last_orientation = initial_orientation;
	m_interpolationState->bIsValid = true;

	return true;
}

Eigen::Quaternionf ExternalOrientationFilter::getOrientation(float time, float offset_x, float offset_y, float offset_z, float offset_world_x, float offset_world_y, float offset_world_z) const
{
	Eigen::Quaternionf result = Eigen::Quaternionf::Identity();

	// Apply offsets to reset orientation
	const Eigen::Quaternionf local_offset_x_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_x * k_degrees_to_radians, Eigen::Vector3f::UnitX()));
	const Eigen::Quaternionf local_offset_y_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_y * k_degrees_to_radians, Eigen::Vector3f::UnitY()));
	const Eigen::Quaternionf local_offset_z_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_z * k_degrees_to_radians, Eigen::Vector3f::UnitZ()));
	const Eigen::Quaternionf local_offset_quat = local_offset_y_quat * local_offset_z_quat * local_offset_x_quat;

	const Eigen::Quaternionf world_offset_x_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_x * k_degrees_to_radians, Eigen::Vector3f::UnitX()));
	const Eigen::Quaternionf world_offset_y_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_y * k_degrees_to_radians, Eigen::Vector3f::UnitY()));
	const Eigen::Quaternionf world_offset_z_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_z * k_degrees_to_radians, Eigen::Vector3f::UnitZ()));
	const Eigen::Quaternionf world_offset_quat = world_offset_y_quat * world_offset_z_quat * world_offset_x_quat;

	if (m_state->bIsValid)
	{
		Eigen::Quaternionf orientation = m_state->orientation;
		Eigen::Vector3f ang_velocity = m_state->angular_velocity;

		if (m_interpolationState->bIsValid)
		{
			bool orientationInterpolation = true;

#if !defined(IS_TESTING_KALMAN)
			orientationInterpolation = DeviceManager::getInstance()->m_tracker_manager->getConfig().angular_interpolation;
#endif

			if (orientationInterpolation)
			{
				orientation = m_interpolationState->getInterpolatedOrientation(orientation);
				ang_velocity = m_interpolationState->getInterpolatedVelocity(ang_velocity);
			}
		}

		Eigen::Quaternionf predicted_orientation = orientation * local_offset_quat;

		if (fabsf(time) > k_real_epsilon)
		{
			const Eigen::Vector3f local_velocity = eigen_vector3f_clockwise_rotate(local_offset_quat, ang_velocity);

			const Eigen::Quaternionf &quaternion_derivative =
				eigen_angular_velocity_to_quaternion_derivative(predicted_orientation, local_velocity);

			predicted_orientation = Eigen::Quaternionf(
				predicted_orientation.coeffs()
				+ quaternion_derivative.coeffs()*time).normalized();
		}

		result = (world_offset_quat * m_state->reset_orientation) * predicted_orientation;
	}

	return result;
}

Eigen::Quaternionf ExternalOrientationFilter::getResetOrientation() const
{
	return m_state->reset_orientation;
}

Eigen::Vector3f ExternalOrientationFilter::getAngularVelocityRadPerSec(float offset_x, float offset_y, float offset_z) const
{
	if (!m_state->bIsValid)
		return Eigen::Vector3f::Zero();

	// Rotate by Axis
	const Eigen::Quaternionf local_offset_x_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_x * k_degrees_to_radians, Eigen::Vector3f::UnitX()));
	const Eigen::Quaternionf local_offset_y_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_y * k_degrees_to_radians, Eigen::Vector3f::UnitY()));
	const Eigen::Quaternionf local_offset_z_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_z * k_degrees_to_radians, Eigen::Vector3f::UnitZ()));
	const Eigen::Quaternionf local_offset_quat = local_offset_y_quat * local_offset_z_quat * local_offset_x_quat;

	const Eigen::Vector3f local_velocity = eigen_vector3f_clockwise_rotate(local_offset_quat, m_state->angular_velocity);

	return local_velocity;
}

Eigen::Vector3f ExternalOrientationFilter::getAngularAccelerationRadPerSecSqr(float offset_x, float offset_y, float offset_z) const
{
	if (!m_state->bIsValid)
		return Eigen::Vector3f::Zero();

	// Rotate by Axis
	const Eigen::Quaternionf local_offset_x_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_x * k_degrees_to_radians, Eigen::Vector3f::UnitX()));
	const Eigen::Quaternionf local_offset_y_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_y * k_degrees_to_radians, Eigen::Vector3f::UnitY()));
	const Eigen::Quaternionf local_offset_z_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_z * k_degrees_to_radians, Eigen::Vector3f::UnitZ()));
	const Eigen::Quaternionf local_offset_quat = local_offset_y_quat * local_offset_z_quat * local_offset_x_quat;

	const Eigen::Vector3f local_acceleration = eigen_vector3f_clockwise_rotate(local_offset_quat, m_state->angular_acceleration);

	return local_acceleration;
}

// -- OrientationFilterExternal --
void OrientationFilterExternal::update(
	const t_high_resolution_timepoint timestamp,
	const PoseFilterPacket &packet)
{
#ifdef WIN32
	float optical_delta_time = m_state->getOpticalTime(timestamp);
	float imu_delta_time = m_state->getImuTime(timestamp);

	m_interpolationState->apply_timestamp_state(timestamp);

	if (packet.controllerDeviceId < 0)
		return;

	std::string pipeName = "\\\\.\\pipe\\PSMoveSerivceEx\\VirtPSmoveStream_";

	char indexStr[20];
	pipeName.append(itoa(packet.controllerDeviceId, indexStr, 10));

	if (orientationPipe == INVALID_HANDLE_VALUE)
	{
		orientationPipe = CreateNamedPipe(
			pipeName.c_str(),
			PIPE_ACCESS_INBOUND,
			PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_NOWAIT,
			1,
			sizeof(pipeBuffer),
			sizeof(pipeBuffer),
			NMPWAIT_USE_DEFAULT_WAIT,
			NULL
		);

		if (orientationPipe != INVALID_HANDLE_VALUE)
		{
			if(showMessage)
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


	if (orientationPipe == INVALID_HANDLE_VALUE)
		return;

	BOOL connected = ConnectNamedPipe(orientationPipe, NULL);
	if (connected)
		SERVER_LOG_INFO("OrientationFilterExternal::update") << "ConnectNamedPipe success, index " << packet.controllerDeviceId;

	if (!connected)
		connected = (GetLastError() == ERROR_PIPE_CONNECTED);

	if (!connected)
	{
		if (GetLastError() != ERROR_PIPE_LISTENING) {
			if(showMessage)
				SERVER_LOG_ERROR("OrientationFilterExternal::update") << "ConnectNamedPipe failed, index " << packet.controllerDeviceId << ", GLE=" << GetLastError() << ".";

			showMessage = false;
			DisconnectNamedPipe(orientationPipe);
		}
	}

	if (!connected)
		return;

	DWORD dwRead;
	BOOL success = ReadFile(orientationPipe, pipeBuffer, 128, &dwRead, NULL);
	if (!success)
	{
		switch (GetLastError())
		{
			case ERROR_BROKEN_PIPE:
			{
				if(showMessage)
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
				if(showMessage)
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
	
	while(*p) {
		vector.push_back(std::string(p));
		p += vector.back().size() + 1;
	}

	int __size = 0;
	bool isValid = false;

	Eigen::Quaternionf new_orientation = Eigen::Quaternionf::Identity();
	Eigen::Vector3f new_angular_velocity = Eigen::Vector3f::Zero();

	// Get orientation
	__size += 4;
	if (vector.size() >= __size)
	{
		new_orientation.x() = (float)_atof_l(vector[__size - 4].c_str(), localeInvariant);
		new_orientation.y() = (float)_atof_l(vector[__size - 3].c_str(), localeInvariant);
		new_orientation.z() = (float)_atof_l(vector[__size - 2].c_str(), localeInvariant);
		new_orientation.w() = (float)_atof_l(vector[__size - 1].c_str(), localeInvariant);

		isValid = true;
	}

	// Get reset orientation
	__size += 4;
	if (vector.size() >= __size)
	{
		Eigen::Quaternionf reset_orientation;
		reset_orientation.x() = (float)_atof_l(vector[__size - 4].c_str(), localeInvariant);
		reset_orientation.y() = (float)_atof_l(vector[__size - 3].c_str(), localeInvariant);
		reset_orientation.z() = (float)_atof_l(vector[__size - 2].c_str(), localeInvariant);
		reset_orientation.w() = (float)_atof_l(vector[__size - 1].c_str(), localeInvariant);

		if (eigen_quaternion_is_valid(reset_orientation))
		{
			Eigen::Quaternionf q_inverse = reset_orientation.conjugate();

			eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
			m_state->reset_orientation = q_inverse;
		}
	}

	if (isValid && eigen_quaternion_is_valid(new_orientation))
	{
		float velocity_smoothing_factor = k_lowpass_velocity_smoothing_factor;
		float angular_prediction_cutoff = k_adaptive_prediction_cutoff;

#if !defined(IS_TESTING_KALMAN) 
		if (packet.controllerDeviceId > -1)
		{
			ServerControllerViewPtr ControllerView = DeviceManager::getInstance()->getControllerViewPtr(packet.controllerDeviceId);
			if (ControllerView != nullptr && ControllerView->getIsOpen())
			{
				switch (ControllerView->getControllerDeviceType())
				{
				case CommonDeviceState::PSMove:
				{
					PSMoveController *controller = ControllerView->castChecked<PSMoveController>();
					PSMoveControllerConfig config = *controller->getConfig();

					velocity_smoothing_factor = config.filter_angular_smoothing_factor;
					angular_prediction_cutoff = config.filter_angular_prediction_cutoff;

					break;
				}
				case CommonDeviceState::VirtualController:
				{
					VirtualController *controller = ControllerView->castChecked<VirtualController>();
					VirtualControllerConfig config = *controller->getConfig();

					velocity_smoothing_factor = config.filter_angular_smoothing_factor;
					angular_prediction_cutoff = config.filter_angular_prediction_cutoff;

					break;
				}
				}
			}
		}
#endif

		// Clamp everything to safety
		velocity_smoothing_factor = clampf(velocity_smoothing_factor, 0.01f, 1.0f);
		angular_prediction_cutoff = clampf(angular_prediction_cutoff, 0.0f, (1 << 16));

		Eigen::Quaternionf deltaRotation = m_state->orientation.conjugate() * new_orientation;
		Eigen::AngleAxisf axisAngle(deltaRotation);

		Eigen::Vector3f angularVelocity = axisAngle.axis() * (axisAngle.angle() / imu_delta_time);

		new_angular_velocity = angularVelocity.transpose();

		// Smooth angular velocity
		new_angular_velocity = lowpass_filter_vector3f(velocity_smoothing_factor, m_state->angular_velocity, new_angular_velocity);

		// Do adapting prediction and smoothing
		if (angular_prediction_cutoff > k_real_epsilon)
		{
			const float velocity_speed_sec = new_angular_velocity.norm();
			const float adaptive_time_scale = clampf(velocity_speed_sec / angular_prediction_cutoff, 0.0f, 1.0f);
			
			new_angular_velocity = new_angular_velocity * adaptive_time_scale;
		}

		const Eigen::Vector3f new_angular_acceleration = (new_angular_velocity - m_state->angular_velocity) / imu_delta_time;

		m_state->apply_imu_state(new_orientation, new_angular_velocity, new_angular_acceleration, timestamp);
		m_interpolationState->apply_interpolation_state(new_orientation, new_angular_velocity, new_angular_acceleration, timestamp);
	}
#endif
}

static Eigen::Vector3f lowpass_filter_vector3f(
	const float alpha,
	const Eigen::Vector3f &old_filtered_vector,
	const Eigen::Vector3f &new_vector)
{
	const Eigen::Vector3f filtered_vector = alpha*new_vector + (1.f - alpha)*old_filtered_vector;

	return filtered_vector;
}
