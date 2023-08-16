// -- includes -----
#include "ExternalOrientationFilter.h"
#include "MathAlignment.h"
#include "ServerLog.h"
#include <deque>
#include <map>
#include <vector>

// -- private definitions -----
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
		const t_high_resolution_timepoint timestamp,
		const bool isTemporary)
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

		if(!isTemporary)
			last_imu_timestamp = timestamp;

		// state is valid now that we have had an update
		bIsValid = true;
	}

	void apply_optical_state(
		const Eigen::Quaternionf &new_orientation,
		const t_high_resolution_timepoint timestamp,
		const bool isTemporary)
	{
		if (eigen_quaternion_is_valid(new_orientation))
		{
			orientation = new_orientation;
		}
		else
		{
			SERVER_LOG_WARNING("OrientationFilter") << "Orientation is NaN!";
		}

		if (!isTemporary)
			last_optical_timestamp = timestamp;

		// state is valid now that we have had an update
		bIsValid = true;
	}
};

// -- public interface -----
//-- Orientation Filter --
ExternalOrientationFilter::ExternalOrientationFilter() :
	m_state(new ExternalOrientationFilterState),
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

	return true;
}

Eigen::Quaternionf ExternalOrientationFilter::getOrientation(float time, float offset_x, float offset_y, float offset_z, float offset_world_x, float offset_world_y, float offset_world_z) const
{
	Eigen::Quaternionf result = Eigen::Quaternionf::Identity();

	if (m_state->bIsValid)
	{
		Eigen::Quaternionf predicted_orientation = m_state->orientation;

		if (fabsf(time) > k_real_epsilon)
		{
			const Eigen::Quaternionf &quaternion_derivative =
				eigen_angular_velocity_to_quaternion_derivative(m_state->orientation, m_state->angular_velocity);

			predicted_orientation = Eigen::Quaternionf(
				m_state->orientation.coeffs()
				+ quaternion_derivative.coeffs()*time).normalized();
		}

		// Apply offsets to reset orientation
		const Eigen::Quaternionf local_offset_x_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_x * k_degrees_to_radians, Eigen::Vector3f::UnitX()));
		const Eigen::Quaternionf local_offset_y_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_y * k_degrees_to_radians, Eigen::Vector3f::UnitY()));
		const Eigen::Quaternionf local_offset_z_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_z * k_degrees_to_radians, Eigen::Vector3f::UnitZ()));
		const Eigen::Quaternionf local_offset_quat = local_offset_y_quat * local_offset_z_quat * local_offset_x_quat;

		const Eigen::Quaternionf world_offset_x_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_x * k_degrees_to_radians, Eigen::Vector3f::UnitX()));
		const Eigen::Quaternionf world_offset_y_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_y * k_degrees_to_radians, Eigen::Vector3f::UnitY()));
		const Eigen::Quaternionf world_offset_z_quat = Eigen::Quaternionf(Eigen::AngleAxisf(offset_world_z * k_degrees_to_radians, Eigen::Vector3f::UnitZ()));
		const Eigen::Quaternionf world_offset_quat = world_offset_y_quat * world_offset_z_quat * world_offset_x_quat;

		result = (world_offset_quat * m_state->reset_orientation) * (predicted_orientation * local_offset_quat);
	}

	return result;
}

Eigen::Quaternionf ExternalOrientationFilter::getResetOrientation() const
{
	return m_state->reset_orientation;
}

Eigen::Vector3f ExternalOrientationFilter::getAngularVelocityRadPerSec() const
{
	return m_state->bIsValid ? m_state->angular_velocity : Eigen::Vector3f::Zero();
}

Eigen::Vector3f ExternalOrientationFilter::getAngularAccelerationRadPerSecSqr() const
{
	return m_state->bIsValid ? m_state->angular_acceleration : Eigen::Vector3f::Zero();
}

// -- OrientationFilterExternal --
void OrientationFilterExternal::update(
	const t_high_resolution_timepoint timestamp,
	const PoseFilterPacket &packet)
{
#ifdef WIN32
	float optical_delta_time = (m_state->getOpticalTime(timestamp) / static_cast<float>(packet.stateLookBack));
	float imu_delta_time = (m_state->getImuTime(timestamp) / static_cast<float>(packet.stateLookBack));
	if (packet.isHalfFrame)
	{
		optical_delta_time /= 2.0f;
		imu_delta_time /= 2.0f;
	}

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
		Eigen::Quaternionf deltaRotation = m_state->orientation.conjugate() * new_orientation;
		Eigen::AngleAxisf axisAngle(deltaRotation);

		Eigen::Vector3f angularVelocity = axisAngle.axis() * (axisAngle.angle() / imu_delta_time);

		// Smooth angular velocity, otherwise its too jittery
		const float alpha = 0.2f;
		Eigen::Vector3f angVel = angularVelocity.transpose();
		new_angular_velocity = ((1.f - alpha) * m_state->angular_velocity + alpha * angVel);

		const Eigen::Vector3f new_angular_acceleration = (new_angular_velocity - m_state->angular_velocity) / imu_delta_time;

		m_state->apply_imu_state(new_orientation, new_angular_velocity, new_angular_acceleration, timestamp, packet.isTemporary);
	}
#endif
}
