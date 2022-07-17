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
	double time;

	/* Quaternion measured when controller points towards camera */
	Eigen::Quaternionf reset_orientation;

	/// The amount of time since the optical filter was updated
	double accumulated_optical_time_delta;

	/// The amount of time since the imu filter was updated
	double accumulated_imu_time_delta;

	void reset()
	{
		bIsValid = false;
		orientation = Eigen::Quaternionf::Identity();
		angular_velocity = Eigen::Vector3f::Zero();
		angular_acceleration = Eigen::Vector3f::Zero();
		reset_orientation = Eigen::Quaternionf::Identity();
		time = 0.0;
		accumulated_optical_time_delta = 0.f;
		accumulated_imu_time_delta = 0.f;
	}

	void apply_imu_state(
		const Eigen::Quaternionf &new_orientation,
		const Eigen::Vector3f &new_angular_velocity,
		const Eigen::Vector3f &new_angular_acceleration,
		const float delta_time)
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

		if (is_valid_float(delta_time))
		{
			time = accumulated_imu_time_delta + (double)delta_time;
			accumulated_imu_time_delta = 0.0;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "time delta is NaN!";
		}

		// state is valid now that we have had an update
		bIsValid = true;
	}

	void apply_optical_state(
		const Eigen::Quaternionf &new_orientation,
		const float delta_time)
	{
		if (eigen_quaternion_is_valid(new_orientation))
		{
			orientation = new_orientation;
		}
		else
		{
			SERVER_LOG_WARNING("OrientationFilter") << "Orientation is NaN!";
		}

		if (is_valid_float(delta_time))
		{
			time = accumulated_imu_time_delta + (double)delta_time;
			accumulated_imu_time_delta = 0.0;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "time delta is NaN!";
		}

		// state is valid now that we have had an update
		bIsValid = true;
	}

	void accumulate_optical_delta_time(const float delta_time)
	{
		if (is_valid_float(delta_time))
		{
			accumulated_optical_time_delta += (double)delta_time;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "optical time delta is NaN!";
		}
	}

	void accumulate_imu_delta_time(const float delta_time)
	{
		if (is_valid_float(delta_time))
		{
			accumulated_imu_time_delta += (double)delta_time;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "imu time delta is NaN!";
		}
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
	return m_state->time;
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

Eigen::Quaternionf ExternalOrientationFilter::getOrientation(float time, float offset_x, float offset_y, float offset_z) const
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
		const Eigen::EulerAnglesf offset_euler(offset_x, offset_y, offset_z);
		const Eigen::Quaternionf offset_quat = eigen_euler_angles_to_quaternionf(offset_euler);
		const Eigen::Quaternionf reset_quat = m_state->reset_orientation * offset_quat;

		result = reset_quat * predicted_orientation;
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
void OrientationFilterExternal::update(const float delta_time, const PoseFilterPacket &packet)
{
#ifdef WIN32
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

	if (vector.size() >= 4)
	{
		Eigen::Quaternionf new_orientation;
		new_orientation.x() = (float)_atof_l(vector[0].c_str(), localeInvariant);
		new_orientation.y() = (float)_atof_l(vector[1].c_str(), localeInvariant);
		new_orientation.z() = (float)_atof_l(vector[2].c_str(), localeInvariant);
		new_orientation.w() = (float)_atof_l(vector[3].c_str(), localeInvariant);

		if (eigen_quaternion_is_valid(new_orientation))
		{
			m_state->apply_optical_state(new_orientation, delta_time);
		}
	}

	if (vector.size() >= 8)
	{
		Eigen::Quaternionf reset_orientation;
		reset_orientation.x() = (float)_atof_l(vector[4].c_str(), localeInvariant);
		reset_orientation.y() = (float)_atof_l(vector[5].c_str(), localeInvariant);
		reset_orientation.z() = (float)_atof_l(vector[6].c_str(), localeInvariant);
		reset_orientation.w() = (float)_atof_l(vector[7].c_str(), localeInvariant);

		if (eigen_quaternion_is_valid(reset_orientation))
		{
			Eigen::Quaternionf q_inverse = reset_orientation.conjugate();

			eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
			m_state->reset_orientation = q_inverse;
		}
	}
#endif
}
