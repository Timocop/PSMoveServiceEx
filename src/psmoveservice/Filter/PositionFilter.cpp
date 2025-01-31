// -- includes -----
#include "PositionFilter.h"
#include "MathEigen.h"
#include "ServerLog.h"

#include <chrono>
#include <numeric>

//-- constants -----
// The max distance between samples that we apply low pass filter on the optical position filter
#define k_max_lowpass_smoothing_distance 10.f  // cm

#define k_lowpass_smoothing_power 0.40f

// Used for lowpass filter of accelerometer
#define k_accelerometer_frequency_cutoff 1000.f // Hz

// Decay rate to apply to the jerk
#define k_jerk_decay 0.6f

// Decay rate to apply to the acceleration
#define k_acceleration_decay 0.7f

// Decay rate to apply to the velocity
#define k_velocity_decay 0.7f

// Past this time the complimentary filter will no longer keep doing
// IMU extrapolation of an unseen controller
#define k_max_unseen_position_timeout 10000.f // ms

#define k_lowpass_velocity_smoothing_factor 0.25f

#define k_velocity_adaptive_prediction_cutoff 0.5f

// Kalman measurement error in cm
#define k_kalman_position_error 10.f

// Kalman process noise
#define k_kalman_position_noise 300.0f

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

struct PositionFilterState
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
		const t_high_resolution_duration_milli time_delta = timestamp - last_imu_timestamp;
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
			SERVER_LOG_WARNING("PositionFilter") << "Position is NaN!";
		}

		if (eigen_vector3f_is_valid(new_velocity_m_per_sec))
		{
			velocity_m_per_sec = new_velocity_m_per_sec;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "Velocity is NaN!";
		}

		if (eigen_vector3f_is_valid(new_acceleration_m_per_sec_sqr))
		{
			acceleration_m_per_sec_sqr = new_acceleration_m_per_sec_sqr;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "Acceleration is NaN!";
		}

		if (eigen_vector3f_is_valid(new_accelerometer_g_units))
		{
			accelerometer_g_units = new_accelerometer_g_units;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "Accelerometer is NaN!";
		}

		if (eigen_vector3f_is_valid(new_accelerometer_derivative_g_per_sec))
		{
			accelerometer_derivative_g_per_sec = new_accelerometer_derivative_g_per_sec;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "AccelerometerDerivative is NaN!";
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
			SERVER_LOG_WARNING("PositionFilter") << "Position is NaN!";
		}

		if (eigen_vector3f_is_valid(new_velocity_m_per_sec))
		{
			velocity_m_per_sec = new_velocity_m_per_sec;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "Velocity is NaN!";
		}

		last_optical_timestamp = timestamp;

        // state is valid now that we have had an update
        bIsValid= true;
	}
};

// -- private methods -----
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
static Eigen::Vector3f lowpass_filter_optical_position_using_distance(
	const float delta_time,
    const PoseFilterPacket *filter_packet,
    const PositionFilterState *fusion_state,
	const float smoothing_distance,
	const float smoothing_power);
static Eigen::Vector3f lowpass_filter_optical_position_using_variance(
	const ExponentialCurve &position_variance_curve,
    const PoseFilterPacket *packet,
    const PositionFilterState *state);
static void lowpass_filter_imu_step(
    const float delta_time,
    const PositionFilterConstants *filter_constants,
    const PoseFilterPacket *filter_packet,
    const PositionFilterState *old_state,
    PositionFilterState *new_state);

// -- public interface -----
//-- Orientation Filter -----
PositionFilter::PositionFilter()
    : m_state(new PositionFilterState)
	, m_interpolationState(new PositionInterpolationState)
{
    memset(&m_constants, 0, sizeof(PositionFilterConstants));
    resetState();
}

PositionFilter::~PositionFilter()
{
	delete m_state;
	delete m_interpolationState;
}

bool PositionFilter::getIsStateValid() const
{
    return m_state->bIsValid;
}

double PositionFilter::getTimeInSeconds() const
{
    return m_state->getOpticalTime(std::chrono::high_resolution_clock::now());
}

void PositionFilter::resetState()
{
    m_state->reset();
	m_interpolationState->reset();
}

void PositionFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
}

bool PositionFilter::init(const PositionFilterConstants &constants)
{
    resetState();
    m_constants= constants;

    return true;
}

bool PositionFilter::init(const PositionFilterConstants &constants, const Eigen::Vector3f &initial_position)
{
	resetState();
	m_constants = constants;
	m_state->position_meters = initial_position;
	m_state->bIsValid = true;
	m_interpolationState->last_position_meters = initial_position;
	m_interpolationState->bIsValid = true;

	return true;
}

Eigen::Vector3f PositionFilter::getPositionCm(float time) const
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

Eigen::Vector3f PositionFilter::getVelocityCmPerSec() const
{
    Eigen::Vector3f result= m_state->velocity_m_per_sec * k_meters_to_centimeters;

    return (m_state->bIsValid) ? result : Eigen::Vector3f::Zero();
}

Eigen::Vector3f PositionFilter::getAccelerationCmPerSecSqr() const
{
    Eigen::Vector3f result= m_state->acceleration_m_per_sec_sqr * k_meters_to_centimeters;

    return (m_state->bIsValid) ? result : Eigen::Vector3f::Zero();
}

// -- Position Filters ----
// -- PositionFilterPassThru --
void PositionFilterPassThru::update(
	const t_high_resolution_timepoint timestamp,
	const PoseFilterPacket &packet)
{
	float optical_delta_time = m_state->getOpticalTime(timestamp);
	float imu_delta_time = m_state->getImuTime(timestamp);

	m_interpolationState->apply_timestamp_state(timestamp);

	float velocity_smoothing_factor = k_lowpass_velocity_smoothing_factor;
	float position_prediction_cutoff = k_velocity_adaptive_prediction_cutoff;

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

				velocity_smoothing_factor = config.filter_velocity_smoothing_factor;
				position_prediction_cutoff = config.filter_velocity_prediction_cutoff;

				break;
			}
			case CommonDeviceState::PSDualShock4:
			{
				PSDualShock4Controller *controller = ControllerView->castChecked<PSDualShock4Controller>();
				PSDualShock4ControllerConfig config = *controller->getConfig();

				velocity_smoothing_factor = config.filter_velocity_smoothing_factor;
				position_prediction_cutoff = config.filter_velocity_prediction_cutoff;

				break;
			}
			case CommonDeviceState::VirtualController:
			{
				VirtualController *controller = ControllerView->castChecked<VirtualController>();
				VirtualControllerConfig *config = controller->getConfigMutable();

				velocity_smoothing_factor = config->filter_velocity_smoothing_factor;
				position_prediction_cutoff = config->filter_velocity_prediction_cutoff;

				break;
			}
			}
		}
	}
	else if (packet.hmdDeviceId > -1)
	{
		ServerHMDViewPtr HmdView = DeviceManager::getInstance()->getHMDViewPtr(packet.hmdDeviceId);
		if (HmdView != nullptr && HmdView->getIsOpen())
		{
			switch (HmdView->getHMDDeviceType())
			{
			case CommonDeviceState::Morpheus:
			{
				MorpheusHMD *controller = HmdView->castChecked<MorpheusHMD>();
				MorpheusHMDConfig *config = controller->getConfigMutable();

				velocity_smoothing_factor = config->filter_velocity_smoothing_factor;
				position_prediction_cutoff = config->filter_velocity_prediction_cutoff;

				break;
			}
			case CommonDeviceState::VirtualHMD:
			{
				VirtualHMD *controller = HmdView->castChecked<VirtualHMD>();
				VirtualHMDConfig *config = controller->getConfigMutable();

				velocity_smoothing_factor = config->filter_velocity_smoothing_factor;
				position_prediction_cutoff = config->filter_velocity_prediction_cutoff;

				break;
			}
			}
		}
	}
#endif

	// Clamp everything to safety
	velocity_smoothing_factor = clampf(velocity_smoothing_factor, 0.01f, 1.0f);
	position_prediction_cutoff = clampf(position_prediction_cutoff, 0.0f, (1 << 16));

	// If device isnt tracking, clear old position and velocity to remove over-prediction
	if (!packet.isCurrentlyTracking && !m_resetVelocity)
	{
		m_resetVelocity = true;
	}

	if (packet.has_optical_measurement() && packet.isSynced)
	{
		Eigen::Vector3f old_position_meters = m_state->position_meters;
		Eigen::Vector3f new_position_meters = packet.get_optical_position_in_meters();
		Eigen::Vector3f old_position_meters_sec = m_state->velocity_m_per_sec;
		Eigen::Vector3f new_position_meters_sec;

		if (m_resetVelocity)
		{
			m_resetVelocity = false;

			new_position_meters_sec = Eigen::Vector3f::Zero();
		}
		else
		{
			new_position_meters_sec = Eigen::Vector3f(
				(new_position_meters.x() - old_position_meters.x()) / optical_delta_time,
				(new_position_meters.y() - old_position_meters.y()) / optical_delta_time,
				(new_position_meters.z() - old_position_meters.z()) / optical_delta_time
			);

			Eigen::Vector3f old_velocity_cm = old_position_meters_sec * k_meters_to_centimeters;
			Eigen::Vector3f new_velocity_cm = new_position_meters_sec * k_meters_to_centimeters;

			new_velocity_cm = lowpass_filter_vector3f(velocity_smoothing_factor, old_velocity_cm, new_velocity_cm);

			new_position_meters_sec = new_velocity_cm * k_centimeters_to_meters;

			if (position_prediction_cutoff > k_real_epsilon)
			{
				// Do adapting prediction and smoothing
				const float velocity_speed_sec = new_position_meters_sec.norm();
				const float adaptive_time_scale = clampf(velocity_speed_sec / position_prediction_cutoff, 0.0f, 1.0f);

				new_position_meters_sec = new_position_meters_sec * adaptive_time_scale;
			}
		}

		m_state->apply_optical_state(new_position_meters, new_position_meters_sec, timestamp);
		m_interpolationState->apply_interpolation_state(new_position_meters, new_position_meters_sec, timestamp);
	}
}

// -- PositionFilterLowPassOptical --
void PositionFilterLowPassOptical::update(
	const t_high_resolution_timepoint timestamp,
	const PoseFilterPacket &packet)
{
	float optical_delta_time = m_state->getOpticalTime(timestamp);
	float imu_delta_time = m_state->getImuTime(timestamp);

	m_interpolationState->apply_timestamp_state(timestamp);

	float smoothing_distance = k_max_lowpass_smoothing_distance;
	float smoothing_power = k_lowpass_smoothing_power;
	float velocity_smoothing_factor = k_lowpass_velocity_smoothing_factor;
	float position_prediction_cutoff = k_velocity_adaptive_prediction_cutoff;

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

				smoothing_distance = config.filter_lowpassoptical_distance;
				smoothing_power = config.filter_lowpassoptical_smoothing;
				velocity_smoothing_factor = config.filter_velocity_smoothing_factor;
				position_prediction_cutoff = config.filter_velocity_prediction_cutoff;

				break;
			}
			case CommonDeviceState::PSDualShock4:
			{
				PSDualShock4Controller *controller = ControllerView->castChecked<PSDualShock4Controller>();
				PSDualShock4ControllerConfig config = *controller->getConfig();

				smoothing_distance = config.filter_lowpassoptical_distance;
				smoothing_power = config.filter_lowpassoptical_smoothing;
				velocity_smoothing_factor = config.filter_velocity_smoothing_factor;
				position_prediction_cutoff = config.filter_velocity_prediction_cutoff;

				break;
			}
			case CommonDeviceState::VirtualController:
			{
				VirtualController *controller = ControllerView->castChecked<VirtualController>();
				VirtualControllerConfig *config = controller->getConfigMutable();

				smoothing_distance = config->filter_lowpassoptical_distance;
				smoothing_power = config->filter_lowpassoptical_smoothing;
				velocity_smoothing_factor = config->filter_velocity_smoothing_factor;
				position_prediction_cutoff = config->filter_velocity_prediction_cutoff;

				break;
			}
			}
		}
	}
	else if (packet.hmdDeviceId > -1)
	{
		ServerHMDViewPtr HmdView = DeviceManager::getInstance()->getHMDViewPtr(packet.hmdDeviceId);
		if (HmdView != nullptr && HmdView->getIsOpen())
		{
			switch (HmdView->getHMDDeviceType())
			{
			case CommonDeviceState::Morpheus:
			{
				MorpheusHMD *controller = HmdView->castChecked<MorpheusHMD>();
				MorpheusHMDConfig *config = controller->getConfigMutable();

				smoothing_distance = config->filter_lowpassoptical_distance;
				smoothing_power = config->filter_lowpassoptical_smoothing;
				velocity_smoothing_factor = config->filter_velocity_smoothing_factor;
				position_prediction_cutoff = config->filter_velocity_prediction_cutoff;

				break;
			}
			case CommonDeviceState::VirtualHMD:
			{
				VirtualHMD *controller = HmdView->castChecked<VirtualHMD>();
				VirtualHMDConfig *config = controller->getConfigMutable();

				smoothing_distance = config->filter_lowpassoptical_distance;
				smoothing_power = config->filter_lowpassoptical_smoothing;
				velocity_smoothing_factor = config->filter_velocity_smoothing_factor;
				position_prediction_cutoff = config->filter_velocity_prediction_cutoff;

				break;
			}
			}
		}
	}
#endif

	// Clamp everything to safety
	smoothing_distance = clampf(smoothing_distance, 1.f, (1 << 16));
	smoothing_power = clampf(smoothing_power, 0.1f, 1.f);
	velocity_smoothing_factor = clampf(velocity_smoothing_factor, 0.01f, 1.0f);
	position_prediction_cutoff = clampf(position_prediction_cutoff, 0.0f, (1 << 16));

	// If device isnt tracking, clear old position and velocity to remove over-prediction
	if (!packet.isCurrentlyTracking && !m_resetVelocity)
	{
		m_resetVelocity = true;
	}

	if (packet.has_optical_measurement() && packet.isSynced)
	{
		Eigen::Vector3f old_position_meters = m_state->position_meters;
		Eigen::Vector3f new_position_meters;
		Eigen::Vector3f old_position_meters_sec = m_state->velocity_m_per_sec;
		Eigen::Vector3f new_position_meters_sec;

        if (m_state->bIsValid)
        {
            // New position is blended against the old position
            new_position_meters = lowpass_filter_optical_position_using_distance(
				optical_delta_time, 
				&packet, 
				m_state, 
				smoothing_distance, smoothing_power);
        }
        else
        {
            // If this is the first filter packet, just accept the position as gospel
            new_position_meters = packet.get_optical_position_in_meters();
        }

		if (m_resetVelocity)
		{
			m_resetVelocity = false;

			new_position_meters_sec = Eigen::Vector3f::Zero();
		}
		else
		{
			new_position_meters_sec = Eigen::Vector3f(
				(new_position_meters.x() - old_position_meters.x()) / optical_delta_time,
				(new_position_meters.y() - old_position_meters.y()) / optical_delta_time,
				(new_position_meters.z() - old_position_meters.z()) / optical_delta_time
			);

			Eigen::Vector3f old_velocity_cm = old_position_meters_sec * k_meters_to_centimeters;
			Eigen::Vector3f new_velocity_cm = new_position_meters_sec * k_meters_to_centimeters;

			new_velocity_cm = lowpass_filter_vector3f(velocity_smoothing_factor, old_velocity_cm, new_velocity_cm);

			new_position_meters_sec = new_velocity_cm * k_centimeters_to_meters;

			if (position_prediction_cutoff > k_real_epsilon)
			{
				// Do adapting prediction and smoothing
				const float velocity_speed_sec = new_position_meters_sec.norm();
				const float adaptive_time_scale = clampf(velocity_speed_sec / position_prediction_cutoff, 0.0f, 1.0f);
				
				new_position_meters_sec = new_position_meters_sec * adaptive_time_scale;
			}
		}

		m_state->apply_optical_state(new_position_meters, new_position_meters_sec, timestamp);
    }
}

// -- PositionFilterComplimentaryOpticalIMU --
void PositionFilterComplimentaryOpticalIMU::update(
	const t_high_resolution_timepoint timestamp,
	const PoseFilterPacket &packet)
{
	float optical_delta_time = m_state->getOpticalTime(timestamp);
	float imu_delta_time = m_state->getImuTime(timestamp);

	m_interpolationState->apply_timestamp_state(timestamp);

	static float g_max_unseen_position_timeout= k_max_unseen_position_timeout;

	if (packet.has_optical_measurement() && packet.isSynced)
	{
		Eigen::Vector3f new_position_meters;

		t_high_resolution_timepoint now = std::chrono::high_resolution_clock::now();
		t_high_resolution_duration_milli timeSinceLast = now - lastOpticalFrame;

		if (m_state->bIsValid &&
			timeSinceLast.count() < g_max_unseen_position_timeout)
		{
            // Compute a low-pass filter on the optical position update
			new_position_meters =
				lowpass_filter_optical_position_using_variance(
					m_constants.position_variance_curve, &packet, m_state);
		}
		else
		{
			new_position_meters= packet.get_optical_position_in_meters();
		}

		m_state->apply_optical_state(new_position_meters, timestamp);
		m_interpolationState->apply_interpolation_state(new_position_meters, timestamp);
		lastOpticalFrame = now;
    }

	// Don't bother with IMU state updates until the we've gotten one valid optical update
	if (m_state->bIsValid)
	{
		if (packet.has_imu_measurements())
		{
			PositionFilterState new_imu_state;
			Eigen::Vector3f new_position;

			// Compute the new filter state based on the previous filter state and new sensor data
			lowpass_filter_imu_step(
				imu_delta_time,
				&m_constants,
				&packet,
				m_state,
				&new_imu_state);

			m_state->apply_imu_state(
				new_imu_state.position_meters,
				new_imu_state.velocity_m_per_sec,
				new_imu_state.acceleration_m_per_sec_sqr,
				new_imu_state.accelerometer_g_units, 
				new_imu_state.accelerometer_derivative_g_per_sec,
				timestamp);
		}
	}
}

void PositionFilterKalman::update(
	const t_high_resolution_timepoint timestamp, 
	const PoseFilterPacket & packet)
{
	float optical_delta_time = m_state->getOpticalTime(timestamp);
	float imu_delta_time = m_state->getImuTime(timestamp);

	m_interpolationState->apply_timestamp_state(timestamp);

	float velocity_smoothing_factor = k_lowpass_velocity_smoothing_factor;
	float position_prediction_cutoff = k_velocity_adaptive_prediction_cutoff;
	float kalman_position_error = k_kalman_position_error;
	float kalman_position_noise = k_kalman_position_noise;
	bool kalman_position_noise_disable_cutoff = true;


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

				velocity_smoothing_factor = config.filter_velocity_smoothing_factor;
				position_prediction_cutoff = config.filter_velocity_prediction_cutoff;
				kalman_position_error = config.filter_position_kalman_error;
				kalman_position_noise = config.filter_position_kalman_noise;
				kalman_position_noise_disable_cutoff = config.filter_position_kalman_disable_cutoff;

				break;
			}
			case CommonDeviceState::PSDualShock4:
			{
				PSDualShock4Controller *controller = ControllerView->castChecked<PSDualShock4Controller>();
				PSDualShock4ControllerConfig config = *controller->getConfig();

				velocity_smoothing_factor = config.filter_velocity_smoothing_factor;
				position_prediction_cutoff = config.filter_velocity_prediction_cutoff;
				kalman_position_error = config.filter_position_kalman_error;
				kalman_position_noise = config.filter_position_kalman_noise;
				kalman_position_noise_disable_cutoff = config.filter_position_kalman_disable_cutoff;

				break;
			}
			case CommonDeviceState::VirtualController:
			{
				VirtualController *controller = ControllerView->castChecked<VirtualController>();
				VirtualControllerConfig *config = controller->getConfigMutable();

				velocity_smoothing_factor = config->filter_velocity_smoothing_factor;
				position_prediction_cutoff = config->filter_velocity_prediction_cutoff;
				kalman_position_error = config->filter_position_kalman_error;
				kalman_position_noise = config->filter_position_kalman_noise;
				kalman_position_noise_disable_cutoff = config->filter_position_kalman_disable_cutoff;

				break;
			}
			}
		}
	}
	else if (packet.hmdDeviceId > -1)
	{
		ServerHMDViewPtr HmdView = DeviceManager::getInstance()->getHMDViewPtr(packet.hmdDeviceId);
		if (HmdView != nullptr && HmdView->getIsOpen())
		{
			switch (HmdView->getHMDDeviceType())
			{
			case CommonDeviceState::Morpheus:
			{
				MorpheusHMD *controller = HmdView->castChecked<MorpheusHMD>();
				MorpheusHMDConfig *config = controller->getConfigMutable();

				velocity_smoothing_factor = config->filter_velocity_smoothing_factor;
				position_prediction_cutoff = config->filter_velocity_prediction_cutoff;
				kalman_position_error = config->filter_position_kalman_error;
				kalman_position_noise = config->filter_position_kalman_noise;
				kalman_position_noise_disable_cutoff = config->filter_position_kalman_disable_cutoff;

				break;
			}
			case CommonDeviceState::VirtualHMD:
			{
				VirtualHMD *controller = HmdView->castChecked<VirtualHMD>();
				VirtualHMDConfig *config = controller->getConfigMutable();

				velocity_smoothing_factor = config->filter_velocity_smoothing_factor;
				position_prediction_cutoff = config->filter_velocity_prediction_cutoff;
				kalman_position_error = config->filter_position_kalman_error;
				kalman_position_noise = config->filter_position_kalman_noise;
				kalman_position_noise_disable_cutoff = config->filter_position_kalman_disable_cutoff;

				break;
			}
			}
		}
	}
#endif

	// Clamp everything to safety
	velocity_smoothing_factor = clampf(velocity_smoothing_factor, 0.01f, 1.0f);
	position_prediction_cutoff = clampf(position_prediction_cutoff, 0.0f, (1 << 16));
	kalman_position_error = clampf(kalman_position_error, 0.01f, (1 << 16));
	kalman_position_noise = clampf(kalman_position_noise, 0.01f, (1 << 16));

	// If device isnt tracking, clear old position and velocity to remove over-prediction
	if (!packet.isCurrentlyTracking && !m_resetVelocity)
	{
		m_resetVelocity = true;
	}

	if (packet.has_optical_measurement() && packet.isSynced)
	{
		Eigen::Vector3f old_position_meters = m_state->position_meters;
		Eigen::Vector3f new_position_meters = packet.get_optical_position_in_meters();
		Eigen::Vector3f old_position_meters_sec = m_state->velocity_m_per_sec;
		Eigen::Vector3f new_position_meters_sec;

		if (m_resetVelocity)
		{
			m_resetVelocity = false;

			new_position_meters_sec = Eigen::Vector3f::Zero();
		}
		else
		{
			Eigen::Vector3f old_position_cm = old_position_meters * k_meters_to_centimeters;
			Eigen::Vector3f new_position_cm = new_position_meters * k_meters_to_centimeters;

			lowpass_vector3f_kalman(
				new_position_cm, 
				old_position_cm, 
				kal_pos_gain, 
				kal_pos_err_estimate, 
				kal_pos_current_estimate,
				kalman_position_error, 
				kalman_position_noise, 
				optical_delta_time);

			new_position_meters = new_position_cm * k_centimeters_to_meters;

			new_position_meters_sec = Eigen::Vector3f(
				(new_position_meters.x() - old_position_meters.x()) / optical_delta_time,
				(new_position_meters.y() - old_position_meters.y()) / optical_delta_time,
				(new_position_meters.z() - old_position_meters.z()) / optical_delta_time
			);

			Eigen::Vector3f old_velocity_cm = old_position_meters_sec * k_meters_to_centimeters;
			Eigen::Vector3f new_velocity_cm = new_position_meters_sec * k_meters_to_centimeters;

			new_velocity_cm = lowpass_filter_vector3f(velocity_smoothing_factor, old_velocity_cm, new_velocity_cm);

			/*
			// $TODO WIP Kalman velocity
			lowpass_vector3f_kalman(
				new_velocity_cm,
				old_velocity_cm,
				kal_vel_gain,
				kal_vel_err_estimate,
				kal_vel_current_estimate,
				0.1f,
				1.0f * velocity_smoothing_factor,
				optical_delta_time);*/

			new_position_meters_sec = new_velocity_cm * k_centimeters_to_meters;

			if (!kalman_position_noise_disable_cutoff && position_prediction_cutoff > k_real_epsilon)
			{
				// Do adapting prediction and smoothing
				const float velocity_speed_sec = new_position_meters_sec.norm();
				const float adaptive_time_scale = clampf(velocity_speed_sec / position_prediction_cutoff, 0.0f, 1.0f);

				new_position_meters_sec = new_position_meters_sec * adaptive_time_scale;
			}
		}

		m_state->apply_optical_state(new_position_meters, new_position_meters_sec, timestamp);
		m_interpolationState->apply_interpolation_state(new_position_meters, new_position_meters_sec, timestamp);
	}
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

static Eigen::Vector3f lowpass_filter_optical_position_using_distance(
	float optical_delta_time,
    const PoseFilterPacket *packet,
    const PositionFilterState *state,
	const float smoothing_distance,
	const float smoothing_power)
{
    assert(state->bIsValid);
	assert(packet->has_optical_measurement());

	float new_position_weight = 1.f;

	if (smoothing_distance > k_real_epsilon)
	{
		// Traveling k_max_lowpass_smoothing_distance in one frame should have 0 smoothing
		// Traveling 0+noise cm in one frame should have 60% smoothing
		// We need to convert meters to centimeters otherwise it wont work. I assume value too small for decimal point precision.
		Eigen::Vector3f diff = (packet->get_optical_position_in_meters() - state->position_meters) * k_meters_to_centimeters;
		float distance = diff.norm() / optical_delta_time;
		new_position_weight = clampf01(lerpf(smoothing_power, 1.00f, distance / (smoothing_distance * k_meters_to_centimeters)));
	}

    // New position is blended against the old position
    const Eigen::Vector3f &old_position = state->position_meters;
    const Eigen::Vector3f new_position = packet->get_optical_position_in_meters();
    const Eigen::Vector3f filtered_new_position = lowpass_filter_vector3f(new_position_weight, old_position, new_position);

    return filtered_new_position;
}

static Eigen::Vector3f lowpass_filter_optical_position_using_variance(
	const ExponentialCurve &position_variance_curve,
	const PoseFilterPacket *packet,
	const PositionFilterState *state)
{
	assert(state->bIsValid);
	assert(packet->has_optical_measurement());

	// Compute the amount of variance in position we expect to see for the given screen area
	const float position_variance =
		position_variance_curve.evaluate(packet->tracking_projection_area_px_sqr);

	// Compute the percentage of maximum variance
	const float max_variance_fraction =
		safe_divide_with_default(position_variance, position_variance_curve.MaxValue, 1.f);

	// Trust the new position more when there is less variance
	const float new_position_weight = clampf01(1.f - max_variance_fraction);

	// New position is blended against the old position
	const Eigen::Vector3f &old_position = state->position_meters;
	const Eigen::Vector3f new_position = packet->get_optical_position_in_meters();
	const Eigen::Vector3f filtered_new_position = lowpass_filter_vector3f(new_position_weight, old_position, new_position);

	return filtered_new_position;
}

static void lowpass_filter_imu_step(
    const float delta_time,
    const PositionFilterConstants *filter_constants,
    const PoseFilterPacket *filter_packet,
    const PositionFilterState *old_state,
    PositionFilterState *new_state)
{
    assert(old_state->bIsValid);

    if (delta_time > k_real_epsilon)
    {
        // Gather sensor state from the previous frame
        const Eigen::Vector3f &old_accelerometer= old_state->accelerometer_g_units;
        const Eigen::Vector3f &old_accelerometer_derivative= old_state->accelerometer_derivative_g_per_sec;

        // Gather physics state from the previous frame
        const Eigen::Vector3f &old_position = old_state->position_meters;
        const Eigen::Vector3f &old_velocity = old_state->velocity_m_per_sec;
        const Eigen::Vector3f &old_acceleration = old_state->acceleration_m_per_sec_sqr;

        // Gather new sensor readings
        // Need to negate the accelerometer reading since it points the opposite direction of gravity)
        const Eigen::Vector3f &new_accelerometer= -filter_packet->world_accelerometer;
   
        // Compute the filtered derivative of the accelerometer (a.k.a. the "jerk")
        Eigen::Vector3f new_accelerometer_derivative= Eigen::Vector3f::Zero();
        {
            const Eigen::Vector3f accelerometer_derivative=
                (new_accelerometer - old_accelerometer) / delta_time;

            // Throw out any jerk below the noise threshold
            const Eigen::Vector3f thresholded_derivative= 
                threshold_vector3f(accelerometer_derivative, filter_constants->accelerometer_noise_radius);

            // Apply a decay filter to the jerk
            static float g_jerk_decay= k_jerk_decay;
            new_accelerometer_derivative= thresholded_derivative*g_jerk_decay;
        }

        // The accelerometer is in "g-units", where 1 g-unit = 9.8m/s^2
        const Eigen::Vector3f old_jerk= old_accelerometer_derivative * k_g_units_to_ms2;
        const Eigen::Vector3f new_jerk= new_accelerometer_derivative * k_g_units_to_ms2;

        // Convert the new acceleration by integrating the jerk
        Eigen::Vector3f new_acceleration= 
            0.5f*(old_jerk + new_jerk)*delta_time 
            + old_acceleration;

        // Apply a lowpass filter to the acceleration
        static float g_cutoff_frequency= k_accelerometer_frequency_cutoff;
        new_acceleration= 
            lowpass_filter_vector3f(delta_time, g_cutoff_frequency, old_acceleration, new_acceleration);

        // Apply a decay filter to the acceleration
        static float g_acceleration_decay= k_acceleration_decay;
        new_acceleration*= g_acceleration_decay;

        // new velocity v1 = v0 + 0.5*(a0+a1)*dt, using trapezoid rule integration
        const Eigen::Vector3f new_unclamped_velocity = 
            0.5f*(old_acceleration + new_acceleration)*delta_time
            + old_velocity;

        // Make sure the velocity doesn't exceed the speed limit
        Eigen::Vector3f new_velocity =
            clamp_vector3f(new_unclamped_velocity, filter_constants->max_velocity);
        
        // Apply a decay filter to the acceleration
        static float g_velocity_decay= k_velocity_decay;
        new_velocity*= g_velocity_decay;

        // new position x1 = x0 + 0.5*(v0+v1)*dt, using trapezoid rule integration
        // Pass the updated position out so that the caller can decide
        // how it should be applied to the fusion state 
        // (i.e. blend it with optical position)
        const Eigen::Vector3f new_position = 
            0.5f*(old_velocity + new_velocity)*delta_time
            + old_position;

        // Save out the updated sensor state
        new_state->accelerometer_g_units= new_accelerometer;
        new_state->accelerometer_derivative_g_per_sec= new_accelerometer_derivative;

        // Save out the updated fusion state
        new_state->acceleration_m_per_sec_sqr = new_acceleration;
        new_state->velocity_m_per_sec = new_velocity;
        new_state->position_meters= new_position;
    }
    else
    {
        *new_state= *old_state;
    }
}