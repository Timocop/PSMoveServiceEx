// -- includes -----
#include "PositionFilter.h"
#include "MathEigen.h"
#include "ServerLog.h"

#include <chrono>
#include <numeric>

//-- constants -----
// The max distance between samples that we apply low pass filter on the optical position filter
#define k_max_lowpass_smoothing_distance 10.f * k_centimeters_to_meters // meters

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

#define k_max_optical_prediction_power 10.0f

// -- private definitions -----
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

    /// The number of seconds the filter has been running
    double time;

	/// The amount of time since the optical filter was updated
	double accumulated_optical_time_delta;

	/// The amount of time since the imu filter was updated
	double accumulated_imu_time_delta;

    void reset()
    {
        bIsValid = false;
        position_meters = Eigen::Vector3f::Zero();
        velocity_m_per_sec = Eigen::Vector3f::Zero();
        acceleration_m_per_sec_sqr = Eigen::Vector3f::Zero();
        accelerometer_g_units = Eigen::Vector3f::Zero();
        accelerometer_derivative_g_per_sec = Eigen::Vector3f::Zero();
        origin_position = Eigen::Vector3f::Zero();
        time= 0.0;
		accumulated_optical_time_delta= 0.f;
		accumulated_imu_time_delta= 0.f;
    }

	void apply_imu_state(
		const Eigen::Vector3f &new_position_m,
		const Eigen::Vector3f &new_velocity_m_per_sec,
		const Eigen::Vector3f &new_acceleration_m_per_sec_sqr,
		const Eigen::Vector3f &new_accelerometer_g_units,
		const Eigen::Vector3f &new_accelerometer_derivative_g_per_sec,
		const float delta_time)
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

		if (is_valid_float(delta_time))
		{
			time= accumulated_imu_time_delta + (double)delta_time;
			accumulated_imu_time_delta= 0.0;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "time delta is NaN!";
		}

        // state is valid now that we have had an update
        bIsValid= true;
	}

	void apply_optical_state(
		const Eigen::Vector3f &new_position_m,
		const float delta_time)
	{
		apply_optical_state(new_position_m, Eigen::Vector3f::Zero(), delta_time);
	}

	void apply_optical_state(
		const Eigen::Vector3f &new_position_m,
		const Eigen::Vector3f &new_velocity_m_per_sec,
		const float delta_time)
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

		if (is_valid_float(delta_time))
		{
			time= accumulated_imu_time_delta + (double)delta_time;
			accumulated_imu_time_delta= 0.0;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "time delta is NaN!";
		}

        // state is valid now that we have had an update
        bIsValid= true;
	}

	void accumulate_optical_delta_time(const float delta_time)
	{
		if (is_valid_float(delta_time))
		{
			accumulated_optical_time_delta+= (double)delta_time;
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
			accumulated_imu_time_delta+= (double)delta_time;
		}
		else
		{
			SERVER_LOG_WARNING("PositionFilter") << "imu time delta is NaN!";
		}
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
static Eigen::Vector3f lowpass_prediction_vector3f(
	const float delta_time,
	const Eigen::Vector3f &old_filtered_vector,
	const Eigen::Vector3f &new_vector);
static Eigen::Vector3f lowpass_filter_optical_position_using_distance(
    const PoseFilterPacket *filter_packet,
    const PositionFilterState *fusion_state);
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
{
    memset(&m_constants, 0, sizeof(PositionFilterConstants));
    resetState();
}

PositionFilter::~PositionFilter()
{
    delete m_state;
}

bool PositionFilter::getIsStateValid() const
{
    return m_state->bIsValid;
}

double PositionFilter::getTimeInSeconds() const
{
    return m_state->time;
}

void PositionFilter::resetState()
{
    m_state->reset();
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

	return true;
}

Eigen::Vector3f PositionFilter::getPositionCm(float time) const
{
    Eigen::Vector3f result = Eigen::Vector3f::Zero();

    if (m_state->bIsValid)
    {
        Eigen::Vector3f predicted_position = 
            is_nearly_zero(time)
            ? m_state->position_meters
            : m_state->position_meters + m_state->velocity_m_per_sec * time;

        result= predicted_position - m_state->origin_position;
        result= result * k_meters_to_centimeters;
    }

    return result;
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
	const float delta_time, 
	const PoseFilterPacket &packet)
{
	const int history_queue_length = 5;

	if (packet.has_optical_measurement() && packet.isSynced)
	{
		Eigen::Vector3f old_position_meters;
		Eigen::Vector3f new_position_meters = packet.get_optical_position_in_meters();
		Eigen::Vector3f new_position_meters_sec;

		// Apply basic optical prediction
		// Add the new blend position to blend history
		blendedPositionHistory.push_back(new_position_meters);
		if (blendedPositionHistory.size() > history_queue_length)
		{
			blendedPositionHistory.pop_front();
		}

		old_position_meters = Eigen::Vector3f::Zero();

		int sampleCount = 0;
		for (std::list<Eigen::Vector3f>::iterator it = blendedPositionHistory.begin(); it != blendedPositionHistory.end(); it++)
		{
			old_position_meters += *it;
			sampleCount++;
		}

		if (sampleCount > 0)
		{
			new_position_meters_sec =
				lowpass_prediction_vector3f(delta_time, (old_position_meters / sampleCount), new_position_meters);
		}
		else
		{
			new_position_meters_sec = Eigen::Vector3f::Zero();
		}

		m_state->apply_optical_state(new_position_meters, new_position_meters_sec, delta_time);
	}
	else
	{
		m_state->accumulate_optical_delta_time(delta_time);
	}
}

// -- PositionFilterLowPassOptical --
void PositionFilterLowPassOptical::update(
	const float delta_time, 
	const PoseFilterPacket &packet)
{
	const int history_queue_length = 5;

	if (packet.has_optical_measurement() && packet.isSynced)
	{
		Eigen::Vector3f old_position_meters;
		Eigen::Vector3f new_position_meters;
		Eigen::Vector3f new_position_meters_sec;

        if (m_state->bIsValid)
        {
            // New position is blended against the old position
            new_position_meters = lowpass_filter_optical_position_using_distance(&packet, m_state);
        }
        else
        {
            // If this is the first filter packet, just accept the position as gospel
            new_position_meters = packet.get_optical_position_in_meters();
        }

		// Apply basic optical prediction
		// Add the new blend position to blend history
		blendedPositionHistory.push_back(new_position_meters);
		if (blendedPositionHistory.size() > history_queue_length)
		{
			blendedPositionHistory.pop_front();
		}

		old_position_meters = Eigen::Vector3f::Zero();
		
		int sampleCount = 0;
		for (std::list<Eigen::Vector3f>::iterator it = blendedPositionHistory.begin(); it != blendedPositionHistory.end(); it++)
		{
			old_position_meters += *it;
			sampleCount++;
		}

		if (sampleCount > 0)
		{
			new_position_meters_sec =
				lowpass_prediction_vector3f(delta_time, (old_position_meters / sampleCount), new_position_meters);
		}
		else
		{
			new_position_meters_sec = Eigen::Vector3f::Zero();
		}

		m_state->apply_optical_state(new_position_meters, new_position_meters_sec, delta_time);
    }
	else
	{
		m_state->accumulate_optical_delta_time(delta_time);
	}
}

// -- PositionFilterLowPassIMU --
void PositionFilterLowPassIMU::update(
	const float delta_time,
	const PoseFilterPacket &packet)
{
	if (packet.has_optical_measurement() && packet.isSynced)
	{
		// Use the raw optical position unfiltered
		const Eigen::Vector3f new_position_meters= packet.get_optical_position_in_meters();

		m_state->apply_optical_state(new_position_meters, delta_time);
    }
	else
	{
		m_state->accumulate_optical_delta_time(delta_time);
	}

    if (packet.has_imu_measurements() && m_state->bIsValid)
    {
		PositionFilterState new_state;

        lowpass_filter_imu_step(
            (float)m_state->accumulated_imu_time_delta + delta_time,
            &m_constants,
            &packet,
            m_state,
            &new_state);

		m_state->apply_imu_state(
			new_state.position_meters,
			new_state.velocity_m_per_sec,
			new_state.acceleration_m_per_sec_sqr,
			new_state.accelerometer_g_units, 
			new_state.accelerometer_derivative_g_per_sec,
			delta_time);
    }
	else
	{
		m_state->accumulate_imu_delta_time(delta_time);
	}
}

// -- PositionFilterComplimentaryOpticalIMU --
void PositionFilterComplimentaryOpticalIMU::update(const float delta_time, const PoseFilterPacket &packet)
{
	static float g_max_unseen_position_timeout= k_max_unseen_position_timeout;

	if (packet.has_optical_measurement() && packet.isSynced)
	{
		Eigen::Vector3f new_position_meters;

		if (m_state->bIsValid &&
			m_state->accumulated_optical_time_delta < g_max_unseen_position_timeout)
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

		m_state->apply_optical_state(new_position_meters, delta_time);
    }
	else
	{
		m_state->accumulate_optical_delta_time(delta_time);
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
				(float)m_state->accumulated_imu_time_delta+delta_time,
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
				delta_time);
		}
		else
		{
			m_state->accumulate_imu_delta_time(delta_time);
		}
	}
}

// -- PositionFilterComplimentaryOpticalIMU --
void PositionFilterLowPassExponential::update(const float delta_time, const PoseFilterPacket &packet)
{
	const int history_queue_length = 20;

	if (packet.has_optical_measurement() && packet.isSynced)
    {        
		Eigen::Vector3f new_position_meters;
		Eigen::Vector3f new_velocity_m_per_sec= Eigen::Vector3f::Zero();

        if (m_state->bIsValid)
        {
			// Blend the latest position against the last position in the filter state ...
            Eigen::Vector3f lowpass_position_meters = 
				lowpass_filter_optical_position_using_distance(&packet, m_state);

			// ... Then blend that result with the blended position at the start of the history
			const float k_history_blend = 0.8f;
			Eigen::Vector3f oldPosition = *(blendedPositionHistory.begin());			
			new_position_meters = (lowpass_position_meters * k_history_blend) + (oldPosition * (1.0f - k_history_blend));

			// Add the new blend position to blend history
			blendedPositionHistory.push_back(new_position_meters);
			if (blendedPositionHistory.size() > history_queue_length)
			{
				blendedPositionHistory.pop_front();
			}

			deltaTimeHistory.push_back(delta_time);
			if (deltaTimeHistory.size() > history_queue_length)
			{
				deltaTimeHistory.pop_front();
			}

			float timeHistoryDuration = 0.f;
			for (std::list<float>::iterator it = deltaTimeHistory.begin(); it != deltaTimeHistory.end(); it++)
			{
				timeHistoryDuration += *it;
			}

			if (timeHistoryDuration > 0.f)
			{
				new_velocity_m_per_sec = 
					(blendedPositionHistory.back() - blendedPositionHistory.front()) / timeHistoryDuration;
			}
        }
        else
        {
			while (deltaTimeHistory.size() < history_queue_length)
				deltaTimeHistory.push_back(delta_time);

			while (blendedPositionHistory.size() < history_queue_length)
				blendedPositionHistory.push_back(m_state->position_meters);

            // If this is the first filter packet, just accept the position as gospel
            new_position_meters = packet.get_optical_position_in_meters();
        }

		m_state->apply_optical_state(new_position_meters, new_velocity_m_per_sec, delta_time);
    }
	else
	{
		m_state->accumulate_optical_delta_time(delta_time);
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

static Eigen::Vector3f lowpass_prediction_vector3f(
	const float delta_time,
	const Eigen::Vector3f &old_filtered_vector,
	const Eigen::Vector3f &new_vector)
{
	Eigen::Vector3f new_position_meters_sec;

	// Apply basic optical prediction
	// We need to convert meters to centimeters otherwise it wont work. I assume value too small for decimal point precision.
	Eigen::Vector3f diff = (new_vector - old_filtered_vector) * k_meters_to_centimeters;
	float distance = diff.norm();
	float new_position_weight = clampf01(lerpf(0.40f, 1.00f, distance / (k_max_lowpass_smoothing_distance * k_meters_to_centimeters)));

	static float g_cutoff_frequency = k_accelerometer_frequency_cutoff;
	new_position_meters_sec =
		lowpass_filter_vector3f(
			delta_time, g_cutoff_frequency,
			Eigen::Vector3f::Zero(), ((new_vector - old_filtered_vector) * new_position_weight) * k_max_optical_prediction_power);

	return new_position_meters_sec;
}

static Eigen::Vector3f lowpass_filter_optical_position_using_distance(
    const PoseFilterPacket *packet,
    const PositionFilterState *state)
{
    assert(state->bIsValid);
	assert(packet->has_optical_measurement());

    // Traveling k_max_lowpass_smoothing_distance in one frame should have 0 smoothing
    // Traveling 0+noise cm in one frame should have 60% smoothing
	// We need to convert meters to centimeters otherwise it wont work. I assume value too small for decimal point precision.
    Eigen::Vector3f diff = (packet->get_optical_position_in_meters() - state->position_meters) * k_meters_to_centimeters;
    float distance = diff.norm();
    float new_position_weight = clampf01(lerpf(0.40f, 1.00f, distance / (k_max_lowpass_smoothing_distance * k_meters_to_centimeters)));

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