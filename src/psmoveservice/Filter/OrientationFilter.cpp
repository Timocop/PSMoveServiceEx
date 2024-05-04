// -- includes -----
#include "OrientationFilter.h"
#include "MathAlignment.h"
#include "ServerLog.h"
#include <deque>

//-- constants -----
// Maximum we blend against the optically derived orientation
#define k_max_optical_orientation_weight 0.005f

// Complementary MARG Filter constants
#define k_base_earth_frame_align_weight 0.02f

// Max length of the orientation history we keep
#define k_orientation_history_max 16

// Used for lowpass filter of accelerometer
#define k_accelerometer_frequency_cutoff 1000.f // Hz

// Decay rate to apply to the jerk
#define k_jerk_decay 0.8f

// Decay rate to apply to the acceleration
#define k_acceleration_decay 0.8f

// Decay rate to apply to the velocity
#define k_velocity_decay 0.8f

#define k_madgwick_beta 0.20f
#define k_madgwick_stabil_min_beta 0.02f
#define k_madgwick_gyro_min_deg 1.5f
#define k_madgwick_gyro_max_deg 15.0f
#define k_madgwick_beta_smoothing_factor 0.1f
#define k_madgwick_smart_correct_min_deg 5.0f
#define k_madgwick_smart_correct_max_deg 25.0f
#define k_madgwick_smart_correct_reset_time_ms 500.0f
#define k_madgwick_smart_correct_gravity_stable 0.8f

#define k_lowpass_velocity_smoothing_factor 0.25f

#define k_adaptive_prediction_cutoff 0.25f
#define k_magnetometer_deviation_cutoff 0.1f

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

struct OrientationFilterState
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
        bIsValid= false;
        orientation= Eigen::Quaternionf::Identity();
        angular_velocity = Eigen::Vector3f::Zero();
        angular_acceleration = Eigen::Vector3f::Zero();
        reset_orientation= Eigen::Quaternionf::Identity();
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
            angular_velocity= new_angular_velocity;
        }
        else
        {
            SERVER_LOG_WARNING("OrientationFilter") << "Angular Velocity is NaN!";
        }

        if (eigen_vector3f_is_valid(new_angular_acceleration))
        {
            angular_acceleration= new_angular_acceleration;
        }
        else
        {
            SERVER_LOG_WARNING("OrientationFilter") << "Angular Acceleration is NaN!";
        }

		last_imu_timestamp = timestamp;

        // state is valid now that we have had an update
        bIsValid= true;
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
        bIsValid= true;
    }
};

// -- private methods -----
static Eigen::Vector3f lowpass_filter_vector3f(
	const float alpha,
	const Eigen::Vector3f &old_filtered_vector,
	const Eigen::Vector3f &new_vector);

static bool magnetometer_in_bounds_vector(
	const CommonRawDeviceVector raw_mag_int,
	const EigenFitEllipsoid & ellipsoid,
	float & m_mag_scale, 
	const float smoothing, 
	const float diviation);

static bool magnetometer_in_bounds_vector3f(
	const Eigen::Vector3f raw_mag,
	const EigenFitEllipsoid & ellipsoid, 
	float & m_mag_scale, 
	const float smoothing, 
	const float diviation);

// -- public interface -----
//-- Orientation Filter --
OrientationFilter::OrientationFilter() :
    m_state(new OrientationFilterState),
	m_interpolationState(new OrientationInterpolationState)
{
    memset(&m_constants, 0, sizeof(OrientationFilterConstants));
    resetState();
}

OrientationFilter::~OrientationFilter()
{
    delete m_state;
	delete m_interpolationState;
}

bool OrientationFilter::getIsStateValid() const
{
    return m_state->bIsValid;
}

double OrientationFilter::getTimeInSeconds() const
{
    return m_state->getImuTime(std::chrono::high_resolution_clock::now());
}

void OrientationFilter::resetState()
{
    m_state->reset();
	m_interpolationState->reset();
}

void OrientationFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
    Eigen::Quaternionf q_inverse = m_state->orientation.conjugate();

    eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
    m_state->reset_orientation= q_pose*q_inverse;
}

bool OrientationFilter::init(const OrientationFilterConstants &constants)
{
    resetState();
    m_constants= constants;

    return true;
}

bool OrientationFilter::init(const OrientationFilterConstants &constants, const Eigen::Quaternionf &initial_orientation)
{
	resetState();
	m_constants = constants;
	m_state->orientation = initial_orientation;
	m_state->bIsValid = true;
	m_interpolationState->last_orientation = initial_orientation;
	m_interpolationState->bIsValid = true;

	return true;
}

Eigen::Quaternionf OrientationFilter::getOrientation(float time, float offset_x, float offset_y, float offset_z, float offset_world_x, float offset_world_y, float offset_world_z) const
{
	Eigen::Quaternionf result = Eigen::Quaternionf::Identity();

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

		Eigen::Quaternionf predicted_orientation = orientation;

		if (fabsf(time) > k_real_epsilon)
		{
			const Eigen::Quaternionf &quaternion_derivative =
				eigen_angular_velocity_to_quaternion_derivative(orientation, ang_velocity);

			predicted_orientation = Eigen::Quaternionf(
				orientation.coeffs()
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

Eigen::Quaternionf OrientationFilter::getResetOrientation() const
{
	return m_state->reset_orientation;
}

Eigen::Vector3f OrientationFilter::getAngularVelocityRadPerSec() const
{
    return m_state->bIsValid ? m_state->angular_velocity : Eigen::Vector3f::Zero();
}

Eigen::Vector3f OrientationFilter::getAngularAccelerationRadPerSecSqr() const
{
    return m_state->bIsValid ? m_state->angular_acceleration : Eigen::Vector3f::Zero();
}

// -- OrientationFilterPassThru --
void OrientationFilterPassThru::update(
	const t_high_resolution_timepoint timestamp, 
	const PoseFilterPacket &packet)
{
	m_interpolationState->apply_timestamp_state(timestamp);

	if (packet.has_optical_measurement() && packet.isSynced)
	{
		const Eigen::Quaternionf &new_orientation= packet.optical_orientation;

		m_state->apply_optical_state(new_orientation, timestamp);
		m_interpolationState->apply_interpolation_state(new_orientation, timestamp);
	}
}

void OrientationFilterMadgwickARG::resetState()
{
	OrientationFilterComplementaryMARG::resetState();
	m_smartOrientation = Eigen::Quaternionf::Identity();
	m_smartReset = false;
	m_smartResetTime = 0.0f;
}

// -- OrientationFilterMadgwickARG --
// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
void OrientationFilterMadgwickARG::update(
	const t_high_resolution_timepoint timestamp,
	const PoseFilterPacket &packet)
{
	float optical_delta_time = m_state->getOpticalTime(timestamp);
	float imu_delta_time = m_state->getImuTime(timestamp);

	m_interpolationState->apply_timestamp_state(timestamp);

	float filter_madgwick_beta = k_madgwick_beta;
	bool filter_madgwick_stabilization = false;
	float filter_madgwick_stabilization_min_beta = k_madgwick_stabil_min_beta;
	float filter_madgwick_stabilization_smoothing_factor = k_madgwick_beta_smoothing_factor;
	float velocity_smoothing_factor = k_lowpass_velocity_smoothing_factor;
	float angular_prediction_cutoff = k_adaptive_prediction_cutoff;
	bool filter_madgwick_smart_correct = true;
	bool filter_madgwick_smart_instant = true;

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

				filter_madgwick_beta = config.filter_madgwick_beta;
				filter_madgwick_stabilization = config.filter_madgwick_stabilization;
				filter_madgwick_stabilization_min_beta = config.filter_madgwick_stabilization_min_beta;
				filter_madgwick_stabilization_smoothing_factor = config.filter_madgwick_stabilization_smoothing_factor;
				velocity_smoothing_factor = config.filter_angular_smoothing_factor;
				angular_prediction_cutoff = config.filter_angular_prediction_cutoff;
				filter_madgwick_smart_correct = config.filter_madgwick_smart_correct;
				filter_madgwick_smart_instant = config.filter_madgwick_smart_instant;

				break;
			}
			case CommonDeviceState::PSDualShock4:
			{
				PSDualShock4Controller *controller = ControllerView->castChecked<PSDualShock4Controller>();
				PSDualShock4ControllerConfig config = *controller->getConfig();

				filter_madgwick_beta = config.filter_madgwick_beta;
				filter_madgwick_stabilization = config.filter_madgwick_stabilization;
				filter_madgwick_stabilization_min_beta = config.filter_madgwick_stabilization_min_beta;
				filter_madgwick_stabilization_smoothing_factor = config.filter_madgwick_stabilization_smoothing_factor;
				velocity_smoothing_factor = config.filter_angular_smoothing_factor;
				angular_prediction_cutoff = config.filter_angular_prediction_cutoff;
				filter_madgwick_smart_correct = config.filter_madgwick_smart_correct;
				filter_madgwick_smart_instant = config.filter_madgwick_smart_instant;

				break;
			}
			}
		}
	}

	if (packet.hmdDeviceId > -1)
	{
		ServerHMDViewPtr HmdView = DeviceManager::getInstance()->getHMDViewPtr(packet.hmdDeviceId);
		if (HmdView != nullptr && HmdView->getIsOpen())
		{
			switch (HmdView->getHMDDeviceType())
			{
			case CommonDeviceState::Morpheus:
			{
				MorpheusHMD *hmd = HmdView->castChecked<MorpheusHMD>();
				MorpheusHMDConfig config = *hmd->getConfig();

				filter_madgwick_beta = config.filter_madgwick_beta;
				filter_madgwick_stabilization = config.filter_madgwick_stabilization;
				filter_madgwick_stabilization_min_beta = config.filter_madgwick_stabilization_min_beta;
				filter_madgwick_stabilization_smoothing_factor = config.filter_madgwick_stabilization_smoothing_factor;
				velocity_smoothing_factor = config.filter_angular_smoothing_factor;
				angular_prediction_cutoff = config.filter_angular_prediction_cutoff;

				break;
			}
			}
		}
	}
#endif

	// Clamp everything to safety
	filter_madgwick_beta = clampf(filter_madgwick_beta, 0.0f, 1.0f);
	filter_madgwick_stabilization_min_beta = clampf(filter_madgwick_stabilization_min_beta, 0.0f, 1.0f);
	filter_madgwick_stabilization_smoothing_factor = clampf(filter_madgwick_stabilization_smoothing_factor, 0.01f, 1.0f);
	velocity_smoothing_factor = clampf(velocity_smoothing_factor, 0.01f, 1.0f);
	angular_prediction_cutoff = clampf(angular_prediction_cutoff, 0.0f, (1 << 16));

	if (packet.has_imu_measurements())
	{
		const Eigen::Vector3f &current_omega= packet.imu_gyroscope_rad_per_sec;

		Eigen::Vector3f current_g= packet.imu_accelerometer_g_units;
		eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

		// Current orientation from earth frame to sensor frame
		const Eigen::Quaternionf SEq = m_state->orientation;
		const Eigen::Quaternionf SEq_smart = m_smartOrientation;

		Eigen::Quaternionf corrected_omega;
		Eigen::Quaternionf corrected_smart_omega;
		Eigen::Quaternionf SEq_new = SEq;
		Eigen::Quaternionf SEq_smart_new = SEq_smart;

		{ // Main Madgwick
			// Compute the quaternion derivative measured by gyroscopes
			// Eqn 12) q_dot = 0.5*q*omega
			Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
			Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) *omega;

			if (!current_g.isApprox(Eigen::Vector3f::Zero(), k_normal_epsilon))
			{
				// Get the direction of the gravitational fields in the identity pose		
				Eigen::Vector3f k_identity_g_direction = m_constants.gravity_calibration_direction;

				// Eqn 15) Applied to the gravity vector
				// Fill in the 3x1 objective function matrix f(SEq, Sa) =|f_g|
				Eigen::Matrix<float, 3, 1> f_g;
				eigen_alignment_compute_objective_vector(SEq, k_identity_g_direction, current_g, f_g, NULL);

				// Eqn 21) Applied to the gravity vector
				// Fill in the 4x3 objective function Jacobian matrix: J_gb(SEq)= [J_g]
				Eigen::Matrix<float, 4, 3> J_g;
				eigen_alignment_compute_objective_jacobian(SEq, k_identity_g_direction, J_g);

				// Eqn 34) gradient_F= J_g(SEq)*f(SEq, Sa)
				// Compute the gradient of the objective function
				Eigen::Matrix<float, 4, 1> gradient_f = J_g * f_g;
				Eigen::Quaternionf SEqHatDot =
					Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

				// normalize the gradient
				eigen_quaternion_normalize_with_default(SEqHatDot, *k_eigen_quaternion_zero);

				if (m_smartReset)
				{
					m_beta = 0.9f;
				}
				else
				{
					if (filter_madgwick_stabilization)
					{
						const float gyro_b = ((fminf(fminf(abs(current_omega.x()), abs(current_omega.y())), abs(current_omega.z())))) * k_radians_to_degreees;
						const float gyro_multi = clampf((gyro_b - k_madgwick_gyro_min_deg) / k_madgwick_gyro_max_deg, 0.0f, 1.0f);

						const float new_beta = clampf(filter_madgwick_beta * gyro_multi, filter_madgwick_stabilization_min_beta, 1.0f);
						const float filtered_beta = filter_madgwick_stabilization_smoothing_factor * new_beta + (1.f - filter_madgwick_stabilization_smoothing_factor) * m_beta;

						m_beta = filtered_beta;
					}
					else
					{
						m_beta = filter_madgwick_beta;
					}
				}

				// Compute the estimated quaternion rate of change
				// Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot
				Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()*m_beta);

				// Compute then integrate the estimated quaternion rate
				// Eqn 42) SEq_new = SEq + SEqDot_est*imu_delta_time
				SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*imu_delta_time);
			}
			else
			{
				SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_omega.coeffs()*imu_delta_time);
			}

			// Make sure the net quaternion is a pure rotation quaternion
			SEq_new.normalize();
		}

		if (filter_madgwick_smart_correct)
		{
			{ // Smart Madgwick
			  // Compute the quaternion derivative measured by gyroscopes
			  // Eqn 12) q_dot = 0.5*q*omega
				Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
				Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) *omega;

				if (!current_g.isApprox(Eigen::Vector3f::Zero(), k_normal_epsilon))
				{
					// Get the direction of the gravitational fields in the identity pose		
					Eigen::Vector3f k_identity_g_direction = m_constants.gravity_calibration_direction;

					// Eqn 15) Applied to the gravity vector
					// Fill in the 3x1 objective function matrix f(SEq, Sa) =|f_g|
					Eigen::Matrix<float, 3, 1> f_g;
					eigen_alignment_compute_objective_vector(SEq, k_identity_g_direction, current_g, f_g, NULL);

					// Eqn 21) Applied to the gravity vector
					// Fill in the 4x3 objective function Jacobian matrix: J_gb(SEq)= [J_g]
					Eigen::Matrix<float, 4, 3> J_g;
					eigen_alignment_compute_objective_jacobian(SEq, k_identity_g_direction, J_g);

					// Eqn 34) gradient_F= J_g(SEq)*f(SEq, Sa)
					// Compute the gradient of the objective function
					Eigen::Matrix<float, 4, 1> gradient_f = J_g * f_g;
					Eigen::Quaternionf SEqHatDot =
						Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

					// normalize the gradient
					eigen_quaternion_normalize_with_default(SEqHatDot, *k_eigen_quaternion_zero);

					// Compute the estimated quaternion rate of change
					// Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot
					Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()*0.9);

					// Compute then integrate the estimated quaternion rate
					// Eqn 42) SEq_smart_new = SEq + SEqDot_est*imu_delta_time
					SEq_smart_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*imu_delta_time);
				}
				else
				{
					SEq_smart_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_omega.coeffs()*imu_delta_time);
				}

				// Make sure the net quaternion is a pure rotation quaternion
				SEq_smart_new.normalize();
			}

			const Eigen::Vector3f &world_g = -packet.world_accelerometer;
			const float accel_g = sqrtf(world_g.x() * world_g.x() + world_g.y() * world_g.y() + world_g.z() * world_g.z());
			
			if (m_smartReset)
			{
				// Stop when deviation is too small
				if ((eigen_quaternion_unsigned_angle_between(SEq_smart_new, SEq_new) * k_radians_to_degreees) < k_madgwick_smart_correct_min_deg)
				{
					m_smartReset = false;
				}

				// Stop if gravity is unstable
				if (accel_g > k_madgwick_smart_correct_gravity_stable && accel_g < 1.f + (1.f - k_madgwick_smart_correct_gravity_stable))
				{
					m_smartReset = false;
				}
			}
			else
			{
				if ((eigen_quaternion_unsigned_angle_between(SEq_smart_new, SEq_new) * k_radians_to_degreees) > k_madgwick_smart_correct_max_deg)
				{
					// Make sure smart madgwick its kind of stable
					if (accel_g > k_madgwick_smart_correct_gravity_stable && accel_g < 1.f + (1.f - k_madgwick_smart_correct_gravity_stable))
					{
						m_smartResetTime += imu_delta_time;

						if (m_smartResetTime > (k_madgwick_smart_correct_reset_time_ms / 1000.f))
						{
							if (filter_madgwick_smart_instant)
							{
								SEq_new = SEq_smart_new;
								m_smartResetTime = 0.0f;
							}
							else
							{
								m_smartReset = true;
							}
						}
					}
					else
					{
						if (m_smartResetTime > 0.0f)
							m_smartResetTime = 0.0f;
					}
				}
				else
				{
					if (m_smartResetTime > 0.0f)
						m_smartResetTime = 0.0f;
				}
			}
		}
		else
		{
			if (m_smartReset)
				m_smartReset = false;
		}


		// Save the new quaternion and first derivative back into the orientation state
		// Derive the second derivative
		{
			const Eigen::Quaternionf &new_orientation = SEq_new;
			const Eigen::Quaternionf &new_smart_orientation = SEq_smart_new;
			Eigen::Vector3f new_angular_velocity= current_omega;

			// Smooth angular velocity
			new_angular_velocity = lowpass_filter_vector3f(velocity_smoothing_factor, m_state->angular_velocity, new_angular_velocity);

			// Do adapting prediction and smoothing
			if (angular_prediction_cutoff > k_real_epsilon)
			{ 
				const float velocity_speed_sec = new_angular_velocity.norm();
				const float adaptive_time_scale = clampf(velocity_speed_sec / angular_prediction_cutoff, 0.0f, 1.0f);

				new_angular_velocity = new_angular_velocity * adaptive_time_scale;
			}

			const Eigen::Vector3f new_angular_acceleration = (current_omega - m_state->angular_velocity) / imu_delta_time;

			m_smartOrientation = new_smart_orientation;
			m_state->apply_imu_state(new_orientation, new_angular_velocity, new_angular_acceleration, timestamp);
			m_interpolationState->apply_interpolation_state(new_orientation, new_angular_velocity, new_angular_acceleration, timestamp);
		}
	}
}

void OrientationFilterMadgwickARG::recenterOrientation(const Eigen::Quaternionf & q_pose)
{
	Eigen::Quaternionf q_inverse = m_state->orientation.conjugate();

	eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
	m_state->reset_orientation = q_pose*q_inverse;
}

// -- OrientationFilterMadgwickMARG --
// This algorithm comes from Sebastian O.H. Madgwick's 2010 paper:
// "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
// https://www.samba.org/tridge/UAV/madgwick_internal_report.pdf
void OrientationFilterMadgwickMARG::resetState()
{
    OrientationFilterMadgwickARG::resetState();
    m_omega_bias_x= m_omega_bias_y= m_omega_bias_z= 0.f;
}

void OrientationFilterMadgwickMARG::update(
	const t_high_resolution_timepoint timestamp, 
	const PoseFilterPacket &packet)
{
	float optical_delta_time = m_state->getOpticalTime(timestamp);
	float imu_delta_time = m_state->getImuTime(timestamp);

	m_interpolationState->apply_timestamp_state(timestamp);

	float filter_madgwick_beta = k_madgwick_beta;
	bool filter_madgwick_stabilization = false;
	float filter_madgwick_stabilization_min_beta = k_madgwick_stabil_min_beta;
	float filter_madgwick_stabilization_smoothing_factor = k_madgwick_beta_smoothing_factor;
	float velocity_smoothing_factor = k_lowpass_velocity_smoothing_factor;
	float angular_prediction_cutoff = k_adaptive_prediction_cutoff;
	float filter_magnetometer_deviation_cutoff = k_magnetometer_deviation_cutoff;
	bool filter_madgwick_smart_correct = true;
	bool filter_madgwick_smart_instant = true;

	EigenFitEllipsoid mag_ellipsoid;
	mag_ellipsoid.clear();

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

				filter_madgwick_beta = config.filter_madgwick_beta;
				filter_madgwick_stabilization = config.filter_madgwick_stabilization;
				filter_madgwick_stabilization_min_beta = config.filter_madgwick_stabilization_min_beta;
				filter_madgwick_stabilization_smoothing_factor = config.filter_madgwick_stabilization_smoothing_factor;
				velocity_smoothing_factor = config.filter_angular_smoothing_factor;
				angular_prediction_cutoff = config.filter_angular_prediction_cutoff;
				filter_magnetometer_deviation_cutoff = config.filter_magnetometer_deviation_cutoff;
				filter_madgwick_smart_correct = config.filter_madgwick_smart_correct;
				filter_madgwick_smart_instant = config.filter_madgwick_smart_instant;

				config.getMagnetometerEllipsoid(&mag_ellipsoid);

				break;
			}
			case CommonDeviceState::PSDualShock4:
			{
				PSDualShock4Controller *controller = ControllerView->castChecked<PSDualShock4Controller>();
				PSDualShock4ControllerConfig config = *controller->getConfig();

				filter_madgwick_beta = config.filter_madgwick_beta;
				filter_madgwick_stabilization = config.filter_madgwick_stabilization;
				filter_madgwick_stabilization_min_beta = config.filter_madgwick_stabilization_min_beta;
				filter_madgwick_stabilization_smoothing_factor = config.filter_madgwick_stabilization_smoothing_factor;
				velocity_smoothing_factor = config.filter_angular_smoothing_factor;
				angular_prediction_cutoff = config.filter_angular_prediction_cutoff;
				filter_madgwick_smart_correct = config.filter_madgwick_smart_correct;
				filter_madgwick_smart_instant = config.filter_madgwick_smart_instant;

				break;
			}
			}
		}
	}

	if (packet.hmdDeviceId > -1)
	{
		ServerHMDViewPtr HmdView = DeviceManager::getInstance()->getHMDViewPtr(packet.hmdDeviceId);
		if (HmdView != nullptr && HmdView->getIsOpen())
		{
			switch (HmdView->getHMDDeviceType())
			{
			case CommonDeviceState::Morpheus:
			{
				MorpheusHMD *hmd = HmdView->castChecked<MorpheusHMD>();
				MorpheusHMDConfig config = *hmd->getConfig();

				filter_madgwick_beta = config.filter_madgwick_beta;
				filter_madgwick_stabilization = config.filter_madgwick_stabilization;
				filter_madgwick_stabilization_min_beta = config.filter_madgwick_stabilization_min_beta;
				filter_madgwick_stabilization_smoothing_factor = config.filter_madgwick_stabilization_smoothing_factor;
				velocity_smoothing_factor = config.filter_angular_smoothing_factor;
				angular_prediction_cutoff = config.filter_angular_prediction_cutoff;
				filter_madgwick_smart_correct = config.filter_madgwick_smart_correct;
				filter_madgwick_smart_instant = config.filter_madgwick_smart_instant;

				break;
			}
			}
		}
	}
#endif

	// Clamp everything to safety
	filter_madgwick_beta = clampf(filter_madgwick_beta, 0.0f, 1.0f);
	filter_madgwick_stabilization_min_beta = clampf(filter_madgwick_stabilization_min_beta, 0.0f, 1.0f);
	filter_madgwick_stabilization_smoothing_factor = clampf(filter_madgwick_stabilization_smoothing_factor, 0.01f, 1.0f);
	velocity_smoothing_factor = clampf(velocity_smoothing_factor, 0.01f, 1.0f);
	angular_prediction_cutoff = clampf(angular_prediction_cutoff, 0.0f, (1 << 16));
	filter_magnetometer_deviation_cutoff = clampf(filter_magnetometer_deviation_cutoff, 0.0f, 1.0f);

	if (packet.has_imu_measurements())
	{
		const Eigen::Vector3f &current_omega= packet.imu_gyroscope_rad_per_sec;

		Eigen::Vector3f current_g= packet.imu_accelerometer_g_units;
		eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

		Eigen::Vector3f current_m= packet.imu_magnetometer_unit;
		eigen_vector3f_normalize_with_default(current_m, Eigen::Vector3f::Zero());

		// If there isn't a valid magnetometer or accelerometer vector, fall back to the IMU style update
		if (current_g.isZero(k_normal_epsilon) || current_m.isZero(k_normal_epsilon))
		{
			OrientationFilterMadgwickARG::update(timestamp, packet);
			return;
		}

		const CommonRawDeviceVector current_raw_mag = packet.raw_imu_magnetometer;

		if (!magnetometer_in_bounds_vector(
			current_raw_mag, 
			mag_ellipsoid,
			m_mag_scale, 
			0.8f, 
			filter_magnetometer_deviation_cutoff))
		{
			OrientationFilterMadgwickARG::update(timestamp, packet);
			return;
		}

		// Current orientation from earth frame to sensor frame
		const Eigen::Quaternionf SEq = m_state->orientation;
		const Eigen::Quaternionf SEq_smart = m_smartOrientation;

		Eigen::Quaternionf corrected_omega;
		Eigen::Quaternionf corrected_smart_omega;
		Eigen::Quaternionf SEq_new = SEq;
		Eigen::Quaternionf SEq_smart_new = SEq_smart;

		{ // Main Madgwick

			// Get the direction of the magnetic fields in the identity pose.	
			// NOTE: In the original paper we converge on this vector over time automatically (See Eqn 45 & 46)
			// but since we've already done the work in calibration to get this vector, let's just use it.
			// This also removes the last assumption in this function about what 
			// the orientation of the identity-pose is (handled by the sensor transform).
			Eigen::Vector3f k_identity_m_direction = m_constants.magnetometer_calibration_direction;

			// Get the direction of the gravitational fields in the identity pose
			Eigen::Vector3f k_identity_g_direction = m_constants.gravity_calibration_direction;

			// Eqn 15) Applied to the gravity and magnetometer vectors
			// Fill in the 6x1 objective function matrix f(SEq, Sa, Eb, Sm) =|f_g|
			//                                                               |f_b|
			Eigen::Matrix<float, 3, 1> f_g;
			eigen_alignment_compute_objective_vector(SEq, k_identity_g_direction, current_g, f_g, NULL);

			Eigen::Matrix<float, 3, 1> f_m;
			eigen_alignment_compute_objective_vector(SEq, k_identity_m_direction, current_m, f_m, NULL);

			Eigen::Matrix<float, 6, 1> f_gb;
			f_gb.block<3, 1>(0, 0) = f_g;
			f_gb.block<3, 1>(3, 0) = f_m;

			// Eqn 21) Applied to the gravity and magnetometer vectors
			// Fill in the 4x6 objective function Jacobian matrix: J_gb(SEq, Eb)= [J_g|J_b]
			Eigen::Matrix<float, 4, 3> J_g;
			eigen_alignment_compute_objective_jacobian(SEq, k_identity_g_direction, J_g);

			Eigen::Matrix<float, 4, 3> J_m;
			eigen_alignment_compute_objective_jacobian(SEq, k_identity_m_direction, J_m);

			Eigen::Matrix<float, 4, 6> J_gb;
			J_gb.block<4, 3>(0, 0) = J_g; J_gb.block<4, 3>(0, 3) = J_m;

			// Eqn 34) gradient_F= J_gb(SEq, Eb)*f(SEq, Sa, Eb, Sm)
			// Compute the gradient of the objective function
			Eigen::Matrix<float, 4, 1> gradient_f = J_gb*f_gb;
			Eigen::Quaternionf SEqHatDot =
				Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

			// normalize the gradient to estimate direction of the gyroscope error
			eigen_quaternion_normalize_with_default(SEqHatDot, *k_eigen_quaternion_zero);

			// Eqn 47) omega_err= 2*SEq*SEqHatDot
			// compute angular estimated direction of the gyroscope error
			Eigen::Quaternionf omega_err = Eigen::Quaternionf(SEq.coeffs()*2.f) * SEqHatDot;

			// Eqn 49) omega_corrected = omega - net_omega_bias
			Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
			corrected_omega = Eigen::Quaternionf(omega.coeffs() /*- omega_bias.coeffs()*/);

			// Compute the rate of change of the orientation purely from the gyroscope
			// Eqn 12) q_dot = 0.5*q*omega
			Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq.coeffs() * 0.5f) * corrected_omega;

			if (m_smartReset)
			{
				m_beta = 0.9f;
			}
			else
			{
				if (filter_madgwick_stabilization)
				{
					const float gyro_b = ((fminf(fminf(abs(current_omega.x()), abs(current_omega.y())), abs(current_omega.z())))) * k_radians_to_degreees;
					const float gyro_multi = clampf((gyro_b - k_madgwick_gyro_min_deg) / k_madgwick_gyro_max_deg, 0.0f, 1.0f);

					const float new_beta = clampf(filter_madgwick_beta * gyro_multi, filter_madgwick_stabilization_min_beta, 1.0f);
					const float filtered_beta = filter_madgwick_stabilization_smoothing_factor * new_beta + (1.f - filter_madgwick_stabilization_smoothing_factor) * m_beta;

					m_beta = filtered_beta;
				}
				else
				{
					m_beta = filter_madgwick_beta;
				}
			}

			// Compute the estimated quaternion rate of change
			// Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot
			Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()*m_beta);

			// Compute then integrate the estimated quaternion rate
			// Eqn 42) SEq_new = SEq + SEqDot_est*delta_t
			SEq_new = Eigen::Quaternionf(SEq.coeffs() + SEqDot_est.coeffs()*imu_delta_time);

			// Make sure the net quaternion is a pure rotation quaternion
			SEq_new.normalize();
		}

		if (filter_madgwick_smart_correct)
		{
			{ // Smart Madgwick

			  // Get the direction of the magnetic fields in the identity pose.	
			  // NOTE: In the original paper we converge on this vector over time automatically (See Eqn 45 & 46)
			  // but since we've already done the work in calibration to get this vector, let's just use it.
			  // This also removes the last assumption in this function about what 
			  // the orientation of the identity-pose is (handled by the sensor transform).
				Eigen::Vector3f k_identity_m_direction = m_constants.magnetometer_calibration_direction;

				// Get the direction of the gravitational fields in the identity pose
				Eigen::Vector3f k_identity_g_direction = m_constants.gravity_calibration_direction;

				// Eqn 15) Applied to the gravity and magnetometer vectors
				// Fill in the 6x1 objective function matrix f(SEq_smart, Sa, Eb, Sm) =|f_g|
				//                                                               |f_b|
				Eigen::Matrix<float, 3, 1> f_g;
				eigen_alignment_compute_objective_vector(SEq_smart, k_identity_g_direction, current_g, f_g, NULL);

				Eigen::Matrix<float, 3, 1> f_m;
				eigen_alignment_compute_objective_vector(SEq_smart, k_identity_m_direction, current_m, f_m, NULL);

				Eigen::Matrix<float, 6, 1> f_gb;
				f_gb.block<3, 1>(0, 0) = f_g;
				f_gb.block<3, 1>(3, 0) = f_m;

				// Eqn 21) Applied to the gravity and magnetometer vectors
				// Fill in the 4x6 objective function Jacobian matrix: J_gb(SEq_smart, Eb)= [J_g|J_b]
				Eigen::Matrix<float, 4, 3> J_g;
				eigen_alignment_compute_objective_jacobian(SEq_smart, k_identity_g_direction, J_g);

				Eigen::Matrix<float, 4, 3> J_m;
				eigen_alignment_compute_objective_jacobian(SEq_smart, k_identity_m_direction, J_m);

				Eigen::Matrix<float, 4, 6> J_gb;
				J_gb.block<4, 3>(0, 0) = J_g; J_gb.block<4, 3>(0, 3) = J_m;

				// Eqn 34) gradient_F= J_gb(SEq_smart, Eb)*f(SEq_smart, Sa, Eb, Sm)
				// Compute the gradient of the objective function
				Eigen::Matrix<float, 4, 1> gradient_f = J_gb*f_gb;
				Eigen::Quaternionf SEqHatDot =
					Eigen::Quaternionf(gradient_f(0, 0), gradient_f(1, 0), gradient_f(2, 0), gradient_f(3, 0));

				// normalize the gradient to estimate direction of the gyroscope error
				eigen_quaternion_normalize_with_default(SEqHatDot, *k_eigen_quaternion_zero);

				// Eqn 47) omega_err= 2*SEq_smart*SEqHatDot
				// compute angular estimated direction of the gyroscope error
				Eigen::Quaternionf omega_err = Eigen::Quaternionf(SEq_smart.coeffs()*2.f) * SEqHatDot;

				// Eqn 49) omega_corrected = omega - net_omega_bias
				Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
				corrected_smart_omega = Eigen::Quaternionf(omega.coeffs() /*- omega_bias.coeffs()*/);

				// Compute the rate of change of the orientation purely from the gyroscope
				// Eqn 12) q_dot = 0.5*q*omega
				Eigen::Quaternionf SEqDot_omega = Eigen::Quaternionf(SEq_smart.coeffs() * 0.5f) * corrected_smart_omega;

				// Compute the estimated quaternion rate of change
				// Eqn 43) SEq_est = SEqDot_omega - beta*SEqHatDot
				Eigen::Quaternionf SEqDot_est = Eigen::Quaternionf(SEqDot_omega.coeffs() - SEqHatDot.coeffs()* 0.9f);

				// Compute then integrate the estimated quaternion rate
				// Eqn 42) SEq_smart_new = SEq_smart + SEqDot_est*delta_t
				SEq_smart_new = Eigen::Quaternionf(SEq_smart.coeffs() + SEqDot_est.coeffs()*imu_delta_time);

				// Make sure the net quaternion is a pure rotation quaternion
				SEq_smart_new.normalize();
			}

			const Eigen::Vector3f &world_g = -packet.world_accelerometer;
			const float accel_g = sqrtf(world_g.x() * world_g.x() + world_g.y() * world_g.y() + world_g.z() * world_g.z());
			
			if (m_smartReset)
			{
				// Stop when deviation is too small
				if ((eigen_quaternion_unsigned_angle_between(SEq_smart_new, SEq_new) * k_radians_to_degreees) < k_madgwick_smart_correct_min_deg)
				{
					m_smartReset = false;
				}

				// Stop if gravity is unstable
				if (accel_g > k_madgwick_smart_correct_gravity_stable && accel_g < 1.f + (1.f - k_madgwick_smart_correct_gravity_stable))
				{
					m_smartReset = false;
				}
			}
			else
			{
				if ((eigen_quaternion_unsigned_angle_between(SEq_smart_new, SEq_new) * k_radians_to_degreees) > k_madgwick_smart_correct_max_deg)
				{
					// Make sure smart madgwick its kind of stable
					if (accel_g > k_madgwick_smart_correct_gravity_stable && accel_g < 1.f + (1.f - k_madgwick_smart_correct_gravity_stable))
					{
						m_smartResetTime += imu_delta_time;

						if (m_smartResetTime > (k_madgwick_smart_correct_reset_time_ms / 1000.f))
						{
							if (filter_madgwick_smart_instant)
							{
								SEq_new = SEq_smart_new;
								m_smartResetTime = 0.0f;
							}
							else
							{
								m_smartReset = true;
							}
						}
					}
					else
					{
						if (m_smartResetTime > 0.0f)
							m_smartResetTime = 0.0f;
					}
				}
				else
				{
					if (m_smartResetTime > 0.0f)
						m_smartResetTime = 0.0f;
				}
			}
		}
		else
		{
			if (m_smartReset)
				m_smartReset = false;
		}
		

		// Save the new quaternion and first derivative back into the orientation state
		// Derive the second derivative
		{
			const Eigen::Quaternionf &new_orientation = SEq_new;
			const Eigen::Quaternionf &new_smart_orientation = SEq_smart_new;
			Eigen::Vector3f new_angular_velocity = Eigen::Vector3f(corrected_omega.x(), corrected_omega.y(), corrected_omega.z());

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

			m_smartOrientation = new_smart_orientation;
			m_state->apply_imu_state(new_orientation, new_angular_velocity, new_angular_acceleration, timestamp);
			m_interpolationState->apply_interpolation_state(new_orientation, new_angular_velocity, new_angular_acceleration, timestamp);
		}
	}
}

// -- OrientationFilterComplementaryOpticalARG --
#define COMPLEMENTARY_FILTER_YAW_ONLY_BLEND 0
void OrientationFilterComplementaryOpticalARG::update(
	const t_high_resolution_timepoint timestamp, 
	const PoseFilterPacket &packet)
{
	float optical_delta_time = m_state->getOpticalTime(timestamp);
	float imu_delta_time = m_state->getImuTime(timestamp);

	m_interpolationState->apply_timestamp_state(timestamp);

	// Blend with optical yaw
	if (packet.has_optical_measurement() && packet.isSynced)
    {
		Eigen::Quaternionf new_orientation;

		if (m_state->bIsValid)
		{
			Eigen::Quaternionf optical_orientation= Eigen::Quaternionf::Identity();
			float optical_weight= 0.f;

			if (m_constants.tracking_shape.shape_type == Sphere)
			{
				// Estimate the orientation of the controller 
				// by leveraging the fact that the bulb is offset from the IMU location
				CommonDevicePosition sphere_offset= m_constants.tracking_shape.shape.sphere.center_cm;
				if (getIsStateValid() && 
					(fabsf(sphere_offset.x) > 0 || fabsf(sphere_offset.y) > 0 || fabsf(sphere_offset.z) > 0))
				{
					// TODO: This assumes that the tracking sphere is 
					// aligned along the +Z axis from the IMU

					// Get the basis vectors for the last estimated IMU orientation
					const Eigen::Matrix3f imu_estimated_basis= m_state->orientation.toRotationMatrix();
					const Eigen::Vector3f imu_estimated_XAxis= imu_estimated_basis.col(0);
					const Eigen::Vector3f imu_estimated_YAxis= imu_estimated_basis.col(1);
					const Eigen::Vector3f imu_estimated_ZAxis= imu_estimated_basis.col(2);

					// Compute where the IMU position should be
					// using the last estimated bulb position
					const Eigen::Vector3f imu_estimated_position_cm=
						packet.current_position_cm // last estimated bulb position
						//- imu_estimated_XAxis*sphere_offset.x
						//- imu_estimated_YAxis*sphere_offset.y
						- imu_estimated_ZAxis*sphere_offset.z;

					// Compute an orientation aligned along the vector from the 
					// last estimated IMU position to the current bulb position
					const Eigen::Vector3f optical_ZAxis= packet.optical_position_cm - imu_estimated_position_cm;
					const Eigen::Vector3f optical_YAxis= optical_ZAxis.cross(imu_estimated_XAxis);
					optical_orientation= eigen_quaternion_from_ZY(optical_ZAxis, optical_YAxis);

					// Use the positional variance as the quality measure
					const float optical_variance= 
						m_constants.position_variance_curve.evaluate(packet.tracking_projection_area_px_sqr);
					const float fraction_of_max_orientation_variance =
						safe_divide_with_default(
							optical_variance,
							m_constants.position_variance_curve.MaxValue,
							1.f);
					const float optical_orientation_quality = clampf01(1.f - fraction_of_max_orientation_variance);

					optical_weight= 
						lerp_clampf(0, k_max_optical_orientation_weight, optical_orientation_quality);
				}
			}
			else
			{
				// Use the optical orientation fed into the filter packet
				optical_orientation= packet.optical_orientation;

				// Use the orientation variance as the quality measure
				const float optical_variance= 
					m_constants.orientation_variance_curve.evaluate(packet.tracking_projection_area_px_sqr);
				const float fraction_of_max_orientation_variance =
					safe_divide_with_default(
						optical_variance,
						m_constants.orientation_variance_curve.MaxValue,
						1.f);
				const float optical_orientation_quality = clampf01(1.f - fraction_of_max_orientation_variance);
				
				optical_weight= 
					lerp_clampf(0, k_max_optical_orientation_weight, optical_orientation_quality);
			}
        
			static float g_weight_override= -1.f;
			if (g_weight_override >= 0.f)
			{
				optical_weight= g_weight_override;
			}

			// Blend between the state's orientation and incoming optical orientation
			#if COMPLEMENTARY_FILTER_YAW_ONLY_BLEND
			Eigen::Quaternionf optical_yaw, optical_twist;
			eigen_quaternionf_to_swing_twist(
				packet.optical_orientation, Eigen::Vector3f(0.f, 1.f, 0.f), 
				optical_yaw, optical_twist);
			Eigen::Quaternionf optical_test= optical_yaw * optical_twist;

			Eigen::Quaternionf SEq_new_yaw, SEq_new_twist;
			eigen_quaternionf_to_swing_twist(
				SEq_new, Eigen::Vector3f(0.f, 1.f, 0.f), 
				SEq_new_yaw, SEq_new_twist);
			Eigen::Quaternionf SEq_new_test= SEq_new_yaw * SEq_new_twist;

			Eigen::Quaternionf blended_swing = 
				eigen_quaternion_normalized_lerp(SEq_new_yaw, optical_yaw, optical_weight);

			// Keep the twist from the filtered orientation
			// but use the blended yaw orientation
			new_orientation = blended_swing * SEq_new_twist;
			#else
			new_orientation= 
				eigen_quaternion_normalized_lerp(
					m_state->orientation, packet.optical_orientation, optical_weight);
			#endif
		}
		else
		{
			// Just use the packets optical orientation if the state is uninitialized
			new_orientation= packet.optical_orientation;
		}

		m_state->apply_optical_state(new_orientation, timestamp);
		m_interpolationState->apply_interpolation_state(new_orientation, timestamp);
    }

    OrientationFilterMadgwickARG::update(timestamp, packet);
}

// -- OrientationFilterComplementaryMARG --
void OrientationFilterComplementaryMARG::resetState()
{
    OrientationFilter::resetState();
}

static Eigen::Vector3f threshold_vector3f(const Eigen::Vector3f &vector, const float min_length)
{
	const float length = vector.norm();
	const Eigen::Vector3f result = (length > min_length) ? vector : Eigen::Vector3f::Zero();

	return result;
}

static Eigen::Vector3f lowpass_filter_vector3f(
	const float alpha,
	const Eigen::Vector3f &old_filtered_vector,
	const Eigen::Vector3f &new_vector)
{
	const Eigen::Vector3f filtered_vector = alpha*new_vector + (1.f - alpha)*old_filtered_vector;

	return filtered_vector;
}

static Eigen::Vector3f lowpass_filter_vector3f(
	const float delta_time,
	const float cutoff_frequency,
	const Eigen::Vector3f &old_filtered_vector,
	const Eigen::Vector3f &new_vector)
{
	// https://en.wikipedia.org/wiki/Low-pass_filter
	const float RC = 1.f / (k_real_two_pi*cutoff_frequency);
	const float alpha = clampf01(delta_time / (RC + delta_time));
	const Eigen::Vector3f filtered_vector = lowpass_filter_vector3f(alpha, old_filtered_vector, new_vector);

	return filtered_vector;
}

static bool magnetometer_in_bounds_vector(
	const CommonRawDeviceVector raw_mag_int,
	const EigenFitEllipsoid &ellipsoid,
	float &m_mag_scale,
	const float smoothing,
	const float diviation)
{
	Eigen::Vector3f raw_mag = Eigen::Vector3f(
		static_cast<float>(raw_mag_int.i),
		static_cast<float>(raw_mag_int.j),
		static_cast<float>(raw_mag_int.k));

	return magnetometer_in_bounds_vector3f(raw_mag, ellipsoid, m_mag_scale, smoothing, diviation);
}

static bool magnetometer_in_bounds_vector3f(
	const Eigen::Vector3f raw_mag,
	const EigenFitEllipsoid &ellipsoid,
	float &m_mag_scale,
	const float smoothing,
	const float diviation)
{
	Eigen::Vector3f ellipsoid_sample =
		eigen_alignment_project_point_on_ellipsoid_basis(raw_mag, ellipsoid);

	const float ellipsoid_sample_q = sqrtf(
		ellipsoid_sample.x() * ellipsoid_sample.x() +
		ellipsoid_sample.y() * ellipsoid_sample.y() +
		ellipsoid_sample.z() * ellipsoid_sample.z());

	m_mag_scale = (smoothing*ellipsoid_sample_q + (1.f - smoothing)*m_mag_scale);

	return (m_mag_scale > (1.f - diviation) && m_mag_scale < 1.f + diviation);
}

void OrientationFilterComplementaryMARG::update(
	const t_high_resolution_timepoint timestamp, 
	const PoseFilterPacket &packet)
{
	float optical_delta_time = m_state->getOpticalTime(timestamp);
	float imu_delta_time = m_state->getImuTime(timestamp);

	m_interpolationState->apply_timestamp_state(timestamp);

	bool filter_enable_magnetometer = true;

	bool filter_use_passive_drift_correction = false;
	PassiveDriftCorrectionMethod filter_passive_drift_correction_method = PassiveDriftCorrectionMethod::StableGravity;
	float filter_passive_drift_correction_deadzone = 3.f;
	float filter_passive_drift_correction_gravity_deadzone = 0.8f;
	float filter_passive_drift_correction_delay = 100.f;

	bool filter_use_stabilization = false;
	float filter_stabilization_min_scale = 0.1f;

	float velocity_smoothing_factor = k_lowpass_velocity_smoothing_factor;
	float angular_prediction_cutoff = k_adaptive_prediction_cutoff;
	float filter_magnetometer_deviation_cutoff = k_magnetometer_deviation_cutoff;

	EigenFitEllipsoid mag_ellipsoid;
	mag_ellipsoid.clear();

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

				filter_enable_magnetometer = config.filter_enable_magnetometer;

				filter_use_passive_drift_correction = config.filter_use_passive_drift_correction;
				filter_passive_drift_correction_method = static_cast<PassiveDriftCorrectionMethod>(config.filter_passive_drift_correction_method);
				filter_passive_drift_correction_deadzone = config.filter_passive_drift_correction_deadzone;
				filter_passive_drift_correction_gravity_deadzone = config.filter_passive_drift_correction_gravity_deadzone;
				filter_passive_drift_correction_delay = config.filter_passive_drift_correction_delay;

				filter_use_stabilization = config.filter_use_stabilization;
				filter_stabilization_min_scale = config.filter_stabilization_min_scale;

				velocity_smoothing_factor = config.filter_angular_smoothing_factor;
				angular_prediction_cutoff = config.filter_angular_prediction_cutoff;
				filter_magnetometer_deviation_cutoff = config.filter_magnetometer_deviation_cutoff;

				config.getMagnetometerEllipsoid(&mag_ellipsoid);

				break;
			}
			}
		}
	}
#endif

	// Clamp everything to safety
	filter_passive_drift_correction_deadzone = clampf(filter_passive_drift_correction_deadzone, 0.0f, (1 << 16));
	filter_passive_drift_correction_gravity_deadzone = clampf(filter_passive_drift_correction_gravity_deadzone, 0.0f, 1.0f);
	filter_passive_drift_correction_delay = clampf(filter_passive_drift_correction_delay, 1.0f, (1 << 16));
	filter_stabilization_min_scale = clampf(filter_stabilization_min_scale, 0.0f, 1.0f);
	velocity_smoothing_factor = clampf(velocity_smoothing_factor, 0.01f, 1.0f);
	angular_prediction_cutoff = clampf(angular_prediction_cutoff, 0.0f, (1 << 16));
	filter_magnetometer_deviation_cutoff = clampf(filter_magnetometer_deviation_cutoff, 0.0f, 1.0f);

	if (packet.has_imu_measurements())
	{
		const Eigen::Vector3f &current_omega = packet.imu_gyroscope_rad_per_sec;

		Eigen::Vector3f current_g = packet.imu_accelerometer_g_units;
		eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

		Eigen::Vector3f current_m = packet.imu_magnetometer_unit;
		eigen_vector3f_normalize_with_default(current_m, Eigen::Vector3f::Zero());


		const CommonRawDeviceVector current_raw_mag = packet.raw_imu_magnetometer;

		if (!magnetometer_in_bounds_vector(
			current_raw_mag, 
			mag_ellipsoid, 
			m_mag_scale, 
			0.8f, 
			filter_magnetometer_deviation_cutoff))
		{
			current_m = Eigen::Vector3f::Zero();
		}

		if (!filter_enable_magnetometer)
		{
			current_m = Eigen::Vector3f::Zero();
		}

		// Get the direction of the magnetic fields in the identity pose.	
		Eigen::Vector3f k_identity_m_direction = m_constants.magnetometer_calibration_direction;

		// Get the direction of the gravitational fields in the identity pose
		Eigen::Vector3f k_identity_g_direction = m_constants.gravity_calibration_direction;

		// Angular Rotation (AR) Update
		//-----------------------------
		// Compute the rate of change of the orientation purely from the gyroscope
		// q_dot = 0.5*q*omega
		Eigen::Quaternionf q_current = m_state->orientation;

		Eigen::Quaternionf q_omega = Eigen::Quaternionf(0.f, current_omega.x(), current_omega.y(), current_omega.z());
		Eigen::Quaternionf q_derivative = Eigen::Quaternionf(q_current.coeffs()*0.5f) * q_omega;

		// Integrate the rate of change to get a new orientation
		// q_new= q + q_dot*dT
		Eigen::Quaternionf q_step = Eigen::Quaternionf(q_derivative.coeffs() * imu_delta_time);
		Eigen::Quaternionf ar_orientation = Eigen::Quaternionf(q_current.coeffs() + q_step.coeffs());

		// Make sure the resulting quaternion is normalized
		ar_orientation.normalize();

		// Magnetic/Gravity (MG) Update
		//-----------------------------
		const Eigen::Vector3f* mg_from[2] = { &k_identity_g_direction, &k_identity_m_direction };
		const Eigen::Vector3f* mg_to[2] = { &current_g, &current_m };
		Eigen::Quaternionf mg_orientation;

		// Always attempt to align with the identity_mg, even if we don't get within the alignment tolerance.
		// More often then not we'll be better off moving forward with what we have and trying to refine
		// the alignment next frame.
		eigen_alignment_quaternion_between_vector_frames(
			mg_from, mg_to, 0.1f, q_current, mg_orientation);

		bool doStabilize = false;

		if (filter_use_passive_drift_correction)
		{
			doStabilize = 
				filter_process_passive_drift_correction(
					imu_delta_time,
					packet,
					filter_use_passive_drift_correction,
					filter_passive_drift_correction_method,
					filter_passive_drift_correction_deadzone,
					filter_passive_drift_correction_gravity_deadzone,
					filter_passive_drift_correction_delay);
		}
		else
		{
			doStabilize = true;
		}

		if (doStabilize)
		{
			// Do stabilization to reduce shaking and quick magnetometer/accelerometer motion
			// Scale by gyro amount
			if (filter_use_stabilization)
			{
				filter_process_stabilization(imu_delta_time, packet, filter_stabilization_min_scale);
			}
			else
			{
				// Update the blend weight
				// -- Exponential blend the MG weight from 1 down to k_base_earth_frame_align_weight
				mg_weight = lerp_clampf(mg_weight, k_base_earth_frame_align_weight, 0.9f);
			}
		}

		// Blending Update
		//----------------
		// Save the new quaternion and first derivative back into the orientation state
		// Derive the second derivative
		{
			// The final rotation is a blend between the integrated orientation and absolute rotation from the earth-frame
			const Eigen::Quaternionf new_orientation =
				eigen_quaternion_normalized_lerp(ar_orientation, mg_orientation, mg_weight);
			Eigen::Vector3f new_angular_velocity = current_omega;

			// Smooth angular velocity
			new_angular_velocity = lowpass_filter_vector3f(velocity_smoothing_factor, m_state->angular_velocity, new_angular_velocity);

			// Do adapting prediction and smoothing
			if (angular_prediction_cutoff > k_real_epsilon)
			{ 
				const float velocity_speed_sec = new_angular_velocity.norm();
				const float adaptive_time_scale = clampf(velocity_speed_sec / angular_prediction_cutoff, 0.0f, 1.0f);

				new_angular_velocity = new_angular_velocity * adaptive_time_scale;
			}

			const Eigen::Vector3f new_angular_acceleration = (current_omega - m_state->angular_velocity) / imu_delta_time;

			m_state->apply_imu_state(new_orientation, new_angular_velocity, new_angular_acceleration, timestamp);
			m_interpolationState->apply_interpolation_state(new_orientation, new_angular_velocity, new_angular_acceleration, timestamp);
		}
	}
}

bool OrientationFilterComplementaryMARG::filter_process_passive_drift_correction(
	const float delta_time, 
	const PoseFilterPacket &packet,
	bool filter_use_passive_drift_correction,
	PassiveDriftCorrectionMethod filter_passive_drift_correction_method,
	float filter_passive_drift_correction_deadzone,
	float filter_passive_drift_correction_gravity_deadzone,
	float filter_passive_drift_correction_delay)
{
	const Eigen::Vector3f &current_omega = packet.imu_gyroscope_rad_per_sec;

	// Gather sensor state from the previous frame
	const Eigen::Vector3f &old_accelerometer = last_accelerometer_g_units;
	const Eigen::Vector3f &old_accelerometer_derivative = last_accelerometer_derivative_g_per_sec;
	const Eigen::Vector3f &old_acceleration = last_acceleration_m_per_sec_sqr;

	// Gather new sensor readings
	// Need to negate the accelerometer reading since it points the opposite direction of gravity)
	const Eigen::Vector3f &new_accelerometer = -packet.world_accelerometer;

	// Compute the filtered derivative of the accelerometer (a.k.a. the "jerk")
	Eigen::Vector3f new_accelerometer_derivative = Eigen::Vector3f::Zero();
	{
		const Eigen::Vector3f accelerometer_derivative =
			(new_accelerometer - old_accelerometer) / delta_time;

		// Apply a decay filter to the jerk
		static float g_jerk_decay = k_jerk_decay;
		new_accelerometer_derivative = accelerometer_derivative*g_jerk_decay;
	}

	// The accelerometer is in "g-units", where 1 g-unit = 9.8m/s^2
	const Eigen::Vector3f old_jerk = old_accelerometer_derivative * k_g_units_to_ms2;
	const Eigen::Vector3f new_jerk = new_accelerometer_derivative * k_g_units_to_ms2;

	// Convert the new acceleration by integrating the jerk
	Eigen::Vector3f new_acceleration =
		0.5f*(old_jerk + new_jerk)*delta_time
		+ old_acceleration;

	// Apply a lowpass filter to the acceleration
	static float g_cutoff_frequency = k_accelerometer_frequency_cutoff;
	new_acceleration =
		lowpass_filter_vector3f(delta_time, g_cutoff_frequency, old_acceleration, new_acceleration);

	// Apply a decay filter to the acceleration
	static float g_acceleration_decay = k_acceleration_decay;
	new_acceleration *= g_acceleration_decay;

	// Save out the updated sensor state
	last_accelerometer_g_units = new_accelerometer;
	last_accelerometer_derivative_g_per_sec = new_accelerometer_derivative;
	last_acceleration_m_per_sec_sqr = new_acceleration;

	std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

	const Eigen::Vector3f &world_g = -packet.world_accelerometer;
	const float accel_g = sqrtf(world_g.x() * world_g.x() + world_g.y() * world_g.y() + world_g.z() * world_g.z());

	bool gravity_stable = false;
	bool gyro_stable = false;
	bool accel_stable = false;
	if (accel_g > filter_passive_drift_correction_gravity_deadzone &&
		accel_g < 1.f + (1.f - filter_passive_drift_correction_gravity_deadzone))
	{
		gravity_stable = true;
	}

	if (abs(current_omega.x()) < filter_passive_drift_correction_deadzone &&
		abs(current_omega.y()) < filter_passive_drift_correction_deadzone &&
		abs(current_omega.z()) < filter_passive_drift_correction_deadzone)
	{
		gyro_stable = true;
	}

	if (abs(new_acceleration.x()) < filter_passive_drift_correction_deadzone &&
		abs(new_acceleration.y()) < filter_passive_drift_correction_deadzone &&
		abs(new_acceleration.z()) < filter_passive_drift_correction_deadzone)
	{
		accel_stable = true;
	}

	switch (filter_passive_drift_correction_method)
	{
	case PassiveDriftCorrectionMethod::StableGravity:
	{
		gyro_stable = true;
		accel_stable = true;
		break;
	}
	case PassiveDriftCorrectionMethod::StableGyroAccel:
	{
		gravity_stable = true;
		break;
	}
	}

	if (gyro_stable && accel_stable && gravity_stable)
	{
		if (mg_ignored)
		{
			std::chrono::duration<double, std::milli> stableDuration = now - timeStableDelay;

			if (stableDuration.count() > filter_passive_drift_correction_delay)
			{
				mg_ignored = false;
			}
		}
		else
		{
			return true;
		}
	}
	else
	{
		timeStableDelay = now;
		mg_ignored = true;
		mg_weight = 0.f;
	}

	return false;
}

void OrientationFilterComplementaryMARG::filter_process_stabilization(
	const float delta_time,
	const PoseFilterPacket &packet,
	float filter_stabilization_min_scale)
{
	const Eigen::Vector3f &current_omega = packet.imu_gyroscope_rad_per_sec;

	const float k_gyro_multi = 1.f;

	float gyro_max = fmaxf(abs(current_omega.x()), fmaxf(abs(current_omega.y()), abs(current_omega.z()))) * k_gyro_multi;

	float align_weight = lerp_clampf(0.f, k_base_earth_frame_align_weight, clampf(gyro_max, clampf(filter_stabilization_min_scale, 0.0f, 1.0f), 1.f));

	// Update the blend weight
	// -- Exponential blend the MG weight from 1 down to k_base_earth_frame_align_weight
	mg_weight = lerp_clampf(mg_weight, align_weight, 0.9f);
}
