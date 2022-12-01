// -- includes -----
#include "OpticalOrientationFilter.h"
#include "MathAlignment.h"
#include "ServerLog.h"
#include <deque>
#include <map>
#include <vector>

// Complementary MARG Filter constants
#define k_base_earth_frame_align_weight 0.02f

// -- private definitions -----
struct OpticalOrientationFilterState
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/// Is the current fusion state valid
	bool bIsValid;

	/* Physics State */
	Eigen::Quaternionf optical_orientation;
	Eigen::Quaternionf imu_orientation;
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
		optical_orientation = Eigen::Quaternionf::Identity();
		imu_orientation = Eigen::Quaternionf::Identity();
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
			imu_orientation = new_orientation;
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
			optical_orientation = new_orientation;
		}
		else
		{
			SERVER_LOG_WARNING("OrientationFilter") << "Orientation is NaN!";
		}

		if (is_valid_float(delta_time))
		{
			time = accumulated_optical_time_delta + (double)delta_time;
			accumulated_optical_time_delta = 0.0;
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
OpticalOrientationFilter::OpticalOrientationFilter() :
	m_state(new OpticalOrientationFilterState)
{
	memset(&m_constants, 0, sizeof(OrientationFilterConstants));
	resetState();
}

OpticalOrientationFilter::~OpticalOrientationFilter()
{
	delete m_state;
}

bool OpticalOrientationFilter::getIsStateValid() const
{
	return m_state->bIsValid;
}

double OpticalOrientationFilter::getTimeInSeconds() const
{
	return m_state->time;
}

void OpticalOrientationFilter::resetState()
{
	m_state->reset();
}

void OpticalOrientationFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
	Eigen::Quaternionf q_inverse = getCombinedOrientation().conjugate();

	eigen_quaternion_normalize_with_default(q_inverse, Eigen::Quaternionf::Identity());
	m_state->reset_orientation = q_pose*q_inverse;
}

bool OpticalOrientationFilter::init(const OrientationFilterConstants &constants)
{
	resetState();
	m_constants = constants;

	return true;
}

bool OpticalOrientationFilter::init(const OrientationFilterConstants &constants, const Eigen::Quaternionf &initial_orientation)
{
	resetState();
	m_constants = constants;
	m_state->imu_orientation = initial_orientation;
	m_state->optical_orientation = initial_orientation;
	m_state->bIsValid = true;

	return true;
}

Eigen::Quaternionf OpticalOrientationFilter::getOrientation(float time, float offset_x, float offset_y, float offset_z) const
{
	Eigen::Quaternionf result = Eigen::Quaternionf::Identity();

	if (m_state->bIsValid)
	{
		Eigen::Quaternionf combined_orientation = getCombinedOrientation();

		Eigen::Quaternionf predicted_orientation = combined_orientation;

		if (fabsf(time) > k_real_epsilon)
		{
			const Eigen::Quaternionf &quaternion_derivative =
				eigen_angular_velocity_to_quaternion_derivative(combined_orientation, m_state->angular_velocity);

			predicted_orientation = Eigen::Quaternionf(
				combined_orientation.coeffs()
				+ quaternion_derivative.coeffs()*time).normalized();
		}

		// Apply offsets to reset orientation
		const Eigen::EulerAnglesf offsetX_euler(offset_x, 0.f, 0.f);
		const Eigen::EulerAnglesf offsetY_euler(0.f, offset_y, 0.f);
		Eigen::EulerAnglesf offsetZ_euler(0.f, 0.f, 0.f);
		offsetZ_euler += Eigen::Vector3f(0.f, 0.f, offset_z);

		const Eigen::Quaternionf offsetX_quat = eigen_euler_angles_to_quaternionf(offsetX_euler);
		const Eigen::Quaternionf offsetY_quat = eigen_euler_angles_to_quaternionf(offsetY_euler);
		const Eigen::Quaternionf offsetZ_quat = eigen_euler_angles_to_quaternionf(offsetZ_euler);

		result = (m_state->reset_orientation * offsetY_quat) * (predicted_orientation * offsetX_quat * offsetZ_quat);
	}

	return result;
}

Eigen::Quaternionf OpticalOrientationFilter::getResetOrientation() const
{
	return m_state->reset_orientation;
}

Eigen::Vector3f OpticalOrientationFilter::getAngularVelocityRadPerSec() const
{
	return m_state->bIsValid ? m_state->angular_velocity : Eigen::Vector3f::Zero();
}

Eigen::Vector3f OpticalOrientationFilter::getAngularAccelerationRadPerSecSqr() const
{
	return m_state->bIsValid ? m_state->angular_acceleration : Eigen::Vector3f::Zero();
}

Eigen::Quaternionf OpticalOrientationFilter::getCombinedOrientation() const
{
	{
		const Eigen::EulerAnglesf eul = eigen_quaternionf_to_euler_angles(m_state->imu_orientation);
		printf("m_state->imu_orientation: %f, %f, %f\n", eul.get_x_degrees(), eul.get_y_degrees(), eul.get_z_degrees());
	}

	{
		const Eigen::EulerAnglesf eul = eigen_quaternionf_to_euler_angles(m_state->optical_orientation);
		printf("m_state->optical_orientation: %f, %f, %f\n", eul.get_x_degrees(), eul.get_y_degrees(), eul.get_z_degrees());
	}

	//$TODO combine optical and IMU orientation.

	return m_state->optical_orientation;
}

// -- OrientationFilterExternal --
void OrientationTargetOpticalARG::update(const float delta_time, const PoseFilterPacket &packet)
{
	UpdateOpticalTarget(delta_time, packet);
	UpdateComplementaryMARG(delta_time, packet);
}


void OrientationTargetOpticalARG::UpdateComplementaryMARG(const float delta_time, const PoseFilterPacket &packet)
{
	if (packet.has_imu_measurements())
	{
		// Time delta used for filter update is time delta passed in
		// plus the accumulated time since the packet hasn't has an IMU measurement
		const float total_delta_time = (float)m_state->accumulated_imu_time_delta + delta_time;

		const Eigen::Vector3f &current_omega = packet.imu_gyroscope_rad_per_sec;

		Eigen::Vector3f current_g = packet.imu_accelerometer_g_units;
		eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

		Eigen::Vector3f current_m = Eigen::Vector3f::Zero();

		// Get the direction of the magnetic fields in the identity pose.	
		Eigen::Vector3f k_identity_m_direction = m_constants.magnetometer_calibration_direction;

		// Get the direction of the gravitational fields in the identity pose
		Eigen::Vector3f k_identity_g_direction = m_constants.gravity_calibration_direction;

		// Angular Rotation (AR) Update
		//-----------------------------
		// Compute the rate of change of the orientation purely from the gyroscope
		// q_dot = 0.5*q*omega
		Eigen::Quaternionf q_current = m_state->imu_orientation;

		Eigen::Quaternionf q_omega = Eigen::Quaternionf(0.f, current_omega.x(), 0.f, current_omega.z());
		Eigen::Quaternionf q_derivative = Eigen::Quaternionf(q_current.coeffs()*0.5f) * q_omega;

		// Integrate the rate of change to get a new orientation
		// q_new= q + q_dot*dT
		Eigen::Quaternionf q_step = Eigen::Quaternionf(q_derivative.coeffs() * total_delta_time);
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

		// Blending Update
		//----------------
		// Save the new quaternion and first derivative back into the orientation state
		// Derive the second derivative
		{
			// The final rotation is a blend between the integrated orientation and absolute rotation from the earth-frame
			const Eigen::Quaternionf new_orientation =
				eigen_quaternion_normalized_lerp(ar_orientation, mg_orientation, mg_weight);

			const Eigen::Vector3f new_angular_velocity = Eigen::Vector3f::Zero(); // current_omega;
			const Eigen::Vector3f new_angular_acceleration = Eigen::Vector3f::Zero(); // (current_omega - m_state->angular_velocity) / delta_time;

			m_state->apply_imu_state(new_orientation, new_angular_velocity, new_angular_acceleration, delta_time);
		}

		// Update the blend weight
		// -- Exponential blend the MG weight from 1 down to k_base_earth_frame_align_weight
		mg_weight = lerp_clampf(mg_weight, k_base_earth_frame_align_weight, 0.9f);
	}
	else
	{
		m_state->accumulate_imu_delta_time(delta_time);
	}
}


void OrientationTargetOpticalARG::UpdateOpticalTarget(const float delta_time, const PoseFilterPacket &packet)
{
#if !defined(IS_TESTING_KALMAN)
	if (packet.has_imu_measurements())
	{
		if (packet.controllerDeviceId > -1)
		{
			ServerControllerViewPtr ControllerView = DeviceManager::getInstance()->getControllerViewPtr(packet.controllerDeviceId);
			ServerControllerViewPtr OtherControllerView = DeviceManager::getInstance()->getControllerViewPtr(1);
			if (ControllerView != nullptr && OtherControllerView != nullptr && ControllerView->getIsOpen() && OtherControllerView->getIsOpen())
			{
				IControllerInterface *current_controller = ControllerView->castChecked<IControllerInterface>();
				IControllerInterface *other_controller = OtherControllerView->castChecked<IControllerInterface>();

				CommonDevicePosition dev_pos = ControllerView->getFilteredPose(current_controller->getPredictionTime()).PositionCm;
				CommonDevicePosition dev_other_pos = OtherControllerView->getFilteredPose(other_controller->getPredictionTime()).PositionCm;
				Eigen::Vector3f pos = Eigen::Vector3f(dev_pos.x, dev_pos.y, dev_pos.z);
				Eigen::Vector3f other_pos = Eigen::Vector3f(dev_other_pos.x, dev_other_pos.y, dev_other_pos.z);

				// Calc angle
				Eigen::Vector3f forwardDirection = Eigen::Vector3f(pos.x(), 0.f, pos.z()) - Eigen::Vector3f(other_pos.x(), 0.f, other_pos.z());
				forwardDirection.normalize();

				Eigen::Quaternionf forwardQuat = Eigen::Quaternionf::Identity();
				forwardQuat.setFromTwoVectors(forwardDirection, Eigen::Vector3f::UnitZ());

				Eigen::Vector3f nullVec = Eigen::Vector3f::Zero();
				DeviceManager::getInstance()->m_tracker_manager->applyPlayspaceOffsets(nullVec, forwardQuat, false, false, true);

				Eigen::Quaternionf forwardQuat_inv = forwardQuat.inverse();

				m_state->apply_optical_state(forwardQuat_inv, delta_time);
			}
		}
	}
#endif
}
