//-- includes -----
#include "MathEigen.h"

//-- globals ----
const Eigen::Vector3f g_eigen_vector3f_zero = Eigen::Vector3f( 0, 0, 0 );
const Eigen::Vector3f *k_eigen_vector3f_zero = &g_eigen_vector3f_zero;

const Eigen::Vector3f g_eigen_vector3f_one = Eigen::Vector3f( 1, 1, 1 );
const Eigen::Vector3f *k_eigen_vector3f_one = &g_eigen_vector3f_one;

const Eigen::Quaternionf g_eigen_quaternion_zero = Eigen::Quaternionf( 0, 0, 0, 0 );
const Eigen::Quaternionf *k_eigen_quaternion_zero = &g_eigen_quaternion_zero;

//-- public methods -----
Eigen::Quaternionf
eigen_quaternion_from_ZY(
	const Eigen::Vector3f &in_Z,
	const Eigen::Vector3f &in_Y)
{
	Eigen::Vector3f Z= in_Z;

	eigen_vector3f_normalize_with_default(Z, Eigen::Vector3f::UnitZ());
	Eigen::Vector3f X= Z.cross(in_Y);
	eigen_vector3f_normalize_with_default(X, Eigen::Vector3f::UnitX());
	Eigen::Vector3f Y= X.cross(Z);

	Eigen::Matrix3f mat;
	mat.col(0)= X;
	mat.col(1)= Y;
	mat.col(2)= Z;

	return Eigen::Quaternionf(mat).normalized();
}

// Creates a quaternion that rotates clockwise about the axis for a positive angle
// when appied with psmove_vector_clockwise_rotate()
Eigen::Quaternionf
eigen_quaternion_angle_axis(float radians, const Eigen::Vector3f &axis)
{
	return Eigen::Quaternionf(Eigen::AngleAxisf(radians, axis));
}

Eigen::Quaternionf
eigen_quaternion_normalized_lerp(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b, const float u)
{
	Eigen::Quaternionf q(a.coeffs()*(1.f - u) + b.coeffs()*u);
	q.normalize();

	return q;
}

Eigen::Quaternionf
eigen_quaternion_safe_divide_with_default(const Eigen::Quaternionf &q, const float divisor, const Eigen::Quaternionf &default_result)
{
	Eigen::Quaternionf q_n;

	if (!is_nearly_zero(divisor))
	{
		q_n = Eigen::Quaternionf(q.coeffs() / divisor);
	}
	else
	{
		q_n = default_result;
	}

	return q_n;
}

float
eigen_quaternion_normalize_with_default(Eigen::Quaternionf &inout_v, const Eigen::Quaternionf &default_result)
{
	const float magnitude = inout_v.norm();
	inout_v = eigen_quaternion_safe_divide_with_default(inout_v, magnitude, default_result);
	return magnitude;
}

Eigen::Quaterniond
eigen_quaterniond_safe_divide_with_default(const Eigen::Quaterniond &q, const double divisor, const Eigen::Quaterniond &default_result)
{
	Eigen::Quaterniond q_n;

	if (!is_double_nearly_zero(divisor))
	{
		q_n = Eigen::Quaterniond(q.coeffs() / divisor);
	}
	else
	{
		q_n = default_result;
	}

	return q_n;
}

double
eigen_quaterniond_normalize_with_default(Eigen::Quaterniond &inout_v, const Eigen::Quaterniond &default_result)
{
	const double magnitude = inout_v.norm();
	inout_v = eigen_quaterniond_safe_divide_with_default(inout_v, magnitude, default_result);
	return magnitude;
}

bool
eigen_vector3f_is_valid(const Eigen::Vector3f &v)
{
    return is_valid_float(v.x()) && is_valid_float(v.y()) && is_valid_float(v.z());
}

bool
eigen_quaternion_is_valid(const Eigen::Quaternionf &q)
{
	return is_valid_float(q.x()) && is_valid_float(q.y()) && is_valid_float(q.z()) && is_valid_float(q.w());
}

Eigen::Vector3f
eigen_vector3f_clockwise_rotate(const Eigen::Quaternionf &q, const Eigen::Vector3f &v)
{
	assert_eigen_quaternion_is_normalized(q);

	// Eigen rotates counterclockwise (i.e. q*v*q^-1), 
	// while we want the inverse of that (q^-1*v*q)
	return q.conjugate()._transformVector(v);
}

Eigen::Vector3d
eigen_vector3d_clockwise_rotate(const Eigen::Quaterniond &q, const Eigen::Vector3d &v)
{
	assert_eigen_quaterniond_is_normalized(q);

	// Eigen rotates counterclockwise (i.e. q*v*q^-1), 
	// while we want the inverse of that (q^-1*v*q)
	return q.conjugate()._transformVector(v);
}

Eigen::Matrix3f
eigen_quaternion_to_clockwise_matrix3f(const Eigen::Quaternionf &q)
{
	return q.conjugate().toRotationMatrix();
}

Eigen::Quaternionf
eigen_matrix3f_to_clockwise_quaternion(const Eigen::Matrix3f &m)
{
	Eigen::Quaternionf q(m);

	return q.conjugate();
}

Eigen::Vector3f
eigen_vector3f_divide_by_vector_with_default(
	const Eigen::Vector3f &v,
	const Eigen::Vector3f &divisor,
	const Eigen::Vector3f &default_result)
{
	Eigen::Vector3f result(
		safe_divide_with_default(v.x(), divisor.x(), default_result.x()),
		safe_divide_with_default(v.y(), divisor.y(), default_result.y()),
		safe_divide_with_default(v.z(), divisor.z(), default_result.z()));

	return result;
}

float
eigen_vector3f_normalize_with_default(Eigen::Vector3f &v, const Eigen::Vector3f &default_result)
{
	const float length = v.norm();

	// Use the default value if v is too tiny
	v = (length > k_normal_epsilon) ? (v / length) : default_result;

	return length;
}

double
eigen_vector3d_normalize_with_default(Eigen::Vector3d &v, const Eigen::Vector3d &default_result)
{
	const double length = v.norm();

	// Use the default value if v is too tiny
	v = (length > 0.0001) ? (v / length) : default_result;

	return length;
}

float
eigen_quaternion_unsigned_angle_between(const Eigen::Quaternionf &a, const Eigen::Quaternionf &b)
{
	assert_eigen_quaternion_is_normalized(a);
	assert_eigen_quaternion_is_normalized(b);

	const float radian_diff = fabsf(a.angularDistance(b));

	return radian_diff;
}

Eigen::Quaternionf
eigen_angular_velocity_to_quaternion_derivative(
	const Eigen::Quaternionf &current_orientation,
	const Eigen::Vector3f &ang_vel)
{
	Eigen::Quaternionf omega = Eigen::Quaternionf(0.f, ang_vel.x(), ang_vel.y(), ang_vel.z());
	Eigen::Quaternionf quaternion_derivative = Eigen::Quaternionf(current_orientation.coeffs() * 0.5f) *omega;

	return quaternion_derivative;
}

Eigen::Quaterniond
eigen_angular_velocity_to_quaterniond_derivative(
	const Eigen::Quaterniond &current_orientation,
	const Eigen::Vector3d &ang_vel)
{
	Eigen::Quaterniond omega = Eigen::Quaterniond(0.f, ang_vel.x(), ang_vel.y(), ang_vel.z());
	Eigen::Quaterniond quaternion_derivative = Eigen::Quaterniond(current_orientation.coeffs() * 0.5f) *omega;

	return quaternion_derivative;
}

Eigen::Vector3f
eigen_quaternion_derivative_to_angular_velocity(
	const Eigen::Quaternionf &current_orientation,
	const Eigen::Quaternionf &quaternion_derivative)
{
	Eigen::Quaternionf inv_orientation = current_orientation.conjugate();
	auto q_ang_vel = (quaternion_derivative*inv_orientation).coeffs() * 2.f;
	Eigen::Vector3f ang_vel(q_ang_vel.x(), q_ang_vel.y(), q_ang_vel.z());

	return ang_vel;
}

Eigen::Vector3d
eigen_quaterniond_derivative_to_angular_velocity(
	const Eigen::Quaterniond &current_orientation,
	const Eigen::Quaterniond &quaternion_derivative)
{
	Eigen::Quaterniond inv_orientation = current_orientation.conjugate();
	auto q_ang_vel = (quaternion_derivative*inv_orientation).coeffs() * 2.f;
	Eigen::Vector3d ang_vel(q_ang_vel.x(), q_ang_vel.y(), q_ang_vel.z());

	return ang_vel;
}

Eigen::Quaterniond
eigen_angle_axis_to_quaterniond(
	const Eigen::Vector3d &angle_axis)
{
	Eigen::Vector3d unit_axis = angle_axis;
	const double angle = eigen_vector3d_normalize_with_default(unit_axis, Eigen::Vector3d::Zero());
	return Eigen::Quaterniond(Eigen::AngleAxisd(angle, unit_axis));
}

Eigen::Quaternionf
eigen_angle_axis_to_quaternion(
	const Eigen::Vector3f &angle_axis)
{
	Eigen::Vector3f unit_axis = angle_axis;
	const float angle = eigen_vector3f_normalize_with_default(unit_axis, Eigen::Vector3f::Zero());
	return Eigen::Quaternionf(Eigen::AngleAxisf(angle, unit_axis));
}

//http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/
template<typename T>
Eigen::Quaternion<T> eigen_euler_angles_to_quaternion(const Eigen::EulerAngles<T> &euler_angles)
{
	const T attitude_radians = euler_angles.get_attitude_radians(); // i
	const T heading_radians = euler_angles.get_heading_radians(); // j
	const T bank_radians = euler_angles.get_bank_radians(); // k

	// Assuming the angles are in radians.
	//(x=pitch, y=yaw, z=roll)
	const double c1 = cos(attitude_radians / 2.f);
	const double s1 = sin(attitude_radians / 2.f);
	const double c2 = cos(heading_radians / 2.f);
	const double s2 = sin(heading_radians / 2.f);
	const double c3 = cos(bank_radians / 2.f);
	const double s3 = sin(bank_radians / 2.f);
	Eigen::Quaternion<T> q(
		static_cast<T>(c1*c2*c3 + s1*s2*s3),  // w = cos(theta/2)
		static_cast<T>(s1*c2*c3 - c1*s2*s3),  // x = v.i*sin(theta/2)
		static_cast<T>(c1*s2*c3 + s1*c2*s3),  // y = v.j*sin(theta/2)
		static_cast<T>(c1*c2*s3 - s1*s2*c3)); // z = v.k*sin(theta/2)

	return q;
}

//http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
template<typename T>
Eigen::EulerAngles<T> eigen_quaternion_to_euler_angles(const Eigen::Quaternion<T> &q)
{
	double qw = static_cast<double>(q.w());
	double qx = static_cast<double>(q.x());
	double qy = static_cast<double>(q.y());
	double qz = static_cast<double>(q.z());
	double test = qx*qy + qz*qw;

	double attitude_radians, heading_radians, bank_radians;

	if (test > 0.4999)
	{
		// singularity at north pole
		heading_radians = 2.0 * atan2(qx, qw);
		bank_radians = k_real64_half_pi;
		attitude_radians = 0.0;
	}
	else if (test < -0.4999)
	{
		// singularity at south pole
		heading_radians = -2.0 * atan2(qx, qw);
		bank_radians = -k_real64_half_pi;
		attitude_radians = 0.0;
	}
	else
	{
		double sqx = qx*qx;
		double sqy = qy*qy;
		double sqz = qz*qz;

		heading_radians = atan2(2.0*qy*qw - 2.0*qx*qz, 1.0 - 2.0*sqy - 2.0*sqz);
		bank_radians = asin(2.0*test);
		attitude_radians = atan2(2.0*qx*qw - 2.0*qy*qz, 1.0 - 2.0*sqx - 2.0*sqz);
	}

	return Eigen::EulerAngles<T>(
		static_cast<T>(bank_radians),
		static_cast<T>(heading_radians),
		static_cast<T>(attitude_radians));
}

Eigen::Vector3f
eigen_vector3f_projection(const Eigen::Vector3f &a, const Eigen::Vector3f &b)
{
	const float projected_length_of_a_on_b = a.dot(b.normalized());
	const Eigen::Vector3f a_projected_on_b = projected_length_of_a_on_b * b;

	return a_projected_on_b;
}

Eigen::Quaternionf
eigen_quaternionf_projection(const Eigen::Quaternionf &q, const Eigen::Vector3f &direction)
{
	Eigen::Vector3f r = Eigen::Vector3f(q.x(), q.y(), q.z());
	Eigen::Vector3f proj = eigen_vector3f_projection(r, direction);
	Eigen::Quaternionf rot = Eigen::Quaternionf(q.w(), proj.x(), proj.y(), proj.z());
	rot.normalize();

	return rot;
}

///**
//https://stackoverflow.com/questions/3684269/component-of-a-quaternion-rotation-around-an-axis
//Decompose the rotation on to 2 parts.
//1. Twist - rotation around the "direction" vector
//2. Swing - rotation around axis that is perpendicular to "direction" vector
//The rotation can be composed back by
//rotation = swing * twist
//has singularity in case of swing_rotation close to 180 degrees rotation.
//if the input quaternion is of non-unit length, the outputs are non-unit as well
//otherwise, outputs are both unit
//
void
eigen_quaternionf_to_swing_twist(
	const Eigen::Quaternionf &rotation,
	const Eigen::Vector3f &direction,
	Eigen::Quaternionf &swing,
	Eigen::Quaternionf &twist)
{
	Eigen::Vector3f ra(rotation.x(), rotation.y(), rotation.z()); // rotation axis
	Eigen::Vector3f p = eigen_vector3f_projection(ra, direction); // return projection v1 on to v2  (parallel component)

	twist = Eigen::Quaternionf(rotation.w(), p.x(), p.y(), p.z()).normalized();
	swing = rotation * twist.conjugate();
}

void
eigen_quaternionf_to_yaw_pitch_roll(
	const Eigen::Quaternionf &rot1,
	const Eigen::Quaternionf &rot2,
	Eigen::Quaternionf &yaw,
	Eigen::Quaternionf &pitch,
	Eigen::Quaternionf &roll)
{
	Eigen::Vector3f pitchAxis = rot1 * Eigen::Vector3f::UnitX();
	Eigen::Vector3f rollAxis = rot1 * Eigen::Vector3f::UnitZ();
	Eigen::Vector3f yawAxis = rot1 * Eigen::Vector3f::UnitY();

	Eigen::Quaternionf diffQ = rot2 * rot1.inverse();

	Eigen::Vector3f r = Eigen::Vector3f(diffQ.x(), diffQ.y(), diffQ.z());
	
	if (r.norm() < k_real_epsilon)
	{
		Eigen::Vector3f rotatedPitchAxis = diffQ * pitchAxis;
		Eigen::Vector3f rotatedYawAxis = pitchAxis.cross(rotatedPitchAxis);
		Eigen::Vector3f rotatedRollAxis = diffQ * rollAxis;

		if (rotatedYawAxis.norm() > k_real_epsilon)
		{
			float yawAngle = eigen_vector3f_get_angle(pitchAxis, rotatedPitchAxis);
			yaw = eigen_quaternion_angle_axis(yawAngle * k_degrees_to_radians, rotatedYawAxis);
		}
		else
		{
			yaw = Eigen::Quaternionf::Identity();
		}

		if (rotatedRollAxis.norm() > k_real_epsilon)
		{
			float rollAngle = eigen_vector3f_get_angle(yawAxis, rotatedYawAxis);
			roll = eigen_quaternion_angle_axis(rollAngle * k_degrees_to_radians, rotatedRollAxis);
		}
		else
		{
			roll = Eigen::Quaternionf::Identity();
		}

		pitch = eigen_quaternion_angle_axis(180.f * k_degrees_to_radians, pitchAxis);
	}
	else
	{
		pitch = eigen_quaternionf_projection(diffQ, pitchAxis);
		roll = eigen_quaternionf_projection(diffQ, rollAxis);
		yaw = diffQ * pitch.inverse();
	}
}

float
eigen_vector3f_get_angle(const Eigen::Vector3f &a, const Eigen::Vector3f &b)
{
	float sq = (a.norm() * b.norm());
	if (sq > k_real_epsilon)
	{
		float dot = a.dot(b.normalized()) / sq;
		float arcAcos = acosf(dot);

		return (arcAcos * k_radians_to_degreees);
	}

	return 0.0f;
}

Eigen::Quaterniond
eigen_euler_angles_to_quaterniond(const Eigen::EulerAnglesd &euler_angles)
{
	return eigen_euler_angles_to_quaternion<double>(euler_angles);
}

Eigen::Quaternionf
eigen_euler_angles_to_quaternionf(const Eigen::EulerAnglesf &euler_angles)
{
	return eigen_euler_angles_to_quaternion<float>(euler_angles);
}

Eigen::EulerAnglesd
eigen_quaterniond_to_euler_angles(const Eigen::Quaterniond &q)
{
	return eigen_quaternion_to_euler_angles<double>(q);
}

Eigen::EulerAnglesf
eigen_quaternionf_to_euler_angles(const Eigen::Quaternionf &q)
{
	return eigen_quaternion_to_euler_angles<float>(q);
}

void
lowpass_vector3f_kalman(
	Eigen::Vector3f &new_value,
	const Eigen::Vector3f old_value,
	Eigen::Vector3f &kal_gain,
	Eigen::Vector3f &kal_err_estimate,
	Eigen::Vector3f &kal_current_estimate,
	const float kalman_position_error,
	const float kalman_position_noise,
	const float deltatime)
{
	for (int i = 0; i < 3; ++i)
	{
		lowpass_kalman(new_value[i], old_value[i], kal_gain[i], kal_err_estimate[i], kal_current_estimate[i], kalman_position_error, kalman_position_noise, deltatime);
	}
}

void
lowpass_kalman(
	float &new_value,
	const float old_value,
	float &kal_gain,
	float &kal_err_estimate,
	float &kal_current_estimate,
	const float kalman_position_error,
	const float kalman_position_noise,
	const float deltatime)
{
	if (kal_err_estimate <= k_real_epsilon || (kal_err_estimate + kalman_position_error) <= k_real_epsilon)
	{
		kal_err_estimate = kalman_position_error;
	}

	kal_gain = kal_err_estimate / (kal_err_estimate + kalman_position_error);
	kal_current_estimate = old_value + kal_gain * (new_value - old_value);
	kal_err_estimate = (1.0 - kal_gain) * kal_err_estimate + std::abs(old_value - kal_current_estimate) * kalman_position_noise * deltatime;
	new_value = kal_current_estimate;
}

float lowpass_filter(
	const float alpha,
	const float old_filtered_vector,
	const float new_vector)
{
	return alpha*new_vector + (1.f - alpha)*old_filtered_vector;
}
