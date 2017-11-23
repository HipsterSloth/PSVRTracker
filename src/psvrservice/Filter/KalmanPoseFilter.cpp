//-- includes --
#include "KalmanPoseFilter.h"
#include "MathAlignment.h"

#include <kalman/MeasurementModel.hpp>
#include <kalman/SystemModel.hpp>
#include <kalman/SquareRootBase.hpp>
#include <kalman/SquareRootUnscentedKalmanFilter.hpp>

// The kalman filter runs way to slow in a fully unoptimized build.
// But if we just unoptimize this file in a release build it seems to run ok.
#if defined(_MSC_VER) && defined(UNOPTIMIZE_KALMAN_FILTERS)
#pragma optimize( "", off )
#endif

//-- constants --
#define MEASUREMENT_LED_COUNT   9

enum PoseFilterStateEnum
{
    // Position State
    POSITION_X, // meters
    LINEAR_VELOCITY_X, // meters / s
    LINEAR_ACCELERATION_X, // meters /s^2
    POSITION_Y,
    LINEAR_VELOCITY_Y,
    LINEAR_ACCELERATION_Y,
    POSITION_Z,
    LINEAR_VELOCITY_Z,
    LINEAR_ACCELERATION_Z,
    // Orientation State
    ERROR_QUATERNION_W,
    ERROR_Q_DOT_W,
    ERROR_QUATERNION_X,
    ERROR_Q_DOT_X,
    ERROR_QUATERNION_Y,
    ERROR_Q_DOT_Y,
    ERROR_QUATERNION_Z,
    ERROR_Q_DOT_Z,

    STATE_PARAMETER_COUNT
};

enum MorpheusMeasurementEnum {
    MORPHEUS_ACCELEROMETER_X, // gravity units
    MORPHEUS_ACCELEROMETER_Y,
    MORPHEUS_ACCELEROMETER_Z,
    MORPHEUS_GYROSCOPE_PITCH, // radians/s
    MORPHEUS_GYROSCOPE_YAW,
    MORPHEUS_GYROSCOPE_ROLL,
    MORPHEUS_LED0_X_CM,          // LED 0 Optical Position
    MORPHEUS_LED0_Y_CM,
    MORPHEUS_LED0_Z_CM,
    MORPHEUS_LED1_X_CM,          // LED 1 Optical Position
    MORPHEUS_LED1_Y_CM,
    MORPHEUS_LED1_Z_CM,
    MORPHEUS_LED2_X_CM,          // LED 2 Optical Position
    MORPHEUS_LED2_Y_CM,
    MORPHEUS_LED2_Z_CM,
    MORPHEUS_LED3_X_CM,          // LED 3 Optical Position
    MORPHEUS_LED3_Y_CM,
    MORPHEUS_LED3_Z_CM,
    MORPHEUS_LED4_X_CM,          // LED 4 Optical Position
    MORPHEUS_LED4_Y_CM,
    MORPHEUS_LED4_Z_CM,
    MORPHEUS_LED5_X_CM,          // LED 5 Optical Position
    MORPHEUS_LED5_Y_CM,
    MORPHEUS_LED5_Z_CM,
    MORPHEUS_LED6_X_CM,          // LED 6 Optical Position
    MORPHEUS_LED6_Y_CM,
    MORPHEUS_LED6_Z_CM,
    MORPHEUS_LED7_X_CM,          // LED 7 Optical Position
    MORPHEUS_LED7_Y_CM,
    MORPHEUS_LED7_Z_CM,
    MORPHEUS_LED8_X_CM,          // LED 8 Optical Position
    MORPHEUS_LED8_Y_CM,
    MORPHEUS_LED8_Z_CM,
    MORPHEUS_MEASUREMENT_PARAMETER_COUNT
};

enum PointCloudMeasurementEnum {
    POINT_CLOUD_LED0_X_CM,          // LED 0 Optical Position
    POINT_CLOUD_LED0_Y_CM,
    POINT_CLOUD_LED0_Z_CM,
    POINT_CLOUD_LED1_X_CM,          // LED 1 Optical Position
    POINT_CLOUD_LED1_Y_CM,
    POINT_CLOUD_LED1_Z_CM,
    POINT_CLOUD_LED2_X_CM,          // LED 2 Optical Position
    POINT_CLOUD_LED2_Y_CM,
    POINT_CLOUD_LED2_Z_CM,
    POINT_CLOUD_LED3_X_CM,          // LED 3 Optical Position
    POINT_CLOUD_LED3_Y_CM,
    POINT_CLOUD_LED3_Z_CM,
    POINT_CLOUD_LED4_X_CM,          // LED 4 Optical Position
    POINT_CLOUD_LED4_Y_CM,
    POINT_CLOUD_LED4_Z_CM,
    POINT_CLOUD_LED5_X_CM,          // LED 5 Optical Position
    POINT_CLOUD_LED5_Y_CM,
    POINT_CLOUD_LED5_Z_CM,
    POINT_CLOUD_LED6_X_CM,          // LED 6 Optical Position
    POINT_CLOUD_LED6_Y_CM,
    POINT_CLOUD_LED6_Z_CM,
    POINT_CLOUD_LED7_X_CM,          // LED 7 Optical Position
    POINT_CLOUD_LED7_Y_CM,
    POINT_CLOUD_LED7_Z_CM,
    POINT_CLOUD_LED8_X_CM,          // LED 8 Optical Position
    POINT_CLOUD_LED8_Y_CM,
    POINT_CLOUD_LED8_Z_CM,

    POINT_CLOUD_MEASUREMENT_PARAMETER_COUNT
};

// Arbitrary tuning scale applied to the measurement noise
#define R_SCALE 1.0

// Arbitrary tuning scale applied to the process noise
#define Q_SCALE 1.0

// From: http://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb#Reasonable-Choices-for-the-Parameters
// beta=2 is a good choice for Gaussian problems, 
// kappa=3-n where n is the size of x is a good choice for kappa, 
// 0<=alpha<=1 is an appropriate choice for alpha, 
// where a larger value for alpha spreads the sigma points further from the mean.
#define k_ukf_alpha 0.6
#define k_ukf_beta 2.0
#define k_ukf_kappa -14.0 // 3 - STATE_PARAMETER_COUNT

//-- private methods ---
template <class StateType>
void Q_discrete_2rd_order_white_noise(const double dT, const double var, const int state_index, Kalman::Covariance<StateType> &Q);
template <class StateType>
void Q_discrete_3rd_order_white_noise(const double dT, const double var, const int state_index, Kalman::Covariance<StateType> &Q);

//-- private definitions --
template<typename T>
class PoseStateVector : public Kalman::Vector<T, STATE_PARAMETER_COUNT>
{
public:
    KALMAN_VECTOR(PoseStateVector, T, STATE_PARAMETER_COUNT)

    static PoseStateVector<T> Identity()
    {
        PoseStateVector<T> result= PoseStateVector<T>::Zero();

        result[ERROR_QUATERNION_W] = 1.0;

        return result;
    }

    // Accessors
    Eigen::Vector3d get_position_meters() const { 
        return Eigen::Vector3d((*this)[POSITION_X], (*this)[POSITION_Y], (*this)[POSITION_Z]); 
    }
    Eigen::Vector3d get_linear_velocity_m_per_sec() const {
        return Eigen::Vector3d((*this)[LINEAR_VELOCITY_X], (*this)[LINEAR_VELOCITY_Y], (*this)[LINEAR_VELOCITY_Z]);
    }
    Eigen::Vector3d get_linear_acceleration_m_per_sec_sqr() const {
        return Eigen::Vector3d((*this)[LINEAR_ACCELERATION_X], (*this)[LINEAR_ACCELERATION_Y], (*this)[LINEAR_ACCELERATION_Z]);
    }
    Eigen::Quaterniond get_error_quaterniond() const {
        return Eigen::Quaterniond((*this)[ERROR_QUATERNION_W], (*this)[ERROR_QUATERNION_X], (*this)[ERROR_QUATERNION_Y], (*this)[ERROR_QUATERNION_Z]);
    }
    Eigen::Quaterniond get_error_quaterniond_dot() const {
        return Eigen::Quaterniond((*this)[ERROR_Q_DOT_W], (*this)[ERROR_Q_DOT_X], (*this)[ERROR_Q_DOT_Y], (*this)[ERROR_Q_DOT_Z]);
    }
    Eigen::Vector3d get_angular_velocity_rad_per_sec() const {
        Eigen::Quaterniond q = get_error_quaterniond();
        Eigen::Quaterniond q_dot= get_error_quaterniond_dot();
        return eigen_quaterniond_derivative_to_angular_velocity(q, q_dot);
    }

    // Mutators
    void set_position_meters(const Eigen::Vector3d &p) {
        (*this)[POSITION_X] = p.x(); (*this)[POSITION_Y] = p.y(); (*this)[POSITION_Z] = p.z();
    }
    void set_linear_velocity_m_per_sec(const Eigen::Vector3d &v) {
        (*this)[LINEAR_VELOCITY_X] = v.x(); (*this)[LINEAR_VELOCITY_Y] = v.y(); (*this)[LINEAR_VELOCITY_Z] = v.z();
    }
    void set_linear_acceleration_m_per_sec_sqr(const Eigen::Vector3d &a) {
        (*this)[LINEAR_ACCELERATION_X] = a.x(); (*this)[LINEAR_ACCELERATION_Y] = a.y(); (*this)[LINEAR_ACCELERATION_Z] = a.z();
    }
    void set_error_quaterniond(const Eigen::Quaterniond &q) {
        (*this)[ERROR_QUATERNION_W] = q.w();
        (*this)[ERROR_QUATERNION_X] = q.x();
        (*this)[ERROR_QUATERNION_Y] = q.y();
        (*this)[ERROR_QUATERNION_Z] = q.z();
    }
    void set_error_quaterniond_dot(const Eigen::Quaterniond &q_dot) {
        (*this)[ERROR_Q_DOT_W] = q_dot.w();
        (*this)[ERROR_Q_DOT_X] = q_dot.x();
        (*this)[ERROR_Q_DOT_Y] = q_dot.y();
        (*this)[ERROR_Q_DOT_Z] = q_dot.z();
    }
    void set_angular_velocity_rad_per_sec(const Eigen::Vector3d &v) {
        Eigen::Quaterniond q = get_error_quaterniond();
        Eigen::Quaterniond q_dot = eigen_angular_velocity_to_quaterniond_derivative(q, v);
        (*this)[ERROR_Q_DOT_W] = q_dot.w();
        (*this)[ERROR_Q_DOT_X] = q_dot.x(); 
        (*this)[ERROR_Q_DOT_Y] = q_dot.y();
        (*this)[ERROR_Q_DOT_Z] = q_dot.z();
    }
};
typedef PoseStateVector<double> PoseStateVectord;

/**
* @brief System model for a controller
*
* This is the system model defining how a controller advances from one
* time-step to the next, i.e. how the system state evolves over time.
*/
class PoseSystemModel : public Kalman::SystemModel<PoseStateVectord, Kalman::Vector<double, 0>, Kalman::SquareRootBase>
{
public:
    inline void set_time_step(const double dt) { m_time_step = dt; }

    void init(const PoseFilterConstants &constants)
    {
        use_linear_acceleration = constants.position_constants.use_linear_acceleration;
        m_last_tracking_projection_area_px_sqr = -1.f;
        update_process_noise(constants, 0.f);
    }

    void update_process_noise(const PoseFilterConstants &constants, float tracking_projection_area_px_sqr)
    {
        // Only update the covariance when there is more than a 10px change in position quality
        if (m_last_tracking_projection_area_px_sqr < 0.f || 
            !is_nearly_equal(tracking_projection_area_px_sqr, m_last_tracking_projection_area_px_sqr, 10.f))
        {
            const double mean_position_dT = constants.position_constants.mean_update_time_delta;
            const double mean_orientation_dT = constants.orientation_constants.mean_update_time_delta;

            // Start off using the maximum variance values
            static double q_scale = Q_SCALE;
            const double orientation_variance = 
                q_scale * 
                static_cast<double>(
                    constants.orientation_constants.orientation_variance_curve.evaluate(
                        tracking_projection_area_px_sqr));
            const double position_variance_cm_sqr = 
                q_scale*
                static_cast<double>(
                    constants.position_constants.position_variance_curve.evaluate(
                        tracking_projection_area_px_sqr));
            // variance_meters = variance_cm * (0.01)^2 because ...
            // var(k*x) = sum(k*x_i - k*mu)^2/(N-1) = k^2*sum(x_i - mu)^2/(N-1)
            // where k = k_centimeters_to_meters = 0.01
            const double position_variance_m_sqr = 
                k_centimeters_to_meters*k_centimeters_to_meters*position_variance_cm_sqr;

            // Initialize the process covariance matrix Q
            Kalman::Covariance<PoseStateVectord> Q = Kalman::Covariance<PoseStateVectord>::Zero();
            Q_discrete_3rd_order_white_noise<PoseStateVectord>(mean_position_dT, position_variance_m_sqr, POSITION_X, Q);
            Q_discrete_3rd_order_white_noise<PoseStateVectord>(mean_position_dT, position_variance_m_sqr, POSITION_Y, Q);
            Q_discrete_3rd_order_white_noise<PoseStateVectord>(mean_position_dT, position_variance_m_sqr, POSITION_Z, Q);
            Q_discrete_2rd_order_white_noise<PoseStateVectord>(mean_orientation_dT, orientation_variance, ERROR_QUATERNION_W, Q);
            Q_discrete_2rd_order_white_noise<PoseStateVectord>(mean_orientation_dT, orientation_variance, ERROR_QUATERNION_X, Q);
            Q_discrete_2rd_order_white_noise<PoseStateVectord>(mean_orientation_dT, orientation_variance, ERROR_QUATERNION_Y, Q);
            Q_discrete_2rd_order_white_noise<PoseStateVectord>(mean_orientation_dT, orientation_variance, ERROR_QUATERNION_Z, Q);
            setCovariance(Q);

            // Keep track last tracking projection area we built the covariance matrix for
            m_last_tracking_projection_area_px_sqr = tracking_projection_area_px_sqr;
        }
    }

    /**
    * @brief Definition of (non-linear) state transition function
    *
    * This function defines how the system state is propagated through time,
    * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to
    * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
    * the system control input \f$u\f$.
    *
    * @param [in] x The system state in current time-step
    * @param [in] u The control vector input
    * @returns The (predicted) system state in the next time-step
    */
    PoseStateVectord f(const PoseStateVectord& old_state, const Kalman::Vector<double, 0>& control) const
    {
        // Predicted state vector after transition
        PoseStateVectord new_state;

        // Extract parameters from the old state
        const Eigen::Vector3d old_position_meters = old_state.get_position_meters();
        const Eigen::Vector3d old_linear_velocity_m_per_sec = old_state.get_linear_velocity_m_per_sec();
        const Eigen::Vector3d old_linear_acceleration_m_per_sec_sqr = old_state.get_linear_acceleration_m_per_sec_sqr();

        // Extract parameters from the old state
        const Eigen::Quaterniond old_error_q = old_state.get_error_quaterniond();
        const Eigen::Quaterniond old_error_q_dot = old_state.get_error_quaterniond_dot();

        // Compute the position state update
        Eigen::Vector3d new_position_meters;
        Eigen::Vector3d new_linear_velocity_m_per_sec;
        if (use_linear_acceleration)
        {
            new_position_meters =
                old_position_meters
                + old_linear_velocity_m_per_sec*m_time_step
                + old_linear_acceleration_m_per_sec_sqr*m_time_step*m_time_step*0.5f;			
            new_linear_velocity_m_per_sec = 
                old_linear_velocity_m_per_sec 
                + old_linear_acceleration_m_per_sec_sqr*m_time_step;
        }
        else
        {
            new_position_meters =
                old_position_meters
                + old_linear_velocity_m_per_sec*m_time_step;			
            new_linear_velocity_m_per_sec =
                (m_time_step > k_real64_normal_epsilon)
                ? (new_position_meters - old_position_meters) / m_time_step
                : old_linear_velocity_m_per_sec;
        }

        const Eigen::Vector3d &new_linear_acceleration_m_per_sec_sqr = old_linear_acceleration_m_per_sec_sqr;

        // Compute the quaternion derivative of the current state
        // q_new= q + q_dot*dT
        const Eigen::Quaterniond error_q_step = Eigen::Quaterniond(old_error_q_dot.coeffs() * m_time_step);
        const Eigen::Quaterniond error_q_new = Eigen::Quaterniond(old_error_q.coeffs() + error_q_step.coeffs());

        // Save results to the new state
        new_state.set_position_meters(new_position_meters);
        new_state.set_linear_velocity_m_per_sec(new_linear_velocity_m_per_sec);
        new_state.set_linear_acceleration_m_per_sec_sqr(new_linear_acceleration_m_per_sec_sqr);

        // Save results to the new state
        new_state.set_error_quaterniond(error_q_new.normalized());
        new_state.set_error_quaterniond_dot(old_error_q_dot);

        return new_state;
    }

protected:
    bool use_linear_acceleration;
    double m_time_step;
    float m_last_tracking_projection_area_px_sqr;
};

class PoseSRUKF : public Kalman::SquareRootUnscentedKalmanFilter<PoseStateVectord>
{
public:
    PoseSRUKF(double alpha = 1.0, double beta = 2.0, double kappa = 0.0)
        : Kalman::SquareRootUnscentedKalmanFilter<PoseStateVectord>(alpha, beta, kappa)
    {
    }

    State& getStateMutable()
    {
        return x;
    }
};

template<typename T>
class Morpheus_PoseMeasurementVector : public Kalman::Vector<T, MORPHEUS_MEASUREMENT_PARAMETER_COUNT>
{
public:
    KALMAN_VECTOR(Morpheus_PoseMeasurementVector, T, MORPHEUS_MEASUREMENT_PARAMETER_COUNT)

    // Accessors
    Eigen::Vector3d get_accelerometer() const {
        return Eigen::Vector3d(
            (*this)[MORPHEUS_ACCELEROMETER_X], 
            (*this)[MORPHEUS_ACCELEROMETER_Y], 
            (*this)[MORPHEUS_ACCELEROMETER_Z]);
    }
    Eigen::Vector3d get_gyroscope() const {
        return Eigen::Vector3d(
            (*this)[MORPHEUS_GYROSCOPE_PITCH], 
            (*this)[MORPHEUS_GYROSCOPE_YAW], 
            (*this)[MORPHEUS_GYROSCOPE_ROLL]);
    }
    Eigen::Vector3d get_LED_position_cm(const int led_index) const {
        assert(led_index >= 0  && led_index < MEASUREMENT_LED_COUNT);
        return Eigen::Vector3d(
                (*this)[MORPHEUS_LED0_X_CM + 3*led_index], 
                (*this)[MORPHEUS_LED0_Y_CM + 3*led_index], 
                (*this)[MORPHEUS_LED0_Z_CM + 3*led_index]);
    }

    // Mutators
    void set_accelerometer(const Eigen::Vector3d &a) {
        (*this)[MORPHEUS_ACCELEROMETER_X] = a.x(); 
        (*this)[MORPHEUS_ACCELEROMETER_Y] = a.y(); 
        (*this)[MORPHEUS_ACCELEROMETER_Z] = a.z();
    }
    void set_gyroscope(const Eigen::Vector3d &g) {
        (*this)[MORPHEUS_GYROSCOPE_PITCH] = g.x(); 
        (*this)[MORPHEUS_GYROSCOPE_YAW] = g.y(); 
        (*this)[MORPHEUS_GYROSCOPE_ROLL] = g.z();
    }
    void set_LED_position_cm(const int led_index, const Eigen::Vector3d &p) {
        assert(led_index >= 0  && led_index < MEASUREMENT_LED_COUNT);
        (*this)[MORPHEUS_LED0_X_CM + 3*led_index] = p.x(); 
        (*this)[MORPHEUS_LED0_Y_CM + 3*led_index] = p.y(); 
        (*this)[MORPHEUS_LED0_Z_CM + 3*led_index] = p.z();
    }
};
typedef Morpheus_PoseMeasurementVector<double> Morpheus_PoseMeasurementVectord;

template<typename T>
class PointCloud_PoseMeasurementVector : public Kalman::Vector<T, POINT_CLOUD_MEASUREMENT_PARAMETER_COUNT>
{
public:
    KALMAN_VECTOR(PointCloud_PoseMeasurementVector, T, POINT_CLOUD_MEASUREMENT_PARAMETER_COUNT)

    // Accessors
    Eigen::Vector3d get_LED_position_cm(const int led_index) const {
        assert(led_index >= 0  && led_index < MEASUREMENT_LED_COUNT);
        return Eigen::Vector3d(
                (*this)[POINT_CLOUD_LED0_X_CM + 3*led_index], 
                (*this)[POINT_CLOUD_LED0_Y_CM + 3*led_index], 
                (*this)[POINT_CLOUD_LED0_Z_CM + 3*led_index]);
    }

    // Mutators
    void set_LED_position_cm(const int led_index, const Eigen::Vector3d &p) {
        assert(led_index >= 0  && led_index < MEASUREMENT_LED_COUNT);
        (*this)[POINT_CLOUD_LED0_X_CM + 3*led_index] = p.x(); 
        (*this)[POINT_CLOUD_LED0_Y_CM + 3*led_index] = p.y(); 
        (*this)[POINT_CLOUD_LED0_Z_CM + 3*led_index] = p.z();
    }
};
typedef PointCloud_PoseMeasurementVector<double> PointCloud_PoseMeasurementVectord;

/**
* @brief Orientation Measurement model for measuring PSVR controller
*
* This is the measurement model for measuring the position and magnetometer of the PSVR controller.
* The measurement is given by the optical trackers.
*/
class Morpheus_PoseMeasurementModel : 
    public Kalman::MeasurementModel<PoseStateVectord, Morpheus_PoseMeasurementVectord, Kalman::SquareRootBase>
{
public:
    void init(const PoseFilterConstants &constants)
    {
        identity_gravity_direction = constants.orientation_constants.gravity_calibration_direction.cast<double>();
        m_last_world_orientation = Eigen::Quaterniond::Identity();
        m_last_world_linear_acceleration_m_per_sec_sqr = Eigen::Vector3d::Zero();
		m_last_tracking_projection_area_px_sqr = -1.f;
		update_measurement_covariance(constants, 0.f);

        assert(constants.shape.shape_type == PSVRTrackingShape_PointCloud);
        assert(constants.shape.shape.pointcloud.point_count == MEASUREMENT_LED_COUNT);
        for (int led_index = 0; led_index < MEASUREMENT_LED_COUNT; ++led_index)
        {
            const PSVRVector3f &p= constants.shape.shape.pointcloud.points[led_index];

            m_LED_model_vertices[led_index]= Eigen::Vector3d(
                static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        }
    }

	void update_measurement_covariance(
		const PoseFilterConstants &constants,
		const float tracking_projection_area_px_sqr)
	{
		// Only update the covariance when there is more than a 10px change in position quality
		if (m_last_tracking_projection_area_px_sqr < 0.f ||
			!is_nearly_equal(tracking_projection_area_px_sqr, m_last_tracking_projection_area_px_sqr, 10.f))
		{
			const double position_variance_cm_sqr = 
                static_cast<double>(
                    constants.position_constants.position_variance_curve.evaluate(
                        tracking_projection_area_px_sqr));

			// Update the measurement covariance R
            // Only diagonals used so no need to compute Cholesky
            static float r_accelerometer_scale = R_SCALE;
            static float r_gyro_scale = R_SCALE;
            static float r_position_scale = R_SCALE;
			Kalman::Covariance<Morpheus_PoseMeasurementVectord> R =
				Kalman::Covariance<Morpheus_PoseMeasurementVectord>::Zero();
            R(MORPHEUS_ACCELEROMETER_X, MORPHEUS_ACCELEROMETER_X) = r_accelerometer_scale*constants.orientation_constants.accelerometer_variance.x();
            R(MORPHEUS_ACCELEROMETER_Y, MORPHEUS_ACCELEROMETER_Y) = r_accelerometer_scale*constants.orientation_constants.accelerometer_variance.y();
            R(MORPHEUS_ACCELEROMETER_Z, MORPHEUS_ACCELEROMETER_Z) = r_accelerometer_scale*constants.orientation_constants.accelerometer_variance.z();
            R(MORPHEUS_GYROSCOPE_PITCH, MORPHEUS_GYROSCOPE_PITCH) = r_gyro_scale*constants.orientation_constants.gyro_variance.x();
            R(MORPHEUS_GYROSCOPE_YAW, MORPHEUS_GYROSCOPE_YAW) = r_gyro_scale*constants.orientation_constants.gyro_variance.y();
            R(MORPHEUS_GYROSCOPE_ROLL, MORPHEUS_GYROSCOPE_ROLL) = r_gyro_scale*constants.orientation_constants.gyro_variance.z();
            for (int i = 0; i < MEASUREMENT_LED_COUNT; ++i)
            {
                int offset= 3*i;
			    R(MORPHEUS_LED0_X_CM+offset, MORPHEUS_LED0_X_CM+offset) = r_position_scale*position_variance_cm_sqr;
			    R(MORPHEUS_LED0_Y_CM+offset, MORPHEUS_LED0_Y_CM+offset) = r_position_scale*position_variance_cm_sqr;
			    R(MORPHEUS_LED0_Z_CM+offset, MORPHEUS_LED0_Z_CM+offset) = r_position_scale*position_variance_cm_sqr;
            }
			setCovariance(R);

			// Keep track last position quality we built the covariance matrix for
			m_last_tracking_projection_area_px_sqr = tracking_projection_area_px_sqr;
		}
	}

    void update_world_orientation(const Eigen::Quaterniond &orientation)
    {
        m_last_world_orientation = orientation;
    }

    void update_world_linear_acceleration_m_per_sec_sqr(const Eigen::Vector3d &linear_acceleration)
    {
        m_last_world_linear_acceleration_m_per_sec_sqr = linear_acceleration;
    }

    /**
    * @brief Definition of (possibly non-linear) measurement function
    *
    * This function maps the system state to the measurement that is expected
    * to be received from the sensor assuming the system is currently in the
    * estimated state.
    *
    * @param [in] x The system state in current time-step
    * @returns The (predicted) sensor measurement for the system state
    */
    Morpheus_PoseMeasurementVectord h(const PoseStateVectord& x) const
    {
        Morpheus_PoseMeasurementVectord predicted_measurement;

        // Use the orientation from the state for prediction
        const Eigen::Quaterniond error_orientation = x.get_error_quaterniond();
        const Eigen::Quaterniond world_to_local_orientation = eigen_quaternion_concatenate(m_last_world_orientation, error_orientation).normalized();
        const Eigen::Quaterniond local_to_world_orientation = world_to_local_orientation.conjugate();

        // Use the current linear acceleration from last frame's state
        // and the current orientation from the state to predict
        // what the accelerometer reading will be (in the space of the controller)
        const Eigen::Vector3d &gravity_accel_g_units = identity_gravity_direction;
        const Eigen::Vector3d linear_accel_g_units = m_last_world_linear_acceleration_m_per_sec_sqr * k_ms2_to_g_units;
        const Eigen::Vector3d accel_world = linear_accel_g_units + gravity_accel_g_units;
        const Eigen::Vector3d accel_local = eigen_vector3d_clockwise_rotate(world_to_local_orientation, accel_world);

        // Use the angular velocity from the state to predict what the gyro reading will be
        const Eigen::Vector3d gyro_local = x.get_angular_velocity_rad_per_sec();

        // Save the predictions into the measurement vector
        predicted_measurement.set_accelerometer(accel_local);
        predicted_measurement.set_gyroscope(gyro_local);

        // Compute where we expect to find the tracking LEDs
        Eigen::Affine3d local_to_world= Eigen::Affine3d::Identity();
        local_to_world.linear()= local_to_world_orientation.toRotationMatrix();
        local_to_world.translation()= x.get_position_meters()*k_meters_to_centimeters;

        for (int led_index = 0; led_index < MEASUREMENT_LED_COUNT; ++led_index)
        {
            const Eigen::Vector3d &model_led_position= m_LED_model_vertices[led_index];
            const Eigen::Vector3d predicted_led_position= local_to_world * model_led_position;

            predicted_measurement.set_LED_position_cm(led_index, predicted_led_position);
        }

        return predicted_measurement;
    }

public:
    Eigen::Vector3d m_LED_model_vertices[MEASUREMENT_LED_COUNT]; // in centimeters!
    Eigen::Vector3d identity_gravity_direction;
    Eigen::Quaterniond m_last_world_orientation;
    Eigen::Vector3d m_last_world_linear_acceleration_m_per_sec_sqr;
    double m_time_step;
	float m_last_tracking_projection_area_px_sqr;
};

/**
* @brief Measurement model for measuring DS4 controller
*
* This is the measurement model for measuring the position and orientation of the DS4 controller.
* The measurement is given by the optical trackers.
*/
class PointCloud_PoseMeasurementModel
    : public Kalman::MeasurementModel<PoseStateVectord, PointCloud_PoseMeasurementVectord, Kalman::SquareRootBase>
{
public:
    void init(const PoseFilterConstants &constants)
    {
        m_last_world_orientation = Eigen::Quaterniond::Identity();
		m_last_tracking_projection_area_px_sqr = -1.f;
		update_measurement_covariance(constants, 0.f);

        assert(constants.shape.shape_type == PSVRTrackingShape_PointCloud);
        assert(constants.shape.shape.pointcloud.point_count == MEASUREMENT_LED_COUNT);
        for (int led_index = 0; led_index < MEASUREMENT_LED_COUNT; ++led_index)
        {
            const PSVRVector3f &p= constants.shape.shape.pointcloud.points[led_index];

            m_LED_model_vertices[led_index]= Eigen::Vector3d(
                static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z));
        }
    }

	void update_measurement_covariance(
		const PoseFilterConstants &constants,
		const float tracking_projection_area_px_sqr)
	{
		// Only update the covariance when there is more than a 10px change in position quality
		if (m_last_tracking_projection_area_px_sqr < 0.f ||
			!is_nearly_equal(tracking_projection_area_px_sqr, m_last_tracking_projection_area_px_sqr, 10.f))
		{
			const double position_variance_cm_sqr = 
                static_cast<double>(
                    constants.position_constants.position_variance_curve.evaluate(
                        tracking_projection_area_px_sqr));

			// Update the measurement covariance R
            // Only diagonals used so no need to compute Cholesky
            static float r_position_scale = R_SCALE;
			Kalman::Covariance<PointCloud_PoseMeasurementVectord> R =
				Kalman::Covariance<PointCloud_PoseMeasurementVectord>::Zero();
            for (int i = 0; i < MEASUREMENT_LED_COUNT; ++i)
            {
                int offset= 3*i;
			    R(POINT_CLOUD_LED0_X_CM+offset, POINT_CLOUD_LED0_X_CM+offset) = r_position_scale*position_variance_cm_sqr;
			    R(POINT_CLOUD_LED0_Y_CM+offset, POINT_CLOUD_LED0_Y_CM+offset) = r_position_scale*position_variance_cm_sqr;
			    R(POINT_CLOUD_LED0_Z_CM+offset, POINT_CLOUD_LED0_Z_CM+offset) = r_position_scale*position_variance_cm_sqr;
            }
			setCovariance(R);

			// Keep track last position quality we built the covariance matrix for
			m_last_tracking_projection_area_px_sqr = tracking_projection_area_px_sqr;
		}
	}

    void update_world_orientation(const Eigen::Quaterniond &orientation)
    {
        m_last_world_orientation = orientation;
    }

    /**
    * @brief Definition of (possibly non-linear) measurement function
    *
    * This function maps the system state to the measurement that is expected
    * to be received from the sensor assuming the system is currently in the
    * estimated state.
    *
    * @param [in] x The system state in current time-step
    * @returns The (predicted) sensor measurement for the system state
    */
    PointCloud_PoseMeasurementVectord h(const PoseStateVectord& x) const
    {
        PointCloud_PoseMeasurementVectord predicted_measurement;

        // Use the orientation from the state for prediction
        const Eigen::Quaterniond error_orientation = x.get_error_quaterniond();
        const Eigen::Quaterniond world_to_local_orientation = eigen_quaternion_concatenate(m_last_world_orientation, error_orientation).normalized();
        const Eigen::Quaterniond local_to_world_orientation = world_to_local_orientation.conjugate();

        // Compute where we expect to find the tracking leds
        Eigen::Affine3d local_to_world= Eigen::Affine3d::Identity();
        local_to_world.linear()= local_to_world_orientation.toRotationMatrix();
        local_to_world.translation()= x.get_position_meters()*k_meters_to_centimeters;

        for (int led_index = 0; led_index < MEASUREMENT_LED_COUNT; ++led_index)
        {
            const Eigen::Vector3d &model_led_position= m_LED_model_vertices[led_index];
            const Eigen::Vector3d predicted_led_position= local_to_world * model_led_position;

            predicted_measurement.set_LED_position_cm(led_index, predicted_led_position);
        }

        return predicted_measurement;
    }

public:
    Eigen::Vector3d m_LED_model_vertices[MEASUREMENT_LED_COUNT]; // in centimeters!
    Eigen::Quaterniond m_last_world_orientation;
    double m_time_step;
	float m_last_tracking_projection_area_px_sqr;
};

class KalmanPoseFilterImpl
{
public:
    /// Is the current fusion state valid
    bool bIsValid;

    /// True if we have seen a valid position measurement (>0 position quality)
    bool bSeenPositionMeasurement;

    /// True if we have seen a valid orientation measurement (>0 orientation quality)
    bool bSeenOrientationMeasurement;

    /// Position that's considered the origin position 
    Eigen::Vector3f origin_position_meters; // meters

    /// Used to model how the physics of the controller evolves
    PoseSystemModel system_model;

    /// Unscented Kalman Filter instance
    PoseSRUKF ukf;

    /// The duration the filter has been running
    double time;

    /// The final output of this filter.
    /// This isn't part of the UKF state vector because it's non-linear.
    /// Instead we store an "error quaternion" in the UKF state vector and then apply it 
    /// to this quaternion after a time step and then zero out the error.
    Eigen::Quaterniond world_orientation;

    KalmanPoseFilterImpl()
        : bIsValid(false)
        , bSeenOrientationMeasurement(false)
        , system_model()
        , ukf(k_ukf_alpha, k_ukf_beta, k_ukf_kappa)
        , world_orientation(Eigen::Quaternionf::Identity())
        , time(0.0)
    {
    }

    virtual void init(const PoseFilterConstants &constants)
    {
        bIsValid = false;
        bSeenOrientationMeasurement = false;
        bSeenPositionMeasurement= false;

        world_orientation = Eigen::Quaterniond::Identity();
        origin_position_meters = Eigen::Vector3f::Zero();

        system_model.init(constants);
        ukf.init(PoseStateVectord::Identity());
    }

    virtual void init(
        const PoseFilterConstants &constants,
        const Eigen::Vector3f &initial_position_meters,
        const Eigen::Quaternionf &orientation)
    {
        bIsValid = true;
        bSeenOrientationMeasurement = true;
        bSeenPositionMeasurement= true;

        origin_position_meters = Eigen::Vector3f::Zero();
        world_orientation = orientation.cast<double>();

        PoseStateVectord state_vector = PoseStateVectord::Identity();
        state_vector.set_position_meters(initial_position_meters.cast<double>());

        system_model.init(constants);
        ukf.init(PoseStateVectord::Identity());
        apply_error_to_world_quaternion();
    }

    // -- World Quaternion Accessors --
    inline Eigen::Quaterniond compute_net_world_quaternion() const
    {
        const Eigen::Quaterniond error_quaternion= ukf.getState().get_error_quaterniond();
        const Eigen::Quaterniond output_quaternion = eigen_quaternion_concatenate(world_orientation, error_quaternion).normalized();
        return output_quaternion;
    }

    // -- World Quaternion Mutators --
    inline void set_world_quaternion(const Eigen::Quaterniond &orientation)
    {
        world_orientation = orientation;
        ukf.getStateMutable().set_error_quaterniond(Eigen::Quaterniond::Identity());
    }

    void apply_error_to_world_quaternion()
    {
        set_world_quaternion(compute_net_world_quaternion());
    }
};

class PointCloudKalmanPoseFilterImpl : public KalmanPoseFilterImpl
{
public:
    PointCloud_PoseMeasurementModel measurement_model;

    void init(const PoseFilterConstants &constants) override
    {
        KalmanPoseFilterImpl::init(constants);
        measurement_model.init(constants);
    }

    void init(
        const PoseFilterConstants &constants,
        const Eigen::Vector3f &position,
        const Eigen::Quaternionf &orientation) override
    {
        KalmanPoseFilterImpl::init(constants, position, orientation);
        measurement_model.init(constants);
    }
};

class MorpheusKalmanPoseFilterImpl : public KalmanPoseFilterImpl
{
public:
    Morpheus_PoseMeasurementModel measurement_model;

    void init(const PoseFilterConstants &constants) override
    {
        KalmanPoseFilterImpl::init(constants);
        measurement_model.init(constants);
    }

    void init(
        const PoseFilterConstants &constants,
        const Eigen::Vector3f &position,
        const Eigen::Quaternionf &orientation) override
    {
        KalmanPoseFilterImpl::init(constants, position, orientation);
        measurement_model.init(constants);
    }
};

//-- public interface --
//-- KalmanPoseFilter --
KalmanPoseFilter::KalmanPoseFilter()
    : m_filter(nullptr)
{
    memset(&m_constants, 0, sizeof(PoseFilterConstants));
}

KalmanPoseFilter::~KalmanPoseFilter()
{
    if (m_filter != nullptr)
    {
        delete m_filter;
        m_filter;
    }
}

bool KalmanPoseFilter::init(const PoseFilterConstants &constants)
{
    m_constants = constants;

    // cleanup any existing filter
    if (m_filter != nullptr)
    {
        delete m_filter;
        m_filter;
    }

    // Create and initialize the private filter implementation
    KalmanPoseFilterImpl *filter = new KalmanPoseFilterImpl();
    filter->init(constants);
    m_filter = filter;

    return true;
}

bool KalmanPoseFilter::init(
    const PoseFilterConstants &constants,
    const Eigen::Vector3f &position,
    const Eigen::Quaternionf &orientation)
{
    m_constants = constants;

    // cleanup any existing filter
    if (m_filter != nullptr)
    {
        delete m_filter;
        m_filter;
    }

    // Create and initialize the private filter implementation
    KalmanPoseFilterImpl *filter = new KalmanPoseFilterImpl();
    filter->init(constants, position, orientation);
    m_filter = filter;

    return true;
}

bool KalmanPoseFilter::getIsStateValid() const
{
    return m_filter->bIsValid;
}

bool KalmanPoseFilter::getIsPositionStateValid() const
{
	return getIsStateValid();
}

bool KalmanPoseFilter::getIsOrientationStateValid() const
{
	return getIsStateValid();
}

double KalmanPoseFilter::getTimeInSeconds() const
{
    return m_filter->time;
}

void KalmanPoseFilter::resetState()
{
    m_filter->init(m_constants);
}

void KalmanPoseFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
    m_filter->world_orientation = q_pose.cast<double>();
    m_filter->ukf.init(PoseStateVectord::Identity());
}

Eigen::Quaternionf KalmanPoseFilter::getOrientation(float time) const
{
    Eigen::Quaternionf result = Eigen::Quaternionf::Identity();

    if (m_filter->bIsValid)
    {
        const Eigen::Quaternionf state_orientation = m_filter->compute_net_world_quaternion().cast<float>();
        Eigen::Quaternionf predicted_orientation = state_orientation;

        if (fabsf(time) > k_real_epsilon)
        {
            const Eigen::Quaternionf &quaternion_derivative =
                eigen_angular_velocity_to_quaternion_derivative(result, getAngularVelocityRadPerSec());

            predicted_orientation = Eigen::Quaternionf(
                state_orientation.coeffs()
                + quaternion_derivative.coeffs()*time).normalized();
        }

        result = predicted_orientation;
    }

    return result;
}

Eigen::Vector3f KalmanPoseFilter::getAngularVelocityRadPerSec() const
{
    Eigen::Vector3d ang_vel = m_filter->ukf.getState().get_angular_velocity_rad_per_sec();

    return ang_vel.cast<float>();
}

Eigen::Vector3f KalmanPoseFilter::getAngularAccelerationRadPerSecSqr() const
{
    return Eigen::Vector3f::Zero();
}

Eigen::Vector3f KalmanPoseFilter::getPositionCm(float time) const
{
    Eigen::Vector3f result = Eigen::Vector3f::Zero();

    if (m_filter->bIsValid)
    {
        Eigen::Vector3f state_position_meters= m_filter->ukf.getState().get_position_meters().cast<float>();
		Eigen::Vector3f state_velocity_m_per_sec = m_filter->ukf.getState().get_linear_velocity_m_per_sec().cast<float>();
        Eigen::Vector3f predicted_position =
            is_nearly_zero(time)
            ? state_position_meters
            : state_position_meters + state_velocity_m_per_sec * time;

        result = (predicted_position - m_filter->origin_position_meters) * k_meters_to_centimeters;
    }

    return result;
}

Eigen::Vector3f KalmanPoseFilter::getVelocityCmPerSec() const
{
	Eigen::Vector3d vel= m_filter->ukf.getState().get_linear_velocity_m_per_sec() * k_meters_to_centimeters;

    return vel.cast<float>();
}

Eigen::Vector3f KalmanPoseFilter::getAccelerationCmPerSecSqr() const
{
    Eigen::Vector3d accel= m_filter->ukf.getState().get_linear_acceleration_m_per_sec_sqr() * k_meters_to_centimeters;

	return accel.cast<float>();
}

//-- KalmanPoseFilterPointCloud --
bool KalmanPoseFilterPointCloud::init(const PoseFilterConstants &constants)
{
    KalmanPoseFilter::init(constants);

    PointCloudKalmanPoseFilterImpl *filter = new PointCloudKalmanPoseFilterImpl();
    filter->init(constants);
    m_filter = filter;

    return true;
}

bool KalmanPoseFilterPointCloud::init(
    const PoseFilterConstants &constants,
    const Eigen::Vector3f &position,
    const Eigen::Quaternionf &orientation)
{
    KalmanPoseFilter::init(constants);

    PointCloudKalmanPoseFilterImpl *filter = new PointCloudKalmanPoseFilterImpl();
    filter->init(constants, position, orientation);
    m_filter = filter;

    return true;
}

void KalmanPoseFilterPointCloud::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (m_filter->bIsValid)
    {
        PointCloudKalmanPoseFilterImpl *filter = static_cast<PointCloudKalmanPoseFilterImpl *>(m_filter);

        // Adjust the amount we trust the process model based on the tracking projection area
        filter->system_model.update_process_noise(m_constants, packet.tracking_projection_area_px_sqr);

        // Predict state for current time-step using the filters
        filter->system_model.set_time_step(delta_time);
        filter->ukf.predict(filter->system_model);

        // Get the measurement model for the DS4 from the derived filter impl
        PointCloud_PoseMeasurementModel &measurement_model = filter->measurement_model;

        // If this is the first time we have seen an orientation measurement, snap the orientation state
        if (!m_filter->bSeenOrientationMeasurement && packet.tracking_projection_area_px_sqr > 0.f)
        {
            const Eigen::Quaterniond world_quaternion = packet.optical_orientation.cast<double>();

            measurement_model.update_world_orientation(world_quaternion);

            filter->set_world_quaternion(world_quaternion);
            filter->bSeenOrientationMeasurement = true;
        }

        // Project the current state onto a predicted measurement as a default
        // in case no observation is available
        PointCloud_PoseMeasurementVectord local_measurement = measurement_model.h(filter->ukf.getState());

        // Adjust the amount we trust the optical measurements based on the tracking projection area
        measurement_model.update_measurement_covariance(
            m_constants,
            packet.tracking_projection_area_px_sqr);

        // If available, use the optical LED measurements
        if (packet.tracking_projection_area_px_sqr > 0.f)
        {
            Eigen::Vector3f optical_position_meters= packet.get_optical_position_in_meters();

            assert(packet.optical_tracking_shape_cm.shape_type == PSVRTrackingShape_PointCloud);
            assert(packet.optical_tracking_shape_cm.shape.pointcloud.point_count == MEASUREMENT_LED_COUNT);
            for (int led_index = 0; led_index < MEASUREMENT_LED_COUNT; ++led_index)
            {
                const PSVRVector3f &p= packet.optical_tracking_shape_cm.shape.pointcloud.points[led_index];
                
                local_measurement.set_LED_position_cm(
                    led_index, 
                    Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)));
            }

			// If this is the first time we have seen the position, snap the position state
			if (!m_filter->bSeenPositionMeasurement)
			{
				m_filter->ukf.getStateMutable().set_position_meters(optical_position_meters.cast<double>());
				m_filter->bSeenPositionMeasurement= true;
			}
        }

        // Update UKF
        filter->ukf.update(measurement_model, local_measurement);

        // Apply the orientation error in the UKF state to the output quaternion.
        // Zero out the error in the UKF state vector.
        filter->apply_error_to_world_quaternion();

        // Update the measurement model with the latest estimate of the orientation (without error)
        // so that we can predict what the controller relative sensor measurements will be
        measurement_model.update_world_orientation(filter->world_orientation);

        filter->time+= (double)delta_time;
    }
    else
    {
        m_filter->ukf.init(PoseStateVectord::Identity());
        m_filter->time= 0.0;
        m_filter->bIsValid = true;
    }
}

//-- KalmanPoseFilterMorpheus --
bool KalmanPoseFilterMorpheus::init(const PoseFilterConstants &constants)
{
    KalmanPoseFilter::init(constants);

    MorpheusKalmanPoseFilterImpl *filter = new MorpheusKalmanPoseFilterImpl();
    filter->init(constants);
    m_filter = filter;

    return true;
}

bool KalmanPoseFilterMorpheus::init(
    const PoseFilterConstants &constants,
    const Eigen::Vector3f &position,
    const Eigen::Quaternionf &orientation)
{
    KalmanPoseFilter::init(constants);

    MorpheusKalmanPoseFilterImpl *filter = new MorpheusKalmanPoseFilterImpl();
    filter->init(constants, position, orientation);
    m_filter = filter;

    return true;
}

void KalmanPoseFilterMorpheus::update(const float delta_time, const PoseFilterPacket &packet)
{
    if (m_filter->bIsValid)
    {
        MorpheusKalmanPoseFilterImpl *filter = static_cast<MorpheusKalmanPoseFilterImpl *>(m_filter);

        // Predict state for current time-step using the filters
        filter->system_model.set_time_step(delta_time);
        filter->ukf.predict(filter->system_model);

        // Get the measurement model for the PSVR from the derived filter impl
        Morpheus_PoseMeasurementModel &measurement_model = filter->measurement_model;

        // Update the linear acceleration on the measurement model with last frames acceleration state
        measurement_model.update_world_linear_acceleration_m_per_sec_sqr(
            packet.get_current_acceleration_in_meters_per_second_squared().cast<double>());

        // If this is the first time we have seen an orientation measurement, 
        // snap the orientation state to a best fit alignment of the sensor measurements.
        if (!m_filter->bSeenOrientationMeasurement)
        {
            Eigen::Vector3f current_g = packet.imu_accelerometer_g_units;
            eigen_vector3f_normalize_with_default(current_g, Eigen::Vector3f::Zero());

            Eigen::Vector3f current_m = packet.imu_magnetometer_unit;
            eigen_vector3f_normalize_with_default(current_m, Eigen::Vector3f::Zero());

            const Eigen::Vector3f* mg_from[2] = { 
                &m_constants.orientation_constants.gravity_calibration_direction, 
                &m_constants.orientation_constants.magnetometer_calibration_direction };
            const Eigen::Vector3f* mg_to[2] = { &current_g, &current_m };

            // Always attempt to align even if we don't get within the alignment tolerance.
            // More often then not the attempted alignment is closer to the right answer 
            // then the identity quaternion.
            Eigen::Quaternionf initial_orientation;
            eigen_alignment_quaternion_between_vector_frames(
                mg_from, mg_to, 0.1f, Eigen::Quaternionf::Identity(), initial_orientation);

            measurement_model.update_world_orientation(initial_orientation.cast<double>());

            filter->set_world_quaternion(initial_orientation.cast<double>());
            filter->bSeenOrientationMeasurement = true;
        }

        // Project the current state onto a predicted measurement as a default
        // in case no observation is available
        Morpheus_PoseMeasurementVectord local_measurement = measurement_model.h(filter->ukf.getState());

        // Accelerometer, gyroscope  measurements are always available
        local_measurement.set_accelerometer(packet.imu_accelerometer_g_units.cast<double>());
        local_measurement.set_gyroscope(packet.imu_gyroscope_rad_per_sec.cast<double>());
        m_filter->bSeenOrientationMeasurement = true;

        // Adjust the amount we trust the optical measurements based on the tracking projection area
        measurement_model.update_measurement_covariance(
            m_constants,
            packet.tracking_projection_area_px_sqr);

        // If available, use the optical LED measurements
        if (packet.tracking_projection_area_px_sqr > 0.f)
        {
            Eigen::Vector3f optical_position_meters= packet.get_optical_position_in_meters();

            assert(packet.optical_tracking_shape_cm.shape_type == PSVRTrackingShape_PointCloud);
            assert(packet.optical_tracking_shape_cm.shape.pointcloud.point_count == MEASUREMENT_LED_COUNT);
            for (int led_index = 0; led_index < MEASUREMENT_LED_COUNT; ++led_index)
            {
                const PSVRVector3f &p= packet.optical_tracking_shape_cm.shape.pointcloud.points[led_index];
                
                local_measurement.set_LED_position_cm(
                    led_index, 
                    Eigen::Vector3d(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)));
            }

			// If this is the first time we have seen the position, snap the position state
			if (!m_filter->bSeenPositionMeasurement)
			{
				m_filter->ukf.getStateMutable().set_position_meters(optical_position_meters.cast<double>());
				m_filter->bSeenPositionMeasurement= true;
			}
        }

        // Update UKF
        filter->ukf.update(measurement_model, local_measurement);

        // Apply the orientation error in the UKF state to the output quaternion.
        // Zero out the error in the UKF state vector.
        filter->apply_error_to_world_quaternion();

        // Update the measurement model with the latest estimate of the orientation (without error)
        // so that we can predict what the controller relative sensor measurements will be
        measurement_model.update_world_orientation(filter->world_orientation);
    }
    else
    {
        m_filter->ukf.init(PoseStateVectord::Identity());
        m_filter->bIsValid= true;
    }
}

//-- Private functions --
// Adapted from: https://github.com/rlabbe/filterpy/blob/master/filterpy/common/discretization.py#L52-L53
// Returns the Q matrix for the Discrete Constant White Noise
// - dT is the time step
// - var is the variance in the process noise
// - state_index denotes where in Q the 2x2 covariance matrix should be written

// Q is computed as the G * G^T * variance, where G is the process noise per time step.
// In other words, G = [[.5dt ^ 2][dt]] ^ T for the constant angular velocity model.
template <class StateType>
void Q_discrete_2rd_order_white_noise(
    const double dT, 
    const double var, 
    const int state_index, 
    Kalman::Covariance<StateType> &Q)
{
    const double dT_2 = dT*dT;
    const double dT_3 = dT_2*dT;
    const double dT_4 = dT_2*dT_2;

    const double q4 = var * dT_4;
    const double q3 = var * dT_3;
    const double q2 = var * dT_2;

    // Q = [.5dt^2, dt]*[.5dt^2, dt]^T * variance
    const int &i= state_index;
    Q(i+0,i+0) = 0.25*q4; Q(i+0,i+1) = 0.5*q3;
    Q(i+1,i+0) =  0.5*q3; Q(i+1,i+1) = q2;
}

// Adapted from: https://github.com/rlabbe/filterpy/blob/master/filterpy/common/discretization.py#L55-L57
// Returns the Q matrix for the Discrete Constant White Noise
// - dT is the time step
// - var is the variance in the process noise
// - state_index denotes where in Q the 3x3 covariance matrix should be written

// Q is computed as the G * G^T * variance, where G is the process noise per time step.
// In other words, G = [[.5dt^2][dt][1]] ^ T for the constant linear acceleration model.
template <class StateType>
void Q_discrete_3rd_order_white_noise(
    const double dT,
    const double var,
    const int state_index,
    Kalman::Covariance<StateType> &Q)
{
    const double dT_2 = dT*dT;
    const double dT_3 = dT_2*dT;
    const double dT_4 = dT_2*dT_2;

    const double q4 = var * dT_4;
    const double q3 = var * dT_3;
    const double q2 = var * dT_2;
    const double q1 = var * dT;

    const int &i= state_index;
    Q(i+0,i+0) = 0.25*q4; Q(i+0,i+1) = 0.5*q3; Q(i+0,i+2) = 0.5*q2;
    Q(i+1,i+0) =  0.5*q3; Q(i+1,i+1) =     q2; Q(i+1,i+2) =     q1;
    Q(i+2,i+0) =  0.5*q2; Q(i+2,i+1) =     q1; Q(i+2,i+2) =    1.0;
}