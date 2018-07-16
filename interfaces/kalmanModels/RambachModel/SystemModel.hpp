#ifndef RAMBACHMODEL_SYSTEMMODEL_HPP
#define RAMBACHMODEL_SYSTEMMODEL_HPP

#include <kalman/LinearizedSystemModel.hpp>
#include "datastructure/MathDefinitions.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace FUSION {
namespace RambachModel
{
/**
 * @brief System state vector-type for a 6DOF AR device
 *
 * This is a system state for an AR devices characterized by its position, velocity,
 * orientation, and the accelerometer and gyroscope biases.
 *
 * This state model is based on the paper
 * "Fusion of Unsynchronized Optical Tracker and Inertial Sensor in EKF Framework for In-car Augmented Reality Delay Reduction"
 * Jason Rambach, Alain Pagani, Sebastian Lampe, Ruben Reiser, Manthan Pancholi, Didier Stricker
 * (http://av.dfki.de/~pagani/papers/Rambach2017a_ISMAR.pdf)
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 16>
{
public:
    KALMAN_VECTOR(State, T, 16)
    
    //! X-position
    static constexpr size_t P_X = 0;
    //! Y-position
    static constexpr size_t P_Y = 1;
	//! Z-position
    static constexpr size_t P_Z = 2;

	//! X-velocity
	static constexpr size_t V_X = 3;
	//! Y-velocity
	static constexpr size_t V_Y = 4;
	//! Z-velocity
	static constexpr size_t V_Z = 5;

	//! Orientation quaternion
	//! W-component
	static constexpr size_t Q_W = 6;
	//! X-component
	static constexpr size_t Q_X = 7;
	//! Y-component
	static constexpr size_t Q_Y = 8;
	//! Z-component
	static constexpr size_t Q_Z = 9;

	//! X-acceleration-bias
	static constexpr size_t B_A_X = 10;
	//! Y-acceleration-bias
	static constexpr size_t B_A_Y = 11;
	//! Z-acceleration-bias
	static constexpr size_t B_A_Z = 12;

	//! X-Angular-velocity-bias
	static constexpr size_t B_W_X = 13;
	//! Y-Angular-velocity-bias
	static constexpr size_t B_W_Y = 14;
	//! Z-Angular-velocity-bias
	static constexpr size_t B_W_Z = 15;
    
    T p_x()       const { return (*this)[ P_X ]; }
    T p_y()       const { return (*this)[ P_Y ]; }
	T p_z()       const { return (*this)[ P_Z ]; }

	T v_x()       const { return (*this)[ V_X ]; }
	T v_y()       const { return (*this)[ V_Y ]; }
	T v_z()       const { return (*this)[ V_Z ]; }

	T q_w()       const { return (*this)[ Q_W ]; }
	T q_x()       const { return (*this)[ Q_X ]; }
	T q_y()       const { return (*this)[ Q_Y ]; }
	T q_z()       const { return (*this)[ Q_Z ]; }

	T b_a_x()       const { return (*this)[ B_A_X ]; }
	T b_a_y()       const { return (*this)[ B_A_Y ]; }
	T b_a_z()       const { return (*this)[ B_A_Z ]; }

	T b_w_x()       const { return (*this)[ B_W_X ]; }
	T b_w_y()       const { return (*this)[ B_W_Y ]; }
	T b_w_z()       const { return (*this)[ B_W_Z ]; }


	T& p_x()       { return (*this)[ P_X ]; }
	T& p_y()       { return (*this)[ P_Y ]; }
	T& p_z()       { return (*this)[ P_Z ]; }

	T& v_x()       { return (*this)[ V_X ]; }
	T& v_y()       { return (*this)[ V_Y ]; }
	T& v_z()       { return (*this)[ V_Z ]; }

	T& q_w()       { return (*this)[ Q_W ]; }
	T& q_x()       { return (*this)[ Q_X ]; }
	T& q_y()       { return (*this)[ Q_Y ]; }
	T& q_z()       { return (*this)[ Q_Z ]; }

	T& b_a_x()       { return (*this)[ B_A_X ]; }
	T& b_a_y()       { return (*this)[ B_A_Y ]; }
	T& b_a_z()       { return (*this)[ B_A_Z ]; }

	T& b_w_x()       { return (*this)[ B_W_X ]; }
	T& b_w_y()       { return (*this)[ B_W_Y ]; }
	T& b_w_z()       { return (*this)[ B_W_Z ]; }

    friend std::ostream& operator<<(std::ostream& os, State<T>& state) {
		os << state.p_x() << ", " << state.p_y() << ", " << state.p_z() << ", "
			<< state.v_x() << ", " << state.v_y() << ", " << state.v_z() << ", "
			<< state.q_w() << ", " << state.q_x() << ", " << state.q_y() << ", " << state.q_z() << ", "
			<< state.b_a_x() << ", " << state.b_a_y() << ", " << state.b_a_z() << ", "
			<< state.b_w_x() << ", " << state.b_w_y() << ", " << state.b_w_z();
		return os;
	}
};

/**
 * @brief System control-input vector-type for a 6DOF AR device
 *
 * This is the system control-input of an AR device
 * using the acceleration and the angular speed as input.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(Control, T, 6)
    
	//! X-acceleration
	static constexpr size_t A_X = 0;
	//! Y-acceleration
	static constexpr size_t A_Y = 1;
	//! Z-acceleration
	static constexpr size_t A_Z = 2;

	//! X-Angular-velocity
	static constexpr size_t W_X = 3;
	//! Y-Angular-velocity
	static constexpr size_t W_Y = 4;
	//! Z-Angular-velocity
	static constexpr size_t W_Z = 5;
    
	T a_x()       const { return (*this)[ A_X ]; }
	T a_y()       const { return (*this)[ A_Y ]; }
	T a_z()       const { return (*this)[ A_Z ]; }

	T w_x()       const { return (*this)[ W_X ]; }
	T w_y()       const { return (*this)[ W_Y ]; }
	T w_z()       const { return (*this)[ W_Z ]; }

	T& a_x() { return (*this)[ A_X ]; }
	T& a_y() { return (*this)[ A_Y ]; }
	T& a_z() { return (*this)[ A_Z ]; }

	T& w_x() { return (*this)[ W_X ]; }
	T& w_y() { return (*this)[ W_Y ]; }
	T& w_z() { return (*this)[ W_Z ]; }

    friend std::ostream& operator<<(std::ostream& os, Control<T>& control) {
        os << control.a_x() << ", " << control.a_y() << ", " << control.a_z() << ", "
            << control.w_x() << ", " << control.w_y() << ", " << control.w_z();
        return os;
    }
};

/**
 * @brief System model for a 6DOF AR device
 *
 * This is the system model defining how our AR device moves from one
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef RambachModel::State<T> S;
    
    //! Control type shortcut definition
    typedef RambachModel::Control<T> C;

    double dt = 0.0016; //TODO create get and set methods

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
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;

        Vector<T, 3> gravity(0, -9.80665, 0);
		Vector<T, 3> acc;
		Vector<T, 3> gyro;

		bool no_control = u.isZero(0);

		// New Angular Velocity (corrected)
		if (no_control)
		{
			gyro(0) = T(0.00001);
			gyro(1) = T(0.00001);
			gyro(2) = T(0.00001);
		}
		else
		{
			gyro(0) = u.w_x() - x.b_w_x();
			gyro(1) = u.w_y() - x.b_w_y();
			gyro(2) = u.w_z() - x.b_w_z();
		}
		

		// New Angular Velocity bias
		x_.b_w_x() = x.b_w_x();
		x_.b_w_y() = x.b_w_y();
		x_.b_w_z() = x.b_w_z();


		// New orientation

		// Previous orientation
		Quaternion<T> q(x.q_w(), x.q_x(), x.q_y(), x.q_z());

		// Angular rate as quaternion
		auto wnorm = gyro.norm();
		auto coswnorm = std::cos(dt * wnorm / 2);
		auto sinwnorm = std::sin(dt * wnorm / 2);

		Quaternion<T> qW(coswnorm,
			gyro(0) / wnorm * sinwnorm,
			gyro(1) / wnorm * sinwnorm,
			gyro(2) / wnorm * sinwnorm);

		Quaternion<T> newQ = q * qW;

		x_.q_w() = newQ.w();
		x_.q_x() = newQ.x();
		x_.q_y() = newQ.y();
		x_.q_z() = newQ.z();

		// New Acceleration (corrected)
		
		if (no_control)
		{
			// no acceleration assumption
			acc(0) = T(0);
			acc(1) = T(0);
			acc(2) = T(0);
		}
		else
		{
			acc = Vector<T, 3>(u.a_x() - x.b_a_x(),
				u.a_y() - x.b_a_y(),
				u.a_z() - x.b_a_z());

			// Acceleration as quaternion
			Quaternion<T> qAcc;
			qAcc.w() = 0.;
			qAcc.vec() = acc;

			Quaternion<T> rotatedQAcc = q * qAcc * q.inverse();

			Vector<T, 3> rotatedAcc = rotatedQAcc.vec();

			acc(0) = rotatedAcc(0) + gravity(0);
			acc(1) = rotatedAcc(1) + gravity(1);
			acc(2) = rotatedAcc(2) + gravity(2);
		}

		// New Acceleration bias
		x_.b_a_x() = x.b_a_x();
		x_.b_a_y() = x.b_a_y();
		x_.b_a_z() = x.b_a_z();

		// New velocity
		x_.v_x() = x.v_x() + acc(0) * dt;
		x_.v_y() = x.v_y() + acc(1) * dt;
		x_.v_z() = x.v_z() + acc(2) * dt;

		// New position
		x_.p_x() = x.p_x() + x.v_x() * dt + acc(0) * dt*dt / 2;
		x_.p_y() = x.p_y() + x.v_y() * dt + acc(1) * dt*dt / 2;
		x_.p_z() = x.p_z() + x.v_z() * dt + acc(2) * dt*dt / 2;
        
        // Return transitioned state vector
        return x_;
    }
    
protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    void updateJacobians( const S& x, const C& u )
    {
		// F = df/dx (Jacobian of state transition w.r.t. the state)
		this->F.setZero();

		bool no_control = u.isZero(0);

		Quaternion<T> q(x.q_w(),
			x.q_x(),
			x.q_y(),
			x.q_z());

		Matrix<T, 3, 3> qrm = q.toRotationMatrix();

        Vector<T, 3> acc, w, gravity(0, -9.80665, 0);

		if (no_control)
		{
			//ACCELERATION
			// no acceleration assumption
			acc = Vector<T, 3>(0,
				0,
				0);

			//ANGULAR VELOCITY
			// no angular velocity assumption
			w = Vector<T, 3>(0.00001,
				0.00001,
				0.00001);
		}
		else
		{
			//ACCELERATION
			Vector<T, 3> rawAcc;
			rawAcc = Vector<T, 3>(u.a_x() - x.b_a_x(),
				u.a_y() - x.b_a_y(),
				u.a_z() - x.b_a_z());

			// Acceleration as quaternion
			Quaternion<T> qAcc;
			qAcc.w() = 0.;
			qAcc.vec() = rawAcc;

			Quaternion<T> rotatedQAcc = q * qAcc * q.inverse();

			Vector<T, 3> rotatedAcc = rotatedQAcc.vec();

			acc = Vector<T, 3>(rotatedAcc(0) + gravity(0),
				rotatedAcc(1) + gravity(1),
				rotatedAcc(2) + gravity(2));

			//ANGULAR VELOCITY
			w = Vector<T, 3>(u.w_x() - x.b_w_x(),
				u.w_y() - x.b_w_y(),
				u.w_z() - x.b_w_z());
		}

		T wnorm = std::sqrt(w(0)*w(0) + w(1)*w(1) + w(2)*w(2));
		T coswnorm = std::cos(dt * wnorm / 2);
		T sinwnormwnorm = std::sin(dt * wnorm / 2) / wnorm;

		T wxsin = w(0) * sinwnormwnorm;
		T wysin = w(1) * sinwnormwnorm;
		T wzsin = w(2) * sinwnormwnorm;
		T qwsin = q.w() * sinwnormwnorm;
		T qxsin = q.x() * sinwnormwnorm;
		T qysin = q.y() * sinwnormwnorm;
		T qzsin = q.z() * sinwnormwnorm;

		T qw_sx = q.w() * acc(0);
		T qw_sy = q.w() * acc(1);
		T qw_sz = q.w() * acc(2);
		T qx_sx = q.x() * acc(0);
		T qx_sy = q.x() * acc(1);
		T qx_sz = q.x() * acc(2);
		T qy_sx = q.y() * acc(0);
		T qy_sy = q.y() * acc(1);
		T qy_sz = q.y() * acc(2);
		T qz_sx = q.z() * acc(0);
		T qz_sy = q.z() * acc(1);
		T qz_sz = q.z() * acc(2);

		Matrix<T, 3, 4> qvr;
		qvr << 2 * qw_sx + 2 * qy_sz - 2 * qz_sy, 2 * qx_sx + 2 * qy_sy + 2 * qz_sz, 2 * qw_sz + 2 * qx_sy - 2 * qy_sx, -2 * qw_sy + 2 * qx_sz - 2 * qz_sx,
			2 * qw_sy - 2 * qx_sz + 2 * qz_sx, -2 * qw_sz - 2 * qx_sy + 2 * qy_sx, 2 * qx_sx + 2 * qy_sy + 2 * qz_sz, 2 * qw_sx + 2 * qy_sz - 2 * qz_sy,
			2 * qw_sz + 2 * qx_sy - 2 * qy_sx, 2 * qw_sy - 2 * qx_sz + 2 * qz_sx, -2 * qw_sx - 2 * qy_sz + 2 * qz_sy, 2 * qx_sx + 2 * qy_sy + 2 * qz_sz;

		// partial derivatives of position
		this->F(S::P_X, S::P_X) = 1;
		this->F(S::P_Y, S::P_Y) = 1;
		this->F(S::P_Z, S::P_Z) = 1;


		this->F(S::P_X, S::V_X) = dt;
		this->F(S::P_Y, S::V_Y) = dt;
		this->F(S::P_Z, S::V_Z) = dt;


		this->F(S::P_X, S::Q_W) = qvr(0, 0) * dt * dt / 2;
		this->F(S::P_X, S::Q_X) = qvr(0, 1) * dt * dt / 2;
		this->F(S::P_X, S::Q_Y) = qvr(0, 2) * dt * dt / 2;
		this->F(S::P_X, S::Q_Z) = qvr(0, 3) * dt * dt / 2;

		this->F(S::P_Y, S::Q_W) = qvr(1, 0) * dt * dt / 2;
		this->F(S::P_Y, S::Q_X) = qvr(1, 1) * dt * dt / 2;
		this->F(S::P_Y, S::Q_Y) = qvr(1, 2) * dt * dt / 2;
		this->F(S::P_Y, S::Q_Z) = qvr(1, 3) * dt * dt / 2;

		this->F(S::P_Z, S::Q_W) = qvr(2, 0) * dt * dt / 2;
		this->F(S::P_Z, S::Q_X) = qvr(2, 1) * dt * dt / 2;
		this->F(S::P_Z, S::Q_Y) = qvr(2, 2) * dt * dt / 2;
		this->F(S::P_Z, S::Q_Z) = qvr(2, 3) * dt * dt / 2;


		this->F(S::P_X, S::B_A_X) = -qrm(0, 0) * dt * dt / 2;
		this->F(S::P_X, S::B_A_Y) = -qrm(0, 1) * dt * dt / 2;
		this->F(S::P_X, S::B_A_Z) = -qrm(0, 2) * dt * dt / 2;

		this->F(S::P_Y, S::B_A_X) = -qrm(1, 0) * dt * dt / 2;
		this->F(S::P_Y, S::B_A_Y) = -qrm(1, 1) * dt * dt / 2;
		this->F(S::P_Y, S::B_A_Z) = -qrm(1, 2) * dt * dt / 2;

		this->F(S::P_Z, S::B_A_X) = -qrm(2, 0) * dt * dt / 2;
		this->F(S::P_Z, S::B_A_Y) = -qrm(2, 1) * dt * dt / 2;
		this->F(S::P_Z, S::B_A_Z) = -qrm(2, 2) * dt * dt / 2;

		// partial derivatives of velocity
		this->F(S::V_X, S::V_X) = 1;
		this->F(S::V_Y, S::V_Y) = 1;
		this->F(S::V_Z, S::V_Z) = 1;


		this->F(S::V_X, S::Q_W) = qvr(0, 0) * dt;
		this->F(S::V_X, S::Q_X) = qvr(0, 1) * dt;
		this->F(S::V_X, S::Q_Y) = qvr(0, 2) * dt;
		this->F(S::V_X, S::Q_Z) = qvr(0, 3) * dt;

		this->F(S::V_Y, S::Q_W) = qvr(1, 0) * dt;
		this->F(S::V_Y, S::Q_X) = qvr(1, 1) * dt;
		this->F(S::V_Y, S::Q_Y) = qvr(1, 2) * dt;
		this->F(S::V_Y, S::Q_Z) = qvr(1, 3) * dt;

		this->F(S::V_Z, S::Q_W) = qvr(2, 0) * dt;
		this->F(S::V_Z, S::Q_X) = qvr(2, 1) * dt;
		this->F(S::V_Z, S::Q_Y) = qvr(2, 2) * dt;
		this->F(S::V_Z, S::Q_Z) = qvr(2, 3) * dt;


		this->F(S::V_X, S::B_A_X) = -qrm(0, 0) * dt;
		this->F(S::V_X, S::B_A_Y) = -qrm(0, 1) * dt;
		this->F(S::V_X, S::B_A_Z) = -qrm(0, 2) * dt;

		this->F(S::V_Y, S::B_A_X) = -qrm(1, 0) * dt;
		this->F(S::V_Y, S::B_A_Y) = -qrm(1, 1) * dt;
		this->F(S::V_Y, S::B_A_Z) = -qrm(1, 2) * dt;

		this->F(S::V_Z, S::B_A_X) = -qrm(2, 0) * dt;
		this->F(S::V_Z, S::B_A_Y) = -qrm(2, 1) * dt;
		this->F(S::V_Z, S::B_A_Z) = -qrm(2, 2) * dt;

		// partial derivatives of orientation
		this->F(S::Q_W, S::Q_W) = coswnorm;
		this->F(S::Q_W, S::Q_X) = -wxsin;
		this->F(S::Q_W, S::Q_Y) = -wysin;
		this->F(S::Q_W, S::Q_Z) = -wzsin;

		this->F(S::Q_X, S::Q_W) = wxsin;
		this->F(S::Q_X, S::Q_X) = coswnorm;
		this->F(S::Q_X, S::Q_Y) = wzsin;
		this->F(S::Q_X, S::Q_Z) = -wysin;

		this->F(S::Q_Y, S::Q_W) = wysin;
		this->F(S::Q_Y, S::Q_X) = -wzsin;
		this->F(S::Q_Y, S::Q_Y) = coswnorm;
		this->F(S::Q_Y, S::Q_Z) = wxsin;

		this->F(S::Q_Z, S::Q_W) = wzsin;
		this->F(S::Q_Z, S::Q_X) = wysin;
		this->F(S::Q_Z, S::Q_Y) = -wxsin;
		this->F(S::Q_Z, S::Q_Z) = coswnorm;


		this->F(S::Q_W, S::B_W_X) = qxsin;
		this->F(S::Q_W, S::B_W_Y) = qysin;
		this->F(S::Q_W, S::B_W_Z) = qzsin;

		this->F(S::Q_X, S::B_W_X) = -qwsin;
		this->F(S::Q_X, S::B_W_Y) = qzsin;
		this->F(S::Q_X, S::B_W_Z) = -qysin;

		this->F(S::Q_Y, S::B_W_X) = -qzsin;
		this->F(S::Q_Y, S::B_W_Y) = -qwsin;
		this->F(S::Q_Y, S::B_W_Z) = qxsin;

		this->F(S::Q_Z, S::B_W_X) = qysin;
		this->F(S::Q_Z, S::B_W_Y) = -qxsin;
		this->F(S::Q_Z, S::B_W_Z) = -qwsin;

		// partial derivatives of acceleration bias
		this->F(S::B_A_X, S::B_A_X) = 1;
		this->F(S::B_A_Y, S::B_A_Y) = 1;
		this->F(S::B_A_Z, S::B_A_Z) = 1;

		// partial derivatives of angular velocity bias
		this->F(S::B_W_X, S::B_W_X) = 1;
		this->F(S::B_W_Y, S::B_W_Y) = 1;
		this->F(S::B_W_Z, S::B_W_Z) = 1;



		// W = df/dw (Jacobian of state transition w.r.t. the noise)
		this->W.setZero();

		// partial derivatives of position
		this->W(S::P_X, C::A_X) = qrm(0, 0) * dt * dt / 2;
		this->W(S::P_X, C::A_Y) = qrm(0, 1) * dt * dt / 2;
		this->W(S::P_X, C::A_Z) = qrm(0, 2) * dt * dt / 2;

		this->W(S::P_Y, C::A_X) = qrm(1, 0) * dt * dt / 2;
		this->W(S::P_Y, C::A_Y) = qrm(1, 1) * dt * dt / 2;
		this->W(S::P_Y, C::A_Z) = qrm(1, 2) * dt * dt / 2;

		this->W(S::P_Z, C::A_X) = qrm(2, 0) * dt * dt / 2;
		this->W(S::P_Z, C::A_Y) = qrm(2, 1) * dt * dt / 2;
		this->W(S::P_Z, C::A_Z) = qrm(2, 2) * dt * dt / 2;

		// partial derivatives of velocity
		this->W(S::V_X, C::A_X) = qrm(0, 0) * dt;
		this->W(S::V_X, C::A_Y) = qrm(0, 1) * dt;
		this->W(S::V_X, C::A_Z) = qrm(0, 2) * dt;

		this->W(S::V_Y, C::A_X) = qrm(1, 0) * dt;
		this->W(S::V_Y, C::A_Y) = qrm(1, 1) * dt;
		this->W(S::V_Y, C::A_Z) = qrm(1, 2) * dt;

		this->W(S::V_Z, C::A_X) = qrm(2, 0) * dt;
		this->W(S::V_Z, C::A_Y) = qrm(2, 1) * dt;
		this->W(S::V_Z, C::A_Z) = qrm(2, 2) * dt;

		//partial derivatives of orientation
		this->W(S::Q_W, C::W_X) = qxsin;
		this->W(S::Q_W, C::W_Y) = qysin;
		this->W(S::Q_W, C::W_Z) = qzsin;

		this->W(S::Q_X, C::W_X) = -qwsin;
		this->W(S::Q_X, C::W_Y) = qzsin;
		this->W(S::Q_X, C::W_Z) = -qysin;

		this->W(S::Q_Y, C::W_X) = -qzsin;
		this->W(S::Q_Y, C::W_Y) = -qwsin;
		this->W(S::Q_Y, C::W_Z) = qxsin;

		this->W(S::Q_Z, C::W_X) = qysin;
		this->W(S::Q_Z, C::W_Y) = -qxsin;
		this->W(S::Q_Z, C::W_Z) = -qwsin;

		// partial derivatives of acceleration bias
		this->W(S::B_A_X, C::A_X) = 1;
		this->W(S::B_A_Y, C::A_Y) = 1;
		this->W(S::B_A_Z, C::A_Z) = 1;

		// partial derivatives of angular velocity bias
		this->W(S::B_W_X, C::W_X) = 1;
		this->W(S::B_W_Y, C::W_Y) = 1;
		this->W(S::B_W_Z, C::W_Z) = 1;
    }
};

} // namespace RambachModel
} // namespace FUSION
} // namespace MODULES
} // namespace SolAR

#endif // RAMBACHMODEL_SYSTEMMODEL_HPP
