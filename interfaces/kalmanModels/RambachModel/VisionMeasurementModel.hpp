/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef RAMBACHMODEL_VISIONMEASUREMENTMODEL_HPP
#define RAMBACHMODEL_VISIONMEASUREMENTMODEL_HPP

#include <kalman/LinearizedMeasurementModel.hpp>
#include "datastructure/MathDefinitions.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace FUSION {
namespace RambachModel
{

/**
 * @brief Measurement vector measuring a position and an orientation (i.e. by using a vision-based pose estimation pipeline)
 *
 * @param T Numeric scalar type
 */
template<typename T>
class VisionMeasurement : public Kalman::Vector<T, 7>
{
public:
    KALMAN_VECTOR(VisionMeasurement, T, 7)
    
	//! X-position
	static constexpr size_t P_X = 0;
	//! Y-position
	static constexpr size_t P_Y = 1;
	//! Z-position
	static constexpr size_t P_Z = 2;

	//! Orientation quaternion
	//! W-component
	static constexpr size_t Q_W = 3;
	//! X-component
	static constexpr size_t Q_X = 4;
	//! Y-component
	static constexpr size_t Q_Y = 5;
	//! Z-component
	static constexpr size_t Q_Z = 6;

	T p_x()       const { return (*this)[ P_X ]; }
	T p_y()       const { return (*this)[ P_Y ]; }
	T p_z()       const { return (*this)[ P_Z ]; }

	T q_w()       const { return (*this)[ Q_W ]; }
	T q_x()       const { return (*this)[ Q_X ]; }
	T q_y()       const { return (*this)[ Q_Y ]; }
	T q_z()       const { return (*this)[ Q_Z ]; }

	T& p_x() { return (*this)[ P_X ]; }
	T& p_y() { return (*this)[ P_Y ]; }
	T& p_z() { return (*this)[ P_Z ]; }

	T& q_w() { return (*this)[ Q_W ]; }
	T& q_x() { return (*this)[ Q_X ]; }
	T& q_y() { return (*this)[ Q_Y ]; }
	T& q_z() { return (*this)[ Q_Z ]; }

    friend std::ostream& operator<<(std::ostream& os, VisionMeasurement<T>& vision) {
        os << vision.p_x() << ", " << vision.p_y() << ", " << vision.p_z() << ", "
            << vision.q_w() << ", " << vision.q_x() << ", " << vision.q_y() << ", " << vision.q_z();
        return os;
    }
};

/**
 * @brief Measurement model for measuring position and orientation of a 6DOF AR device
 *
 * This is the measurement model for measuring the position and orientation of our
 * AR device. This could be realized by a vision-based pose estimation pipeline.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class VisionMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, VisionMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef RambachModel::State<T> S;
    
    //! Measurement type shortcut definition
    typedef RambachModel::VisionMeasurement<T> M;
    
	VisionMeasurementModel()
    {
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
        this->H.setZero();

		// partial derivatives of position
		this->H(M::P_X, S::P_X) = 1;
		this->H(M::P_Y, S::P_Y) = 1;
		this->H(M::P_Z, S::P_Z) = 1;

		// partial derivatives of orientation
		this->H(M::Q_W, S::Q_W) = 1;
		this->H(M::Q_X, S::Q_X) = 1;
		this->H(M::Q_Y, S::Q_Y) = 1;
		this->H(M::Q_Z, S::Q_Z) = 1;

        this->V.setIdentity();
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
    M h(const S& x) const
    {
        M measurement;
        
        // Measurement is given by the actual device position and orientation
        measurement.p_x() = x.p_x();
		measurement.p_y() = x.p_y();
		measurement.p_z() = x.p_z();

		measurement.q_w() = x.q_w();
		measurement.q_x() = x.q_x();
		measurement.q_y() = x.q_y();
		measurement.q_z() = x.q_z();
        
        return measurement;
    }
};

} // namespace RambachModel
} // namespace FUSION
} // namespace MODULES
} // namespace SolAR

#endif // RAMBACHMODEL_VISIONMEASUREMENTMODEL_HPP
