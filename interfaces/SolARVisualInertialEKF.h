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

#ifndef SOLARVISUALINERTIALEKF_H
#define SOLARVISUALINERTIALEKF_H

#include "xpcf/component/ComponentBase.h"
#include "api/fusion/IVisualInertialFusion.h"

#include "SolARFusionAPI.h"

#include "datastructure/Image.h"
#include "datastructure/MathDefinitions.h"
#include "SharedFifo.hpp"

#include "kalman/ExtendedKalmanFilter.hpp"

#include "kalmanModels/RambachModel/SystemModel.hpp"
#include "kalmanModels/RambachModel/VisionMeasurementModel.hpp"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace FUSION {

typedef double T;

// Define the model used
typedef RambachModel::State<T> State;
typedef RambachModel::Control<T> Control;
typedef RambachModel::SystemModel<T> SystemModel;
typedef RambachModel::VisionMeasurement<T> VisionMeasurement;
typedef RambachModel::VisionMeasurementModel<T> VisionModel;

class SOLARFUSION_EXPORT_API SolARVisualInertialEKF : public org::bcom::xpcf::ComponentBase,
        public api::fusion::IVisualInertialFusion {
public:
    SolARVisualInertialEKF();

    ~SolARVisualInertialEKF() = default;

    virtual void init(SolAR::api::fusion::VisionData & initialPose) override;

    virtual void addInertialData(SolAR::api::fusion::InertialData & inertialData) override;

    virtual void addVisionData(SolAR::api::fusion::VisionData & visionData) override;

    virtual FrameworkReturnCode process(Transform3Df & pose) override;

    void unloadComponent () override final;

 private:
    Kalman::ExtendedKalmanFilter<State> m_ekf;
    SystemModel m_sys;
    VisionModel m_vm;

    std::chrono::high_resolution_clock::time_point m_lastKalman;

    SharedFifo<SolAR::api::fusion::InertialData> m_inertialQueue;
    SharedFifo<SolAR::api::fusion::VisionData> m_visionQueue;

    void poseToPositionAndOrientation(Transform3Df & tr, Vector<T,3>& position, Quaternion<T>& orientation);

    void kalmanToOpenCVBase(Vector<T, 3>& position, Quaternion<T>& orientation);

    void constructPose(const  Transform3Df &r, const  Vector3f & t, Transform3Df & pose);
};

}
}
}

#endif // SOLARVISUALINERTIALEKF_H
