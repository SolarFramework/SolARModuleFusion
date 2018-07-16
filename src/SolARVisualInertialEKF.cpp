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

#include "SolARVisualInertialEKF.h"

namespace xpcf  = org::bcom::xpcf;
XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::FUSION::SolARVisualInertialEKF)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace FUSION {

    SolARVisualInertialEKF::SolARVisualInertialEKF():ComponentBase(xpcf::toUUID<SolARVisualInertialEKF>())
    {
        addInterface<api::fusion::IVisualInertialFusion>(this);
    }

    void SolARVisualInertialEKF::init(SolAR::api::fusion::VisionData & initialPose)
    {
        Vector<T, 3> position;
        Quaternion<T> orientation;
        poseToPositionAndOrientation(initialPose.pose.inverse(), position, orientation);
        kalmanToOpenCVBase(position, orientation);

        State x;
        x.p_x() = position(0);
        x.p_y() = position(1);
        x.p_z() = position(2);

        x.q_w() = orientation.w();
        x.q_x() = orientation.x();
        x.q_y() = orientation.y();
        x.q_z() = orientation.z();

        // Init filter with initial state
        m_ekf.init(x);

        // Set process noise P = 10
        m_ekf.setCovariance(m_ekf.getCovariance() * 10);

        // Set Control noise covariance and Measurement noise covariance
        Matrix<T, 6, 6> controlCovariance;
        Matrix<T, 7 ,7> measurementCovariance;

        controlCovariance.setZero();
        controlCovariance(Control::A_X, Control::A_X) = 0.175;
        controlCovariance(Control::A_Y, Control::A_Y) = 0.175;
        controlCovariance(Control::A_Z, Control::A_Z) = 0.175;

        controlCovariance(Control::W_X, Control::W_X) = 0.175;
        controlCovariance(Control::W_Y, Control::W_Y) = 0.175;
        controlCovariance(Control::W_Z, Control::W_Z) = 0.175;

        measurementCovariance.setZero();
        measurementCovariance(VisionMeasurement::P_X, VisionMeasurement::P_X) = 0.005;
        measurementCovariance(VisionMeasurement::P_Y, VisionMeasurement::P_Y) = 0.005;
        measurementCovariance(VisionMeasurement::P_Z, VisionMeasurement::P_Z) = 0.005;

        measurementCovariance(VisionMeasurement::Q_W, VisionMeasurement::Q_W) = 0.001;
        measurementCovariance(VisionMeasurement::Q_X, VisionMeasurement::Q_X) = 0.001;
        measurementCovariance(VisionMeasurement::Q_Y, VisionMeasurement::Q_Y) = 0.001;
        measurementCovariance(VisionMeasurement::Q_Z, VisionMeasurement::Q_Z) = 0.001;

        m_sys.setCovariance(controlCovariance);

        m_vm.setCovariance(measurementCovariance);

        m_lastKalman = initialPose.timestamp;
    }

    void SolARVisualInertialEKF::addInertialData(SolAR::api::fusion::InertialData & inertialData)
    {
        m_inertialQueue.push(inertialData);
    }

    void SolARVisualInertialEKF::addVisionData(SolAR::api::fusion::VisionData & visionData)
    {
        m_visionQueue.push(visionData);
    }

    FrameworkReturnCode SolARVisualInertialEKF::process(Transform3Df & pose)
    {
        bool hasLooped = false;
        State x_ekf;

        while (!hasLooped && !m_visionQueue.empty() && !m_inertialQueue.empty())
        {
            SolAR::api::fusion::InertialData inertialData;
            SolAR::api::fusion::VisionData visionData;

            inertialData = m_inertialQueue.back();
            visionData = m_visionQueue.back();

            if (inertialData.timestamp < visionData.timestamp)
            {
                m_inertialQueue.pop(); // no need to get the data again

                Control control;
                control.a_x() = inertialData.accelData[0];
                control.a_y() = inertialData.accelData[1];
                control.a_z() = inertialData.accelData[2];

                control.w_x() = inertialData.gyroData[0];
                control.w_y() = inertialData.gyroData[1];
                control.w_z() = inertialData.gyroData[2];

                //process imu
                m_sys.dt = std::chrono::duration_cast<std::chrono::seconds>(inertialData.timestamp - m_lastKalman).count();
                x_ekf = m_ekf.predict(m_sys, control);
                m_lastKalman = inertialData.timestamp;
            }
            else
            {
                hasLooped = true;

                m_visionQueue.pop(); // same here

                m_sys.dt = (visionData.timestamp - m_lastKalman).count() * 0.000000001;
                x_ekf = m_ekf.predict(m_sys);

                if (visionData.isPoseValid)
                {
                    Vector<T, 3> position;
                    Quaternion<T> orientation;
                    VisionMeasurement vision;

                    Transform3Df inv = visionData.pose.inverse();

                    poseToPositionAndOrientation(inv, position, orientation);
                    kalmanToOpenCVBase(position, orientation);

                    vision.p_x() = position(0);
                    vision.p_y() = position(1);
                    vision.p_z() = position(2);

                    vision.q_w() = orientation.w();
                    vision.q_x() = orientation.x();
                    vision.q_y() = orientation.y();
                    vision.q_z() = orientation.z();

                    x_ekf = m_ekf.update(m_vm, vision);
                    m_lastKalman = visionData.timestamp;

                    orientation.w() = x_ekf.q_w();
                    orientation.x() = x_ekf.q_x();
                    orientation.y() = x_ekf.q_y();
                    orientation.z() = x_ekf.q_z();

                    orientation.normalize();

                    // apply normalized quaternion
                    x_ekf.q_w() = orientation.w();
                    x_ekf.q_x() = orientation.x();
                    x_ekf.q_y() = orientation.y();
                    x_ekf.q_z() = orientation.z();

                    m_ekf.init(x_ekf);
                }
            }
        }

        if (hasLooped)
        {
            Vector<T, 3> position;
            Quaternion<T> orientation;

            position(0) = T(x_ekf.p_x());
            position(1) = T(x_ekf.p_y());
            position(2) = T(x_ekf.p_z());

            orientation.w() = T(x_ekf.q_w());
            orientation.x() = T(x_ekf.q_x());
            orientation.y() = T(x_ekf.q_y());
            orientation.z() = T(x_ekf.q_z());

            kalmanToOpenCVBase(position, orientation);

            pose = constructPose(Transform3Df(orientation.toRotationMatrix().cast<float>()), position.cast<float>()); // cast to float to prevent Eigen's "MIXED TYPES WHEN USING DOUBLE"
            pose = pose.inverse();

            return FrameworkReturnCode::_SUCCESS;
        }

        return FrameworkReturnCode::_ERROR_;
    }

    void SolARVisualInertialEKF::poseToPositionAndOrientation(Transform3Df & tr, Vector<T,3>& position, Quaternion<T>& orientation)
    {
        position = Vector<T,3>(tr(0, 3), tr(1, 3), tr(2, 3));

        orientation = Quaternion<T>();
        orientation.w() = std::sqrt(1 + tr(0, 0) + tr(1, 1) + tr(2, 2)) / 2;
        orientation.x() = (tr(2, 1) - tr(1, 2)) / (4 * orientation.w());
        orientation.y() = (tr(0, 2) - tr(2, 0)) / (4 * orientation.w());
        orientation.z() = (tr(1, 0) - tr(0, 1)) / (4 * orientation.w());
    }

    void SolARVisualInertialEKF::kalmanToOpenCVBase(Vector<T, 3>& position, Quaternion<T>& orientation)
    {
        position(1) = -position(1);

        orientation.w() = -orientation.w();
        orientation.y() = -orientation.y();
    }

    Transform3Df SolARVisualInertialEKF::constructPose(const  Transform3Df &r, const  Vector3f & t)
    {

        Transform3Df m_poseTransform;
        m_poseTransform(0,0) =  r(0,0);  m_poseTransform(0,1) = r(0,1);   m_poseTransform(0,2) = r(0,2);
        m_poseTransform(1,0) =  r(1,0);  m_poseTransform(1,1) = r(1,1);   m_poseTransform(1,2) = r(1,2);
        m_poseTransform(2,0) =  r(2,0);  m_poseTransform(2,1) = r(2,1);   m_poseTransform(2,2) = r(2,2);

        m_poseTransform(0,3) =   t[0];
        m_poseTransform(1,3) =   t[1];
        m_poseTransform(2,3) =   t[2];

        m_poseTransform(3,0) = 0; m_poseTransform(3,1) = 0; m_poseTransform(3,2) = 0; m_poseTransform(3,3) = 1;

        return m_poseTransform;
    }
}
}
}
