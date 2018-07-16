/**
 * @copyright Copyright (c) 2015 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include <iostream>

#include "xpcf/module/ModuleFactory.h"

#include "SolARVisualInertialEKF.h"

namespace xpcf=org::bcom::xpcf;

XPCF_DECLARE_MODULE("2afa5c9c-85e0-11e8-adc0-fa7ae01bbebc", "SolARModuleFusion")


extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID,SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
     xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
     errCode = xpcf::tryCreateComponent<SolAR::MODULES::FUSION::SolARVisualInertialEKF>(componentUUID,interfaceRef);
    return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::MODULES::FUSION::SolARVisualInertialEKF)
XPCF_END_COMPONENTS_DECLARATION
