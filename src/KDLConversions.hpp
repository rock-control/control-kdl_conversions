#ifndef KDL_CONVERSIONS_HPP
#define KDL_CONVERSIONS_HPP

#include <iostream>
#include <kdl/frames.hpp>
#include <base/samples/rigid_body_state.h>

namespace kdl_conversions
{
void KDL2RigidBodyState(const KDL::Frame &in, base::samples::RigidBodyState& out);
void KDL2RigidBodyState(const KDL::Twist &in, base::samples::RigidBodyState& out);
void KDL2RigidBodyState(const KDL::Frame &pose_in, const KDL::Twist &twist_in, base::samples::RigidBodyState& out);

void RigidBodyState2KDL(const base::samples::RigidBodyState& in, KDL::Frame& out);
void RigidBodyState2KDL(const base::samples::RigidBodyState& in, KDL::Twist& out);
void RigidBodyState2KDL(const base::samples::RigidBodyState& in, KDL::Frame& pose_out, KDL::Twist& twist_out);

} // end namespace kdl_conversions

#endif
