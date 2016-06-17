#ifndef KDL_CONVERSIONS_HPP
#define KDL_CONVERSIONS_HPP

#include <iostream>
#include <kdl/frames.hpp>
#include <base/samples/rigid_body_state.h>
#include <base/JointLimits.hpp>
#include <base/samples/Joints.hpp>
#include<kdl/jntarray.hpp>

namespace kdl_conversions
{
void KDL2RigidBodyState(const KDL::Frame &in, base::samples::RigidBodyState& out);
void KDL2RigidBodyState(const KDL::Twist &in, base::samples::RigidBodyState& out);
void KDL2RigidBodyState(const KDL::Frame &pose_in, const KDL::Twist &twist_in, base::samples::RigidBodyState& out);

void RigidBodyState2KDL(const base::samples::RigidBodyState& in, KDL::Frame& out);
void RigidBodyState2KDL(const base::samples::RigidBodyState& in, KDL::Twist& out);
void RigidBodyState2KDL(const base::samples::RigidBodyState& in, KDL::Frame& pose_out, KDL::Twist& twist_out);

void baseJointLimits2KDLJntLimitArray(const std::string& startJoint, const std::string& endJoint,const base::JointLimits& jointLimits, KDL::JntArray& qMin, KDL::JntArray& qMax);
void baseJointLimits2KDLJntLimitArray(const base::JointLimits& jointLimits, KDL::JntArray& qMin, KDL::JntArray& qMax);

void baseJoints2KDLJntArray(const std::string& startJoint, const std::string& endJoint, const base::samples::Joints& jointSamples, KDL::JntArray& jntArray);
void baseJoints2KDLJntArray(const base::samples::Joints& jointSamples, KDL::JntArray& jntArray);

void KDLJntArray2baseJoints(const std::string& startJoint, const std::string& endJoint, const KDL::JntArray& jntArray, base::samples::Joints& jointSamples);
void KDLJntArray2baseJoints(const KDL::JntArray& jntArray, base::samples::Joints& jointSamples);

} // end namespace kdl_conversions

#endif
