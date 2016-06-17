#include "KDLConversions.hpp"

namespace kdl_conversions
{

void KDL2RigidBodyState(const KDL::Frame &in, base::samples::RigidBodyState& out){
    out.position << in.p(0), in.p(1), in.p(2);
    double x, y, z, w;
    in.M.GetQuaternion( x, y, z, w);
    out.orientation = base::Quaterniond(w, x, y, z);
}

void KDL2RigidBodyState(const KDL::Twist &in, base::samples::RigidBodyState& out){
    out.velocity << in(0), in(1), in(2);
    out.angular_velocity << in(3), in(4), in(5);
}

void KDL2RigidBodyState(const KDL::Frame &pose_in, const KDL::Twist &twist_in, base::samples::RigidBodyState& out){
    KDL2RigidBodyState(pose_in, out);
    KDL2RigidBodyState(twist_in, out);
}

void RigidBodyState2KDL(const base::samples::RigidBodyState& in, KDL::Frame& out){
    out.p = KDL::Vector(in.position(0), in.position(1), in.position(2));
    out.M = KDL::Rotation::Quaternion( in.orientation.x(),
                                       in.orientation.y(),
                                       in.orientation.z(),
                                       in.orientation.w());
}

void RigidBodyState2KDL(const base::samples::RigidBodyState& in, KDL::Twist& out){
    out = KDL::Twist(KDL::Vector(in.velocity(0), in.velocity(1), in.velocity(2)),
                     KDL::Vector(in.angular_velocity(0), in.angular_velocity(1), in.angular_velocity(2)));
}

void RigidBodyState2KDL(const base::samples::RigidBodyState& in, KDL::Frame& pose_out, KDL::Twist& twist_out){
    RigidBodyState2KDL(in, pose_out);
    RigidBodyState2KDL(in, twist_out);
}

void BaseVector3dToKDLVector(const base::Vector3d& in, KDL::Vector& out){
    for(uint i = 0; i < 3; i++)
        out(i) = in(i);
}

void WrenchToKDLWrench(const base::samples::Wrench& in, KDL::Wrench& out){
    BaseVector3dToKDLVector(in.force, out.force);
    BaseVector3dToKDLVector(in.torque, out.torque);
}

void baseJointLimits2KDLJntLimitArray(const base::JointLimits& jointLimits, KDL::JntArray& qMin, KDL::JntArray& qMax)
{
  qMin.resize(jointLimits.names.size());
  qMax.resize(jointLimits.names.size());
  for(unsigned int i=0; i<jointLimits.names.size(); i++)
  {
    qMin(i) = jointLimits[i].min.position;
    qMax(i) = jointLimits[i].max.position;
  }
}

void baseJointLimits2KDLJntLimitArray(const std::string& startJoint, const std::string& endJoint, const base::JointLimits& jointLimits, KDL::JntArray& qMin, KDL::JntArray& qMax)
{
  qMin.resize(jointLimits.names.size());
  qMax.resize(jointLimits.names.size());
  for(unsigned int i=jointLimits.mapNameToIndex(startJoint); i<jointLimits.mapNameToIndex(endJoint)+1; i++)
  {
    qMin(i) = jointLimits[i].min.position;
    qMax(i) = jointLimits[i].max.position;
  }
}

void baseJoints2KDLJntArray(const base::samples::Joints& jointSamples, KDL::JntArray& jntArray)
{
  jntArray.resize(jointSamples.names.size());
  
  for(unsigned int i=0; i<jointSamples.names.size(); i++)
  {
    jntArray(i) = jointSamples.elements[i].position;
  }
}

void baseJoints2KDLJntArray(const std::string& startJoint, const std::string& endJoint, const base::samples::Joints& jointSamples, KDL::JntArray& jntArray)
{
  jntArray.resize(jointSamples.names.size());
  for(unsigned int i=jointSamples.mapNameToIndex(startJoint); i<jointSamples.mapNameToIndex(endJoint)+1; i++)
  {
    jntArray(i) = jointSamples.elements[i].position;
  }
}

void KDLJntArray2baseJoints(const KDL::JntArray& jntArray, base::samples::Joints& jointSamples)
{
  if(jntArray.rows()!=jointSamples.elements.size())
  {
    throw std::runtime_error("KDLJntArray2baseJoints: Mismatch in number of elements in KDL Joint Array annd base Joint Samples");
  }
  else
  {
    for(unsigned int i=0; i<jointSamples.names.size(); i++)
    {
      jointSamples.elements[i].position = jntArray(i);
    }
  }
}

void KDLJntArray2baseJoints(const std::string& startJoint, const std::string& endJoint, const KDL::JntArray& jntArray, base::samples::Joints& jointSamples)
{
  unsigned start= jointSamples.mapNameToIndex(startJoint);
  unsigned end= jointSamples.mapNameToIndex(endJoint);
  
  if(jntArray.rows()!= end-start+1)
  {
    throw std::runtime_error("KDLJntArray2baseJoints: Mismatch in number of elements in KDL Joint Array annd base Joint Samples");
  }
  else
  {
    for(unsigned int i=jointSamples.mapNameToIndex(startJoint); i<jointSamples.mapNameToIndex(endJoint)+1; i++)
    {
      jointSamples.elements[i].position = jntArray(i);
    }
  }    
}

}
