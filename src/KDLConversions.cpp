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

}
