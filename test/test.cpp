#include <boost/test/unit_test.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <kdl/frames_io.hpp>
#include <stdlib.h>

using namespace kdl_conversions;
using namespace std;


BOOST_AUTO_TEST_CASE(from_kdl_to_base_types)
{
    srand (time(NULL));

    const double X_ROT = (rand()%1000)/1000.0;;
    const double Y_ROT = (rand()%1000)/1000.0;;
    const double Z_ROT = (rand()%1000)/1000.0;;

    const double X = (rand()%1000)/1000.0;;
    const double Y = (rand()%1000)/1000.0;;
    const double Z = (rand()%1000)/1000.0;;

    const double X_VEL = (rand()%1000)/1000.0;;
    const double Y_VEL = (rand()%1000)/1000.0;;
    const double Z_VEL = (rand()%1000)/1000.0;;

    const double X_ROT_VEL = (rand()%1000)/1000.0;;
    const double Y_ROT_VEL = (rand()%1000)/1000.0;;
    const double Z_ROT_VEL = (rand()%1000)/1000.0;;

    cout<<"Pose Data: "<<endl;
    cout<<"Position: ["<<X<<", "<<Y<<", "<<Z<<"]"<<endl;
    cout<<"Orientation: "<<endl;
    cout<<"Rotating by "<<X_ROT<<" rad around x-axis"<<endl;
    cout<<"Then rotating by "<<Y_ROT<<" rad around new y-axis"<<endl;
    cout<<"Then Rotating by "<<Z_ROT<<" rad around new z-axis"<<endl;
    cout<<endl;

    cout<<"Twist Data: "<<endl;
    cout<<"Translational Vel: ["<<X_VEL<<", "<<Y_VEL<<", "<<Z_VEL<<"]"<<endl;
    cout<<"Rotational Vel:  ["<<X_ROT_VEL<<", "<<Y_ROT_VEL<<", "<<Z_ROT_VEL<<"]"<<endl;
    cout<<endl;

    KDL::Frame f_in;
    f_in.Identity();
    f_in.p = KDL::Vector(X,Y,Z);
    f_in.M.DoRotX(X_ROT);
    f_in.M.DoRotX(Y_ROT);
    f_in.M.DoRotZ(Z_ROT);
    KDL::Twist tw_in(KDL::Vector(X_VEL, Y_VEL, Z_VEL), KDL::Vector(X_ROT_VEL, Y_ROT_VEL, Z_ROT_VEL));
    for(int i = 0; i < 9; i++ )
        if(fabs(f_in.M.data[i]) < 1e-5 )
            f_in.M.data[i] = 0;

    cout<<"Pose KDL: "<<endl;
    cout<<f_in<<endl;
    double rpy[3], q[4];
    f_in.M.GetRPY(rpy[0], rpy[1], rpy[2]);
    f_in.M.GetQuaternion(q[0], q[1], q[2], q[3]);
    cout<<"RPY: "<<rpy[0]<<" "<<rpy[1]<< " "<<rpy[2]<<endl;
    cout<<"Quaternion: "<<q[0]<<" "<<q[1]<< ""<<q[2]<<" "<<q[3]<<endl;
    cout<<"Twist KDL: "<<tw_in<<endl;

    cout<<".............................."<<endl;

    base::samples::RigidBodyState rbs;
    KDL2RigidBodyState(f_in, tw_in, rbs);

    cout<<"Rigid Body State: "<<endl;
    for(int i = 0; i < 4; i++ ){
        for(int j = 0; j < 4; j++){
            if(fabs(rbs.getTransform()(i,j)) < 1e-5 )
                cout<<0<<" ";
            else
                cout<<rbs.getTransform()(i,j)<<" ";
        }
        cout<<endl;
    }
    cout<<endl;
    cout<<"RPY: "<<rbs.getRoll()<<" "<<rbs.getPitch()<<" "<<rbs.getYaw()<<endl;
    cout<<"Quaternion: "<<rbs.orientation.x()<<" "<<rbs.orientation.y()<<" "<<rbs.orientation.z()<<" "<<rbs.orientation.w()<<endl;
    cout<<"Twist: "<<endl; for(int i = 0; i < 3; i++) cout<<rbs.velocity(i)<<" ";  for(int i = 0; i < 3; i++) cout<<rbs.angular_velocity(i)<<" "; cout<<endl;
    cout<<".............................."<<endl;

    KDL::Frame f_out;
    KDL::Twist tw_out;
    RigidBodyState2KDL(rbs, f_out, tw_out);
    cout<<"Pose KDL Out: "<<endl;
    for(int i = 0; i < 9; i++ )
        if(fabs(f_out.M.data[i]) < 1e-5 )
            f_out.M.data[i] = 0;
    cout<<f_out<<endl;
    f_out.M.GetRPY(rpy[0], rpy[1], rpy[2]);
    f_out.M.GetQuaternion(q[0], q[1], q[2], q[3]);
    cout<<"RPY: "<<rpy[0]<<" "<<rpy[1]<< " "<<rpy[2]<<endl;
    cout<<"Quaternion: "<<q[0]<<" "<<q[1]<< ""<<q[2]<<" "<<q[3]<<endl;
    cout<<"Twist: "<<tw_out<<endl;

    for(int i = 0; i < 3; i++ )
        BOOST_CHECK_EQUAL( fabs(f_in.p(i) - f_out.p(i)) < 1e-5, true );
    for(int i = 0; i < 6; i++ )
        BOOST_CHECK_EQUAL( fabs(tw_in(i) - tw_out(i)) < 1e-5, true );
    for(int i = 0; i < 9; i++ )
        BOOST_CHECK_EQUAL( fabs(f_in.M.data[i] - f_out.M.data[i]) < 1e-5, true );

}
