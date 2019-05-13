#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetLinkState.h>

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

#include <cmath>
#include <string>

#include <dji_m100_gazebo/pid.h>

#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif
// Copied from dji_sdk.h
enum M100FlightStatus{
    ON_GROUND = 1,
    TAKINGOFF = 2,
    IN_AIR = 3,
    LANDING = 4,
    LANDED = 5
};

namespace gazebo
{
    class DJI_ROS_ControlPlugin : public ModelPlugin{
        public:
            DJI_ROS_ControlPlugin(){}
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
                std::cout<<"\033[1;32m DJI ROS Control Plugin is successfully plugged in to "<<_model->GetName()<<"\033[0m\n";
                this->model = _model;
                this->world = _model->GetWorld();
                this->base_link = model->GetLink();
                this->gimbal_yaw_link = _model->GetLink("gimbal_yaw_link");

                std::cout<<"\033[1;32m Gimbal Yaw link exists : "<<this->gimbal_yaw_link->GetName()<<"\033[0m\n";
                
                ros::SubscribeOptions vel_ops = ros::SubscribeOptions::create<geometry_msgs::Vector3Stamped>(
                    "dji_sdk/velocity", 1000,
                    boost::bind(&DJI_ROS_ControlPlugin::velocityCallback, this, _1),
                    ros::VoidPtr(), &this->callback_queue);

                this->velocity_subscriber = nh.subscribe(vel_ops);

                ros::SubscribeOptions imu_ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
                    "dji_sdk/imu", 1000,
                    boost::bind(&DJI_ROS_ControlPlugin::imuCallback, this, _1),
                    ros::VoidPtr(), &this->callback_queue);
                this->imu_subscriber = nh.subscribe(imu_ops);

                ros::SubscribeOptions pos_ops = ros::SubscribeOptions::create<geometry_msgs::PointStamped>(
                    "dji_sdk/local_position", 1000,
                    boost::bind(&DJI_ROS_ControlPlugin::localPositionCallback, this, _1),
                    ros::VoidPtr(), &this->callback_queue);
                this->local_position_subscriber = nh.subscribe(pos_ops);
                
                ros::SubscribeOptions gimbal_ops = ros::SubscribeOptions::create<geometry_msgs::Vector3Stamped>(
                    "dji_sdk/gimbal_angle", 1000,
                    boost::bind(&DJI_ROS_ControlPlugin::gimbalOrientationCallback, this, _1),
                    ros::VoidPtr(), &this->callback_queue);
                this->local_position_subscriber = nh.subscribe(gimbal_ops);

                ros::SubscribeOptions flight_status_ops = ros::SubscribeOptions::create<std_msgs::UInt8>(
                    "dji_sdk/flight_status", 1000,
                    boost::bind(&DJI_ROS_ControlPlugin::flightStatusCallback, this, _1),
                    ros::VoidPtr(), &this->callback_queue);
                this->flight_status_subscriber = nh.subscribe(flight_status_ops);
            
                this->max_force = -1;
                this->motion_small_noise = 0;
                this->motion_drift_noise = 0;
                this->motion_drift_noise_time = 1.0;

                if(_sdf->HasElement("maxForce")) this->max_force = _sdf->GetElement("maxForce")->Get<double>();
                if(_sdf->HasElement("motionSmallNoise")) this->motion_small_noise = _sdf->GetElement("motionSmallNoise")->Get<double>();
                if(_sdf->HasElement("motionDriftNoise")) this->motion_drift_noise = _sdf->GetElement("motionDriftNoise")->Get<double>();
                if(_sdf->HasElement("motionDriftNoiseTime")) this->motion_drift_noise_time = _sdf->GetElement("motionDriftNoiseTime")->Get<double>();

                #if GAZEBO_MAJOR_VERSION >= 8
                    this->inertia = this->base_link->GetInertial()->PrincipalMoments();
                    this->mass = this->base_link->GetInertial()->Mass();
                #else
                    this->inertia = this->base_link->GetInertial()->GetPrincipalMoments();
                    this->mass = this->base_link->GetInertial()->GetMass();
                #endif
                this->controllers_.roll.Load(_sdf, "rollpitch");
                this->controllers_.pitch.Load(_sdf, "rollpitch");
                this->controllers_.yaw.Load(_sdf, "yaw");
                this->controllers_.velocity_x.Load(_sdf, "velocityXY");
                this->controllers_.velocity_y.Load(_sdf, "velocityXY");
                this->controllers_.velocity_z.Load(_sdf, "velocityZ");
                
                this->reset();
                this->flight_state = M100FlightStatus::LANDED;
                this->m_timeAfterCmd = 0;
                this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DJI_ROS_ControlPlugin::OnUpdate,this));
            }
            // The values are fetched with Inertial Measurement Unit. It is error prone.
            // Acceleration can be computed with simulation-based variables.
            void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
            {
                #if GAZEBO_MAJOR_VERSION >= 8
                    this->pose.Rot().Set(-imu->orientation.w, -imu->orientation.x, imu->orientation.y, imu->orientation.z);
                    this->euler = this->pose.Rot().Euler();
                    this->angular_velocity = this->pose.Rot().RotateVector(math::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
                #else
                    this->pose.rot.Set(-imu->orientation.w, -imu->orientation.x, imu->orientation.y, imu->orientation.z);
                    this->euler = this->pose.rot.GetAsEuler();
                    this->angular_velocity = this->pose.rot.RotateVector(math::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
                #endif
            }
            void flightStatusCallback(const std_msgs::UInt8::ConstPtr& flight_status)
            {
                if(flight_status->data == M100FlightStatus::TAKINGOFF && (this->flight_state == M100FlightStatus::ON_GROUND || this->flight_state == M100FlightStatus::LANDED))
                    this->flight_state = M100FlightStatus::TAKINGOFF;
                else if(flight_status->data == M100FlightStatus::LANDING && (this->flight_state == M100FlightStatus::IN_AIR || this->flight_state == M100FlightStatus::TAKINGOFF))
                    this->flight_state = M100FlightStatus::LANDING;
            }
            void velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& velocity_msg)
            {
                this->velocity_command.linear.x = velocity_msg->vector.y;
                this->velocity_command.linear.y = -velocity_msg->vector.x;
                this->velocity_command.linear.z = velocity_msg->vector.z;
                this->velocity_command.angular.x = 0;
                this->velocity_command.angular.y = 0;
                this->velocity_command.angular.z = 0;
                #if GAZEBO_MAJOR_VERSION >= 8
                    static common::Time last_sim_time = this->world->SimTime();
                #else
                    static common::Time last_sim_time = this->world->GetSimTime();
                #endif
                static double time_counter_for_drift_noise = 0;
                static double drift_noise[4] = {0.0, 0.0, 0.0, 0.0};
                // Get simulator time
                #if GAZEBO_MAJOR_VERSION >= 8
                    common::Time cur_sim_time = this->world->SimTime();
                #else
                    common::Time last_sim_time = this->world->GetSimTime();
                #endif
                double dt = (cur_sim_time - last_sim_time).Double();
                // save last time stamp
                last_sim_time = cur_sim_time;

                // generate noise
                if(time_counter_for_drift_noise > this->motion_drift_noise_time)
                {
                    drift_noise[0] = 2*this->motion_drift_noise*(drand48()-0.5);
                    drift_noise[1] = 2*this->motion_drift_noise*(drand48()-0.5);
                    drift_noise[2] = 2*this->motion_drift_noise*(drand48()-0.5);
                    drift_noise[3] = 2*this->motion_drift_noise*(drand48()-0.5);
                    time_counter_for_drift_noise = 0.0;
                }
                time_counter_for_drift_noise += dt;

                this->velocity_command.angular.x += drift_noise[0] + 2*this->motion_small_noise*(drand48()-0.5);
                this->velocity_command.angular.y += drift_noise[1] + 2*this->motion_small_noise*(drand48()-0.5);
                this->velocity_command.angular.z += drift_noise[3] + 2*this->motion_small_noise*(drand48()-0.5);
                this->velocity_command.linear.z += drift_noise[2] +  2*this->motion_small_noise*(drand48()-0.5);
            }
            void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& position_msg)
            {
                #if GAZEBO_MAJOR_VERSION >= 8
                    math::Vector3d old_position(this->pose.Pos());
                    this->pose.Pos().Set(position_msg->point.x,-position_msg->point.y,position_msg->point.z);    
                #else
                    math::Vector3d old_position(this->pose.pos);
                    this->pose.pos.Set(position_msg->point.x,-position_msg->point.y,position_msg->point.z);    
                #endif
            }
            void reset()
            {
                this->controllers_.roll.reset();
                this->controllers_.pitch.reset();
                this->controllers_.yaw.reset();
                this->controllers_.velocity_x.reset();
                this->controllers_.velocity_y.reset();
                this->controllers_.velocity_z.reset();

                this->base_link->SetForce(math::Vector3d(0,0,0));
                this->base_link->SetTorque(math::Vector3d(0,0,0));

                // reset state
                this->pose.Reset();
                this->linear_velocity.Set();
                this->angular_velocity.Set();
                this->position.Set();
                this->acceleration.Set();
                this->euler.Set();
            }
            void gimbalOrientationCallback(const geometry_msgs::Vector3Stamped::ConstPtr& gimbal_orientation_msg)
            {
                this->gimbal_orientation.X() = gimbal_orientation_msg->vector.x;
                this->gimbal_orientation.Y() = gimbal_orientation_msg->vector.y;
                this->gimbal_orientation.Z() = gimbal_orientation_msg->vector.z;
            }
            void OnUpdate(){
                this->callback_queue.callAvailable();
                // Get simulator time
                #if GAZEBO_MAJOR_VERSION >= 8
                    common::Time sim_time = this->world->SimTime();
                #else
                    common::Time sim_time = this->world->GetSimTime();
                #endif
                double dt = (sim_time - this->last_time).Double();
                if (dt == 0.0) return;
                UpdateState(dt);
                UpdateDynamics(dt);
                
                // save last time stamp
                this->last_time = sim_time;   
                #if GAZEBO_MAJOR_VERSION >= 8
                    math::Pose3d gimbal_pose = this->gimbal_yaw_link->WorldPose();
                    gimbal_pose.Set(gimbal_pose.Pos(),this->gimbal_orientation);
                #else
                    math::Pose3d gimbal_pose = this->gimbal_yaw_link->GetWorldPose();
                    gimbal_pose.Set(gimbal_pose.pos,this->gimbal_orientation);
                #endif
                this->gimbal_yaw_link->SetWorldPose(gimbal_pose);
            }
            void UpdateState(double dt)
            {
                if(this->flight_state == M100FlightStatus::TAKINGOFF){
                    this->m_timeAfterCmd += dt;
                    if (this->m_timeAfterCmd > 0.5){
                        this->flight_state = M100FlightStatus::IN_AIR;
                        std::cout << "Entering flying model!" << std::endl;
                    }
                }else if(this->flight_state == M100FlightStatus::LANDING){
                    this->m_timeAfterCmd += dt;
                    if(this->m_timeAfterCmd > 1.0){
                        this->flight_state = M100FlightStatus::LANDED;
                        std::cout << "Landed!" <<std::endl;
                    }
                }else
                    this->m_timeAfterCmd = 0;
            }
            void UpdateDynamics(double dt)
            {
                math::Vector3d force, torque;
                // Use Gazebo for states
                #if GAZEBO_MAJOR_VERSION >= 8
                    this->pose = this->base_link->WorldPose();
                    this->angular_velocity = this->base_link->WorldAngularVel();
                    this->euler = pose.Rot().Euler();    
                    this->acceleration = (this->base_link->WorldLinearVel() - this->linear_velocity) / dt;
                    this->linear_velocity = this->base_link->WorldLinearVel();
                #else
                    this->pose = this->base_link->GetWorldPose();
                    this->angular_velocity = this->base_link->GetWorldAngularVel();
                    this->euler = this->pose.rot.GetAsEuler();
                    this->acceleration = (this->base_link->GetWorldLinearVel() - this->linear_velocity) / dt;
                    this->linear_velocity = this->base_link->GetWorldLinearVel();
                #endif
                // .............
                //convert the acceleration and velocity into the body frame
                #if GAZEBO_MAJOR_VERSION >= 8
                    math::Vector3d body_vel = this->pose.Rot().RotateVector(this->linear_velocity);
                    math::Vector3d body_acc = this->pose.Rot().RotateVector(this->acceleration);
                    math::Vector3d poschange = this->pose.Pos() - this->position;
                    this->position = pose.Pos();
                    // Get gravity
                    math::Vector3d gravity_body = this->pose.Rot().RotateVector(this->world->Gravity());
                    double gravity = gravity_body.Length();
                #else
                    math::Vector3d body_vel = this->pose.rot.RotateVector(this->linear_velocity);
                    math::Vector3d body_acc = this->pose.rot.RotateVector(this->acceleration);
                    math::Vector3d poschange = this->pose.pos - this->position;
                    this->position = pose.pos;
                    // Get gravity
                    math::Vector3d gravity_body = this->pose.rot.RotateVector(this->world->Gravity());
                    double gravity = gravity_body.GetLength();
                   
                #endif
                 double load_factor = gravity * gravity / this->world->Gravity().Dot(gravity_body);  // Get gravity
                // Rotate vectors to coordinate frames relevant for control
                #if GAZEBO_MAJOR_VERSION >= 8
                    math::Quaterniond heading_quaternion(cos(this->euler.Z()/2),0,0,sin(this->euler.Z()/2));
                #else
                    math::Quaterniond heading_quaternion(cos(this->euler.z/2),0,0,sin(this->euler.z/2));
                #endif

                math::Vector3d velocity_xy = heading_quaternion.RotateVectorReverse(this->linear_velocity);
                math::Vector3d acceleration_xy = heading_quaternion.RotateVectorReverse(this->acceleration);
                #if GAZEBO_MAJOR_VERSION >= 8
                    math::Vector3d angular_velocity_body = this->pose.Rot().RotateVectorReverse(this->angular_velocity);
                #else
                    math::Vector3d angular_velocity_body = this->pose.rot.RotateVectorReverse(this->angular_velocity);
                #endif
                force.Set(0.0, 0.0, 0.0);
                torque.Set(0.0, 0.0, 0.0);
                
                
                if(this->flight_state == M100FlightStatus::IN_AIR )
                {
                    #if GAZEBO_MAJOR_VERSION >= 8
                        double pitch_command =  this->controllers_.velocity_x.update(this->velocity_command.linear.x, velocity_xy.X(), acceleration_xy.X(), dt) / gravity;
                        double roll_command  = -this->controllers_.velocity_y.update(this->velocity_command.linear.y, velocity_xy.Y(), acceleration_xy.Y(), dt) / gravity;
                        torque.X() = inertia.X() *  controllers_.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
                        torque.Y() = inertia.Y() *  controllers_.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
                    #else
                        double pitch_command =  this->controllers_.velocity_x.update(this->velocity_command.linear.x, velocity_xy.x, acceleration_xy.x, dt) / gravity;
                        double roll_command  = -this->controllers_.velocity_y.update(this->velocity_command.linear.y, velocity_xy.y, acceleration_xy.y, dt) / gravity;
                        torque.x = inertia.x *  controllers_.roll.update(roll_command, euler.x, angular_velocity_body.x, dt);
                        torque.y = inertia.y *  controllers_.pitch.update(pitch_command, euler.y, angular_velocity_body.y, dt);
                    #endif
                }
                else
                {
                    #if GAZEBO_MAJOR_VERSION >= 8
                        torque.X() = inertia.X() *  controllers_.roll.update(this->velocity_command.angular.x, euler.X(), angular_velocity_body.X(), dt);
                        torque.Y() = inertia.Y() *  controllers_.pitch.update(this->velocity_command.angular.y, euler.Y(), angular_velocity_body.Y(), dt);
                    #else
                        torque.x = inertia.x *  controllers_.roll.update(this->velocity_command.angular.x, euler.x, angular_velocity_body.x, dt);
                        torque.y = inertia.y *  controllers_.pitch.update(this->velocity_command.angular.y, euler.y, angular_velocity_body.y, dt);
                    #endif
                }
                #if GAZEBO_MAJOR_VERSION >= 8
                    torque.Z() = inertia.Z() *  controllers_.yaw.update(this->velocity_command.angular.z, angular_velocity.Z(), 0, dt);
                    double pid_update = controllers_.velocity_z.update(this->velocity_command.linear.z,  this->linear_velocity.Z(), this->acceleration.Z(), dt);
                    force.Z()  = mass      * (pid_update + load_factor * gravity);
                    if (this->max_force > 0.0 && force.Z() > this->max_force) force.Z() = this->max_force;
                    if (force.Z() < 0.0) force.Z() = 0.0;
                #else
                    torque.z = inertia.z *  controllers_.yaw.update(this->velocity_command.angular.z, angular_velocity.z, 0, dt);
                    double pid_update = controllers_.velocity_z.update(this->velocity_command.linear.z,  this->linear_velocity.z, this->acceleration.z, dt);
                    force.z  = mass      * (pid_update + load_factor * gravity);
                    if (this->max_force > 0.0 && force.z > this->max_force) force.z = this->max_force;
                    if (force.z < 0.0) force.z = 0.0;
                #endif


                
                
                // process robot state information
                if(this->flight_state == M100FlightStatus::LANDED || this->flight_state == M100FlightStatus::ON_GROUND)
                {
                    //NOOP
                }
                else if(this->flight_state == M100FlightStatus::IN_AIR)
                {
                    this->base_link->AddRelativeForce(force);
                    this->base_link->AddRelativeTorque(torque);
                }
                else if(this->flight_state == M100FlightStatus::TAKINGOFF)
                {
                
                    this->base_link->AddRelativeForce(force*1.5);
                    this->base_link->AddRelativeTorque(torque*1.5);
                }
                else if(this->flight_state == M100FlightStatus::LANDING)
                {
                    this->base_link->AddRelativeForce(force*0.8);
                    this->base_link->AddRelativeTorque(torque*0.8);
                }
            }
        private:
            ros::NodeHandle nh;
            physics::WorldPtr world;
            physics::ModelPtr model;
            physics::LinkPtr base_link,gimbal_yaw_link;
            event::ConnectionPtr update_connection;

            math::Pose3d pose;
            math::Vector3d linear_velocity,angular_velocity,acceleration,position;
            geometry_msgs::Twist velocity_command;
            math::Vector3d gimbal_orientation;
            math::Vector3d euler;

            ros::Subscriber imu_subscriber;
            ros::Subscriber velocity_subscriber;
            ros::Subscriber local_position_subscriber;
            ros::Subscriber gimbal_orientation_subscriber;
            ros::Subscriber flight_status_subscriber;

            double max_force, motion_small_noise, motion_drift_noise, motion_drift_noise_time;

            math::Vector3d inertia;
            double mass;

            struct Controllers {
                PIDController roll;
                PIDController pitch;
                PIDController yaw;
                PIDController velocity_x;
                PIDController velocity_y;
                PIDController velocity_z;
            } controllers_;

            int flight_state;
            ros::CallbackQueue callback_queue;
            common::Time last_time;
            double m_timeAfterCmd;

    };
    GZ_REGISTER_MODEL_PLUGIN(DJI_ROS_ControlPlugin)
}