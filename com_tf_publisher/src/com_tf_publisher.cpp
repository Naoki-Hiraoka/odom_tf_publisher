#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>

namespace com_tf_publisher {
  class com_tf_publisher
  {
  public:
    com_tf_publisher() {
      ros::NodeHandle nh, pnh("~");

      cnoid::BodyLoader bodyLoader;
      std::string fileName;
      pnh.getParam("model",fileName);
      this->robot_ = bodyLoader.load(fileName);
      if(!this->robot_){
        ROS_FATAL_STREAM("failed to load model[" << fileName << "]");
        exit(1);
      }

      pnh.param("frame_id", frame_id_, std::string("odom"));
      pnh.param("com_frame_id", com_frame_id_, std::string("com"));
      pnh.param("base_frame_id", base_frame_id_, std::string("base_link"));

      sub_ = nh.subscribe("joint_states", 1, &com_tf_publisher::topicCallback, this); // keep only latest ones to avoid latency because waitForTransform in callback function takes much time.

    }

    void topicCallback(const sensor_msgs::JointState::ConstPtr& msg) {
      if(msg->name.size() != msg->position.size()) return;
      for(int i=0;i<msg->name.size();i++){
        const cnoid::LinkPtr& link = robot_->link(msg->name[i]);
        if(!link) continue;
        link->q() = msg->position[i];
      }

      // get base frame
      if (!this->tfListener_.waitForTransform(this->frame_id_, this->base_frame_id_, msg->header.stamp, ros::Duration(1.0))) {
        ROS_ERROR("failed to lookup transform between %s and %s",
                  this->frame_id_.c_str(),
                  this->base_frame_id_.c_str());
        return;
      }
      tf::StampedTransform transform;
      tfListener_.lookupTransform(this->frame_id_, this->base_frame_id_, msg->header.stamp, transform);
      Eigen::Affine3d transform_pose;
      tf::transformTFToEigen(transform,transform_pose);
      this->robot_->rootLink()->T() = transform_pose;

      this->robot_->calcForwardKinematics();
      this->robot_->calcCenterOfMass();

      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = msg->header.stamp;
      transformStamped.header.frame_id = this->frame_id_;
      transformStamped.child_frame_id = this->com_frame_id_;
      transformStamped.transform.translation.x = this->robot_->centerOfMass()[0];
      transformStamped.transform.translation.y = this->robot_->centerOfMass()[1];
      transformStamped.transform.translation.z = this->robot_->centerOfMass()[2];
      transformStamped.transform.rotation.x = 0.0;
      transformStamped.transform.rotation.y = 0.0;
      transformStamped.transform.rotation.z = 0.0;
      transformStamped.transform.rotation.w = 1.0;
      br.sendTransform(transformStamped);
    }

  protected:
    cnoid::BodyPtr robot_;
    tf::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster br;
    ros::Subscriber sub_;

    std::string frame_id_;
    std::string com_frame_id_;
    std::string base_frame_id_;
  private:
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "com_tf_publisher");
  ros::NodeHandle nh;
  com_tf_publisher::com_tf_publisher c;
  ros::spin();
}
