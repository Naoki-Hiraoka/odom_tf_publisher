#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>


namespace odom_tf_publisher {
  class odom_tf_publisher
  {
  public:
    odom_tf_publisher() {
      ros::NodeHandle nh, pnh("~");
      tfListener_.reset(new tf::TransformListener());

      pnh.param("child_frame_id", child_frame_id_,
                std::string(""));

      odomSub_ = nh.subscribe("odom", 1, &odom_tf_publisher::odomCallback, this); // keep only latest ones to avoid latency because waitForTransform in callback function takes much time.

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      // odom
      Eigen::Affine3d odom_pose;
      tf::poseMsgToEigen(msg->pose.pose, odom_pose);

      if(this->child_frame_id_ != "" && this->child_frame_id_ != msg->child_frame_id){
        // convert frame

        if (!this->tfListener_->waitForTransform(this->child_frame_id_, msg->child_frame_id, msg->header.stamp, ros::Duration(1.0))) {
          ROS_ERROR("failed to lookup transform between %s and %s",
                    this->child_frame_id_.c_str(),
                    msg->child_frame_id.c_str());
          return;
        }

        tf::StampedTransform transform;
        tfListener_->lookupTransform(this->child_frame_id_, msg->child_frame_id, msg->header.stamp, transform);
        Eigen::Affine3d transform_pose;
        tf::transformTFToEigen(transform,transform_pose);

        odom_pose = odom_pose * transform_pose.inverse();
      }

      geometry_msgs::TransformStamped odom_coords;
      odom_coords.header = msg->header;
      if(this->child_frame_id_!="") odom_coords.child_frame_id = this->child_frame_id_;
      else odom_coords.child_frame_id = msg->child_frame_id;
      tf::transformEigenToMsg(odom_pose, odom_coords.transform);
      std::vector<geometry_msgs::TransformStamped> tf_transforms;
      tf_transforms.push_back(odom_coords);
      this->tfBroadcaster_.sendTransform(tf_transforms);
    }

  protected:
    boost::shared_ptr<tf::TransformListener> tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;
    ros::Subscriber odomSub_;

    std::string child_frame_id_;
  private:
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_tf_publisher");
  odom_tf_publisher::odom_tf_publisher c;
  ros::AsyncSpinner spinner(10); // Use many threads to increase frequency because waitForTransform in callback function takes much time.
  spinner.start();
  ros::waitForShutdown();
}
