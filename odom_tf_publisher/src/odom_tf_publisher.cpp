#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


namespace odom_tf_publisher {
  class odom_tf_publisher
  {
  public:
    odom_tf_publisher()
      : tfListener_(tfBuffer_)
    {
      ros::NodeHandle nh, pnh("~");

      pnh.param("child_frame_id", child_frame_id_,
                std::string(""));
      pnh.param("frame_id_overwrite", frame_id_overwrite_,
                std::string(""));
      pnh.param("child_frame_id_overwrite", child_frame_id_overwrite_,
                std::string(""));

      odomSub_ = nh.subscribe("odom", 1, &odom_tf_publisher::odomCallback, this); // keep only latest ones to avoid latency because waitForTransform in callback function takes much time.

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      // odom
      Eigen::Affine3d odom_pose;
      tf::poseMsgToEigen(msg->pose.pose, odom_pose);
      std::string odom_frame_id = (this->frame_id_overwrite_ == "") ? msg->header.frame_id : this->frame_id_overwrite_;
      std::string odom_child_frame_id = (this->child_frame_id_overwrite_ == "") ? msg->child_frame_id : this->child_frame_id_overwrite_;

      if(this->child_frame_id_ != "" && this->child_frame_id_ != odom_child_frame_id){
        // convert frame

        Eigen::Affine3d transform_pose;
        try{
          geometry_msgs::TransformStamped transform = transform = tfBuffer_.lookupTransform(this->child_frame_id_, odom_child_frame_id, msg->header.stamp, ros::Duration(1.0));
          tf::transformMsgToEigen(transform.transform,transform_pose);
        }catch (std::exception& ex) {
          ROS_ERROR_STREAM(ex.what());
          return;
        }

        odom_pose = odom_pose * transform_pose.inverse();
      }

      geometry_msgs::TransformStamped odom_coords;
      odom_coords.header = msg->header;
      odom_coords.header.frame_id = odom_frame_id;
      if(this->child_frame_id_!="") odom_coords.child_frame_id = this->child_frame_id_;
      else odom_coords.child_frame_id = odom_child_frame_id;
      tf::transformEigenToMsg(odom_pose, odom_coords.transform);
      std::vector<geometry_msgs::TransformStamped> tf_transforms;
      tf_transforms.push_back(odom_coords);
      this->tfBroadcaster_.sendTransform(tf_transforms);
    }

  protected:
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    ros::Subscriber odomSub_;

    std::string child_frame_id_;
    std::string frame_id_overwrite_;
    std::string child_frame_id_overwrite_;
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
