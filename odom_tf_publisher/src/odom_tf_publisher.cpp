#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros_extension/message_filter2.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


namespace odom_tf_publisher {
  class odom_tf_publisher
  {
  public:
    odom_tf_publisher()
      : tfListener_(tfBuffer_),
        tf2Filter_(odomSub_, tfBuffer_, "", 10, 0)
    {
      ros::NodeHandle pnh("~");

      pnh.param("child_frame_id", child_frame_id_,
                std::string(""));
      pnh.param("frame_id_overwrite", frame_id_overwrite_,
                std::string(""));
      pnh.param("child_frame_id_overwrite", child_frame_id_overwrite_,
                std::string(""));
      pnh.param("invert_odom", invert_odom_,
                false);
      int queue_size_;
      pnh.param("queue_size", queue_size_,
                10);

      // avoid calling waitForTransform in callback function because it takes much time and reduces callback hz.
      if(this->child_frame_id_ != ""){
        tf2Filter_.setTargetFrame(this->child_frame_id_);
      }else{
        tf2Filter_.setTargetFrames(std::vector<std::string>()); // filterしない
      }
      if(this->child_frame_id_overwrite_ != ""){
        tf2Filter_.setFrameIdFunc([=](const boost::shared_ptr<nav_msgs::Odometry const>& message){
            return this->child_frame_id_overwrite_;
          });
      }else{
        tf2Filter_.setFrameIdFunc([=](const boost::shared_ptr<nav_msgs::Odometry const>& message){
            return message->child_frame_id;
          });
      }

      tf2Filter_.setQueueSize(queue_size_);

      odomSub_.subscribe(nh_, "odom", queue_size_);
      tf2Filter_.registerCallback(boost::bind(&odom_tf_publisher::odomCallback, this, _1));
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      if(!std::isfinite(msg->pose.pose.position.x) ||
         !std::isfinite(msg->pose.pose.position.y) ||
         !std::isfinite(msg->pose.pose.position.z) ||
         !std::isfinite(msg->pose.pose.orientation.x) ||
         !std::isfinite(msg->pose.pose.orientation.y) ||
         !std::isfinite(msg->pose.pose.orientation.z) ||
         !std::isfinite(msg->pose.pose.orientation.w)
         ){
        // ビジュアルオドメトリセンサを使用していると、たまにnanが入ることがあるので.
        ROS_ERROR("input odom message is not finite. ignore this message");
        return;
      }
      // odom
      Eigen::Isometry3d odom_pose;
      tf::poseMsgToEigen(msg->pose.pose, odom_pose);
      std::string odom_frame_id = (this->frame_id_overwrite_ == "") ? msg->header.frame_id : this->frame_id_overwrite_;
      std::string odom_child_frame_id = (this->child_frame_id_overwrite_ == "") ? msg->child_frame_id : this->child_frame_id_overwrite_;

      if(this->child_frame_id_ != "" && this->child_frame_id_ != odom_child_frame_id){
        // convert frame

        Eigen::Isometry3d transform_pose;
        try{
          geometry_msgs::TransformStamped transform = tfBuffer_.lookupTransform(this->child_frame_id_, odom_child_frame_id, msg->header.stamp, ros::Duration(0.0));
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
      if(this->invert_odom_){
        std::swap(odom_coords.header.frame_id, odom_coords.child_frame_id);
        odom_pose = odom_pose.inverse();
      }
      tf::transformEigenToMsg(odom_pose, odom_coords.transform);
      this->tfBroadcaster_.sendTransform(odom_coords);
    }

  protected:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    message_filters::Subscriber<nav_msgs::Odometry> odomSub_;
    tf2_ros_extension::MessageFilter2<nav_msgs::Odometry> tf2Filter_;

    std::string child_frame_id_;
    std::string frame_id_overwrite_;
    std::string child_frame_id_overwrite_;
    bool invert_odom_;
  private:
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_tf_publisher");
  odom_tf_publisher::odom_tf_publisher c;
  ros::spin();
}
