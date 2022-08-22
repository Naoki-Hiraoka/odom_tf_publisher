#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


namespace tftree_merger {
  class tftree_merger
  {
  public:
    tftree_merger()
      : tfListener_(tfBuffer_)
    {
      ros::NodeHandle nh, pnh("~");

      pnh.param("parenttree_rootframe", parenttree_rootframe_,std::string(""));
      pnh.param("parenttree_targetframe", parenttree_targetframe_,std::string(""));
      pnh.param("childtree_rootframe", childtree_rootframe_,std::string(""));
      pnh.param("childtree_targetframe", childtree_targetframe_,std::string(""));

      timer_ = nh.createTimer(ros::Duration(1.0 / 10),
                              boost::bind(&tftree_merger::periodicTimerCallback, this, _1));

    }

    void periodicTimerCallback(const ros::TimerEvent& event){
      geometry_msgs::TransformStamped parentTransform;
      try{
        parentTransform = this->tfBuffer_.lookupTransform(this->parenttree_rootframe_, this->parenttree_targetframe_, ros::Time(0), ros::Duration(3));
      } catch (std::exception& ex) {
        ROS_ERROR_STREAM(ex.what());
        return;
      }

      geometry_msgs::TransformStamped childTransform;
      try{
        childTransform = this->tfBuffer_.lookupTransform(this->childtree_rootframe_, this->childtree_targetframe_, ros::Time(0), ros::Duration(3));
      } catch (std::exception& ex) {
        ROS_ERROR_STREAM(ex.what());
        return;
      }

      // stampをそろえる
      if(parentTransform.header.stamp > childTransform.header.stamp){
        try{
          parentTransform = this->tfBuffer_.lookupTransform(this->parenttree_rootframe_, this->parenttree_targetframe_, childTransform.header.stamp);
        } catch (std::exception& ex) {
          ROS_ERROR_STREAM(ex.what());
          return;
        }
      }else if(parentTransform.header.stamp < childTransform.header.stamp){
        try{
          childTransform = this->tfBuffer_.lookupTransform(this->childtree_rootframe_, this->childtree_targetframe_, parentTransform.header.stamp);
        } catch (std::exception& ex) {
          ROS_ERROR_STREAM(ex.what());
          return;
        }
      }

      Eigen::Affine3d parentT;
      tf::transformMsgToEigen(parentTransform.transform,parentT);
      Eigen::Affine3d childT;
      tf::transformMsgToEigen(childTransform.transform,childT);

      Eigen::Affine3d trans = parentT * childT.inverse();

      geometry_msgs::TransformStamped tf_coords;
      tf_coords.header.stamp = parentTransform.header.stamp;
      tf_coords.header.frame_id = this->parenttree_rootframe_;
      tf_coords.child_frame_id = this->childtree_rootframe_;
      tf::transformEigenToMsg(trans, tf_coords.transform);
      this->tfBroadcaster_.sendTransform(tf_coords);
    }

  protected:
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    ros::Timer timer_;

    std::string parenttree_rootframe_;
    std::string parenttree_targetframe_;
    std::string childtree_rootframe_;
    std::string childtree_targetframe_;
  private:
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tftree_merger");
  tftree_merger::tftree_merger c;
  ros::AsyncSpinner spinner(5); // Use many threads to increase frequency because waitForTransform in callback function takes much time.
  spinner.start();
  ros::waitForShutdown();
}
