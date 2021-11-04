#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>


namespace tftree_merger {
  class tftree_merger
  {
  public:
    tftree_merger() {
      ros::NodeHandle nh, pnh("~");
      tfListener_.reset(new tf::TransformListener());

      pnh.param("parenttree_rootframe", parenttree_rootframe_,std::string(""));
      pnh.param("parenttree_targetframe", parenttree_targetframe_,std::string(""));
      pnh.param("childtree_rootframe", childtree_rootframe_,std::string(""));
      pnh.param("childtree_targetframe", childtree_targetframe_,std::string(""));

      timer_ = nh.createTimer(ros::Duration(1.0 / 10),
                              boost::bind(&tftree_merger::periodicTimerCallback, this, _1));

    }

    void periodicTimerCallback(const ros::TimerEvent& event){
      ros::Time now = ros::Time::now();
      if (!this->tfListener_->waitForTransform(this->parenttree_rootframe_, this->parenttree_targetframe_, now, ros::Duration(1.0))) {
        ROS_ERROR_STREAM("failed to lookup transform between " << this->parenttree_rootframe_ << " and " << this->parenttree_targetframe_);
        return;
      }
      Eigen::Affine3d parentT;
      {
        tf::StampedTransform transform;
        tfListener_->lookupTransform(this->parenttree_rootframe_, this->parenttree_targetframe_, now, transform);
        tf::transformTFToEigen(transform,parentT);
      }
      if (!this->tfListener_->waitForTransform(this->childtree_rootframe_, this->childtree_targetframe_, now, ros::Duration(1.0))) {
        ROS_ERROR_STREAM("failed to lookup transform between " << this->childtree_rootframe_ << " and " << this->childtree_targetframe_);
        return;
      }
      Eigen::Affine3d childT;
      {
        tf::StampedTransform transform;
        tfListener_->lookupTransform(this->childtree_rootframe_, this->childtree_targetframe_, now, transform);
        tf::transformTFToEigen(transform,childT);
      }

      Eigen::Affine3d trans = parentT * childT.inverse();

      geometry_msgs::TransformStamped tf_coords;
      tf_coords.header.stamp = now;
      tf_coords.header.frame_id = this->parenttree_rootframe_;
      tf_coords.child_frame_id = this->childtree_rootframe_;
      tf::transformEigenToMsg(trans, tf_coords.transform);
      this->tfBroadcaster_.sendTransform(tf_coords);
    }

  protected:
    boost::shared_ptr<tf::TransformListener> tfListener_;
    tf::TransformBroadcaster tfBroadcaster_;
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
