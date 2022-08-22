#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>


namespace base_footprint_publisher {
  class base_footprint_publisher
  {
  public:
    base_footprint_publisher()
      : tfListener_(tfBuffer_)
    {
      ros::NodeHandle nh, pnh("~");

      pnh.param("rleg_frame", rleg_frame_,std::string("rleg_end_coords"));
      pnh.param("lleg_frame", lleg_frame_,std::string("lleg_end_coords"));
      pnh.param("world_frame", world_frame_,std::string("odom"));
      pnh.param("frame_id", frame_id_,std::string("base_footprint"));

      timer_ = nh.createTimer(ros::Duration(1.0 / 50),
                              boost::bind(&base_footprint_publisher::periodicTimerCallback, this, _1));

    }

    void periodicTimerCallback(const ros::TimerEvent& event){
      geometry_msgs::TransformStamped rlegTransform;
      try{
        rlegTransform = this->tfBuffer_.lookupTransform(this->world_frame_, this->rleg_frame_, ros::Time(0), ros::Duration(3));
      } catch (std::exception& ex) {
        ROS_ERROR_STREAM(ex.what());
        return;
      }

      geometry_msgs::TransformStamped llegTransform;
      try{
        llegTransform = this->tfBuffer_.lookupTransform(this->world_frame_, this->lleg_frame_, ros::Time(0), ros::Duration(3));
      } catch (std::exception& ex) {
        ROS_ERROR_STREAM(ex.what());
        return;
      }

      // stampをそろえる
      if(rlegTransform.header.stamp > llegTransform.header.stamp){
        try{
          rlegTransform = this->tfBuffer_.lookupTransform(this->world_frame_, this->rleg_frame_, llegTransform.header.stamp);
        } catch (std::exception& ex) {
          ROS_ERROR_STREAM(ex.what());
          return;
        }
      }else if(rlegTransform.header.stamp < llegTransform.header.stamp){
        try{
          llegTransform = this->tfBuffer_.lookupTransform(this->world_frame_, this->lleg_frame_, rlegTransform.header.stamp);
        } catch (std::exception& ex) {
          ROS_ERROR_STREAM(ex.what());
          return;
        }
      }

      Eigen::Affine3d rlegT;
      tf::transformMsgToEigen(rlegTransform.transform,rlegT);
      Eigen::Affine3d llegT;
      tf::transformMsgToEigen(llegTransform.transform,llegT);

      Eigen::Affine3d midT;
      midT.translation() = (rlegT.translation() + llegT.translation()) * 0.5;
      Eigen::AngleAxisd rleg2llegR(rlegT.linear().transpose() * llegT.linear());
      midT.linear() = rlegT.linear() * Eigen::AngleAxisd(rleg2llegR.angle() * 0.5, rleg2llegR.axis()); // Eigen::Quaternionのslerpは、90度回転した姿勢で不自然な遠回り補間をするので使ってはならない

      // Zはrlegとllegの低い方の値
      midT.translation()[2] = std::min(rlegT.translation()[2],llegT.translation()[2]);
      // Z軸は鉛直
      {
        Eigen::Vector3d localZaxis = midT.linear() * Eigen::Vector3d::UnitZ();
        Eigen::Vector3d worldZaxis = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d cross = localZaxis.cross(worldZaxis);
        double dot = std::min(1.0,std::max(-1.0,localZaxis.dot(worldZaxis))); // acosは定義域外のときnanを返す
        if(cross.norm()==0){
          if(dot == -1) midT.linear() = -midT.linear();
          else midT.linear() = midT.linear();
        }else{
          double angle = std::acos(dot); // 0~pi
          Eigen::Vector3d axis = cross.normalized(); // include sign
          midT.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(angle, axis)) * midT.linear();
        }
      }

      geometry_msgs::TransformStamped tf_coords;
      tf_coords.header.stamp = rlegTransform.header.stamp;
      tf_coords.header.frame_id = this->world_frame_;
      tf_coords.child_frame_id = this->frame_id_;
      tf::transformEigenToMsg(midT, tf_coords.transform);
      this->tfBroadcaster_.sendTransform(tf_coords);
    }

  protected:
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;
    ros::Timer timer_;

    std::string rleg_frame_;
    std::string lleg_frame_;
    std::string world_frame_;
    std::string frame_id_;

  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_footprint_publisher");
  base_footprint_publisher::base_footprint_publisher c;
  ros::spin();
}
