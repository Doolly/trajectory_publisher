#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <string>

class TrajectoryPublisher {
public:
  TrajectoryPublisher()
      : polling_(true), prev_tf_timestamp_(0.0), tf_topic_("/tf"),
        tf_polling_hz_(250), frame_id_("/world"), child_frame_id_("/imu") {
    getParams();

    trajectory_.header.frame_id = frame_id_;
    trajectory_publisher_ =
        nh_.advertise<nav_msgs::Path>("trajectory", 1, true);

    if (polling_) {
      if (testTransform()) {
        ROS_INFO("Succeed in getting transformation in polling mode");
        timer_ = nh_.createTimer(ros::Duration(1.0 / tf_polling_hz_),
                                 &TrajectoryPublisher::timerCallback, this);
      } else {
        ROS_ERROR("Fail to get transformation in polling mode");
      }
    } else {
      tf_subscriber_ =
          nh_.subscribe(tf_topic_, 256, &TrajectoryPublisher::tfCallback, this);
    }
  }

private:
  void getParams() {
    ros::NodeHandle private_nh("~");
    // [ToDo] get tf_topic_, polling_, tf_polling_hz_
  }

  bool testTransform() const {
    int test_hz = 10000;
    double test_duration = 10.0;
    bool isTfValid = false;
    for (int i = 0; i < static_cast<int>(test_duration * test_hz) && !isTfValid;
         ++i) {
      isTfValid =
          tf_listener_.canTransform(frame_id_, child_frame_id_, ros::Time());
      ros::WallDuration(1.0 / test_hz).sleep();
    }
    return isTfValid;
  }

  void tfCallback(const tf::tfMessage::ConstPtr &tf_msg) {
    // [ToDo] implement it
    ROS_INFO("%s ++", __func__);
  }

  void timerCallback(const ros::TimerEvent &) {
    tf::StampedTransform tf;
    tf_listener_.lookupTransform(frame_id_, child_frame_id_, ros::Time(0), tf);
    const ros::Time &curr_tf_timestamp = tf.stamp_;
    const bool &newTf = checkCurrentTimestamp(curr_tf_timestamp);
    // Add tf to trajectory only when the tf is new
    if (newTf) {
      addTfToTrajectory(tf);
      publishTrajectory();
    }
  }

  bool checkCurrentTimestamp(const ros::Time &curr_tf_timestamp) {
    if (curr_tf_timestamp < prev_tf_timestamp_) {
      ROS_WARN("Current timestamp is smaller than previous timestamp");
      return false;
    } else if (curr_tf_timestamp == prev_tf_timestamp_) {
      return false;
    }
    prev_tf_timestamp_ = curr_tf_timestamp;
    return true;
  }

  void addTfToTrajectory(const tf::StampedTransform &tf) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    if (trajectory_.poses.empty()) {
      pose.header.seq = 0;
    } else {
      pose.header.seq = trajectory_.poses.back().header.seq + 1;
    }
    pose.header.stamp = tf.stamp_;
    pose.pose.orientation.w = tf.getRotation().getW();
    pose.pose.orientation.x = tf.getRotation().getX();
    pose.pose.orientation.y = tf.getRotation().getY();
    pose.pose.orientation.z = tf.getRotation().getZ();
    pose.pose.position.x = tf.getOrigin().getX();
    pose.pose.position.y = tf.getOrigin().getY();
    pose.pose.position.z = tf.getOrigin().getZ();
    trajectory_.header.seq++;
    trajectory_.header.stamp = tf.stamp_;
    trajectory_.poses.push_back(pose);
  }

  void publishTrajectory() const { trajectory_publisher_.publish(trajectory_); }

private:
  ros::NodeHandle nh_;
  bool polling_;
  ros::Time prev_tf_timestamp_;
  ros::Publisher trajectory_publisher_;
  nav_msgs::Path trajectory_;
  // Callback for subscriber
  ros::Subscriber tf_subscriber_;
  std::string tf_topic_;
  // Timer for polling
  ros::Timer timer_;
  int tf_polling_hz_;
  tf::TransformListener tf_listener_;
  std::string frame_id_;
  std::string child_frame_id_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "trajectory_publisher");
  TrajectoryPublisher tp;
  ros::spin();
  exit(EXIT_SUCCESS);
}
