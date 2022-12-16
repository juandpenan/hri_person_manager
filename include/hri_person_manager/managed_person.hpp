#ifndef MANAGED_PERSON_H
#define MANAGED_PERSON_H

#include <iostream>
#include <chrono>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/buffer.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <hri/FeatureTracker.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>


namespace hri
{
const std::string NS("/humans/persons/");

// after that period of time, the location confidence of the person will be reduced to 0,
// and its TF transform won't be published anymore
const std::chrono::seconds LIFETIME_UNTRACKED_PERSON(10);

const std::string PERSON("person_");
const std::string ANONYMOUS("anonymous_person_");

class ManagedPerson : public rclcpp::Node
{
public:

  ManagedPerson(hri::ID id, tf2::BufferCore& tf_buffer,
                const std::string& reference_frame);

  ~ManagedPerson();

  void setFaceId(hri::ID id);
  void setBodyId(hri::ID id);
  void setVoiceId(hri::ID id);

  void setAnonymous(bool anonymous);
  bool anonymous() const
  {
    return _anonymous;
  }

  void setAlias(hri::ID id);
  hri::ID alias() const
  {
    return _alias;
  }


  void setLocationConfidence(float confidence);
  float locationConfidence() const
  {
    return _loc_confidence;
  }

  hri::ID id() const
  {
    return _id;
  }

  std::string tfFrame() const
  {
    return _tf_frame;
  }

  bool activelyTracked() const
  {
    return _actively_tracked;
  }

  void update(hri::ID face_id, hri::ID body_id, hri::ID voice_id,
              std::chrono::milliseconds elapsed_time);

private:
  void publishFrame();

 
  hri::ID _id;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr face_id_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr body_id_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_id_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alias_pub; 
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr anonymous_pub; 
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr loc_confidence_pub;


  bool _actively_tracked;

  std::string _tf_frame;
  std::string _tf_reference_frame;

  tf2::BufferCore* _tf_buffer;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_br;
  

  geometry_msgs::msg::TransformStamped _transform;
  bool _had_transform_at_least_once;

  hri::ID _face_id;
  hri::ID _body_id;
  hri::ID _voice_id;
  float _loc_confidence;
  bool _loc_confidence_dirty;
  bool _anonymous;

  std_msgs::msg::String id_msg;
  std_msgs::msg::Float32 float_msg;
  std_msgs::msg::Bool bool_msg;

  hri::ID _alias;

  std::chrono::milliseconds _time_since_last_seen;
};


}  // namespace hri
#endif