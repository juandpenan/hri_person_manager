#include "hri_person_manager/managed_person.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>


// using namespace hri;
namespace hri
{
ManagedPerson::ManagedPerson(
  hri::ID id, rclcpp::Node::SharedPtr node, tf2::BufferCore & tf_buffer,
  const std::string & reference_frame)
: _id(id)
  , _node(node)
  , _actively_tracked(false)
  , _tf_reference_frame(reference_frame)
  , _tf_buffer(&tf_buffer)
  , _had_transform_at_least_once(false)
  , _loc_confidence(0.)
  , _loc_confidence_dirty(false)
  , _anonymous(false)
  , _time_since_last_seen(0)
{
  rclcpp::NodeOptions node_options;

  node_options.start_parameter_event_publisher(false);
  node_options.start_parameter_services(false);
  auto node_params = _node->get_node_parameters_interface();
  auto node_topics = _node->get_node_topics_interface();
  auto qos = rclcpp::SystemDefaultsQoS();

  callback_group_ = _node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, true);
  rclcpp::PublisherOptionsWithAllocator<std::allocator<void>> options;
  
  options.callback_group = callback_group_;

  face_id_pub = rclcpp::create_publisher<std_msgs::msg::String>(node_topics, NS + id + "/face_id", qos, options);
  body_id_pub = rclcpp::create_publisher<std_msgs::msg::String>(node_topics, NS + id + "/body_id", qos, options);
  voice_id_pub = rclcpp::create_publisher<std_msgs::msg::String>(node_topics, NS + id + "/voice_id", qos, options);
  alias_pub = rclcpp::create_publisher<std_msgs::msg::String>(node_topics, NS + id + "/alias", qos, options);
  anonymous_pub = rclcpp::create_publisher<std_msgs::msg::Bool>(node_topics, NS + id + "/anonymous", qos, options);
  loc_confidence_pub = rclcpp::create_publisher<std_msgs::msg::Float32>(node_topics, NS + id + "/location_confidence", qos, options);


  setAnonymous((id.substr(0, ANONYMOUS.size()) == ANONYMOUS) ? true : false);

  _tf_frame = _anonymous ? id : hri::PERSON + id;
}

ManagedPerson::~ManagedPerson()
{
  RCLCPP_DEBUG_STREAM(_node->get_logger(), "Closing all topics related to person <" << _id);
  // TODO IMPLEMENT SHUTDOWN WHEN AVAILABLE
  // face_id_pub.shutdown();
  // body_id_pub.shutdown();
  // voice_id_pub.shutdown();
  // alias_pub.shutdown();
  // anonymous_pub.shutdown();
  // loc_confidence_pub.shutdown();
}

void ManagedPerson::setFaceId(hri::ID id)
{
  if (id != _face_id) {
    RCLCPP_INFO_STREAM(
      _node->get_logger(), "[person <" << _id << ">] face_id updated to <" << id << ">");
  }
  _face_id = id;
  id_msg.data = id;
  face_id_pub->publish(id_msg);
}

void ManagedPerson::setBodyId(ID id)
{
  if (id != _body_id) {
    RCLCPP_INFO_STREAM(
      _node->get_logger(), "[person <" << _id << ">] body_id updated to <" << id << ">");
  }

  _body_id = id;
  id_msg.data = id;
  body_id_pub->publish(id_msg);
}

void ManagedPerson::setVoiceId(ID id)
{
  if (id != _voice_id) {
    RCLCPP_INFO_STREAM(
      _node->get_logger(), "[person <" << _id << ">] voice_id updated to <" << id << ">");
  }
  _voice_id = id;
  id_msg.data = id;
  voice_id_pub->publish(id_msg);
}

void ManagedPerson::setAnonymous(bool anonymous)
{
  if (anonymous && _anonymous != anonymous) {
    RCLCPP_WARN_STREAM(_node->get_logger(), "new anonymous person " << _id);

  }
  _anonymous = anonymous;
  bool_msg.data = anonymous;
  anonymous_pub->publish(bool_msg);
}

void ManagedPerson::setAlias(ID id)
{
  if (id != _alias) {
    RCLCPP_INFO_STREAM(
      _node->get_logger(), "[person <" << _id << ">] set to be alias of <" << _alias << ">");
  }
  _alias = id;
  id_msg.data = id;
  alias_pub->publish(id_msg);
}

void ManagedPerson::setLocationConfidence(float confidence)
{
  _loc_confidence = confidence;
  float_msg.data = confidence;
  loc_confidence_pub->publish(float_msg);
}


void ManagedPerson::update(ID face_id, ID body_id, ID voice_id, std::chrono::milliseconds elapsed_time)
{
  // a person is considered 'actively tracked' if at least one of its face/body/voice is tracked
  _actively_tracked = !face_id.empty() || !body_id.empty() || !voice_id.empty();

  setFaceId(face_id);
  setBodyId(body_id);
  setVoiceId(voice_id);

  if (_actively_tracked) {
    _time_since_last_seen = std::chrono::milliseconds(0);
    if (_loc_confidence != 1.) {
      _loc_confidence = 1.;
      _loc_confidence_dirty = true;
    }
  } else { // *not* actively tracked
    if (_time_since_last_seen > LIFETIME_UNTRACKED_PERSON) {
      if (_loc_confidence != 0.) {
        _loc_confidence = 0.;
        _loc_confidence_dirty = true;
        RCLCPP_WARN_STREAM(
          _node->get_logger(), "[person <" << _id << ">] not seen for more than "
                                          << LIFETIME_UNTRACKED_PERSON.count()
                                          << ". Not publishing tf frame anymore.");
      }
    } else { // not tracked, but lifetime *not yet expired*
      _time_since_last_seen += elapsed_time;
      _loc_confidence = 1. - _time_since_last_seen / LIFETIME_UNTRACKED_PERSON;
      _loc_confidence_dirty = true;
    }
  }


  if (_loc_confidence_dirty) {
    setLocationConfidence(_loc_confidence);
    _loc_confidence_dirty = false;
  }

  publishFrame();
}


void ManagedPerson::publishFrame()
{
  /////////////////////////////////////////////
  // publish TF frame of the person


  std::string target_frame;

  if (!_face_id.empty()) {
    target_frame = std::string("face_") + _face_id;
  } else if (!_body_id.empty()) {
    target_frame = std::string("head_") + _body_id;
  } else if (!_voice_id.empty()) {
    target_frame = std::string("voice_") + _voice_id;
  }

  if (!target_frame.empty()) {

    if (_tf_buffer->canTransform(_tf_reference_frame, target_frame, tf2::TimePointZero)) {
      RCLCPP_INFO_STREAM_ONCE(
        _node->get_logger(), "[person <" << _id << ">] broadcast transform "
                                        << _tf_reference_frame << " <-> " << target_frame);
      try {
        _transform =
          _tf_buffer->lookupTransform(_tf_reference_frame, target_frame, tf2::TimePointZero);
        
        _transform.header.stamp = _node->get_clock()->now();
        _transform.child_frame_id = _tf_frame;

        _tf_br->sendTransform(_transform);
        _had_transform_at_least_once = true;
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(_node->get_logger(), ex.what());
      }
    } else {
      RCLCPP_INFO_STREAM_ONCE(
        _node->get_logger(), "[person <" << _id << ">] can not publish transform (either reference frame <"
                                        << _tf_reference_frame << "> or target frame <"
                                        << target_frame << "> are not available)");
    }
  } else {
    if (!_had_transform_at_least_once) {
      RCLCPP_INFO_STREAM_ONCE(
        _node->get_logger(), "[person <" << _id << ">] no face, body or voice TF frame avail. Can not yet broadcast frame <"
                                        << _tf_frame << ">.");
    } else {
      // publish the last known transform, until loc_confidence == 0
      if (_loc_confidence > 0) {
        _transform.header.stamp = _node->get_clock()->now();
        try
        { 
          _tf_br->sendTransform(_transform);
        }
        catch(tf2::TransformException & ex)
        {
          RCLCPP_WARN(_node->get_logger(), ex.what());
        }
        
        
      }
    }
  }
}
} // namespace HRI

