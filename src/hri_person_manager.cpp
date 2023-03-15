#include <hri_msgs/msg/ids_match.hpp>
#include <hri_msgs/msg/ids_list.hpp>
#include <std_srvs/srv/empty.hpp>


#include "tf2_ros/transform_listener.h"
#include <hri/hri.hpp>
#include <thread>
#include <array>
#include <functional>

#include "hri/body.hpp"
#include "hri/face.hpp"
#include "hri/voice.hpp"
#include "hri_person_manager/person_matcher.hpp"
#include "hri_person_manager/managed_person.hpp"
#include "rclcpp/rclcpp.hpp"



using namespace std::placeholders;
using namespace std::chrono_literals;

// after TIME_TO_DISAPPEAR seconds *without* actively seeing the person, the person tf
// frame is not published anymore.
const float TIME_TO_DISAPPEAR = 10.;  // secs

enum UpdateType
{
  NEW_FEATURE,
  RELATION,
  REMOVE
};

typedef std::tuple<UpdateType, hri::ID, hri::FeatureType, hri::ID, hri::FeatureType, float> Association;

class PersonManager : public rclcpp::Node
{
public:
  PersonManager()
  :  Node("hri_person_manager"), tfListener(tfBuffer), person_matcher()
  {
    hri_listener = std::make_shared<hri::HRIListener>();
    tracked_persons_pub = this->create_publisher<hri_msgs::msg::IdsList>(
      "/humans/persons/tracked",
      1);
    known_persons_pub = this->create_publisher<hri_msgs::msg::IdsList>("/humans/persons/known", 1);
    humans_graph_pub = this->create_publisher<std_msgs::msg::String>("/humans/graph", 1);

    candidates = this->create_subscription<hri_msgs::msg::IdsMatch>(
      "/humans/candidate_matches", 10, std::bind(&PersonManager::onCandidateMatch, this, _1));

    this->declare_parameter("reference_frame", "map");
    this->declare_parameter("match_threshold", 0.5);
    this->declare_parameter("create_features_from_candidate_matches", true);

    _create_features_from_candidate_matches =
      this->get_parameter("create_features_from_candidate_matches").get_value<bool>();
    _reference_frame =
      this->get_parameter("reference_frame").get_parameter_value().get<std::string>();
    match_threshold =
      this->get_parameter("match_threshold").get_parameter_value().get<float>();


    person_matcher.set_threshold(match_threshold);


    timer_ = this->create_wall_timer(
      100ms, std::bind(&PersonManager::timer_callback, this));


    reset_service =
      create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&PersonManager::reset, this, _1, _2, _3));

    hri_listener->onFace(bind(&PersonManager::onFace, this, _1));
    hri_listener->onFaceLost(bind(&PersonManager::onFeatureLost, this, _1));
    hri_listener->onBody(bind(&PersonManager::onBody, this, _1));
    hri_listener->onBodyLost(bind(&PersonManager::onFeatureLost, this, _1));
    hri_listener->onVoice(bind(&PersonManager::onVoice, this, _1));
    hri_listener->onVoiceLost(bind(&PersonManager::onFeatureLost, this, _1));    

    RCLCPP_INFO(
      this->get_logger(),
      "hri_person_manager ready. Waiting for candidate associations on /humans/candidate_matches");

    t0 = this->get_clock()->now();
  }

  void timer_callback()
  {
    std::chrono::nanoseconds elapsed_time(this->get_clock()->now().nanoseconds() - t0.nanoseconds());
    publish_persons(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time));
    t0 = this->get_clock()->now();
  }

  bool reset(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
  {
    RCLCPP_WARN(
      this->get_logger(), "Clearing all associations between persons, faces, bodies, voices");
    person_matcher.reset();

    persons.clear();
    previously_tracked.clear();

    hri_msgs::msg::IdsList persons_list;
    tracked_persons_pub->publish(persons_list);
    known_persons_pub->publish(persons_list);

    return true;
  }

  void onCandidateMatch(hri_msgs::msg::IdsMatch::SharedPtr match)
  {
    hri::FeatureType type1, type2;
    hri::ID id1, id2;

    id1 = match->id1;

    if (id1.empty()) {
      RCLCPP_ERROR(this->get_logger(), "received an empty id for id1");
      return;
    }

    id2 = match->id2;

    if (id2.empty() && match->id2_type != hri_msgs::msg::IdsMatch::UNSET) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(), "received an empty id for id2, with type set to " << match->id2_type);
      return;
    }

    if (id1 == id2) {
      RCLCPP_ERROR(this->get_logger(), "candidate_matches with identical id1 and id2. Skipping.");
      return;
    }

    switch (match->id1_type) {
      case hri_msgs::msg::IdsMatch::PERSON:
        type1 = hri::FeatureType::person;
        break;

      case hri_msgs::msg::IdsMatch::FACE:
        type1 = hri::FeatureType::face;
        break;

      case hri_msgs::msg::IdsMatch::BODY:
        type1 = hri::FeatureType::body;
        break;

      case hri_msgs::msg::IdsMatch::VOICE:
        type1 = hri::FeatureType::voice;
        break;

      default:
        RCLCPP_ERROR_STREAM(
          this->get_logger(), "received an invalid type for id1: " << match->id1_type);
        return;
    }

    switch (match->id2_type) {
      case hri_msgs::msg::IdsMatch::PERSON:
        type2 = hri::FeatureType::person;
        break;

      case hri_msgs::msg::IdsMatch::FACE:
        type2 = hri::FeatureType::face;
        break;

      case hri_msgs::msg::IdsMatch::BODY:
        type2 = hri::FeatureType::body;
        break;

      case hri_msgs::msg::IdsMatch::VOICE:
        type2 = hri::FeatureType::voice;
        break;

      default:
        RCLCPP_ERROR_STREAM(
          this->get_logger(), "received an invalid type for id2: " << match->id2_type);
        return;
    }


    {
      updates.push_back({RELATION, id1, type1, id2, type2, match->confidence});
    }
  }

  void onFace(hri::FaceWeakConstPtr face)
  {
    hri::ID id;

    if (auto face_ptr = face.lock()) {
      id = face_ptr->id();
      if (id.size() == 0)
      {
        RCLCPP_ERROR_STREAM(
          this->get_logger(), "got invalid new face: empty id! skipping");
        return;
      }
      // create a new node in the graph
      updates.push_back(
        {NEW_FEATURE, id, hri::FeatureType::face, id, hri::FeatureType::face, 0.0});
    }
  }

  void onBody(hri::BodyWeakConstPtr body)
  {
    hri::ID id;

    if (auto body_ptr = body.lock())
    {
      id = body_ptr->id();

      if (id.size() == 0)
      {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),"got invalid new body: empty id! skipping");
        return;
      }
      // create a new node in the graph
      updates.push_back({ NEW_FEATURE, id, hri::FeatureType::body, id, hri::FeatureType::body, 0. });
    }
  }

  void onVoice(hri::VoiceWeakConstPtr voice)
  {
    hri::ID id;

    if (auto voice_ptr = voice.lock())
    {
      id = voice_ptr->id();

      if (id.size() == 0)
      {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),"got invalid new voice: empty id! skipping");
        return;
      }

      // create a new node in the graph
      updates.push_back({ NEW_FEATURE, id, hri::FeatureType::voice, id, hri::FeatureType::voice, 0. });
    }
  }


  void onFeatureLost(hri::ID id)
  {
    if (id.size() == 0)
    {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),"a feature was removed, but empty id! skipping");
      return;
    }

    updates.push_back({ REMOVE, id, hri::FeatureType::invalid, id, hri::FeatureType::invalid, 0. });
  }

  void initialize_person(hri::ID id)
  {
    persons[id] = std::make_shared<hri::ManagedPerson>(id, shared_from_this(), tfBuffer, _reference_frame);

    publish_known_persons();
  }

  void publish_known_persons()
  {
    // publish an updated list of all known persons
    hri_msgs::msg::IdsList persons_list;
    std::vector<hri::ID> known;
    for (auto const & kv : persons) {
      persons_list.ids.push_back(kv.first);
      known.push_back(kv.first);
    }

    if (known != previously_known)
    {
      persons_list.header.stamp = this->get_clock()->now();
      known_persons_pub->publish(persons_list);
      previously_known = known;
    }
  }

  
  void publish_persons(std::chrono::milliseconds elapsed_time)
  {
    ///////////////////////////////////
    // first: housekeeping -> update the graph with all the last changes
    UpdateType update_type;
    hri::ID id1, id2;
    hri::FeatureType type1, type2;
    float likelihood;

    if (!updates.empty()) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Updating graph:");
    }
    for (auto u : updates) {
      std::tie(update_type, id1, type1, id2, type2, likelihood) = u;

      switch (update_type) {
        case NEW_FEATURE:
          {
            RCLCPP_INFO_STREAM(this->get_logger(), "- New feature: " << id1 << " (" << type1 << ")");
            person_matcher.update({ { id1, type1, id2, type2, 1.0 } });
          }
          break;

        case REMOVE:
          {
            RCLCPP_INFO_STREAM(this->get_logger(), "- Remove ID: " << id1);
            person_matcher.erase(id1);
          }
          break;

        case RELATION:
          {
          RCLCPP_INFO_STREAM(
            this->get_logger(), "- Update relation: " << id1 << " (" << type1 << ") <--> "
                                                      << id2 << " (" << type2 << "); likelihood=" << likelihood);
          person_matcher.update({{id1, type1, id2, type2, likelihood}}, _create_features_from_candidate_matches);
          }
          break;
      }
    }
    updates.clear();
    //////////////////////////


    //////////////////////////
    //   MAIN ALGORITHM     //
    //////////////////////////

    auto person_associations = person_matcher.get_all_associations();
    //////////////////////////

    ////////////////////////////////////////////

    if (humans_graph_pub->get_subscription_count() > 0)
    {
      std_msgs::msg::String graphviz;
      graphviz.data = person_matcher.get_graphviz();
      humans_graph_pub->publish(graphviz);
    }

    ////////////////////////////////////////////
    // go over all the persons in the graph, and process their
    // associations

    for (const auto & kv : person_associations) {
      hri::ID id = kv.first;

      // new person? first, create it (incl its publishers)
      if (persons.find(id) == persons.end()) {
        initialize_person(id);
      }

      auto person = persons[id];

      auto association = kv.second;

      hri::ID face_id, body_id, voice_id;

      if (association.find(hri::FeatureType::face) != association.end()) {
        face_id = association.at(hri::face);
      }
      if (association.find(hri::FeatureType::body) != association.end()) {
        body_id = association.at(hri::body);
      }
      if (association.find(hri::FeatureType::voice) != association.end()) {
        voice_id = association.at(hri::voice);
      }

      ////////////////////////////////////////////
      // publish the face, body, voice id corresponding to the person
      person->update(face_id, body_id, voice_id, elapsed_time);
    }
    ////////////////////////////////////////////
    // if an anonymous person is *not* present anymore in the associations, it
    // has disappeared, and must be removed from the system.
    std::vector<hri::ID> to_delete;
    for (auto const& kv : persons)
    {
      if (kv.second->anonymous() && !person_associations.count(kv.first))
      {
        to_delete.push_back(kv.first);
      }
    }
    for (const auto& p : to_delete)
    {
      persons.erase(p);
    }

    ////////////////////////////////////////////
    // publish the list of currently actively tracked persons
    hri_msgs::msg::IdsList persons_list;
    std::vector<hri::ID> actively_tracked;

    for (auto const & kv : persons) {
      if (kv.second->activelyTracked()) {
        actively_tracked.push_back(kv.first);
        persons_list.ids.push_back(kv.first);
      }
    }

    if (actively_tracked != previously_tracked) {
      tracked_persons_pub->publish(persons_list);
      previously_tracked = actively_tracked;
    }
    publish_known_persons();
  }

  void set_threshold(float threshold)
  {
    person_matcher.set_threshold(threshold);
  }

private:
  PersonMatcher person_matcher;

  std::map<hri::ID, std::shared_ptr<hri::ManagedPerson>> persons;
  std::vector<hri::ID> previously_known, previously_tracked;

  std::vector<Association> updates;

  // hold the list of faces/bodies/voices that are already associated to a person
  // (so that we do not create un-needed anonymous persons)
  std::set<hri::ID> associated_faces;
  std::set<hri::ID> associated_bodies;
  std::set<hri::ID> associated_voices;

  std::shared_ptr<hri::HRIListener> hri_listener{nullptr};

  // actively tracked persons (eg, one of face_id, body_id or voice_id is not empty for that person)
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr tracked_persons_pub;
  // known persons: either actively tracked ones, or not tracked anymore (but
  // still known to the robot)
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr known_persons_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr humans_graph_pub;


  tf2::BufferCore tfBuffer;
  tf2_ros::TransformListener tfListener;  

  rclcpp::Time t0;

  float match_threshold;
  std::string _reference_frame;
  bool _create_features_from_candidate_matches;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<hri_msgs::msg::IdsMatch>::SharedPtr candidates;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PersonManager>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
