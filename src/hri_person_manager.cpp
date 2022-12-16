

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



using namespace hri;
using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

// after TIME_TO_DISAPPEAR seconds *without* actively seeing the person, the person tf
// frame is not published anymore.
const float TIME_TO_DISAPPEAR = 10.;  // secs

enum UpdateType
{
  RELATION,
  REMOVE
};

typedef std::tuple<UpdateType, hri::ID, FeatureType, hri::ID, FeatureType, float> Association;

class PersonManager : public rclcpp::Node
{
public:
  PersonManager()
    :  Node("hri_person_manager"),tfListener(tfBuffer), person_matcher()
  {
    
     

    tracked_persons_pub = this->create_publisher<hri_msgs::msg::IdsList>("/humans/persons/tracked", 1);
    known_persons_pub = this->create_publisher<hri_msgs::msg::IdsList>("/humans/persons/known", 1);
    humans_graph_pub = this->create_publisher<std_msgs::msg::String>("/humans/graph", 1);


    candidates = this->create_subscription<hri_msgs::msg::IdsMatch>(
        "/humans/candidate_matches", 10,std::bind(&PersonManager::onCandidateMatch, this, _1));

    this->declare_parameter("reference_frame", "map");
    this->declare_parameter("match_threshold", 0.5);
    _reference_frame =
      this->get_parameter("reference_frame").get_parameter_value().get<std::string>();
    match_threshold =
      this->get_parameter("match_threshold").get_parameter_value().get<float>();

      
    person_matcher.set_threshold(match_threshold);
 

    timer_ = this->create_wall_timer(
      100ms, std::bind(&PersonManager::timer_callback, this));

      
    reset_service = create_service<std_srvs::srv::Empty>("reset",std::bind(&PersonManager::reset,this,_1,_2,_3));        
    
    hri_listener.onFace(bind(&PersonManager::onFace, this, _1));
    hri_listener.onFaceLost(bind(&PersonManager::onFeatureLost, this, _1));
    hri_listener.onBody(bind(&PersonManager::onBody, this, _1));
    hri_listener.onBodyLost(bind(&PersonManager::onFeatureLost, this, _1));
    hri_listener.onVoice(bind(&PersonManager::onVoice, this, _1));
    hri_listener.onVoiceLost(bind(&PersonManager::onFeatureLost, this, _1));

    t0 = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(),"hri_person_manager ready. Waiting for candidate associations on /humans/candidate_matches");
  }

  void timer_callback()
  {
    float match_threshold =
      this->get_parameter("match_threshold").get_parameter_value().get<float>();

    chrono::nanoseconds elapsed_time(this->get_clock()->now().nanoseconds() - t0.nanoseconds());
    publish_persons(chrono::duration_cast<chrono::milliseconds>(elapsed_time));

    std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", "world")};
    this->set_parameters(all_new_parameters);
  }

  bool reset(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
  {
    RCLCPP_WARN(this->get_logger(),"Clearing all associations between persons, faces, bodies, voices");
    person_matcher.reset();

    persons.clear();
    previously_tracked.clear();
    anonymous_persons.clear();

    hri_msgs::msg::IdsList persons_list;
    tracked_persons_pub->publish(persons_list);
    known_persons_pub->publish(persons_list);

    return true;
  }

  void onCandidateMatch(hri_msgs::msg::IdsMatch::SharedPtr match)
  {
    FeatureType type1, type2;
    ID id1, id2;

    id1 = match->id1;

    if (id1.empty())
    {
       RCLCPP_ERROR(this->get_logger(),"received an empty id for id1");
      return;
    }

    id2 = match->id2;

    if (id2.empty() && match->id2_type != hri_msgs::msg::IdsMatch::UNSET)
    {
       RCLCPP_ERROR_STREAM(this->get_logger(),"received an empty id for id2, with type set to " << match->id2_type);
      return;
    }

    if (id1 == id2)
    {
       RCLCPP_ERROR(this->get_logger(),"candidate_matches with identical id1 and id2. Skipping.");
      return;
    }

    switch (match->id1_type)
    {
      case hri_msgs::msg::IdsMatch::PERSON:
        type1 = FeatureType::person;
        break;

      case hri_msgs::msg::IdsMatch::FACE:
        type1 = FeatureType::face;
        break;

      case hri_msgs::msg::IdsMatch::BODY:
        type1 = FeatureType::body;
        break;

      case hri_msgs::msg::IdsMatch::VOICE:
        type1 = FeatureType::voice;
        break;

      default:
         RCLCPP_ERROR_STREAM(this->get_logger(),"received an invalid type for id1: " << match->id1_type);
        return;
    }

    switch (match->id2_type)
    {
      case hri_msgs::msg::IdsMatch::PERSON:
        type2 = FeatureType::person;
        break;

      case hri_msgs::msg::IdsMatch::FACE:
        type2 = FeatureType::face;
        break;

      case hri_msgs::msg::IdsMatch::BODY:
        type2 = FeatureType::body;
        break;

      case hri_msgs::msg::IdsMatch::VOICE:
        type2 = FeatureType::voice;
        break;

      case hri_msgs::msg::IdsMatch::UNSET:
        type2 = FeatureType::person;
        id2 = hri::ANONYMOUS;
        break;

      default:
         RCLCPP_ERROR_STREAM(this->get_logger(),"received an invalid type for id2: " << match->id2_type);
        return;
    }


    {
      updates.push_back({ RELATION, id1, type1, id2, type2, match->confidence });
    }
  }

  void onFace(FaceWeakConstPtr face)
  {
    ID id;

    if (auto face_ptr = face.lock())
    {
      id = face_ptr->id();
      // if already associated, do nothing
      if (associated_faces.count(id) != 0)
      {
        return;
      }

      // ...otherwise, create an anonymous person
      updates.push_back(
          { RELATION, id, FeatureType::face, hri::ANONYMOUS, FeatureType::person, 1.0 });
    }
  }

  void onBody(BodyWeakConstPtr body)
  {
    ID id;

    if (auto body_ptr = body.lock())
    {
      id = body_ptr->id();

      // if already associated, do nothing
      if (associated_bodies.count(id) != 0)
      {
        return;
      }

      // ...otherwise, create an anonymous person
      updates.push_back(
          { RELATION, id, FeatureType::body, hri::ANONYMOUS, FeatureType::person, 1.0 });
    }
  }

  void onVoice(VoiceWeakConstPtr voice)
  {
    ID id;

    if (auto voice_ptr = voice.lock())
    {
      id = voice_ptr->id();

      // if already associated, do nothing
      if (associated_voices.count(id) != 0)
      {
        return;
      }

      // ...otherwise, create an anonymous person
      updates.push_back(
          { RELATION, id, FeatureType::voice, hri::ANONYMOUS, FeatureType::person, 1.0 });
    }
  }


  void onFeatureLost(ID id)
  {
    updates.push_back({ REMOVE, id, FeatureType::person, id, FeatureType::person, 0. });
  }

  void update(ID id1, FeatureType type1, ID id2, FeatureType type2, float confidence)
  {
    // only id2 can be anonymous (cf onFace/onBody/onVoice above)
    if (id2 == hri::ANONYMOUS)
    {
      // if id1 is 'egbd4', id2 becomes 'anonymous_person_' -> 'anonymous_person_egbd4'
      // to create a 'unique' anonymous person for corresponding body part
      id2 += id1;

      anonymous_persons.insert(id1);
    }
    else
    {
      // Remove previously-anonymous persons if needed

      // id1 is a person associated to id2, and id2 previously had an anonymous person attached?
      // => remove the anonymous person
      if ((type1 == FeatureType::person) && anonymous_persons.count(id2) != 0)
      {
         RCLCPP_WARN_STREAM(this->get_logger(),"removing anonymous person "
                        << hri::ANONYMOUS + id2 << " as it is not anonymous anymore");
        anonymous_persons.erase(id2);
        auto removed_persons = person_matcher.erase(hri::ANONYMOUS + id2);
        for (auto const& id : removed_persons)
        {
          remove_person(id);
        }
      }
      else if ((type2 == FeatureType::person) && anonymous_persons.count(id1) != 0)
      {
         RCLCPP_WARN_STREAM(this->get_logger(),"removing anonymous person "
                        << hri::ANONYMOUS + id1 << " as it is not anonymous anymore");
        anonymous_persons.erase(id1);
        auto removed_persons = person_matcher.erase(hri::ANONYMOUS + id1);
        for (auto const& id : removed_persons)
        {
          remove_person(id);
        }
      }

      // id1 & id2 are not persons, id1 associated to id2, both have an anonymous
      // person attached?
      // => remove one
      else if (anonymous_persons.count(id2) != 0 && anonymous_persons.count(id1) != 0)
      {
        auto removed_persons = person_matcher.erase(hri::ANONYMOUS + id2);
        for (auto const& id : removed_persons)
        {
          remove_person(id);
        }
      }
    }

    person_matcher.update({ { id1, type1, id2, type2, confidence } });

    // after an update, we might have new orphaned nodes (if the update sets a confidence of 0)
    if (confidence == 0.0)
    {
      auto removed_persons = person_matcher.clear_orphans();

      for (auto const& id : removed_persons)
      {
        remove_person(id);
      }
    }


    // TODO:
    // If you have the following associations:
    // f1 -> anon_p1
    // b2 -> f1
    // and you add:
    // b2 -> p2
    // then anon_p1 should be removed (since f1 is now indirectly associated with p2)
    //
    // This is not handled yet.
  }

  void initialize_person(hri::ID id)
  {
    persons[id] = std::make_shared<ManagedPerson>(id, tfBuffer, _reference_frame);

    publishKnownPersons();
  }

  void publishKnownPersons()
  {
    // publish an updated list of all known persons
    hri_msgs::msg::IdsList persons_list;

    for (auto const& kv : persons)
    {
      persons_list.ids.push_back(kv.first);
    }

    known_persons_pub->publish(persons_list);
  }

  void remove_person(ID id)
  {
    if (!persons.count(id))
    {
      return;
    }

    // unlike anonymous persons, non-anonymous person can not be removed -- they
    // can merely become untracked.
    if (persons[id]->anonymous())
    {
      // delete the person (the ManagedPerson destructor will also shutdown the
      // corresponding topics)
      persons.erase(id);

      anonymous_persons.erase(id);
    }

    // publish an updated list of known/tracked persons
    hri_msgs::msg::IdsList persons_list;
    for (auto const& kv : persons)
    {
      if (kv.second->activelyTracked())
      {
        persons_list.ids.push_back(kv.first);
      }
    }

    tracked_persons_pub->publish(persons_list);

    publishKnownPersons();
  }

  void publish_persons(chrono::milliseconds elapsed_time)
  {
    ///////////////////////////////////
    // first: housekeeping -> update the graph with all the last changes
    UpdateType update_type;
    ID id1, id2;
    FeatureType type1, type2;
    float p;

    if (!updates.empty())
    {
       RCLCPP_INFO_STREAM(this->get_logger(),"Updating graph:");
    }
    for (auto u : updates)
    {
      std::tie(update_type, id1, type1, id2, type2, p) = u;

      switch (update_type)
      {
        case REMOVE:
        {
           RCLCPP_INFO_STREAM(this->get_logger(),"- Remove ID: " << id1);

          // while erasing the id id1, the person matcher might create
          // orphans that are as well deleted by the person_matcher.
          // The ids of the orphans that happened to be persons
          // are returned by PersonMatcher::erase to then remove these
          // persons from eg /humans/persons/tracked
          auto removed_persons = person_matcher.erase(id1);

          for (auto const& id : removed_persons)
          {
            remove_person(id);
          }
        }
        break;
        case RELATION:
          RCLCPP_INFO_STREAM(this->get_logger(),"- Update relation: " << id1 << " (" << type1 << ") <--> "
                                                << id2 << " (" << type2 << "); p=" << p);
          update(id1, type1, id2, type2, p);
          break;
      }
    }
    updates.clear();
    //////////////////////////

    auto person_associations = person_matcher.get_all_associations();

    std_msgs::msg::String graphviz;
    graphviz.data = person_matcher.get_graphviz();
    humans_graph_pub->publish(graphviz);

    associated_faces.clear();
    associated_bodies.clear();
    associated_voices.clear();

    for (const auto& kv : person_associations)
    {
      ID id = kv.first;

      // new person? first, create it (incl its publishers)
      if (persons.find(id) == persons.end())
      {
        initialize_person(id);
      }

      auto person = persons[id];

      auto association = kv.second;

      ID face_id, body_id, voice_id;

      if (association.find(FeatureType::face) != association.end())
      {
        face_id = association.at(face);
        associated_faces.insert(face_id);
      }
      if (association.find(FeatureType::body) != association.end())
      {
        body_id = association.at(body);
        associated_bodies.insert(body_id);
      }
      if (association.find(FeatureType::voice) != association.end())
      {
        voice_id = association.at(voice);
        associated_voices.insert(voice_id);
      }

      ////////////////////////////////////////////
      // publish the face, body, voice id corresponding to the person
      person->update(face_id, body_id, voice_id, elapsed_time);
    }

    ////////////////////////////////////////////
    // publish the list of currently actively tracked persons
    hri_msgs::msg::IdsList persons_list;
    vector<ID> actively_tracked;

    for (auto const& kv : persons)
    {
      if (kv.second->activelyTracked())
      {
        actively_tracked.push_back(kv.first);
        persons_list.ids.push_back(kv.first);
      }
    }

    if (actively_tracked != previously_tracked)
    {
      tracked_persons_pub->publish(persons_list);
      previously_tracked = actively_tracked;
    }
  }



private:
 
  PersonMatcher person_matcher; 

  map<hri::ID, std::shared_ptr<ManagedPerson>> persons;
  vector<hri::ID> previously_tracked;
  set<hri::ID> anonymous_persons;

  vector<Association> updates;

  // hold the list of faces/bodies/voices that are already associated to a person
  // (so that we do not create un-needed anonymous persons)
  set<hri::ID> associated_faces;
  set<hri::ID> associated_bodies;
  set<hri::ID> associated_voices;

  HRIListener hri_listener;

  // actively tracked persons (eg, one of face_id, body_id or voice_id is not empty for that person)
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr tracked_persons_pub;
  // known persons: either actively tracked ones, or not tracked anymore (but
  // still known to the robot)
  rclcpp::Publisher<hri_msgs::msg::IdsList>::SharedPtr known_persons_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr humans_graph_pub;


  tf2::BufferCore tfBuffer;
  tf2_ros::TransformListener tfListener;

  float match_threshold;

  rclcpp::Time t0;

  std::string _reference_frame;

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



