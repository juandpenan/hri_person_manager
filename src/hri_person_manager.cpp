#include <hri_msgs/StringHRI.h>
#include <hri_msgs/BoolHRI.h>
#include <hri_msgs/Float32HRI.h>

#include <hri_msgs/IdsMatch.h>
#include <hri_msgs/IdsList.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <hri/hri.h>
#include <hri/base.h>
#include <thread>
#include <chrono>
#include <map>
#include <array>

#include "person_matcher.h"
#include "managed_person.h"
#include "ros/node_handle.h"

using namespace ros;
using namespace hri;
using namespace std;


// after TIME_TO_DISAPPEAR seconds *without* actively seeing the person, the person tf
// frame is not published anymore.
const float TIME_TO_DISAPPEAR = 10.;  // secs

const std::string NS("/humans/persons/");

class PersonManager
{
public:
  PersonManager(NodeHandle& nh, const string& reference_frame)
    : _nh(nh), _reference_frame(reference_frame), tfListener(tfBuffer)
  {
    tracked_persons_pub = _nh.advertise<hri_msgs::IdsList>(NS + "/tracked", 1, true);
    known_persons_pub = _nh.advertise<hri_msgs::IdsList>(NS + "/known", 1, true);

    face_id_pub = _nh.advertise<hri_msgs::StringHRI>(NS + "/face_id", 1, true);
    body_id_pub = _nh.advertise<hri_msgs::StringHRI>(NS + "/body_id", 1, true);
    voice_id_pub = _nh.advertise<hri_msgs::StringHRI>(NS + "/voice_id", 1, true);
    alias_pub = _nh.advertise<hri_msgs::StringHRI>(NS + "/alias", 1, true);
    anonymous_pub = _nh.advertise<hri_msgs::BoolHRI>(NS + "/anonymous", 1, true);
    loc_confidence_pub = _nh.advertise<hri_msgs::Float32HRI>(NS + "/location_confidence", 1);



    candidates = _nh.subscribe<hri_msgs::IdsMatch>(
        "/humans/candidate_matches", 10, bind(&PersonManager::onCandidateMatch, this, _1));



    ROS_INFO("hri_person_manager ready. Waiting for candidate associations on /humans/candidate_matches");
  }

  bool reset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    ROS_WARN("Clearing all associations between persons, faces, bodies, voices");
    person_matcher.reset();

    persons.clear();
    previously_tracked.clear();
    anonymous_persons.clear();

    hri_msgs::IdsList persons_list;
    tracked_persons_pub.publish(persons_list);
    known_persons_pub.publish(persons_list);

    return true;
  }

  void onCandidateMatch(hri_msgs::IdsMatchConstPtr match)
  {
    FeatureType type1, type2;
    ID id1, id2;

    id1 = match->id1;

    if (id1.empty())
    {
      ROS_ERROR("received an empty id for id1");
      return;
    }

    id2 = match->id2;

    if (id2.empty() && match->id2_type != hri_msgs::IdsMatch::UNSET)
    {
      ROS_ERROR_STREAM("received an empty id for id2, with type set to " << match->id2_type);
      return;
    }

    switch (match->id1_type)
    {
      case hri_msgs::IdsMatch::PERSON:
        type1 = FeatureType::person;
        break;

      case hri_msgs::IdsMatch::FACE:
        type1 = FeatureType::face;
        break;

      case hri_msgs::IdsMatch::BODY:
        type1 = FeatureType::body;
        break;

      case hri_msgs::IdsMatch::VOICE:
        type1 = FeatureType::voice;
        break;

      default:
        ROS_ERROR_STREAM("received an invalid type for id1: " << match->id1_type);
        return;
    }

    switch (match->id2_type)
    {
      case hri_msgs::IdsMatch::PERSON:
        type2 = FeatureType::person;
        break;

      case hri_msgs::IdsMatch::FACE:
        type2 = FeatureType::face;
        break;

      case hri_msgs::IdsMatch::BODY:
        type2 = FeatureType::body;
        break;

      case hri_msgs::IdsMatch::VOICE:
        type2 = FeatureType::voice;
        break;

      case hri_msgs::IdsMatch::UNSET:
        type2 = FeatureType::person;
        id2 = hri::ANONYMOUS;
        break;

      default:
        ROS_ERROR_STREAM("received an invalid type for id2: " << match->id2_type);
        return;
    }

    float confidence = match->confidence;
    if (id2 == hri::ANONYMOUS)
    {
      // if id1 is 'egbd4', id2 becomes 'anonymous_person_' -> 'anonymous_person_egbd4'
      // to create a 'unique' anonymous person for corresponding body part
      id2 += id1;

      anonymous_persons.push_back(id1);
    }
    else
    {
      // Remove previously-anonymous persons if needed

      // id1 is a person associated to id2, and id2 previously had an anonymous person attached?
      // => remove the anonymous person
      if ((type1 == FeatureType::person) &&
          std::find(anonymous_persons.begin(), anonymous_persons.end(), id2) !=
              anonymous_persons.end())
      {
        ROS_WARN_STREAM("removing anonymous person "
                        << hri::ANONYMOUS + id2 << " as it is not anonymous anymore");
        person_matcher.erase(hri::ANONYMOUS + id2);
        persons.erase(hri::ANONYMOUS + id2);
        anonymous_persons.erase(
            std::remove(anonymous_persons.begin(), anonymous_persons.end(), id2),
            anonymous_persons.end());

        publishKnownPersons();
      }
      // id1 & id2 are not persons, id1 associated to id2, both have an anonymous
      // person attached?
      // => remove one
      else if (std::find(anonymous_persons.begin(), anonymous_persons.end(), id2) !=
                   anonymous_persons.end() &&
               std::find(anonymous_persons.begin(), anonymous_persons.end(), id1) !=
                   anonymous_persons.end())
      {
        person_matcher.erase(hri::ANONYMOUS + id2);
        persons.erase(hri::ANONYMOUS + id2);
        anonymous_persons.erase(
            std::remove(anonymous_persons.begin(), anonymous_persons.end(), id2),
            anonymous_persons.end());

        publishKnownPersons();
      }
    }


    person_matcher.update({ { id1, type1, id2, type2, confidence } });


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

  void initialize_person(ID id)
  {
    // publish an updated list of all known persons
    // *before* creating the instance of ManagedPerson.
    // Doing so ensures that the client will see on /humans/persons/known the new person,
    // and will be expecting its ID on the other topics.
    hri_msgs::IdsList persons_list;
    for (auto const& kv : persons)
    {
      persons_list.ids.push_back(kv.first);
    }
    persons_list.ids.push_back(id);
    persons_list.header.stamp = ros::Time::now();
    known_persons_pub.publish(persons_list);

    persons[id] = make_shared<ManagedPerson>(_nh, id, face_id_pub, body_id_pub, voice_id_pub,
                                             alias_pub, anonymous_pub, loc_confidence_pub,
                                             tfBuffer, _reference_frame);
  }

  void publishKnownPersons()
  {
    // publish an updated list of all known persons
    hri_msgs::IdsList persons_list;

    for (auto const& kv : persons)
    {
      persons_list.ids.push_back(kv.first);
    }
    known_persons_pub.publish(persons_list);
  }

  void remove_person(ID id)
  {
    // delete the person (the ManagedPerson destructor will also shutdown the
    // corresponding topics)
    persons.erase(id);

    // publish an updated list of tracked persons
    hri_msgs::IdsList persons_list;

    for (auto const& kv : persons)
    {
      persons_list.ids.push_back(kv.first);
    }
    persons_list.header.stamp = ros::Time::now();
    tracked_persons_pub.publish(persons_list);
  }

  void publish_persons(chrono::milliseconds elapsed_time)
  {
    auto person_associations = person_matcher.get_all_associations();

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
      }
      if (association.find(FeatureType::body) != association.end())
      {
        body_id = association.at(body);
      }
      if (association.find(FeatureType::voice) != association.end())
      {
        voice_id = association.at(voice);
      }

      ////////////////////////////////////////////
      // publish the face, body, voice id corresponding to the person
      person->update(face_id, body_id, voice_id, elapsed_time);
    }

    ////////////////////////////////////////////
    // publish the list of currently actively tracked persons
    hri_msgs::IdsList persons_list;
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
      tracked_persons_pub.publish(persons_list);
      previously_tracked = actively_tracked;
    }
  }

  void set_threshold(float threshold)
  {
    person_matcher.set_threshold(threshold);
  }

private:
  NodeHandle& _nh;

  map<ID, shared_ptr<ManagedPerson>> persons;
  vector<ID> previously_tracked;
  vector<ID> anonymous_persons;

  // actively tracked persons (eg, one of face_id, body_id or voice_id is not empty for that person)
  Publisher tracked_persons_pub;
  // known persons: either actively tracked ones, or not tracked anymore (but
  // still known to the robot)
  Publisher known_persons_pub;

  Publisher face_id_pub;
  Publisher body_id_pub;
  Publisher voice_id_pub;
  Publisher alias_pub;
  Publisher anonymous_pub;
  Publisher loc_confidence_pub;

  PersonMatcher person_matcher;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  string _reference_frame;

  ros::Subscriber candidates;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hri_person_manager");
  ros::NodeHandle nh;


  float match_threshold;
  ros::param::param<float>("/humans/match_threshold", match_threshold, 0.5);

  string reference_frame;
  ros::param::param<string>("/humans/reference_frame", reference_frame, "map");


  PersonManager pm(nh, reference_frame);

  pm.set_threshold(match_threshold);

  ros::ServiceServer reset_service =
      nh.advertiseService("/hri_person_manager/reset", &PersonManager::reset, &pm);

  ros::Rate loop_rate(10);

  auto t0 = ros::Time::now();
  while (ros::ok())
  {
    t0 = ros::Time::now();
    loop_rate.sleep();
    ros::spinOnce();

    chrono::nanoseconds elapsed_time((ros::Time::now() - t0).toNSec());
    pm.publish_persons(chrono::duration_cast<chrono::milliseconds>(elapsed_time));
  }

  return 0;
}
