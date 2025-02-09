^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hri_person_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2022-10-14)
------------------
* track /h/{f|b|v}/tracked to automatically create/remove anonymous persons
* use Boost graph bundled properties to store vertex ids
  Until now, ID where stored in a separate map. This would cause major issues when
  a node was removed as Boost would re-assign vertex ids that would not match anymore
  the ID that we had stored.
* publish dot graph of humans on /humans/graph
* more tests -- tests do not pass for a strange reason, need investigating
* add accessor to set/get a person's alias
* anonymous_persons can be removed by publishing a candidate match with confidence=0
* [minor] improved console logging behaviour
* Contributors: Séverin Lemaignan

0.2.4 (2022-07-12)
------------------
* missing 'break' in a switch leading to mis-handling anonymous persons
* Contributors: Séverin Lemaignan

0.2.3 (2022-06-01)
------------------
* update to new IdsMatch.msg
* Contributors: Séverin Lemaignan

0.2.2 (2022-06-01)
------------------
* attempting to fix unit-test compilation error on ferrum
* Contributors: Séverin Lemaignan

0.2.1 (2022-05-27)
------------------
* fix variable shadowing/initialisation order
* add missing dep on std_srvs
* Contributors: Séverin Lemaignan

0.2.0 (2022-05-27)
------------------
* increase candidate_matches subscriber queue to ensure no message missed
* large refactor, moving 'managed persons' to their own class
  While here:
  - added a /hri_person_manager/reset service to clear all existing
  associations;
  - updated unit-tests to latest libhri (0.5.0)
* make reference tf frame a parameter /humans/reference_frame
* Publish the person's tf frame + /location_confidence
  While here, encapsulate code in a PersonManager class
* support anonymous persons, ie persons that we are aware of because a face,
  body, voice has been detected, but that is not yet identified.
* publish separately /humans/persons/tracked (actively tracked) and /humans/persons/known
* Contributors: Séverin Lemaignan

0.1.0 (2022-03-06)
------------------
* ROS wrapper + test of ROS node
* complete implementation of PersonMatcher algo. Tests pass.
* Contributors: Séverin Lemaignan
