/*
 * STUDENT CODE - OPTIONAL - EDIT, REPLACE, OR DELETE
 */

#include "seafloor_survey.h"

#include <algorithm>

SeafloorSurvey::SeafloorSurvey(int robotCount)
    : robotCount(robotCount), debrisByRobot(robotCount)
{
}

void SeafloorSurvey::recordDebris(Location location)
{
  // Appending keeps the vector sorted (see header).
  debrisByRobot[location.robot].push_back(location.segment);
}

int SeafloorSurvey::countOnPath(int robot, int low, int high) const
{
  const std::vector<int> &debris = debrisByRobot[robot];
  if (debris.empty() || low > high)
  {
    return 0;
  }
  // Binary search for the first debris >= low and the first debris > high.
  auto first = std::lower_bound(debris.begin(), debris.end(), low);
  auto last = std::upper_bound(first, debris.end(), high);
  return static_cast<int>(last - first);
}

int SeafloorSurvey::getSignalStrength(Location location) const
{
  const int low = location.segment - SIGNAL_RANGE;
  const int high = location.segment + SIGNAL_RANGE;

  // The paths form a circle, so robot 0 and robot N-1 are neighbors.
  const int robot = location.robot;
  const int left = (robot + robotCount - 1) % robotCount;
  const int right = (robot + 1) % robotCount;

  int strength = countOnPath(robot, low, high);
  // With only 1 or 2 robots a neighbor can be the same path; count each path once.
  if (left != robot)
  {
    strength += countOnPath(left, low, high);
  }
  if (right != robot && right != left)
  {
    strength += countOnPath(right, low, high);
  }
  return strength;
}
