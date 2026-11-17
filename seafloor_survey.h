/*
 * STUDENT CODE - OPTIONAL - EDIT, REPLACE, OR DELETE
 *
 * SeafloorSurvey remembers every piece of debris the robots have found
 * and answers "how much debris is near this spot?" (signal strength).
 *
 * Storage: one vector of segment numbers per robot path.
 * A robot only ever drives forward, so debris on a path is discovered in
 * increasing segment order. push_back therefore keeps each vector sorted
 * for free, and counting the debris inside a range of segments is two
 * binary searches (O(log k) for k debris on that path).
 */

#pragma once

#include <vector>

#include "mothership.h"

class SeafloorSurvey
{
public:
  explicit SeafloorSurvey(int robotCount);

  // Debris on a path must be recorded in increasing segment order
  // (always true, because robots only drive forward).
  void recordDebris(Location location);

  // Number of debris found on one path with low <= segment <= high.
  int countOnPath(int robot, int low, int high) const;

  // Signal strength from the assignment: debris within SIGNAL_RANGE
  // segments of location.segment, on the robot's own path and on the
  // paths immediately to its left and right.
  int getSignalStrength(Location location) const;

private:
  int robotCount;
  std::vector<std::vector<int>> debrisByRobot;
};
