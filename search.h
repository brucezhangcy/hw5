/*
 * STUDENT CODE - EDIT THIS FILE
 * Keep the class name and required interface.
 * Otherwise, add or change any data members or
 * member functions.
 */

#pragma once

#include <vector>

#include "mothership.h"
#include "robot_queue.h"
#include "seafloor_survey.h"

class Search
{
public:
  explicit Search(Mothership &mothership); // REQUIRED

  SearchResult findShipwreck(); // REQUIRED

private:
  Mothership &mothership; // REQUIRED

  int robotCount;
  SeafloorSurvey seafloorSurvey; // every debris found so far
  RobotQueue robotQueue;         // AVL tree: which robot to drive next

  std::vector<int> pathEnd;       // last segment a robot can safely reach
  std::vector<int> position;      // current segment of each robot
  std::vector<int> transitTarget; // depth a robot should jump to (0 = none)
  std::vector<char> isScout;      // scouts are explored first

  void setUp();
  int leftOf(int robot) const;
  int rightOf(int robot) const;

  long long priorityOf(int robot) const;
  long long stayLimit() const;
  void refreshPriority(int robot);

  bool driveStint(int robot);
  void handleDebris(int robot);
  void activateNeighbors(int robot);
  void finishRobot(int robot);
  void makeScout(int robot);
};
