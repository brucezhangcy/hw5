/*
 * STUDENT CODE
 */

#include "robot_queue.h"

RobotQueue::RobotQueue(int robotCount)
    : keys(robotCount, 0), queued(robotCount, 0)
{
}

bool RobotQueue::empty() const
{
  return entries.empty();
}

bool RobotQueue::contains(int robot) const
{
  return queued[robot] != 0;
}

void RobotQueue::insert(int robot, long long key)
{
  keys[robot] = key;
  queued[robot] = 1;
  entries.insert(RobotEntry{key, robot});
}

void RobotQueue::update(int robot, long long key)
{
  if (keys[robot] == key)
  {
    return; // nothing changed, skip the erase + insert
  }
  entries.erase(RobotEntry{keys[robot], robot});
  keys[robot] = key;
  entries.insert(RobotEntry{key, robot});
}

void RobotQueue::remove(int robot)
{
  entries.erase(RobotEntry{keys[robot], robot});
  queued[robot] = 0;
}

int RobotQueue::bestRobot() const
{
  return entries.begin()->robot;
}

long long RobotQueue::bestKey() const
{
  return entries.begin()->key;
}
