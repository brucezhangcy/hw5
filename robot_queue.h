/*
 * STUDENT CODE
 *
 * RobotQueue keeps the robots that still have path left to explore,
 * ordered by a priority key. A LOWER key means "drive this robot sooner".
 *
 * It is an AVL tree (AVLSet from trees.h) of (key, robot) pairs, plus an
 * array holding each robot's current key so we can find its tree entry.
 *
 *   best robot:     begin()                 O(1)
 *   change a key:   erase old, insert new   O(log n)
 *   add / remove:                           O(log n)
 *
 * Changing a key follows the "Ordered sets: updating an element" example
 * from examples/using_data_structures.cpp: there is no in-place update,
 * so we erase the old entry and insert the new one.
 */

#pragma once

#include <vector>

#include "trees.h"

struct RobotEntry
{
  long long key;
  int robot;

  bool operator<(const RobotEntry &other) const
  {
    if (key != other.key)
    {
      return key < other.key;
    }
    return robot < other.robot; // break ties so every entry is unique
  }
};

class RobotQueue
{
public:
  explicit RobotQueue(int robotCount);

  bool empty() const;
  bool contains(int robot) const;

  void insert(int robot, long long key); // robot must not already be queued
  void update(int robot, long long key); // robot must be queued
  void remove(int robot);                // robot must be queued

  int bestRobot() const;      // queue must not be empty
  long long bestKey() const;  // queue must not be empty

private:
  AVLSet<RobotEntry> entries;
  std::vector<long long> keys;  // keys[robot] = key stored in the tree
  std::vector<char> queued;     // queued[robot] = is the robot in the tree?
};
