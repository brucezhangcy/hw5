/*
 * STUDENT CODE - EDIT THIS FILE
 *
 * SEARCH HEURISTIC
 * ----------------
 * 1. Scouts first. Debris around any wreck site is spread over three
 *    neighboring paths, so if we only drive every third robot ("scouts")
 *    at least one of them must cross every debris cluster. This cuts the
 *    cost of the early, blind part of the search to about one third.
 *
 * 2. Best-first by priority. Every robot that still has path to explore
 *    sits in an AVL tree (RobotQueue) keyed by
 *
 *        priority = depth  -  SIGNAL_BONUS * signalStrength(depth)
 *                          +  NON_SCOUT_PENALTY   (if not a scout)
 *
 *    and we always drive the robot with the LOWEST priority. With no
 *    debris this is "shallowest robot first", so the scouts advance
 *    together. Debris near a robot lowers its priority, so robots in a
 *    promising area are allowed to run ahead of the others.
 *
 * 3. Follow dense debris. When a robot finds a lot of debris on its own
 *    path (ACTIVATION_COUNT within ACTIVATION_WINDOW segments) it is
 *    probably at a wreck site. Its two neighbors become scouts and are
 *    sent straight to that depth, because the wreck may be on either of
 *    them. A former wreck site eventually runs out of debris, the
 *    priorities of its robots rise again, and the search moves on.
 *
 * 4. Fewer switches. Switching robots costs SWITCH_COST ticks, so once a
 *    robot is chosen it keeps driving (a "stint") until its priority is
 *    worse than the best waiting robot by more than STAY_BONUS.
 *
 * Every robot stays in the tree until it reaches the end of its path, so
 * if the wreck is never found every reachable segment has been explored
 * and "not findable" is the correct answer.
 */

#include "search.h"

#include <algorithm>
#include <climits>

namespace
{
  // Values below were picked by reasoning about the debris layout and then
  // checked against the performance tests; nearby values work about as well.

  // Depth "discount" for each piece of debris near a robot.
  constexpr long long SIGNAL_BONUS = 2000;
  // A non-scout waits until scouts are this much deeper than it. Large,
  // because every debris cluster is already crossed by some scout.
  constexpr long long NON_SCOUT_PENALTY = 150000;
  // Keep driving the active robot until another is better by this much.
  // Long stints make the 200-tick switch cost small compared to driving
  // (a 2000-segment stint pays at most 200 extra ticks, about 10%).
  constexpr long long STAY_BONUS = 2000;
  // ACTIVATION_COUNT debris within ACTIVATION_WINDOW segments on one path
  // is far above background debris and marks a likely wreck site.
  constexpr int ACTIVATION_WINDOW = 1000;
  constexpr int ACTIVATION_COUNT = 7;
  // Neighbors start this far before the dense area, so they see all of it.
  constexpr int TRANSIT_MARGIN = 2000;
}

Search::Search(Mothership &mothership)
    : mothership(mothership),
      robotCount(mothership.getRobotCount()),
      seafloorSurvey(robotCount),
      robotQueue(robotCount),
      pathEnd(robotCount, TETHER_LENGTH),
      position(robotCount, 0),
      transitTarget(robotCount, 0),
      isScout(robotCount, 0)
{
}

int Search::leftOf(int robot) const
{
  return (robot + robotCount - 1) % robotCount;
}

int Search::rightOf(int robot) const
{
  return (robot + 1) % robotCount;
}

void Search::setUp()
{
  // The abyss list is unordered. One pass turns it into an array so we can
  // look up any robot's safe end in O(1) instead of scanning the list.
  for (const Location &abyss : mothership.getAbysses())
  {
    pathEnd[abyss.robot] = std::min(pathEnd[abyss.robot], abyss.segment - 1);
  }

  for (int robot = 0; robot < robotCount; ++robot)
  {
    pathEnd[robot] = std::max(pathEnd[robot], 0);
    position[robot] = mothership.getRobotPosition(robot);
    isScout[robot] = (robotCount <= 3 || robot % 3 == 0);
  }

  // A scout that cannot move cannot cover its neighbors, so they scout instead.
  for (int robot = 0; robot < robotCount; ++robot)
  {
    if (isScout[robot] && position[robot] >= pathEnd[robot])
    {
      isScout[leftOf(robot)] = 1;
      isScout[rightOf(robot)] = 1;
    }
  }

  for (int robot = 0; robot < robotCount; ++robot)
  {
    if (position[robot] < pathEnd[robot])
    {
      robotQueue.insert(robot, priorityOf(robot));
    }
  }
}

long long Search::priorityOf(int robot) const
{
  int depth = position[robot];
  long long penalty = isScout[robot] ? 0 : NON_SCOUT_PENALTY;
  if (transitTarget[robot] > depth)
  {
    // On its way to a dense area: judge it by where it is going.
    depth = transitTarget[robot];
  }
  const int signal = seafloorSurvey.getSignalStrength(Location{robot, depth});
  return depth + penalty - SIGNAL_BONUS * signal;
}

long long Search::stayLimit() const
{
  if (robotQueue.empty())
  {
    return LLONG_MAX;
  }
  return robotQueue.bestKey() + STAY_BONUS;
}

void Search::refreshPriority(int robot)
{
  if (robotQueue.contains(robot))
  {
    robotQueue.update(robot, priorityOf(robot));
  }
}

void Search::makeScout(int robot)
{
  if (!isScout[robot])
  {
    isScout[robot] = 1;
    refreshPriority(robot);
  }
}

SearchResult Search::findShipwreck()
{
  setUp();

  while (!robotQueue.empty())
  {
    const int robot = robotQueue.bestRobot();
    robotQueue.remove(robot); // the active robot is out of the tree while it drives

    if (mothership.getActiveRobot() != robot)
    {
      mothership.switchToRobot(robot);
    }

    if (driveStint(robot))
    {
      return SearchResult{true, Location{robot, position[robot]}};
    }
  }

  // Every robot reached the end of its path without finding the wreck.
  return SearchResult{false, Location{}};
}

bool Search::driveStint(int robot)
{
  long long limit = stayLimit();

  while (position[robot] < pathEnd[robot])
  {
    // Never ask for more than the safe distance left (abyss / tether).
    const int distance = std::min(MAX_SEGMENTS_PER_DRIVE, pathEnd[robot] - position[robot]);
    const DriveResult result = mothership.driveActiveRobot(distance);
    position[robot] += result.segmentsMoved;

    if (result.reason == DriveStopReason::ShipwreckFound)
    {
      return true;
    }

    if (result.reason == DriveStopReason::DebrisFound)
    {
      seafloorSurvey.recordDebris(Location{robot, position[robot]});
      handleDebris(robot);
      limit = stayLimit(); // neighbors' priorities may have changed
    }

    if (priorityOf(robot) > limit)
    {
      break; // another robot is now clearly more promising
    }
  }

  if (position[robot] < pathEnd[robot])
  {
    robotQueue.insert(robot, priorityOf(robot));
  }
  else
  {
    finishRobot(robot);
  }
  return false;
}

void Search::handleDebris(int robot)
{
  const int segment = position[robot];
  if (seafloorSurvey.countOnPath(robot, segment - ACTIVATION_WINDOW, segment) >= ACTIVATION_COUNT)
  {
    activateNeighbors(robot);
  }
  // The new debris is within signal range of the neighbors too.
  refreshPriority(leftOf(robot));
  refreshPriority(rightOf(robot));
}

void Search::activateNeighbors(int robot)
{
  const int target = std::max(0, position[robot] - ACTIVATION_WINDOW - TRANSIT_MARGIN);
  for (int neighbor : {leftOf(robot), rightOf(robot)})
  {
    if (neighbor == robot)
    {
      continue;
    }
    isScout[neighbor] = 1;
    const bool alreadyTravelling = transitTarget[neighbor] > position[neighbor];
    if (!alreadyTravelling && position[neighbor] < target)
    {
      transitTarget[neighbor] = std::min(target, pathEnd[neighbor]);
    }
    refreshPriority(neighbor);
  }
}

void Search::finishRobot(int robot)
{
  // This path is fully explored. If it was a scout, its neighbors must
  // now be explored on their own, so they become scouts too.
  if (isScout[robot])
  {
    makeScout(leftOf(robot));
    makeScout(rightOf(robot));
  }
}
