#include "pathPlanning.h"

#include <iostream>
#include <vector>
#include <math.h>

Point::Point(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

double Point::distanceTo(Point nextPoint)
{
    return sqrt(pow(nextPoint.x - x, 2) +
                pow(nextPoint.y - y, 2) +
                pow(nextPoint.z - z, 2) * 1.0);
}

Point Point::moveTowards(Point pointTowards, double distance)
{
    double magDist = distanceTo(pointTowards);
    double xu = x + (pointTowards.x - x) / magDist * distance;
    double yu = y + (pointTowards.y - y) / magDist * distance;
    double zu = z + (pointTowards.z - z) / magDist * distance;
    return Point(xu, yu, zu);
}

Point crossP(Point u, Point v)
{
    double wx = u.y * v.z - u.z * v.y;
    double wy = u.z * v.x - u.x * v.z;
    double wz = u.x * v.y - u.y * v.x;
    Point w = {wx, wy, wz};
    return w;
}

Pose::Pose(Point position)
{
    this->position = position;
}

void Pose::reorientTo(Point target)
{
    double mag = position.distanceTo(target);
    Point forward = {(target.x - position.x) / mag,
                     (target.y - position.y) / mag,
                     (target.z - position.z) / mag};
    rotMatrix[0][0] = forward.x;
    rotMatrix[1][0] = forward.y;
    rotMatrix[2][0] = forward.z;

    Point upDir = crossP(forward, {0, 1, 0});
    if (forward.x == 0 && forward.y == 1 && forward.z == 0)
    {
        upDir.x = 0;
        upDir.y = 0;
        upDir.z = 1;
    }
    mag = upDir.distanceTo({0, 0, 0});
    rotMatrix[0][2] = upDir.x / mag;
    rotMatrix[1][2] = upDir.y / mag;
    rotMatrix[2][2] = upDir.z / mag;

    Point leftDir = crossP(upDir, forward);
    mag = leftDir.distanceTo({0, 0, 0});
    rotMatrix[0][1] = leftDir.x / mag;
    rotMatrix[1][1] = leftDir.y / mag;
    rotMatrix[2][1] = leftDir.z / mag;
}

Robot::Robot()
{
    nextWaypoint = 0;
    distanceToEnd = 0;
    distanceToWaypoint = 0;
    stepMaxDistance = 10;
}

Robot::Robot(Pose initialPose)
{
    currentPose = initialPose;
    Robot();
}

void Robot::addWaypoint(Point waypoint)
{
    path.push_back(waypoint);
}

void Robot::updateDistanceToEnd()
{
    distanceToEnd = 0;
    Point currentPoint = currentPose.position;
    for (std::vector<Point>::iterator it = path.begin() + nextWaypoint; it != path.end(); it++)
    {
        distanceToEnd += currentPoint.distanceTo(*it);
        currentPoint = *it;
    }
}

void Robot::updateDistanceToWaypoint()
{
    distanceToWaypoint = currentPose.position.distanceTo(path.at(nextWaypoint));
}

void Robot::updateDistances()
{
    updateDistanceToWaypoint();
    updateDistanceToEnd();
}

void Robot::moveStep()
{
    double stepDistance = 0;
    while (distanceToWaypoint <= (stepMaxDistance - stepDistance))
    {
        currentPose.position = path.at(nextWaypoint);
        stepDistance = distanceToWaypoint;
        if (nextWaypoint + 1 == path.size())
        {
            updateDistances();
            return;
        }
        nextWaypoint++;
        updateDistances();
        currentPose.reorientTo(path.at(nextWaypoint));
        std::cout << "Waypoint " << nextWaypoint - 1 << " reached: "
                  << stepMaxDistance - stepDistance << " cm to complete step.\n";
    }
    currentPose.position = currentPose.position.moveTowards(path.at(nextWaypoint), stepMaxDistance - stepDistance);
    updateDistances();
    std::cout << "Step completed at: "
              << distanceToWaypoint << " cm to next waypoint.\n\t"
              << distanceToEnd << " cm to end.\n";
}

void Robot::followPath()
{
    if (path.size() == 0)
    {
        std::cout << "No path to follow\n";
        return;
    }
    updateDistances();
    currentPose.reorientTo(path.at(nextWaypoint));
    std::cout << "Starting moving: " << distanceToEnd << " cm to end.\n";
    while (distanceToEnd > 0)
    {
        moveStep();
    }
    std::cout << "End reached\n";
}