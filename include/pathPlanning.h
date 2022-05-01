#include <vector>


class Point
{
public:
    double x;
    double y;
    double z;

    Point(double x = 0, double y = 0, double z = 0);
    double distanceTo(Point);
    Point moveTowards(Point, double);
};
Point crossP (Point, Point);

class Pose
{
public:
    double rotMatrix [3][3];
    Point position;
    Pose(Point position = {0, 0, 0});

    void reorientTo(Point);
};

class Robot
{
private:
    Pose currentPose;
    std::vector<Point> path;
    double distanceToEnd;
    double distanceToWaypoint;
    int nextWaypoint;
    double stepMaxDistance;
    void updateDistanceToEnd();
    void updateDistanceToWaypoint();
    void updateDistances();
    void moveStep();

public:
    Robot();
    Robot(Pose);
    void addWaypoint(Point);
    void followPath();
};