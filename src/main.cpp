#include "pathPlanning.h"

#include <iostream>
#include <vector>
#include <math.h>


int main(int, char **)
{
    std::cout << "Robot challenge start\n";
    Robot r;
    r.addWaypoint({9,0,0});
    r.addWaypoint({15,0,0});
    r.addWaypoint({15,8,0});
    r.addWaypoint({15,15,0});
    r.addWaypoint({15,15,15});
    r.followPath();
}
