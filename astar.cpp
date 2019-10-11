#include "astar.hpp"
#include <iostream>

using namespace std;

int main() {
    aStar a = aStar(new point(0,0), new point(0,0));

    cout << "A* running..." << endl;

    a.runAlgorithm();

    cout << "A* finalized." << endl;

    return 0;
};
