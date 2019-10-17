// GPL v3
// Dragos-Ronald Rugescu

#include "astar.hpp"
#include <iostream>

using namespace std;

int main() {
    aStar a = aStar(100, new point(0,0), new point(6,6));

    a.setMapSize(100);

    std::cout << "Map size: ";

    auto m = a.getMapSize();

    printCoords(m);

    std::cout << "A* running..." << std::endl;

    a.runAlgorithm();

    std::cout << "A* finalized." << std::endl;

    for (auto n : a.returnPath()) {
      std::cout << "Path point : " << n << std::endl;
    }

    return 0;
};
