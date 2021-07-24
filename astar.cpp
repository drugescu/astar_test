// GPL v3
// Dragos-Ronald Rugescu

#include "astar.hpp"
#include <iostream>

using namespace std;

int main() {
    aStar a = aStar(10, new point(0,0), new point(6,6));

    a.setMapSize(10); // Redundant but to demonstrate

    std::cout << "Map size: ";

    auto m = a.getMapSize();

    printCoords(m);

    a.setInaccessible(1, 1);

    a.printMap();

    std::cout << "A* running..." << std::endl;

    a.runAlgorithm();

    std::cout << "A* finalized." << std::endl;
    std::cout << "Printing path:" << std::endl;

    for (auto n : a.returnPath()) {
      std::cout << " Path point : " << n;
    }

    std::cout << "Done." << std::endl;

    return 0;
};
