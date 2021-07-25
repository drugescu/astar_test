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
    a.setInaccessible(2, 1);
    a.setInaccessible(1, 2);
    a.setInaccessible(1, 3);
    a.setInaccessible(3, 1);
    a.setInaccessible(5, 5);

    std::cout << "Printing map: " << std::endl;
    a.printMap();

    std::cout << "A* running..." << std::endl;

    a.runAlgorithm();

    std::cout << "A* finalized." << std::endl;
    std::cout << "Printing path:" << std::endl;

    for (auto n : a.getPath()) {
      std::cout << " Path point : " << n;
    }

    std::cout << "Printing map with final path: " << std::endl;
    a.printMap(WITH_PATH);

    std::cout << "Done." << std::endl;

    return 0;
};
