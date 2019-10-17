// GPL v3
// Dragos-Ronald Rugescu
//
// Prototype
// A star algorithm class - matrix and graph
//   Variables:
//        _Object origin
//        _Object destination
//        vector open
//        vector closed
//        Object Heuristic
//   Functions:
//        setOrigin(_Object origin)
//        setDestination(_Object destination)
//        runAlgorithm() -> int [0, 1] as [not_found, found]
//        returnPath() -> vector of std::pair<int,int>
//
//   origin, destination = std::pair<int,int>

#include <array>
#include <queue>
#include <vector>
#include <utility>
#include <memory>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

#define MANHATTAN_DISTANCE  1
#define EUCLIDEAN_DISTANCE  2

#define INITIAL_SIZE      100

#define COST                1

// std::find - O(n)
#define contains(vec, item)     (std::find(vec.begin(), vec.end(), item)) != vec.end()

// Defs and structs

class point;

typedef std::vector<point> aStarList;

typedef std::pair<int,int> coords;

enum Direction { NORTH = 0, SOUTH, EAST, WEST, NORTH_EAST, SOUTH_EAST, NORTH_WEST, SOUTH_WEST };

constexpr std::initializer_list<Direction> dirList = { NORTH, SOUTH, EAST, WEST, NORTH_EAST, SOUTH_EAST, NORTH_WEST, SOUTH_WEST };

// Print out coords

void printCoords(coords p) {
  std::cout << "Point : { " << p.first << ", " << p.second << " }" << std::endl;
}

 std::ostream& operator<< (std::ostream &out, const coords& p) {
  out << "Coords {" << p.first << "," << p.second << "}" << std::endl;
  return out;
}

// Point class

class point {
    public:
      coords pos;
      float f = 0.0f;
      float g = 0.0f;
      float h = 0.0f;

      coords parent = coords(0,0);

      point() { };

      point(int a, int b) {
        pos.first = a;
        pos.second = b;
      }

      point(point p, int deltaX, int deltaY) {
        pos.first  = p.pos.first + deltaX;
        pos.second = p.pos.second + deltaY;

        f = p.f;
        g = p.g;
        h = p.h;

        parent = p.pos;
      }

      void print() {
        std::cout << "Coords: " << pos.first << "," << pos.second << ", f = "
                     << f << ", g = " << g << ", h = " << h;

        std::cout << ", Parent = {" << parent.first << "," << parent.second << "}" << std::endl;
      }

      bool operator== (const point& b) {
        return ((this->pos.first == b.pos.first) && (this->pos.second == b.pos.second));
      }

      bool operator!= (const point& b) {
        return ((this->pos.first != b.pos.first) || (this->pos.second != b.pos.second));
      }

      // Used for std::sort and other algorithms
      bool operator< (const point& b) {
        return (this->f < b.f);
      }

      point& operator= (const point& p) {
        pos.first = p.pos.first; pos.second = p.pos.first;
        f = p.f; g = g; h = p.h;

        return *this;
      }

      friend std::ostream& operator<< (std::ostream &out, const point& p) {
        out << "Point {" << p.pos.first << "," << p.pos.second << "}, f = "
            << p.f << ", g = " << p.g << ", h = " << p.h;

        out << ", Parent = {" << p.parent.first << "," << p.parent.second << "}" << std::endl;

        return out;
      }

      void setScores(int g, int h) {
        this->g = g;
        this->h = h;
        this->f = g + h;
      }
};

bool operator< (const point&a, const point& b) {
  return (a.f < b.f);
}

// Interface

class Heuristic {
    private:
        int internalHeuristic = EUCLIDEAN_DISTANCE;
    public:
        Heuristic() {};

        void setHeuristic(int num);
        int getHeuristic();

        float distanceOp(const point p1, const point p2);
};

// Implementation

void Heuristic::setHeuristic(int num) {
    internalHeuristic = num;
}

int Heuristic::getHeuristic() { return internalHeuristic; }

float Heuristic::distanceOp(const point p1, const point p2) {
    switch (internalHeuristic)
    {
    case MANHATTAN_DISTANCE:
        return abs(p1.pos.first - p2.pos.first) + abs(p1.pos.second - p2.pos.second);

    case EUCLIDEAN_DISTANCE:
        return sqrt(pow(p1.pos.first - p2.pos.first, 2) + pow(p1.pos.second - p2.pos.second, 2));

    default:
        return 0.0f;
        break;
    }
}

// Interface

class aStar {
    private:

        uint16_t sizeM, sizeN;

        point origin, destination;

        aStarList open, closed;

        Heuristic h;

        MatrixXd m;

        point path_start;

    public:
        // Constructor versions
        aStar();
        aStar(int size);
        aStar(const point origin, const point destination);
        aStar(int size, const point origin, const point destination);
        aStar(int sizeM, int sizeN, const point origin, const point destination);
        aStar(const point* origin, const point* destination);
        aStar(int size, const point* origin, const point* destination);
        aStar(int sizeM, int sizeN, const point* origin, const point* destination);

        // Origin
        void setOrigin(const point origin);
        auto const getOrigin() { return origin; }

        // Destination
        void setDestination(const point destination);
        auto const getDestination() { return destination; }

        // Size of internal matrix
        void setMapSize(int size);
        void setMapSize(int sizeM, int sizeN);
        coords getMapSize();

        // Heuristic
        void setHeuristic();
        Heuristic& getHeuristic() { return h; }

        // Algorithm
        int runAlgorithm();

        // Result
        std::vector<coords> returnPath();
};

// Implementation

#pragma region aStar_constructors

aStar::aStar() {
    point o(0,0), d(0,0);

    o.f = o.g = o.h = d.f = d.g = d.h = 0.0f;

    origin = o;
    destination = d;

    // Initialize map with basic size
    m.resize(INITIAL_SIZE, INITIAL_SIZE);
}

aStar::aStar(int size) {
    point o(0,0), d(0,0);

    o.f = o.g = o.h = d.f = d.g = d.h = 0.0f;

    origin = o;
    destination = d;

    // Initialize map with basic size
    m.resize(size, size);
}

aStar::aStar(int size, const point origin, const point destination) {
    // Initialize origin
    aStar::origin = origin;
    aStar::destination = destination;

    // Initialize map with basic size
    m.resize(size, size);
}

aStar::aStar(int sizeM, int sizeN, const point origin, const point destination) {
    // Initialize origin
    aStar::origin = origin;
    aStar::destination = destination;

    // Initialize map with basic size
    m.resize(sizeM, sizeN);
}

aStar::aStar(const point origin, const point destination) {
    aStar::origin = origin;
    aStar::destination = destination;

    m.resize(INITIAL_SIZE, INITIAL_SIZE);
}

aStar::aStar(const point* origin, const point* destination) {
    aStar::origin = *origin;
    aStar::destination = *destination;

    m.resize(INITIAL_SIZE, INITIAL_SIZE);
}

aStar::aStar(int size, const point* origin, const point* destination) {
    // Initialize origin
    aStar::origin = *origin;
    aStar::destination = *destination;

    // Initialize map with basic size
    m.resize(size, size);
}

aStar::aStar(int sizeM, int sizeN, const point* origin, const point* destination) {
    // Initialize origin
    aStar::origin = *origin;
    aStar::destination = *destination;

    // Initialize map with basic size
    m.resize(sizeM, sizeN);
}

#pragma endregion

// Map size

void aStar::setMapSize(int size) {
  this->sizeM = this->sizeN = size;
  this->m.resize(size, size);
}

void aStar::setMapSize(int sizeM, int sizeN) {
  this->sizeM = sizeM;
  this->sizeN = sizeN;
  this->m.resize(sizeM, sizeN);
}

coords aStar::getMapSize() { return *(new coords(sizeN, sizeM)); }

// Heuristic

void aStar::setHeuristic() {};

// Result

std::vector<coords> aStar::returnPath() {

    auto path = std::vector<coords>();

    path.push_back(path_start.pos);
    //while (current_node != nullptr) {
      //path.push_back(*current_node);
      //current_node = &(*current_node->parent);
    //}

    return path;
}

// Algorithm

point getAdjacent(point p, int dir) {
  switch (dir)
  {
    case (NORTH)      : return point(p, -COST,     0);
    case (NORTH_WEST) : return point(p, -COST, -COST);
    case (NORTH_EAST) : return point(p, -COST,  COST);
    case (SOUTH)      : return point(p,  COST,     0);
    case (SOUTH_WEST) : return point(p,  COST, -COST);
    case (SOUTH_EAST) : return point(p,  COST,  COST);
    case (WEST)       : return point(p,     0, -COST);
    case (EAST)       : return point(p,     0,  COST);

    default           : return point();
  }
}

std::vector<point> generateChildren(point p) {
    std::vector<point> children;

    // Check map and see if children are all valid - paths exist
    for (auto DIR : dirList) {
      children.push_back(getAdjacent(p, DIR));
    }

    return children;
}

int aStar::runAlgorithm() {
    // Let initial lists be empty
    aStarList emptyO, emptyC;
    swap(open, emptyO);
    swap(open, emptyC);

    // Put start node on open list - O(1)
    open.push_back(origin);

    // Loop until you find the end
    while(!open.empty()) {

      // Sort the list - (N*logN)
      std::sort(open.begin(), open.end());

      // Remove from open - O(1)
      auto p = open.front();
      std::cout << "Removing best point : " << p;
      open.erase(open.begin()); // O(1) best case

      // Add to closed - O(1)
      closed.push_back(p);

      // If goal, return
      if (p == destination) {
        std::cout << "Arrived at destination." << std::endl;
        path_start = p;
        return EXIT_SUCCESS;
      }

      // Generate children
      // For each child
      for (auto child : generateChildren(p)) {

        std::cout << "Generated child : " << child;

        // If in closed list, continue
        if (contains(closed, child))
          continue;

        // Update g and h - don't forget to look at the map for cost
        // Replace basic COST with COST*map(parent_height-child_height) unless
        //   child has height - 1 in which case you cannot use the child...
        //   but this will be handles in child legality step above
        //   calculateCost fnction required here.
        // Create f, g, h
        //   child.g = curentnode.g + distance
        //   child.h = heuristic to end
        //   child.f = sum
        child.setScores(p.g + COST,
                        getHeuristic().distanceOp(child, getDestination()));

        std::cout << "Changed child : " << child;
        // Child is already in open, and g higher that open, continue
        // Add child to open list
        open.push_back(child);
      }
    }

    return EXIT_FAILURE;
}
