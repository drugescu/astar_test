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
//#include <boost/numeric/ublas/matrix.hpp>
//#include <boost/numeric/ublas/io.hpp>
#include <Eigen/Dense>

using namespace Eigen;

#define MANHATTAN_DISTANCE  1
#define EUCLIDEAN_DISTANCE  2

#define INITIAL_SIZE      100

//using namespace boost::numeric::ublas;

// Defs and structs

typedef std::pair<int,int> coords;

// Print out coords

void printCoords(coords p) {
  std::cout << "Point : { " << p.first << ", " << p.second << "}" << std::endl;
}
/*template <>
std::ostream& coords::operator<< (std::ostream &out) {
    out << "Pair {" << this->first << "," << this->second << std::endl;
    return out;
}*/

// Point class

class point {
    public:
      coords pos;
      float f;
      float g;
      float h;

      point() {};

      point(int a, int b) {
        pos.first = a;
        pos.second = b;
      }

      void print() { std::cout << "Coords: " << pos.first << "," << pos.second << ", f = "
                     << f << ", g = " << g << ", h = " << h << std::endl; }

      bool operator== (const point& b) {
        return ((this->pos.first == b.pos.first) && (this->pos.second == b.pos.second));
      }

      bool operator< (const point& b) {
        return (this->f < b.f);
      }

      friend std::ostream& operator<< (std::ostream &out, const point& p) {
        out << "Point {" << p.pos.first << "," << p.pos.second << "}, f = "
            << p.f << ", g = " << p.g << ", h = " << p.h << std::endl;
        return out;
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
        Heuristic();

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

        std::priority_queue<point> open, closed;

        //matrix<float> m(int, int);
        MatrixXd m;

        std::vector<point> path;

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

        // Algorithm
        int runAlgorithm();

        // Result
        std::vector<point> returnPath();
};

// Implementation

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

std::vector<point> aStar::returnPath() {
    auto path = new std::vector<point>();
    return *path;
}

// Algorithm

int aStar::runAlgorithm() {
    // Let initial lists be empty
    std::priority_queue<point> emptyO, emptyC;
    swap(open, emptyO);
    swap(open, emptyC);

    // Put start node on open list, it also sorts the queue
    open.push(origin);

    // Loop until you find the end
    while(!open.empty()) {

      // Remove from open
      auto p = open.top();
      p.print();
      open.pop();

      // Add to closed
      closed.emplace(p);

      // If goal, return
      if (p == destination) {
        std::cout << "Arrived at destination." << std::endl;
        return EXIT_SUCCESS;
      }

      // Generate children
      // For each child
        // If in closed list, continue
        // Create f, g, h
        //   child.g = curentnode.g + distance
        //   child.h = heuristic to end
        //   child.f = sum
        // Child is already in open, and g huigher that open , continue
        // Add child to open listl
    }

    return EXIT_FAILURE;
}
