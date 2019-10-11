// Prototype
// A star algorithm class - matrix
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

#include <queue>
#include <vector>
#include <utility>
#include <memory>
#include <cmath>
#include <algorithm>
#include <iostream>

#define MANHATTAN_DISTANCE  1
#define EUCLIDEAN_DISTANCE  2

using namespace std;

// Defs and structs

typedef pair<int,int> coords;

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

      void print() { cout << "Coords: " << pos.first << "," << pos.second << ", f = "
                     << f << ", g = " << g << ", h = " << h << endl; }

      bool operator== (const point& b) {
        return ((this->pos.first == b.pos.first) && (this->pos.second == b.pos.second));
      }

};

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
        point origin;
        point destination;

        queue<point> open, closed;

        vector<point> path;

    public:
        aStar();
        aStar(const point origin, const point destination);
        aStar(const point* origin, const point* destination);

        void setOrigin(const point origin);
        auto const getOrigin() { return origin; }

        void setDestination(const point destination);
        auto const getDestination() { return destination; }

        void setHeuristic();
        int runAlgorithm();

        vector<point> returnPath();
};

// Implementation

queue<point> sortQueue(queue<point> in) {
  vector<point> v;
  auto inp = in;

  while (inp.empty() == false) {
    v.push_back(inp.front());
    inp.pop();
  }

  sort(v.begin(), v.end(), [](const point &a, const point &b) { return a.f < b.f; });

  for (int i = 0; i < inp.size(); i++) {
    inp.push(v[i]);
  }

  return inp;
}

aStar::aStar() {
    point o(0,0), d(0,0);

    o.f = o.g = o.h = d.f = d.g = d.h = 0.0f;

    origin = o;
    destination = d;
}

aStar::aStar(const point origin, const point destination) {
    aStar::origin = origin;
    aStar::destination = destination;
}

aStar::aStar(const point* origin, const point* destination) {
    aStar::origin = *origin;
    aStar::destination = *destination;
}

void aStar::setHeuristic() {};

vector<point> aStar::returnPath() {
    auto path = new vector<point>();
    return *path;
}

int aStar::runAlgorithm() {
    // Let initial lists be empty
    queue<point> emptyO, emptyC;
    swap(open, emptyO);
    swap(open, emptyC);

    // Put start node on open list
    open.emplace(origin);

    // Loop until you find the end
    while(!open.empty()) {

      // Get the current node - least f
      open = sortQueue(open);

      // Remove from open
      auto p = open.back();
      p.print();
      open.pop();

      // Add to closed
      closed.emplace(p);

      // If goal, return
      if (p == destination) {
        cout << "Arrived at destination." << endl;
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
