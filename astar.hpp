// GPL v3
// Dragos-Ronald Rugescu
//
// A star algorithm class - matrix and graph

#include "utils.hpp"

using namespace Eigen;

// std::find - O(n)
#define contains(vec, item)     (std::find(vec.begin(), vec.end(), item)) != vec.end()

// Defs and structs

class point; // Far declaration

typedef std::vector<point> aStarList;

typedef std::pair<int,int> coords;

enum Direction { NORTH = 0, SOUTH, EAST, WEST, NORTH_EAST, SOUTH_EAST, NORTH_WEST, SOUTH_WEST };

constexpr std::initializer_list<Direction> dirList = { NORTH, SOUTH, EAST, WEST, NORTH_EAST, SOUTH_EAST, NORTH_WEST, SOUTH_WEST };

bool matchPointCoords(const point& p, const coords& c); // Far declaration

auto find_item(std::vector<point> vec, coords c) {
  auto cs = std::vector<coords>();
  cs.push_back(c);

  return std::find_first_of(vec.begin(), vec.end(), cs.begin(), cs.end(), matchPointCoords);
}

// Print out coords

void printCoords(coords p) {
  std::cout << "Point : { " << p.first << ", " << p.second << " }" << std::endl;
}

std::ostream& operator<< (std::ostream& out, const coords& p) {
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

      coords parent = coords(INEXISTENT, INEXISTENT);

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
        pos.first = p.pos.first; pos.second = p.pos.second;
        f = p.f; g = p.g; h = p.h;

        parent = p.parent;

        return *this;
      }

      friend std::ostream& operator<< (std::ostream &out, const point& p) {
        out << "Point {" << p.pos.first << "," << p.pos.second << "}, f = "
            << p.f << ", g = " << p.g << ", h = " << p.h;

        out << ", Parent = " << p.parent;

        return out;
      }

      void setScores(int g, int h) {
        this->g = g;
        this->h = h;
        this->f = g + h;
      }

};

bool matchPointCoords(const point& p, const coords& c) { return (p.pos == c); }

bool operator< (const point&a, const point& b) {
  return (a.f < b.f);
}

// Interface

class Heuristic {
    private:
        int internalHeuristic = EUCLIDEAN_DISTANCE;
    public:
        Heuristic() {};

        // Getter, setter and delta
        float getDelta(const coords& p1, const coords& p2, const int& power);
        void setHeuristic(int num);
        int getHeuristic();

        float distanceOp(const point p1, const point p2);
};

// Implementation

void Heuristic::setHeuristic(int num) {
    internalHeuristic = num;
}

int Heuristic::getHeuristic() { return internalHeuristic; }

float Heuristic::getDelta(const coords& p1, const coords& p2, const int& power) {
  if (power == IDENTITY)
    return abs(p1.first - p2.first) + abs(p1.second - p2.second);
  else if (power == SQUARED)
    return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));
  else if (power == MINIMUM)
    return std::min(abs(p1.first - p2.first), abs(p2.second - p2.second));
  else
    return -std::numeric_limits<float>::infinity();
}

float Heuristic::distanceOp(const point p1, const point p2) {
    switch (internalHeuristic)
    {
    case MANHATTAN_DISTANCE:
        return getDelta(p1.pos, p2.pos, IDENTITY);

    case EUCLIDEAN_DISTANCE:
        return getDelta(p1.pos, p2.pos, SQUARED);

    case OCTOGONAL_DISTANCE:
        return (10.0f) * getDelta(p1.pos, p2.pos, IDENTITY)
             + (-6.0f) * getDelta(p1.pos, p2.pos, MINIMUM);

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

        point path_start = point(INEXISTENT, INEXISTENT);

        std::vector<coords> path;

        std::vector<coords> returnPath();

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

        // Collision map
        void setInaccessible(const point p);
        void setInaccessible(const coords& c);
        void setInaccessible(int x, int y);

        // Algorithm
        std::vector<point> generateChildren(point p);
        point getAdjacent(point p, int dir);
        bool isValid(const coords& p);
        int runAlgorithm();

        // Result
        void printMap();
        void printMap(bool with_path);
        std::vector<coords> getPath();
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

#pragma region MapSize

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

#pragma endregion

#pragma region Heuristics

// Heuristic

void aStar::setHeuristic() {};

#pragma endregion

#pragma region Results and Validity

// Result

bool aStar::isValid(const coords& p) {
  if (p == coords(INEXISTENT, INEXISTENT))
    return false;

  if (p.first > sizeM || p.second > sizeN)
    return false;

  if (p.first < 0 || p.second < 0)
    return false;

  if (this->m(p.first, p.second) == INACCESSIBLE)
    return false;

  return true;
}

std::vector<coords> aStar::returnPath() {

    auto path = std::vector<coords>();
    auto current_point = path_start;

    // If there has been a run
    if (isValid(current_point.pos)) {
      // Insert into final path the destination
      path.insert(path.begin(), path_start.pos);
      debug << "  Inserting dest. in path : " << path_start;

      // While parentage of nodes exist
      while(isValid(current_point.parent)) {
        // Find parent
        auto p = find_item(closed, current_point.parent);

        debug << "  Found parent : " << *p;

        // Insert into final path
        path.insert(path.begin(), p->pos);

        // Look for next parent
        current_point = *p;
      }
    }

    return path;
}

#pragma endregion

#pragma region Collision Map

// Collision map
void aStar::setInaccessible(const point p) {
  aStar::setInaccessible(p.pos);
}

void aStar::setInaccessible(const coords& c) {
  this->m(c.first, c.second) = INACCESSIBLE;
  debug << "Set inaccessible location @" << c;
}

void aStar::setInaccessible(const int x, const int y) {
  coords c = coords(x, y);
  setInaccessible(c);
}

#pragma endregion

#pragma region Algorithm

// Algorithm

point aStar::getAdjacent(point p, int dir) {
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

std::vector<point> aStar::generateChildren(point p) {
    std::vector<point> children;

    // Check map and see if children are all valid - paths exist
    for (auto DIR : dirList) {
      auto a = getAdjacent(p, DIR);

      if (isValid(a.pos))
        children.push_back(a);
    }

    return children;
}

int aStar::runAlgorithm() {

    // Let initial lists be empty
    aStarList emptyO, emptyC;
    swap(open, emptyO);
    swap(open, emptyC);
    path.clear();

    // Put start node on open list - O(1)
    path_start = point(INEXISTENT, INEXISTENT);
    open.push_back(origin);
    debug << "Pushed origin : " << origin;

    // Loop until you find the end
    while(!open.empty()) {

      debug << "--------------------A*---------------------" << std::endl;

      // Sort the list - (N*logN)
      std::sort(open.begin(), open.end());
      debug << "Sorted list: " << std::endl;
      for (auto i : open)
        std::cout << i;

      // Remove from open - O(1)
      auto p = open.front();
      debug << "Removing best point : " << p;
      open.erase(open.begin()); // O(1) best case

      // Add to closed - O(1)
      closed.push_back(p);

      // If goal, return
      if (p == destination) {
        debug << "Arrived at destination." << std::endl;
        path_start = p;
        path = returnPath();
        return EXIT_SUCCESS;
      }

      // Generate children
      // For each child
      for (auto child : generateChildren(p)) {

        debug << "Generated child : " << child;

        // If in closed list, continue
        if (contains(closed, child))
          continue;

        // Update g and h - don't forget to look at the map for cost
        // Replace basic COST with COST*map(parent_height-child_height) unless
        //   child has height - 1 in which case you cannot use the child...
        //   but this will be handles in child legality step above
        //   calculateCost fnction required here.
        child.setScores(p.g + COST,
                        getHeuristic().distanceOp(child, getDestination()));

        debug << "Updated child H : " << child;
        // Child is already in open, and g higher that open, continue
        // Add child to open list
        open.push_back(child);
      }

      debug << "Open list: " << std::endl;
      for (auto i : open)
        std::cout << i;
    }

    return EXIT_FAILURE;
}

std::vector<coords> aStar::getPath() {
  return path;
}

void aStar::printMap() {
  aStar::printMap(WITHOUT_PATH);
}

void aStar::printMap(bool with_path) {
  char p;
  int val;
  coords c;

  for (int i = 0; i < this->sizeM; i++) {
    for (int j = 0; j < this->sizeN; j++) {
      val = this->m(i,j);
      c = coords(i,j);

      switch (val)
      {
      case INACCESSIBLE:
        p = '#';
        break;
      
      default:
        p = '.';
        break;
      }

      if (origin.pos == c)
        std::cout << "O";
      else if (destination.pos == c)
        std::cout << "X";
      else
      {
        if (!path.empty() && (with_path != WITHOUT_PATH))
        {
          auto pt = std::find(path.begin(), path.end(), c);
          if (pt != path.end())
            std::cout << "*";
          else
            std::cout << p;
        }
        else
          std::cout << p;
      }
    }
    std::cout << std::endl;
  }
}

#pragma endregion