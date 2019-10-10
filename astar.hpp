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

#include <vector>
#include <utility>
#include <cmath>

#define MANHATTAN_DISTANCE  1
#define EUCLIDEAN_DISTANCE  2

using namespace std;

// Interface

typedef pair<int,int> point;

class Heuristic {
    private:
        int internalHeuristic;
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
        return abs(p1.first - p2.first) + abs(p1.second - p2.second);

    case EUCLIDEAN_DISTANCE:
        return sqrt(pow(p1.first - p2.first, 2) + pow(p1.second - p2.second, 2));

    default:
        break;
    }
}

// Interface

class aStar {
    private:
        point origin;
        point destination;

    public:
        aStar();
        aStar(const point origin, const point destination);

        void setOrigin(const point origin);
        auto const getOrigin() { return origin; }

        void setDestination(const point destination);
        auto const getDestination() { return destination; }

        void setHeuristic();
        int runAlgorithm();

        vector<point> returnPath();
};

// Implementation

aStar::aStar() {
    origin = make_pair(0, 0);
    destination = make_pair(0, 0);
}

aStar::aStar(const point origin, const point destination) {
    aStar::origin = origin;
    aStar::destination = destination;
}

void aStar::setHeuristic() {};

vector<point> aStar::returnPath() {
    auto path = new vector<point>();
    return *path;
}

int aStar::runAlgorithm() {
    return EXIT_SUCCESS;
}
