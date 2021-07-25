#include <array>
#include <queue>
#include <vector>
#include <utility>
#include <memory>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "Eigen/Dense"
#include <limits>

#undef DEBUG_FLAG
// #define DEBUG_FLAG

#ifdef DEBUG_FLAG
#define debug std::cout
#else
#define debug 0 && std::cout
#endif

#define MANHATTAN_DISTANCE  1
#define EUCLIDEAN_DISTANCE  2
#define OCTOGONAL_DISTANCE  3

#define INITIAL_SIZE      100

#define COST                1
#define INEXISTENT         -1
#define INACCESSIBLE       -1

#define WITH_PATH           1
#define WITHOUT_PATH        0

#define IDENTITY            1
#define SQUARED             2
#define MINIMUM             3

#define MIN_WEIGHT          1.0f
#define EIGTH_WEIGHT        30.63f
#define QUARTER_WEIGHT      61.25f
#define HALF_WEIGHT         122.5f
#define MAX_WEIGHT          255.0f
