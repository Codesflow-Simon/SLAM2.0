#include <Eigen/Dense>
#include <math.h>
#include <map>
#include <vector>
#include <fstream>      // std::ofstream

using namespace Eigen;
using namespace std;

typedef Eigen::Vector3d Vector3;

double norm(Vector3 vec);

double distanceBetween(Vector3 a, Vector3 b);

vector<double> removeZeros(vector<double> input);