#include <Eigen/Dense>
#include <math.h>
#include <map>
#include <vector>
#include <fstream>      // std::ofstream
#include <random>

using namespace Eigen;
using namespace std;

typedef Eigen::Vector3d Vector3;

Vector3 isoRandomVector(double sigma) {
  default_random_engine generator;
  normal_distribution<double> dist;
  dist = normal_distribution<double>(0, sigma);
  return Vector3(dist(generator), dist(generator), dist(generator));
}

/* A simple function to calculate the norm of a vector 3 */
double norm(Vector3 vec) {
  double sq_x = vec.x() * vec.x();
  double sq_y = vec.y() * vec.y();
  double sq_z = vec.z() * vec.z();
  return sqrt(sq_x + sq_y + sq_z);
}

double distanceBetween(Vector3 a, Vector3 b) {
  Vector3 displacement = a - b;
  return norm(displacement);
}

vector<double> removeZeros(vector<double> input) {
  auto output = vector<double>();
  vector<double>::iterator it = output.begin();

  for (auto const& in: input) {
    if (in != 0) {
      output.insert(it, in);
      it = output.end();
    }
  }
  return output;
} 
