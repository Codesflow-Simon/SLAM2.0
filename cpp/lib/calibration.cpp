#include <nlohmann/json.hpp>
#include <set>
#include <map>
// #include "sensor.h"
#include "cpp/lib/emulator.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // Point3 (x,y,z)
using Sensor = Emulator;

// Estimate prior noise to be about 20cm
noiseModel::Diagonal::shared_ptr anchor_noise_model = noiseModel::Isotropic::Sigma(3,0.2);
noiseModel::Diagonal::shared_ptr calibration_tag_noise_model = noiseModel::Isotropic::Sigma(3,0.1);
noiseModel::Diagonal::shared_ptr distance_noise_model = noiseModel::Isotropic::Sigma(1,0.1);


Eigen::Matrix<double, 5, 3> calibrate(Sensor sensor) {
  // Todo, save calibration
  json data = sensor.getJson();
  const int anchors = data["meas"]["d"].size();
  const string tagID = data["id"];
  const vector<string> anchorIDs = data["meas"]["a"];

  Eigen::Matrix<double, 5, 3> anchorMatrix;
  anchorMatrix << -0.6, 0.9,  2.1,  // Ground truth
                   1.2, 0.1,  0.3,  
                  -0.3, 0.3, -1.6,
                   0.7, 2.1, -0.4, 
                  -1.8, 1.5,  0.1;

  Values values;
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();

  map<string, Key> ID_KeyMap;

  for (int i=0; i<anchors; i++) {
    ID_KeyMap.insert({anchorIDs[i], (Key)i});
    values.insert((Key)i, (Point3)anchorMatrix.row(i));
    graph -> add(PriorFactor<Point3>((Key)i, (Point3)anchorMatrix.row(i), anchor_noise_model));
  }

  // Construct range map between anchors
  // TODO: ensure this works for real sensors
  map<string, map<string, double>> rangeMap;
  sensor.sendA2a();
  for (int i=0; i<anchors; i++) {
    json data = sensor.getJson();
    string firstKey = data["id"];
    map<string, double> innerMap;

    for (int j=0; j<anchors-1; j++) {
      string secondKey = data["meas"]["a"][j];
      double value = data["meas"]["d"][j];
      innerMap.insert({secondKey, value});
    }

    rangeMap.insert({firstKey, innerMap});
  }

  // Anchor pairwise calibration
  // TODO: ensure this works for real sensors
  for (auto const& id_a : anchorIDs) {
    for (auto const& id_b : anchorIDs) {
      auto id_pair = set<string>({id_a, id_b});
      if (id_a != id_b) {       // Pair has not been processed yet
        Key first_key = ID_KeyMap.at(id_a);
        Key second_key = ID_KeyMap.at(id_b);
        double distance = rangeMap.at(id_a).at(id_b);
        
        graph -> add(RangeFactor<Point3>(first_key, second_key, distance, distance_noise_model));
      }
    }
  }

  // Postitional calibration
  // TODO: ensure this works for real sensors
  Point3 calibration_points[8] = {
    Point3({0,0,0}),    
    Point3({1,0,0}),    
    Point3({1,0,1}),
    Point3({0,0,1}),
    Point3({0,1,1}),
    Point3({0,1,0}),
    Point3({1,1,0}),
    Point3({1,1,1})
  };

  // Outer loop: calibration point on unit cube
  for (int i=0; i<8; i++) {
    auto point = calibration_points[i];
    json data = sensor.getJson((Vector3)point);

    // Inner loop: anchors
    for (int j=0; j<anchors; j++) {
      string anchorID = data["meas"]["a"][j];
      double distance = data["meas"]["d"][j];
      Key anchorKey = ID_KeyMap.at(anchorID);
      values.insert(X(8*i+j), point);           // Key for combination of anchor and calibration point
      graph -> add(PriorFactor<Point3>(X(8*i+j), point, calibration_tag_noise_model));
      graph -> add(RangeFactor<Point3>(X(8*i+j), anchorKey, distance, distance_noise_model));
    }    
  }

  LevenbergMarquardtOptimizer optimizer(*graph, values);
  values = optimizer.optimize(); 

  Eigen::Matrix<double, 5, 3> outputMatrix;

  for (int i=0; i<anchors; i++) {
    outputMatrix.row(i) = values.at<Point3>((Key)i);
    cout << "anchor " << i << " at " << values.at<Point3>((Key)i);
  }
  cout << endl;

  return outputMatrix;
}
