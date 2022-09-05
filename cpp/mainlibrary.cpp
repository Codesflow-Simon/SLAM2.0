#include <iostream>
#include <nlohmann/json.hpp>
#include <vector>
#include <map>
#include <list>
#include <math.h>
#include <unistd.h>

// #include "cpp/lib/sensor.h"
#include "cpp/lib/util.h"
// #include "cpp/lib/factors.h"
#include "cpp/lib/emulator.h"
#include "cpp/lib/calibration.h"
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace gtsam;

using json = nlohmann::json;
using Sensor = Emulator;

namespace factorgraph {

ofstream debugLog;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

// Noise models
noiseModel::Diagonal::shared_ptr prior_pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.02, 0.02, 0.02, 0.2, 0.2, 0.2).finished()); // rad,rad,rad,m, m, m
noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.2); 
noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 0.1);
noiseModel::Diagonal::shared_ptr mag_noise_model = noiseModel::Isotropic::Sigma(3, 0.2);
noiseModel::Diagonal::shared_ptr distance_noise_model = noiseModel::Isotropic::Sigma(1, 0.1);
noiseModel::Diagonal::shared_ptr anchor_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);

void logMeasurements(json data) {
  debugLog << "acc = " << data["acc"] << endl;
  debugLog << "omega = " << data["gyro"] << endl;
  debugLog << "range = " << data["meas"]["d"] << endl;
  debugLog << "mag = " << data["mag"] << endl;
}

Emulator getEmulator() {
  Emulator emulator = Emulator();
  emulator.setMeasurementError(0.1);

  emulator.setAnchor(Anchor(Vector3(-0.6, 0.9,  2.1), "2e4f"));
  emulator.setAnchor(Anchor(Vector3( 1.2, 0.1,  0.3), "3bfc"));
  emulator.setAnchor(Anchor(Vector3(-0.3, 0.3, -1.6), "3a85"));
  emulator.setAnchor(Anchor(Vector3( 0.7, 2.1, -0.4), "3dd2"));
  emulator.setAnchor(Anchor(Vector3(-1.8, 1.5,  0.1), "2e01"));
  return emulator;
}

boost::shared_ptr<PreintegrationParams> getPreintParams() {
  auto params = PreintegrationParams::MakeSharedU(9.81);
  params->setAccelerometerCovariance(I_3x3 * 0.1);
  params->setGyroscopeCovariance(I_3x3 * 0.1);
  params->setIntegrationCovariance(I_3x3 * 0.1);
  params->setUse2ndOrderCoriolis(false);
  params->setOmegaCoriolis(Vector3(0, 0, 0));
  return params;
}

int main() {  
  debugLog.open("log", ofstream::out | ofstream::trunc);
  debugLog << "Begin" << endl;

  Sensor sensor = getEmulator();
  auto anchorPos = calibrate(sensor);
  sensor.clearA2a();

  json data = sensor.getJson();
  logMeasurements(data);
  const int anchors = data["meas"]["d"].size();
  const string tagID = data["id"];
  const vector<string> anchorIDs = data["meas"]["a"];

  auto params = getPreintParams();
  
  debugLog << "Initialising structures" << endl;
  ISAM2 isam;
  ISAM2Result result;
  Pose3 initialPose = Pose3();

  gtsam::NonlinearFactorGraph graph;
  graph.addPrior(X(0), initialPose, prior_pose_noise);

  Values values;
  values.insert(X(0), initialPose);

  // Initalise anchors and create map
  map<string, Key> ID_KeyMap;
  for (int i=0; i<anchors; i++) {
    ID_KeyMap.insert({anchorIDs[i], (Key)i});
    values.insert((Key)i, (Point3)anchorPos.row(i));
    graph.addPrior((Key)i, (Point3)anchorPos.row(i), anchor_noise_model);
  }

  // IMU priors
  graph.addPrior(B(0), imuBias::ConstantBias(), bias_noise_model);
  values.insert(B(0), imuBias::ConstantBias());
  
  graph.addPrior(V(0), Vector3(0,0,0), velocity_noise_model);
  values.insert(V(0), Vector3(0,0,0));

  int k = 0; //Step counter
  int minK = 20;
  double prevTime = -1;
  bool initalised = false;
  
  NavState prevState = NavState();
  NavState propState = NavState();

  imuBias::ConstantBias prevBias = imuBias::ConstantBias();
  imuBias::ConstantBias propBias = imuBias::ConstantBias();

  PreintegratedImuMeasurements accum(params);

  while (k < 100) {
    k++;
    debugLog << "on loop: " << k << endl;

    json data = sensor.getJson();
    logMeasurements(data);

    // First: odometry
    // Bias terms
    graph.add(BetweenFactor<imuBias::ConstantBias>(B(k-1), B(k), imuBias::ConstantBias(), bias_noise_model));
    values.insert(B(k), imuBias::ConstantBias());

    if (k > 1) {
      Vector3 measuredAcc;
      Vector3 measuredOmega;

      for (int i=0; i<3; i++){
        measuredAcc(i,0) = data["acc"].at(i);
        measuredOmega(i,0) = data["gyro"].at(i);
        data["acc"].get<vector<double>>();
      }

      accum.integrateMeasurement((Vector3)measuredAcc, (Vector3)measuredOmega,0.05); // Emulator is unsing constant dt

      graph.add(ImuFactor(X(k - 1), V(k - 1), X(k), V(k), B(k), accum));
    }

    // Second: projections
    propState = accum.predict(prevState, prevBias);
    values.insert(X(k), propState.pose());
    values.insert(V(k), propState.v());

    // Third: range factors
    const string mainID = data["id"];

    for (int i=0; i<data["meas"]["d"].size(); i++) {

      string secondID = data["meas"]["a"][i];
      Key anchorKey = ID_KeyMap.at(secondID);
      double measurement = data["meas"]["d"].at(i);
      
      if (mainID == tagID) {
        graph.add(RangeFactor<Pose3, Point3>(X(k), anchorKey, measurement, distance_noise_model));
      }
    }

    if (k > minK) {
      debugLog << "optimising" << endl;
      if (!initalised) {
        debugLog << "batch optimisation" << endl;
        gtsam::LevenbergMarquardtOptimizer batchOptimizer(graph, values);
        values = batchOptimizer.optimize();
        initalised = true;
      }
      debugLog << "isam update" << endl;
      result = isam.update(graph, values);
      graph = NonlinearFactorGraph(); // Reset structures once passed to isam
      values = Values();
    }

    prevTime = (double)data["ts"];
    accum.resetIntegration();
  }

  gtsam::Values finalResult = isam.calculateEstimate();
  debugLog << finalResult.at<Pose3>(X(k)) << endl;

  return 0; 
}

} // End namespace