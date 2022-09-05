#pragma once
#include <nlohmann/json.hpp>
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

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using Sensor = Emulator;

// TODO: generalise this return type
Eigen::Matrix<double, 5, 3> calibrate(Sensor sensor);