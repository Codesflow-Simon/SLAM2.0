#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <random>
#include <string>

#include <nlohmann/json.hpp>

#include "util.h"

using namespace std;
using namespace Eigen;

using json = nlohmann::json;

class Anchor {
    public:
        string ID;
        Vector3 location;

        Anchor(Vector3 location_, string ID_) {
            location = location_;
            ID = ID_;
        }

        bool equals(const Anchor& other) const {
            const string otherID = other.ID;
            return otherID == ID;
        }
};

ostream& operator<<(std::ostream &strm, const Anchor &a) {
    return strm << "Anchor(\"" << a.ID << "\", x=" << a.location.x() << ", y=" << a.location.y() << ", z=" << a.location.z() << ")";
}

class Emulator {
    private:
        vector<Anchor> anchors;
        vector<Anchor>::iterator it = anchors.begin();

        double error;
        int a2aSent = -1;
        double time = 0;
        const double dt = 0.05;

        string tagID = "0b05";
        default_random_engine generator;
        normal_distribution<double> dist;

        vector<double> sample(Vector3 tag) {
            int idx = 0;
            vector<double> output = vector<double>(anchors.size());
            for (auto const& anchor : anchors) {
                double noise = dist(generator);
                double distance = distanceBetween(anchor.location, tag);

                output.at(idx) = distance + noise;
                
                idx++;
            }
            return output;
        }

    vector<double> get_random_vector(Vector3 mean, double isotropic_sigma) {
        vector<double> out(3);
        normal_distribution<double> dist = normal_distribution<double>(0, isotropic_sigma);
        out.at(0) = mean.x() + dist(generator);
        out.at(1) = mean.y() + dist(generator);
        out.at(2) = mean.z() + dist(generator);
        return out;
    }

    public:
        void setAnchor(Anchor anchor) {
            anchors.insert(it, anchor);    
            it = anchors.end();
        }

        vector<Anchor> getAnchors(){
            return anchors;
        }

        void setMeasurementError(double sigma) {
            dist = normal_distribution<double>(0, sigma);
        }

        json getTagData(Vector3 tag) {
            json output = json();

            auto measurement = sample(tag);
            output["id"] = tagID;
            output["acc"] = get_random_vector(Vector3(0,0,9.81), 0.2);
            output["gyro"] = get_random_vector(Vector3(0,0,0), 0.2);
            output["mag"] = {0, 0, 21};
            output["ts"] = (double)time;

            json meas = json();
        
            auto ids = vector<string>();
            vector<string>::iterator it = ids.begin();
            for (auto const& anchor: anchors) {
                ids.insert(it, anchor.ID);
                it = ids.end();
            }

            meas["a"] = ids;
            meas["d"] = measurement;      
            output["meas"] = meas;

            return output;
        }
        json getJson() {
            return getJson(Vector3(0,0,0));
        }

        json getJson(Vector3 tag) {

            json output = json();
            if (a2aSent==-1) {
                return getTagData(tag);
            } else {
                Anchor subject = anchors.at(a2aSent);

                auto measurement = sample(subject.location);
                output["id"] = subject.ID;
                output["acc"] = {0,0,-9.81};
                output["gyro"] = {0.0, 0.0, 0.0};
                output["mag"] = {0, 0, 21};
                output["ts"] = time;             
    
                json meas = json();

                auto ids = vector<string>();
                vector<string>::iterator it = ids.begin();
                for (auto const& anchor: anchors) {
                    if (anchor.equals(subject))
                        {continue;}
                    ids.insert(it, anchor.ID);
                    it = ids.end();
                }

                meas["a"] = ids;
                meas["d"] = removeZeros(measurement);  
                output["meas"] = meas;
            }
            a2aSent--;
            time += dt;
            return output;
        }

        void sendA2a() {
            a2aSent = anchors.size()-1;
        }

        void clearA2a() {
            a2aSent = -1;
        }
};

Vector3 addAnchorNoise(Vector3 anchor, double sigma) {
    default_random_engine generator;
    auto dist = normal_distribution<double>(0, sigma);
    double x = dist(generator);
    double y = dist(generator);
    double z = dist(generator);
    auto noise = Vector3(x,y,z);
    return anchor + noise;
}
