#include <iostream>
#include <nlohmann/json.hpp>
#include "cpp/lib/sensor.h"

using namespace std;

using json = nlohmann::json;

Sensor sensor;

int main() {
    // Prints out mass sensor infomation
    while (true) {
        json json = sensor.getJson();
        sensor.sendA2a();
        if (json == nlohmann::json::parse("{}")) {
            continue;
        } else {
           cout << json << endl;
        }
    }
    return 0;
}