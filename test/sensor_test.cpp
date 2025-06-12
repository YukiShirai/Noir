#include "sensor.h"

#include <gtest/gtest.h>

#include <memory>

using namespace noir;

class SensorInterfaceTest : public ::testing::Test {
   protected:
    std::vector<std::shared_ptr<Sensor>> sensors;
    void SetUp() override {
        sensors.push_back(std::make_shared<ForceSensor>("ft"));
        sensors.push_back(std::make_shared<VisionSensor>("vision"));
    }
};

TEST_F(SensorInterfaceTest, ReadAndGetMeasurement) {
    for (const auto& sensor : sensors) {
        sensor->read();
        SensorData data = sensor->getMeasurement();

        if (std::holds_alternative<Eigen::Vector3d>(data)) {
            Eigen::Vector3d f = std::get<Eigen::Vector3d>(data);
            EXPECT_EQ(f.size(), 3);
        } else if (std::holds_alternative<int>(data)) {
            int frame = std::get<int>(data);
            EXPECT_GE(frame, 0);
        } else {
            FAIL() << "Unsupported sensor type in variant";
        }
    }
}
