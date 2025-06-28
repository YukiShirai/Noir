#ifndef SENSOR_H
#define SENSOR_H
#include <Eigen/Dense>
#include <string>
#include <variant>

namespace noir {

// define potential different sensor reading type here:
using SensorData = std::variant<Eigen::Vector3d, int>;

class Sensor {
   protected:
    std::string m_name;

   public:
    explicit Sensor(std::string_view name) : m_name(name) {}
    virtual ~Sensor() = default;

    virtual void read() const = 0;

    const std::string& getName() const { return m_name; }

    virtual SensorData getMeasurement() const = 0;
};

class ForceSensor : public Sensor {
   private:
    Eigen::Vector3d m_force{0.0, 0.0, 0.0};

   public:
    explicit ForceSensor(std::string_view name) : Sensor(name) {}

    void read() const override;

    SensorData getMeasurement() const override { return m_force; }
};

class VisionSensor : public Sensor {
   private:
    int m_frame{};

   public:
    explicit VisionSensor(std::string_view name) : Sensor(name) {}

    void read() const override;

    SensorData getMeasurement() const override { return m_frame; }
};

}  // namespace noir

#endif  // SENSOR_H
