#include <helpers.h>


class LeftTofSensor {
    public:
        LeftTofSensor() {
            TOF tof_sensor;
            tof_sensor_ = tof_sensor;
        }

        void addValue() {
            // get left sensor reading 
            double sensor_value = tof_sensor_.getDistance(); // subject to change cus azum
            if(sensor_value == -1) return;
    
            // add sensor reading to queue
            while (buffer_.size() >= 10) {
                buffer_.pop();
            }
            buffer_.push(sensor_value);
        }

        int getValue() {
            return buffer_.back();
        }

        void clearValues() {
            std::queue<int> empty;
            std::swap(buffer_, empty);
        }

        bool shouldAdjustRight() {
            if (buffer_.size() < 4) {
                return false;
            }
            return (buffer_.front() - buffer_.back() > TOLERANCE);
        }

        bool shouldAdjustLeft() {
            if(buffer_.size() < 4) {
                return false;
            }
            return (buffer_.back() - buffer_.front() > TOLERANCE); 
        }
}