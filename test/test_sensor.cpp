#include "utils.h"
#include "testutils.h"
#include "analogmockinput.h"
#include "sensor.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::AtLeast;
using ::testing::InSequence;
using ::testing::Return;

TEST(sensor_tests, sensor_inits_analog_device) {
    auto mock_analog_input = std::unique_ptr<AnalogMockInput>(new AnalogMockInput());
    EXPECT_CALL(*mock_analog_input, init())                  
      .Times(AtLeast(1));

    Sensor sensor(std::move(mock_analog_input));
    sensor.init();
}

TEST(sensor_tests, sensor_gathers_data_from_analog) {
    constexpr int sensor_readings_num = 10;
    distribution value_distribution(MIN_SENSOR_VALUE,MAX_SENSOR_VALUE);
    std::vector<int> sensor_values;

    for(int i = 0; i < sensor_readings_num; i++) {
        sensor_values.push_back(value_distribution(rng));
    }

    auto mock_analog_input = std::unique_ptr<AnalogMockInput>(new AnalogMockInput());

    InSequence s;
    for (int i = 0; i <= sensor_readings_num; i++) {
        EXPECT_CALL(*mock_analog_input, get_value())
            .WillOnce(Return(sensor_values[i]))
            .RetiresOnSaturation();
    }
      
    Sensor sensor(std::move(mock_analog_input));

    for (int i = 0; i <= sensor_readings_num; i++) {
        sensor.measure_brightness();
        double expected_max_brightness = *std::max_element(sensor_values.begin(), sensor_values.begin() + i+1);
        EXPECT_DOUBLE_EQ(sensor.get_denoised_value(), expected_max_brightness) << "Problem with iteration: " << i;
    }
}