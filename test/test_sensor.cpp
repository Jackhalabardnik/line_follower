#include "analogmockinput.h"
#include "sensor.h"
#include "testutils.h"
#include "utils.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::AtLeast;
using ::testing::InSequence;
using ::testing::Return;

TEST(sensorTests, sensorInitsAnalogDevice) {
  auto mockAnalogInput =
      std::unique_ptr<AnalogMockInput>(new AnalogMockInput());
  EXPECT_CALL(*mockAnalogInput, init()).Times(AtLeast(1));

  Sensor sensor(std::move(mockAnalogInput));
  sensor.init();
}

TEST(sensorTests, sensorGathersDataFromAnalogAndDenoisesIt) {
  randomDistribution valueDistribution(SensorUtils::MIN_SENSOR_VALUE,
                                  SensorUtils::MAX_SENSOR_VALUE);
  std::vector<int> sensorValues;

  for (int i = 0; i < SensorUtils::WINDOW_SIZE; i++) {
    sensorValues.push_back(valueDistribution(randomGenerator));
  }

  auto mockAnalogInput =
      std::unique_ptr<AnalogMockInput>(new AnalogMockInput());

  InSequence s;
  for (int i = 0; i <= SensorUtils::WINDOW_SIZE; i++) {
    EXPECT_CALL(*mockAnalogInput, getValue())
        .WillOnce(Return(sensorValues[i]))
        .RetiresOnSaturation();
  }

  Sensor sensor(std::move(mockAnalogInput));

  for (int i = 0; i <= SensorUtils::WINDOW_SIZE; i++) {
    sensor.measureBrightness();
    double expected_max_brightness =
        *std::max_element(sensorValues.begin(), sensorValues.begin() + i + 1);
    EXPECT_DOUBLE_EQ(sensor.getDenoisedValue(), expected_max_brightness)
        << "Problem with iteration: " << i;
  }
}

TEST(sensorTests, sensorKeepsOnlyReadingsFromWindow) {
  auto mockAnalogInput =
      std::unique_ptr<AnalogMockInput>(new AnalogMockInput());

  InSequence s;
  EXPECT_CALL(*mockAnalogInput, getValue())
      .WillOnce(Return(SensorUtils::MAX_SENSOR_VALUE))
      .RetiresOnSaturation();

  for (int i = 0; i <= SensorUtils::WINDOW_SIZE - 1; i++) {
    EXPECT_CALL(*mockAnalogInput, getValue())
        .WillOnce(Return(SensorUtils::MIN_SENSOR_VALUE))
        .RetiresOnSaturation();
  }

  EXPECT_CALL(*mockAnalogInput, getValue())
      .WillOnce(Return(SensorUtils::MAX_SENSOR_VALUE - 1))
      .RetiresOnSaturation();

  Sensor sensor(std::move(mockAnalogInput));

  for (int i = 0; i <= SensorUtils::WINDOW_SIZE + 1; i++) {
    sensor.measureBrightness();
  }
  EXPECT_EQ(sensor.getDenoisedValue(), SensorUtils::MAX_SENSOR_VALUE - 1);
}