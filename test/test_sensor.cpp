#include "analogmockinput.h"
#include "sensor.h"
#include "testutils.h"
#include "utils.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using ::testing::AtLeast;
using ::testing::InSequence;
using ::testing::Return;

TEST(sensorTests, sensorInitsAnalogDevice)
{
  auto mockAnalogInput = std::make_unique<AnalogMockInput>();
  EXPECT_CALL(*mockAnalogInput, init()).Times(AtLeast(1));

  Sensor sensor(std::move(mockAnalogInput));
  sensor.init();
}

TEST(sensorTests, sensorGathersDataFromAnalogAndDenoisesIt)
{
  randomDistribution valueDistribution(SensorUtils::MIN_SENSOR_VALUE,
                                       SensorUtils::MAX_SENSOR_VALUE);
  std::vector<int> sensorValues;

  for (int i = 0; i < SensorUtils::WINDOW_SIZE; i++)
  {
    sensorValues.push_back(valueDistribution(randomGenerator));
  }

  auto mockAnalogInput = std::make_unique<AnalogMockInput>();

  InSequence s;
  for (int i = 0; i < SensorUtils::WINDOW_SIZE; i++)
  {
    EXPECT_CALL(*mockAnalogInput, getValue())
        .WillOnce(Return(sensorValues[i]))
        .RetiresOnSaturation();
  }

  Sensor sensor(std::move(mockAnalogInput));

  for (int i = 0; i < SensorUtils::WINDOW_SIZE; i++)
  {
    sensor.measureBrightness();
    double expected_max_brightness =
        *std::max_element(sensorValues.begin(), sensorValues.begin() + i + 1);
    EXPECT_DOUBLE_EQ(sensor.getDenoisedValue(), expected_max_brightness)
        << "Problem with iteration: " << i;
  }
}

TEST(sensorTests, sensorKeepsOnlyReadingsFromWindow)
{
  auto mockAnalogInput = std::make_unique<AnalogMockInput>();

  InSequence s;
  EXPECT_CALL(*mockAnalogInput, getValue())
      .WillOnce(Return(SensorUtils::MAX_SENSOR_VALUE));

  EXPECT_CALL(*mockAnalogInput, getValue())
      .Times(SensorUtils::WINDOW_SIZE)
      .WillRepeatedly(Return(SensorUtils::MIN_SENSOR_VALUE));

  EXPECT_CALL(*mockAnalogInput, getValue())
      .WillOnce(Return(SensorUtils::MAX_SENSOR_VALUE - 1));

  Sensor sensor(std::move(mockAnalogInput));

  for (int i = 0; i <= SensorUtils::WINDOW_SIZE + 1; i++)
  {
    sensor.measureBrightness();
  }
  EXPECT_EQ(sensor.getDenoisedValue(), SensorUtils::MAX_SENSOR_VALUE - 1);
}

TEST(sensorTests, sensorIsCalibratingWhite)
{
  constexpr double maxSensorReading = SensorUtils::MAX_SENSOR_VALUE,
                   midSensorReading = SensorUtils::MAX_SENSOR_VALUE * 0.9,
                   smallSensorReading = SensorUtils::MAX_SENSOR_VALUE * 0.8,
                   maxBrightness = 0,
                   midBrightness = 100.0 - (smallSensorReading - SensorUtils::MIN_SENSOR_VALUE) / (midSensorReading - SensorUtils::MIN_SENSOR_VALUE) * 100.0;
  auto mockAnalogInput = std::make_unique<AnalogMockInput>();
  InSequence s;
  EXPECT_CALL(*mockAnalogInput, getValue())
      .WillOnce(Return(maxSensorReading));

  EXPECT_CALL(*mockAnalogInput, getValue())
      .Times(SensorUtils::WINDOW_SIZE)
      .WillRepeatedly(Return(midSensorReading));

  EXPECT_CALL(*mockAnalogInput, getValue())
      .Times(SensorUtils::WINDOW_SIZE)
      .WillRepeatedly(Return(smallSensorReading));

  EXPECT_CALL(*mockAnalogInput, getValue())
      .WillOnce(Return(midSensorReading))
      .WillOnce(Return(maxSensorReading));

  Sensor sensor(std::move(mockAnalogInput));

  sensor.setCalibrationState(SensorUtils::CalibrationState::WHITE);

  sensor.measureBrightness(); // read max value

  EXPECT_DOUBLE_EQ(sensor.getBrightnessPercentage(), maxBrightness);

  for (int i = 0; i < SensorUtils::WINDOW_SIZE; i++) // move max value out of window
  {
    sensor.measureBrightness(); // read mid value
  }

  EXPECT_DOUBLE_EQ(sensor.getBrightnessPercentage(), maxBrightness);

  sensor.setCalibrationState(SensorUtils::CalibrationState::NONE);

  for (int i = 0; i < SensorUtils::WINDOW_SIZE; i++) // move mid value out of window
  {
    sensor.measureBrightness(); // read small value
  }

  EXPECT_DOUBLE_EQ(sensor.getBrightnessPercentage(), midBrightness);

  sensor.measureBrightness(); // read mid value

  EXPECT_DOUBLE_EQ(sensor.getBrightnessPercentage(), maxBrightness);

  sensor.measureBrightness(); // read max value

  EXPECT_DOUBLE_EQ(sensor.getBrightnessPercentage(), maxBrightness);
}

TEST(sensorTests, sensorIsCalibratingBlack)
{
  constexpr double maxSensorReading = 500,
                   midSensorReading = 200,
                   smallSensorReading = 100,
                   minBrightness = 100,
                   midBrightness = 100.0 - (maxSensorReading - midSensorReading) / (SensorUtils::MAX_SENSOR_VALUE - midSensorReading) * 100.0;
  auto mockAnalogInput = std::make_unique<AnalogMockInput>();
  InSequence s;
  EXPECT_CALL(*mockAnalogInput, getValue())
      .WillOnce(Return(smallSensorReading))
      .WillOnce(Return(midSensorReading))
      .WillOnce(Return(maxSensorReading));

  EXPECT_CALL(*mockAnalogInput, getValue())
      .Times(SensorUtils::WINDOW_SIZE)
      .WillRepeatedly(Return(midSensorReading));

  EXPECT_CALL(*mockAnalogInput, getValue())
      .Times(SensorUtils::WINDOW_SIZE)
      .WillRepeatedly(Return(smallSensorReading));

  Sensor sensor(std::move(mockAnalogInput));

  sensor.setCalibrationState(SensorUtils::CalibrationState::BLACK);

  sensor.measureBrightness(); // read small value

  EXPECT_DOUBLE_EQ(sensor.getBrightnessPercentage(), minBrightness);

  sensor.measureBrightness(); // read mid value

  EXPECT_DOUBLE_EQ(sensor.getBrightnessPercentage(), minBrightness);

  sensor.setCalibrationState(SensorUtils::CalibrationState::NONE);

  sensor.measureBrightness(); // read max value

  EXPECT_DOUBLE_EQ(sensor.getBrightnessPercentage(), midBrightness);

  for (int i = 0; i < SensorUtils::WINDOW_SIZE; i++) // move max value out of window
  {
    sensor.measureBrightness(); // read mid value
  }

  EXPECT_DOUBLE_EQ(sensor.getBrightnessPercentage(), minBrightness);

  for (int i = 0; i < SensorUtils::WINDOW_SIZE; i++) // move mid value out of window
  {
    sensor.measureBrightness(); // read small value
  }

  EXPECT_DOUBLE_EQ(sensor.getBrightnessPercentage(), minBrightness);
}