#include "robotpid.h"
#include "testutils.h"
#include "utils.h"

#include <gtest/gtest.h>

double middleMean(double left, double right) {
    return (left+right)/2 * PIDRatios::MID_MUL;
}

TEST(PIDTests, AdjustSpeedForMiddleSensors) {
    constexpr double startingSpeed = 50;

    double leftSensorValue = generate_random_number(0, 49), rightSensorValue = generate_random_number(51,100);

    double expectedHigerSpeed = startingSpeed + middleMean(leftSensorValue, rightSensorValue),
           expectedLowerSpeed = startingSpeed - middleMean(leftSensorValue, rightSensorValue);

    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,0,leftSensorValue,rightSensorValue,0,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedHigerSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedLowerSpeed);
    }
    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,0,rightSensorValue,leftSensorValue,0,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedLowerSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedHigerSpeed);
    }

}

TEST(PIDTests, AdjustSpeedForInterSensors) {
    constexpr double startingSpeed = 50;

    double sensorValue = generate_random_number(PIDRatios::DOWN_SENSOR_BUFFER, PIDRatios::UP_SENSOR_BUFFER);

    double expectedHigerSpeed = startingSpeed + sensorValue*PIDRatios::INTER_MUL,
           expectedLowerSpeed = startingSpeed - sensorValue*PIDRatios::INTER_MUL;

    bound_value(expectedHigerSpeed, 0.0, 100.0);
    bound_value(expectedLowerSpeed, 0.0, 100.0);

    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,sensorValue,0,0,0,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedLowerSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedHigerSpeed);
    }
    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,0,0,0,sensorValue,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedHigerSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedLowerSpeed);
    }

}

TEST(PIDTests, AdjustSpeedForOuterSensors) {
    constexpr double startingSpeed = 50;

    double sensorValue = generate_random_number(PIDRatios::DOWN_SENSOR_BUFFER, PIDRatios::UP_SENSOR_BUFFER);

    double expectedHigerSpeed = startingSpeed + sensorValue*PIDRatios::OUTER_MUL,
           expectedLowerSpeed = startingSpeed - sensorValue*PIDRatios::OUTER_MUL;

    bound_value(expectedHigerSpeed, 0.0, 100.0);
    bound_value(expectedLowerSpeed, 0.0, 100.0);

    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({sensorValue,0,0,0,0,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedLowerSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedHigerSpeed);
    }
    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,0,0,0,0,sensorValue});
        EXPECT_EQ(result.leftEngineSpeed, expectedHigerSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedLowerSpeed);
    }

}

TEST(PIDTests, NoSpeedChange) {
    constexpr double startingSpeed = 50;
    constexpr RobotEngineSpeed expectedData = {startingSpeed, startingSpeed};

    double sensorValue = generate_random_number(PIDRatios::DOWN_SENSOR_BUFFER, PIDRatios::UP_SENSOR_BUFFER);

    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,0,0,0,0,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedData.leftEngineSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedData.rightEngineSpeed);
    }
    
    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({100,100,100,100,100,100});
        EXPECT_EQ(result.leftEngineSpeed, expectedData.leftEngineSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedData.rightEngineSpeed);
    }
    
    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,0,100,100,0,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedData.leftEngineSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedData.rightEngineSpeed);
    }

    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,0,sensorValue,sensorValue,0,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedData.leftEngineSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedData.rightEngineSpeed);
    }

    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,sensorValue,0,0,sensorValue,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedData.leftEngineSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedData.rightEngineSpeed);
    }

    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({sensorValue,0,0,0,0,sensorValue});
        EXPECT_EQ(result.leftEngineSpeed, expectedData.leftEngineSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedData.rightEngineSpeed);
    }
}

TEST(PIDTests, AdjustSpeedForMiddleSensorsOmnitOtherData) {
    constexpr double startingSpeed = 50;

    double leftSensorValue = generate_random_number(0, 49), rightSensorValue = generate_random_number(51,100);

    double expectedHigerSpeed = startingSpeed + middleMean(leftSensorValue, rightSensorValue),
           expectedLowerSpeed = startingSpeed - middleMean(leftSensorValue, rightSensorValue);

    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,leftSensorValue,leftSensorValue,rightSensorValue,0,rightSensorValue});
        EXPECT_EQ(result.leftEngineSpeed, expectedHigerSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedLowerSpeed);
    }
    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({rightSensorValue,0,rightSensorValue,leftSensorValue,leftSensorValue,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedLowerSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedHigerSpeed);
    }

}

TEST(PIDTests, AdjustSpeedForInterSensorsWhenOutroInfo) {
    constexpr double startingSpeed = 50;

    double sensorValue = generate_random_number(PIDRatios::DOWN_SENSOR_BUFFER, PIDRatios::UP_SENSOR_BUFFER);

    double expectedHigerSpeed = startingSpeed + sensorValue*PIDRatios::INTER_MUL,
           expectedLowerSpeed = startingSpeed - sensorValue*PIDRatios::INTER_MUL;

    bound_value(expectedHigerSpeed, 0.0, 100.0);
    bound_value(expectedLowerSpeed, 0.0, 100.0);

    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({sensorValue,sensorValue,0,0,0,0});
        EXPECT_EQ(result.leftEngineSpeed, expectedLowerSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedHigerSpeed);
    }
    {
        RobotPID pid(startingSpeed, 100, 0);
        auto result = pid.calculatePID({0,0,0,0,sensorValue,sensorValue});
        EXPECT_EQ(result.leftEngineSpeed, expectedHigerSpeed);
        EXPECT_EQ(result.rightEngineSpeed, expectedLowerSpeed);
    }

}