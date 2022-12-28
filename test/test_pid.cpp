#include "robotpid.h"
#include "testutils.h"
#include "utils.h"

#include <gtest/gtest.h>

TEST(PIDTests, NoSpeedChange) {
    constexpr double startingSpeed = 50;
    constexpr RobotEngineSpeed expectedData = {startingSpeed, startingSpeed};

    double sensorValue = generate_random_number(PIDRatios::DOWN_SENSOR_BUFFER+1, PIDRatios::UP_SENSOR_BUFFER-1);

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
