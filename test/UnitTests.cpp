#define CATCH_CONFIG_MAIN
#include "catch.hpp"

double normalise_angle(double angle){
    if(angle > 0){
        angle = 360 - (abs(angle));
    } 

    if(angle < 0){
        angle = abs(angle);
    }

    return angle;
}

float angle_difference(float TargetAngle, float Current_Theta){
    float difference = (TargetAngle - Current_Theta + 540);
    difference = fmod(difference,360.f);
    difference -= 180;

    return fabs(difference);
}

double target_angle(double xDiff, double yDiff){
    return atan2(yDiff,xDiff) * 180/M_PI;
}

TEST_CASE("Ensure that angles are normalised to acceptable range", "[single file]"){
    REQUIRE(normalise_angle(-350) == 350);
    REQUIRE(normalise_angle(10) == 350);
}

TEST_CASE("Correctly evaluate difference between angles", "[single file]"){
    REQUIRE(angle_difference(20.f,10.f) == 10);
    REQUIRE(angle_difference(90.f,30.f) == 60);
    REQUIRE(angle_difference(359.f,1.f) == 2);
    REQUIRE(angle_difference(1.f,359.f) == 2);
    REQUIRE(angle_difference(270.f, 90.f) == 180);
}

TEST_CASE("Assess angle to destination in degrees", "[single file]"){
    REQUIRE(target_angle(3, 3) == 45);
    REQUIRE(target_angle(0,3) == 90);
}