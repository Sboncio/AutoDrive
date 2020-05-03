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


float quaternion_to_degrees(double quat0, double quat1, double quat2, double quat3){

    double Orientation = atan2(2 * (quat0 * quat3 + quat1 * quat2), 1 - 2 * ((quat2 * quat2) + (quat3 * quat3)));

    Orientation = Orientation * (180/M_PI);

    return Orientation;
}

float rads_to_degrees(float rad){
    return rad * 180.f/M_PI;
}

TEST_CASE("Ensure that angles are normalised to acceptable range", "[single file]"){
    REQUIRE(normalise_angle(-350) == 350);
    REQUIRE(normalise_angle(10) == 350);
    REQUIRE(normalise_angle(180) == 180);
    REQUIRE(normalise_angle(-40) == 40);
    REQUIRE(normalise_angle(90) == 270);
    REQUIRE(normalise_angle(0.0001) == 360);
    REQUIRE(normalise_angle(-0.0001) == 0);
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
    REQUIRE(target_angle(0, -3) == -90);
    REQUIRE(target_angle(2,-2) == -45);
}

TEST_CASE("Correctly convert quaternion values to degrees", "[single file]"){
    REQUIRE(quaternion_to_degrees(0,0,1,0) == 180);
}

TEST_CASE("Correctly convert radians to degrees", "[single file]"){
    REQUIRE(rads_to_degrees(1) == 57.29578f);
    REQUIRE(rads_to_degrees(3) == 171.88734f);
    REQUIRE(rads_to_degrees(-2) == -114.59156f);
    REQUIRE(rads_to_degrees(5) == 286.47891f);
}