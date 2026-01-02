#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "main.h"              // IWYU pragma: keep
#include "lemlib/api.hpp"
#include <cstdio>
//#include <string>
//#include <vector>
//#include <functional>

extern lemlib::Chassis chassis;

ASSET(SAWP_1);
ASSET(SAWP_2);
ASSET(SAWP_3);


void red_score_sort(void* param){ //YOU ARE RED ALLIANCE
    long_goal();
    while(true){
    if(optical.get_hue() > 190 && optical.get_hue() < 220){
        outtake();
    }
    
    else {
        stop_intakes();
        break;
    }
    pros::delay(20);
    }   

    
}



void blue_score_sort(){ // YOU ARE BLUE ALLIANCE 
    long_goal();
    while(true){
    if(optical.get_hue() > 5 && optical.get_hue() < 22){
        outtake();
    }
    }
}

void move_until_distance(double travel_until_distance){
    while(true){
        double current_distance = fwrd_distance.get(); 
        if (current_distance < travel_until_distance){
            chassis.cancelAllMotions();
            chassis.waitUntilDone();
            pros::delay(300);
            chassis.setPose(0,0,0);

            break;
        }
    pros::delay(10); // cool down
    }
}

void test(){
    
}   

void sigSAWP(){
    chassis.setPose(0,0,0);

    chassis.moveToPoint(0, 29.7, 1000);
    intake();
    chassis.turnToHeading(90,1000);
    pros::delay(400);
    
    match_load.extend();
    
    chassis.moveToPoint(11, 30, 1000, {.maxSpeed = 60});
    
    pros::delay(400);

    chassis.moveToPoint(-25.5, 30.5, 2000,{.forwards = false,.maxSpeed = 80});
    pros::delay(1000);
    long_goal();
    match_load.retract();
    pros::delay(500);
    chassis.turnToHeading(190.5, 1000);
    stop_intakes();
    intake();
    
    chassis.moveToPoint(-21, 8, 2000,{.maxSpeed = 90});

    pros::delay(300);

    match_load.extend();
    pros::delay(500);
    match_load.retract();
    //chassis.turnToHeading(-180, 1000);
    chassis.moveToPoint(-19, -35, 2000,{.maxSpeed = 80});
    pros::delay(800);
    //match_load.extend();
    chassis.waitUntilDone();

    chassis.turnToHeading(133, 1000);

    chassis.moveToPoint(-30, -24, 1000,{.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();

    medium_top();
    pros::delay(700);

    intake();


}