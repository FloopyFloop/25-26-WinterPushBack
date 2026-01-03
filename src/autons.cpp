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
    pros::delay(1000);
    match_load.extend();
    pros::delay(500);
    match_load.retract();

    chassis.turnToHeading(133, 1000);

    chassis.moveToPoint(-29, -27, 1000,{.forwards = false, .maxSpeed = 80});
    pros::delay(500);

    medium_top();
    pros::delay(700);
    stop_intakes();
    

    chassis.moveToPoint(8, -59, 1000, {.maxSpeed = 90});
    chassis.waitUntilDone();
    chassis.turnToHeading(90,1000);
    intake();
    pros::delay(400);
    
    match_load.extend();

    
    
    chassis.moveToPoint(20, -57, 1000,{.maxSpeed = 80});
    
    chassis.moveToPoint(-15, -58, 1000,{.forwards = false});
    
    pros::delay(400);
    match_load.retract();
    pros::delay(300);
    long_goal();
}

void leftQUAL(){
    chassis.setPose(0,0,0);
    wing.extend();
    chassis.moveToPoint(-12, 21.5, 1000, {.maxSpeed = 70});
    intake();
    pros::delay(900);
    match_load.extend();
    chassis.moveToPoint(-32, 36.6, 3000,{.maxSpeed = 70});
    pros::delay(300);
    match_load.retract();
    pros::delay(700);
    chassis.moveToPoint(-8, 27, 1000,{.forwards = false});
    chassis.turnToHeading(-140, 1000);
    //chassis.waitUntilDone();
    chassis.moveToPoint(-1.5, 33.5, 1000,{.forwards = false, .maxSpeed = 80});
    pros::delay(500);

    medium_top();
    pros::delay(1000);
    chassis.moveToPoint(-35.75, 0, 2000,{.maxSpeed = 80});
    chassis.waitUntilDone();
    stop_intakes();

    chassis.turnToHeading(-180, 1000);
    pros::delay(200);
    match_load.extend();
    intake();
    chassis.moveToPoint(-37.5, -15, 1300,{.maxSpeed = 90});
    //pros::delay(200);
    chassis.moveToPoint(-35.5, 23, 1000,{.forwards= false, .maxSpeed = 70});
    match_load.retract();
    pros::delay(800);

    long_goal();

    pros::delay(800);

    chassis.moveToPoint(-35.5, 7, 1000);

    chassis.turnToHeading(-140, 1000);
    
    chassis.moveToPoint(-27.25, 15, 1000,{.forwards = false,.maxSpeed = 90});
    pros::delay(300);
    wing.retract();
    chassis.turnToHeading(-180, 1000);

    chassis.moveToPoint(-27.1, 35, 1000,{.forwards = false});


}  


void sevenWingRight(){
    chassis.setPose(0,0,0);
    wing.extend();
    chassis.moveToPoint(8, 20.5, 1000, {.maxSpeed = 90});
    intake();
    pros::delay(300);
    match_load.extend();

    pros::delay(1200);
    
    chassis.turnToPoint(35, 0, 1000);

    pros::delay(100);

    match_load.retract();

    chassis.moveToPoint(37, 0, 1000,{.maxSpeed = 90});
    chassis.turnToHeading(180,1000);

    pros::delay(200);
    match_load.extend();
    intake();
    chassis.moveToPoint(37.5, -15, 1000,{.maxSpeed = 80});

    chassis.moveToPoint(36.5, 20, 2000,{.forwards =false,.maxSpeed = 70});
    pros::delay(400);
    match_load.retract();
    pros::delay(400);

    long_goal();

    pros::delay(1100);

    chassis.moveToPoint(36, 7, 1000,{.maxSpeed = 80});

    chassis.turnToHeading(245, 1000);

    chassis.moveToPoint(45.5, 14, 1000,{.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(180, 1000);
    wing.retract();
    chassis.moveToPoint(45.4, 38, 1000,{.forwards = false});
    

}

void fourWingRight(){
    chassis.setPose(0,0,0);
    wing.extend();
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

    chassis.moveToPoint(-10, 30.5, 1000);

    chassis.turnToHeading(140, 1000);

    chassis.moveToPoint(-22.6, 36.5, 1000,{.forwards = false, .maxSpeed = 80});

    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-42, 37, 1000,{.forwards =false});
    wing.retract();
    

    
}

void skills(){
    chassis.setPose(0,0,0);

    
}

void getOffLine(){
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 5, 1000);
}