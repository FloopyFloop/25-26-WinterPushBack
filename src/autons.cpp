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
    chassis.cancelAllMotions();   

    chassis.moveToPoint(30, 40, 1000);
    move_until_distance(20);
    chassis.turnToHeading(90, 1000);
}

