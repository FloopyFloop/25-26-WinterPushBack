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
    pros::delay(900);
    long_goal();
    match_load.retract();
    pros::delay(400);

    chassis.turnToPoint(-21, -3, 1000);

    chassis.moveToPoint(-21, -3, 1000);

    chassis.moveToPoint(-19, -35, 2000,{.maxSpeed = 90});
    pros::delay(800);
    match_load.extend();

    /*
    chassis.turnToHeading(190.5, 1000);
    stop_intakes();
    intake();
    
    chassis.moveToPoint(-20, 5, 2000,{.maxSpeed = 60});

    pros::delay(550);

    match_load.extend();
    pros::delay(500);
    match_load.retract();
    //chassis.turnToHeading(-180, 1000);
    chassis.moveToPoint(-19, -35, 2000,{.maxSpeed = 90});
    */
    pros::delay(1000);
    match_load.extend();
    pros::delay(500);
    match_load.retract();

    chassis.turnToHeading(133, 1000);

    chassis.moveToPoint(-27, -28, 1000,{.forwards = false, .maxSpeed = 110});
    pros::delay(500);

    medium_top();
    pros::delay(700);
    stop_intakes();
    

    chassis.moveToPoint(8, -57, 1000, {.maxSpeed = 100});
    chassis.waitUntilDone();
    chassis.turnToHeading(90,1000);
    intake();
    pros::delay(400);
    
    match_load.extend();

    chassis.moveToPoint(25, -55, 1200,{.maxSpeed = 100});
    
    chassis.moveToPoint(-15, -56, 1000,{.forwards = false});
    
    pros::delay(400);
    match_load.retract();
    outtake();
    pros::delay(300);
    long_goal();

}

void regionalSAWP(){
    //chassis.setPose(-3.38,15.95,0);
    chassis.setPose(0,0,0);

    chassis.moveToPoint(0,-5,1000,{.forwards = false, .maxSpeed = 110, .minSpeed = 30});
    intake();
    chassis.moveToPoint(-5, 45, 2000,{.maxSpeed = 85});
    chassis.turnToHeading(-90, 1000);
    pros::delay(150);

    match_load.extend();
    pros::delay(150);
    chassis.moveToPoint(-20, 46, 1200,{.maxSpeed = 80});
    pros::delay(120);

    chassis.moveToPoint(20, 45, 900,{.forwards = false,.maxSpeed = 100});

    pros::delay(800);
    long_goal();
    match_load.retract();
    pros::delay(750);
    chassis.turnToHeading(-200, 1000, {.minSpeed = 20});
    intake();
    pros::delay(300);
    chassis.moveToPoint(15, 26, 1000, {.maxSpeed = 80});
    chassis.turnToHeading(-45, 1000);
    chassis.moveToPoint(27, 13, 1000,{.forwards = false, .maxSpeed = 80});
    pros::delay(200);

    medium_top();

    pros::delay(500);
    chassis.swingToHeading(-180, lemlib::DriveSide::LEFT, 1000,{.earlyExitRange = 15});
    //chassis.moveToPoint(15, 27, 1000, {.minSpeed = 30, .earlyExitRange = 3});
    pros::delay(400);
    outtake();
    chassis.moveToPoint(19, -13, 2000,{.minSpeed = 40});
    intake();
    chassis.moveToPoint(20, -18, 1000, {.minSpeed = 10});
    chassis.turnToPoint(3,-48, 1000,{.minSpeed = 40});
    chassis.moveToPoint(3, -45, 1000);

    chassis.turnToHeading(-90, 1000,{.minSpeed = 30});

    chassis.moveToPoint(20, -48.5, 1000,{.forwards = false, .minSpeed = 30});
    pros::delay(150);
    long_goal();
    /*
    chassis.turnToHeading(-45, 1000);

    chassis.moveToPoint(24, 13, 1000,{.forwards = false , .maxSpeed = 70});

    medium_top();
    */
    pros::delay(150000);
}

void leftQUAL(){
    chassis.setPose(0,0,0);
    wing.extend();
    chassis.moveToPoint(-12, 23.5, 3000, {.maxSpeed = 50});
    intake();
    pros::delay(900);
    match_load.extend();
    chassis.moveToPoint(-33, 37.6, 3000,{.maxSpeed = 70});
    pros::delay(300);
    match_load.retract();
    pros::delay(700);
    chassis.moveToPoint(-7, 26, 1000,{.forwards = false});
    chassis.turnToHeading(-140, 1000);
    //chassis.waitUntilDone();
    chassis.moveToPoint(0, 32, 1000,{.forwards = false, .maxSpeed =50});
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
    chassis.moveToPoint(-35.5, -18, 1200,{.maxSpeed = 70});
    //pros::delay(200);
    chassis.moveToPoint(-35.5, 23, 1000,{.forwards= false, .maxSpeed = 70});
    match_load.retract();
    pros::delay(800);

    long_goal();

    pros::delay(800);
    /*
    chassis.moveToPoint(-35.5, 7, 1000);

    chassis.turnToHeading(-140, 1000);
    
    chassis.moveToPoint(-25.25, 15, 1000,{.forwards = false,.maxSpeed = 90});
    pros::delay(300);
    wing.retract();
    chassis.turnToHeading(-180, 1000);

    chassis.moveToPoint(-27.1, 35, 1000,{.forwards = false});
    */

}  


void sevenWingRight(){
    chassis.setPose(0,0,0);
    wing.extend();
    chassis.moveToPoint(8, 22.5, 1000, {.maxSpeed = 90});
    intake();
    pros::delay(300);
    match_load.extend();

    pros::delay(1200);
    
    chassis.turnToPoint(35, 0, 1000);

    pros::delay(100);

    match_load.retract();

    chassis.moveToPoint(35, 0, 1000,{.maxSpeed = 90});
    chassis.turnToHeading(180,1000);

    pros::delay(200);
    match_load.extend();
    intake();
    chassis.moveToPoint(34, -15, 1600,{.maxSpeed = 80});

    chassis.moveToPoint(35.25, 20, 2000,{.forwards = false,.maxSpeed = 80});

    pros::delay(400);
    outtake();
    pros::delay(200);
    
    intake();
    pros::delay(200);
    long_goal();
    match_load.retract();
    pros::delay(1500);
    
    chassis.moveToPoint(44.5, 10, 2000);

    chassis.turnToHeading(180, 1000);
    wing.retract();
    chassis.moveToPoint(44.5, 35, 1000,{.forwards = false});
    
    
    
    /*
    chassis.moveToPoint(36, 7, 1000,{.maxSpeed = 80, .earlyExitRange = 0.5});

    chassis.turnToHeading(245, 1000);

    chassis.moveToPoint(45.5, 14, 1000,{.forwards = false, .maxSpeed = 80,.earlyExitRange = 0.5});
    chassis.turnToHeading(180, 1000);
    wing.retract();
    chassis.moveToPoint(45.4, 38, 1000,{.forwards = false});
    */

}

void fourWingRight(){
    chassis.setPose(0,0,0);
    wing.extend();
    chassis.moveToPoint(0, 29.7, 1000,{.earlyExitRange = 0.5});
    intake();
    chassis.turnToHeading(90,1000);
    pros::delay(200);
    
    match_load.extend();
    
    chassis.moveToPoint(11, 30, 1000, {.maxSpeed = 60});
    
    pros::delay(600);

    chassis.moveToPoint(-25.5, 30.5, 2000,{.forwards = false,.maxSpeed = 80});
    pros::delay(1000);
    long_goal();
    match_load.retract();
    pros::delay(700);

    chassis.moveToPoint(-15, 38.25, 1000);

    chassis.turnToHeading(90, 1000);

    wing.retract();

    chassis.moveToPoint(-35, 38.5, 700, {.forwards = false});

    /*
    chassis.moveToPoint(-10, 30.5, 1000);

    chassis.turnToHeading(140, 1000);

    chassis.moveToPoint(-22.6, 36.5, 1000,{.forwards = false, .maxSpeed = 80});

    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-42, 37, 1000,{.forwards =false});
    wing.retract();
    */

    
}

void skills(){



    chassis.setPose(-50,-19.234, 180);

    chassis.moveToPoint(-50, -46.9,1000, {.minSpeed = 20});

    chassis.turnToHeading(270, 1000);
    pros::delay(300);
    intake();
    match_load.extend();
    chassis.moveToPoint(-70, -46.8, 1000,{.maxSpeed = 70});
    pros::delay(1500);
    
    chassis.moveToPoint(-50, -46.8, 1000,{.forwards= false});
    chassis.moveToPoint(-70, -46.8, 1000,{.maxSpeed = 70});
    pros::delay(1500);

    chassis.moveToPoint(-50, -46.8, 1000,{.forwards= false});
    chassis.moveToPoint(-70, -46.8, 1000,{.maxSpeed = 70});
    pros::delay(1800);

    chassis.moveToPoint(-50, -46.9,1000, {.forwards = false});
    chassis.moveToPoint(-35, -63, 1000, {.forwards = false, .maxSpeed = 80});

    chassis.turnToHeading(270,1000);

    wing.extend();
    chassis.moveToPoint(42, -61, 3000, {.forwards = false ,.maxSpeed = 100});
    
    chassis.turnToHeading(0,1000);
    match_load.retract();
    chassis.moveToPoint(42.25, -48.9, 1000);
    chassis.turnToHeading(90, 1000);
    pros::delay(300);
    //match_load.extend();

    chassis.moveToPoint(20, -48.5, 1400,{.forwards = false, .maxSpeed = 90}); 
    pros::delay(700);
    outtake();
    pros::delay(350);
    long_goal();
    pros::delay(3000);
    
    
    
    chassis.setPose(0,0,0); // position reset
    match_load.extend();
    intake();
    chassis.moveToPoint(-2, 20, 2000);
    chassis.moveToPoint(-2, 35, 2000,{.maxSpeed = 80});
    pros::delay(1300);
    chassis.moveToPoint(-2, 20, 1000,{.forwards = false});
    chassis.moveToPoint(-2, 35, 2000,{.maxSpeed = 80});
    pros::delay(1300);
    chassis.moveToPoint(-2, 20, 1000,{.forwards = false});
    chassis.moveToPoint(-2, 35, 2000,{.maxSpeed = 80});
    pros::delay(1300);

    chassis.moveToPoint(-2, -10, 1000,{.forwards = false, .maxSpeed = 60});
    pros::delay(550);
    outtake();
    pros::delay(350);
    long_goal();
    pros::delay(2000);
    match_load.retract();
    chassis.moveToPoint(-2, 15, 1000);
    chassis.moveToPoint(-2, -20, 1000,{.forwards = false, .maxSpeed = 60});
    
    chassis.setPose(0,0,0);
    long_goal();

    chassis.moveToPoint(0, 10, 1000);
    chassis.turnToPoint(-25, 30, 1000);
    chassis.moveToPoint(-25, 30, 1000);
    chassis.turnToHeading(-90, 1000);

    chassis.moveToPoint(-30, 30, 1000);
    chassis.waitUntilDone();
    match_load.extend();
    pros::delay(300);
    chassis.moveToPoint(-45, 30, 5000,{.minSpeed = 125});
    pros::delay(1500);
    match_load.retract();

    /*
    chassis.setPose(0,0,0);

    chassis.moveToPoint(0,10,1000,{.minSpeed = 20});

    chassis.turnToHeading(-90, 1000);

    chassis.moveToPoint(-92, 15, 2000,{.maxSpeed = 90});
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 1000);
    pros::delay(300);
    intake();
    match_load.extend();
    pros::delay(200);
    chassis.moveToPoint(-93, 15, 1000);
    chassis.moveToPoint(-93, 34, 1000,{.maxSpeed = 80});
    pros::delay(1300);

    chassis.moveToPoint(-93, 20, 1000,{.forwards = false});
    chassis.moveToPoint(-93, 34, 1000,{.maxSpeed = 80});
    pros::delay(1300);

    chassis.moveToPoint(-93, 5, 1000,{.forwards = false});


    chassis.moveToPoint(-105, 5, 1000,{.forwards = false});
    chassis.turnToHeading(0, 1000);

    chassis.moveToPoint(-105, -71, 2000, {.forwards = false, .maxSpeed = 90});

    chassis.turnToHeading(90, 1000);

    chassis.moveToPoint(-89, -70.5, 1000);
    chassis.turnToHeading(180,1000);
    */

   
}


void sevenWingLeft(){

    chassis.setPose(0,0,0);
    wing.extend();
    chassis.moveToPoint(-8, 20.5, 1000, {.maxSpeed = 90});
    intake();
    pros::delay(300);
    match_load.extend();

    pros::delay(1200);
    
    chassis.turnToPoint(-35, 0, 1000);

    pros::delay(100);

    match_load.retract();

    chassis.moveToPoint(-37, 0, 1000,{.maxSpeed = 90});
    chassis.turnToHeading(-180,1000);

    pros::delay(200);
    match_load.extend();
    intake();
    chassis.moveToPoint(-37.5, -15, 1000,{.maxSpeed = 80});

    chassis.moveToPoint(-36.5, 20, 2000,{.forwards =false,.maxSpeed = 70});
    pros::delay(400);
    match_load.retract();
    pros::delay(400);

    long_goal();

    pros::delay(1100);

    chassis.moveToPoint(-36, 7, 1000,{.maxSpeed = 80});

    chassis.turnToHeading(-245, 1000);

    chassis.moveToPoint(-45.5, 14, 1000,{.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(-180, 1000);
    wing.retract();
    chassis.moveToPoint(-45.4, 38, 1000,{.forwards = false});
}

void getOffLine(){
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 5, 1000);
}