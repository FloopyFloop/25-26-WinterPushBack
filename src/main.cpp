#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
//  #include "liblvgl/display/lv_display.h"
//#include "liblvgl/widgets/image/lv_image.h"
//#include "pros/apix.h"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/optical.hpp"
//#include <cstddef>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-2, 3, -4}, pros::MotorGearset::blue); // left motor group - ports 3, 4, 5 (reversed)
pros::MotorGroup rightMotors({-7, 8, 9}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Distance fwrd_distance(10);
pros::Motor intake_1(-5);
pros::Motor intake_2(-10);
pros::Optical optical(6);


pros::adi::Pneumatics tripleDown ('A',false);
pros::adi::Pneumatics tripleUp ('B',false);
pros::adi::Pneumatics wing ('C',false);
pros::adi::Pneumatics match_load ('D',false);

/*

git push commands

git add .
git commit -m "feb break"
git push

*/


// Inertial Sensor on port 10
pros::Imu imu(6);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed

pros::Rotation horizontalEnc(-7);

// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-1);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
//lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 0);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
//lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0.5);

// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)

lemlib::TrackingWheel horizontal(&horizontalEnc,1, 0);

// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative) 
lemlib::TrackingWheel vertical(&verticalEnc,1,0.6);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            7.4, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            80, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             7, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             80, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(5, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(5, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // exp
                                  // 3.o curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

/*
void display_img_from_c_array(){
    LV_IMAGE_DECLARE(nylock_logo_1);
    
    lv_obj_t* img = lv_image_create(lv_screen_active());

    lv_image_set_src(img, &nylock_logo_1_map);
}
*/


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    /*
    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
    */
    chassis.setPose(0,0,0);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging

    


    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    
}

void intake() {
    intake_2.move_voltage(12000);
    intake_1.move_voltage(-12000);
    tripleUp.extend();
    tripleDown.retract();
    
}

void long_goal() {
    
    intake_2.move_voltage(12000);
    intake_1.move_voltage(-12000);
    tripleDown.extend();
    tripleUp.extend();
}

void medium_top() {
    
    intake_2.move_voltage(12000);
    intake_1.move_voltage(-12000);
    tripleDown.retract();
    tripleUp.retract();
    
   
}

void outtake() {
   
    intake_2.move_voltage(-12000);
    intake_1.move_voltage(12000);
    tripleDown.extend();
    tripleUp.extend();
}


void stop_intakes() {
    intake_1.move_voltage(0);
    intake_2.move_voltage(0);
  
    
}
void opcontrol() {
    static bool slow_intake_mode = false;

    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.arcade(leftY, rightX);

        lemlib::Pose pose = chassis.getPose();
        controller.print(0, 0, "X:%.2f Y:%.2f T:%.2f", pose.x, pose.y, pose.theta);

        pros::delay(10);
        bool didAction = false;

        // Toggle slow intake mode with Y button
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            slow_intake_mode = !slow_intake_mode;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake();
            didAction = true;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            outtake();
            didAction = true;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            long_goal();
            didAction = true;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (slow_intake_mode) {
                intake_2.move_voltage(8000);
                intake_1.move_voltage(-8000);
                tripleDown.retract();
                tripleUp.retract();
            } else {
                medium_top();
            }
            didAction = true;
        }

        if (!didAction) stop_intakes();

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            static bool wing_state;
            wing_state = !wing_state;
            if (wing_state) wing.extend();
            else wing.retract();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            static bool match_load_state = false;
            match_load_state = !match_load_state;
            if (match_load_state) match_load.extend();
            else match_load.retract();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            chassis.setPose(0,0,0);
            pros::delay(50);
            chassis.moveToPoint(-6.75, 6, 1000, {.minSpeed = 40, .earlyExitRange = 1});
            chassis.turnToHeading(1, 1000, {.minSpeed = 30, .earlyExitRange = 2});
        }
    }
}

        


     
