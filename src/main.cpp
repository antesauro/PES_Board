#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "modules/Fahrmechanismus/ColorSensor.h"
#include "modules/Fahrmechanismus/debug_print.h"
#include "modules/Fahrmechanismus/line_array_module.h"
#include "modules/Fahrmechanismus/motor_module.h"
#include "modules/Fahrmechanismus/servo_module.h"
#include "modules/ultrasonic_module.h"

static constexpr int PICKUP_HOUSE_DISTANCE_MM = 100;
static constexpr int DELIVERY_HOUSE_DISTANCE_MM = 50;

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition at the end

// main runs as an own thread
int main()
{
    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // main loop period in ms (50 Hz)
    const int print_period_ms = 250;    // print interval in ms
    Timer main_task_timer;
    Timer color_pause_timer;

    /* INPUT OBJECTS*/

    // attach button fall function address to user button object
    user_button.fall(&toggle_do_execute_main_fcn);

    /* OUTPUT OBJECTS*/
    // led on nucleo board
    DigitalOut user_led(LED1);

    // additional led
    // create DigitalOut object to command extra led, you need to add an additional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via the resistor
    DigitalOut led1(PB_9);

    // --- adding variables and objects and applying functions starts here ---

    ColorSensor color_sensor(PB_3);

    /* MODULE OBJECTS*/
    LineArrayModule line_array_module;
    UltrasonicModule ultrasonic_module;
    ServoModule servo_module;
    MotorModule motor_module;

    int print_cycle_counter = 0;
    const int print_cycle_divider = print_period_ms / main_task_period_ms;

    float startup_rotation = 0.0f; // Motor rotation variable for the startup sequence
    float distance_traveled = 0.0f;
    float last_pause_rotation = -10.0f; // track cooldown distance

    /* ROBOT STATES DECLARATION*/
    enum RobotState {
        INITIAL,
        READY,
        START,
        DRIVE,
        RETRIEVE,
        DELIVER,
        SLEEP,
        COLOR_PAUSE,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

    // Emergency toggle for the while loop
    // the loop will run as long as no emergeny has been met
    int toggle_emergency = 0;

    // --- code that runs every cycle at the start goes here ---

    // start timer
    main_task_timer.start();
    // start timer
    main_task_timer.start();

    // Turn the sensor LED on for calibration!
    color_sensor.switchLed(ON);

    while (true) {
        // Read the RAW, uncalibrated frequencies directly from the sensor
        const float *raw_hz = color_sensor.readRawColor();

        printf("RAW Hz -> R: %5.0f | G: %5.0f | B: %5.0f | C: %5.0f\n", raw_hz[0], raw_hz[1], raw_hz[2], raw_hz[3]);

        thread_sleep_for(500); // Print twice a second
        /*
            while (!toggle_emergency) {
                main_task_timer.reset();

                ultrasonic_module.update();

                // state machine
                switch (robot_state) {
                    case RobotState::INITIAL:
                        printInitialState();

                        motor_module.initialize();
                        servo_module.initialize();
                        servo_module.center();
                        robot_state = RobotState::READY;
                        break;

                    case RobotState::READY:
                        printReadyState();

                        if (do_execute_main_task) {
                            robot_state = RobotState::START;
                            led1 = 1;
                            startup_rotation = motor_module.getRotation(); // Registers initial Rotation of Drive DC
        Motor

                        } else {
                            // the following code block gets executed only once
                            if (do_reset_all_once) {
                                do_reset_all_once = false;
                                // --- variables and objects that should be reset go here ---
                                // reset variables and objects
                                robot_state = RobotState::INITIAL;
                                servo_module.disable();
                                motor_module.disable();
                                ultrasonic_module.reset();
                                startup_rotation = 0.0f;
                                distance_traveled = 0.0f;
                                color_pause_timer.stop();
                                led1 = 0;
                            }
                        }

                        break;
                    case RobotState::START: {
                        const bool do_print = (print_cycle_counter == 0);
                        line_array_module.update(do_print);

                        distance_traveled =
                            motor_module.getRotation() - startup_rotation; // Calculate distance traveled by Drive Motor
                        static constexpr float DRIVE_MAX_RPS = 0.5f;

                        // First intersection encounter (noch testen mit Abstand!)
                        if (distance_traveled >= 0.2f && distance_traveled < 0.8f) {
                            motor_module.setVelocity(0.5f);      // force speed to not block
                            servo_module.setSteeringAngle(0.6f); // set turn angle for left turn
                        } else if (distance_traveled >= 0.8f && distance_traveled < 1.5f) {
                            motor_module.setVelocity(0.6f);      // force speed to not block
                            servo_module.setSteeringAngle(0.7f); // set turn angle for left turn to smooth out
                        } else {
                            // normal line follow
                            float drive_scale = line_array_module.driveVoltage() / 12.0f;
                            motor_module.setVelocity(drive_scale * DRIVE_MAX_RPS);
                            servo_module.setSteeringAngle(line_array_module.steeringCommand());
                        }

                        if (distance_traveled >= 1.5f) {
                            robot_state = RobotState::DRIVE;
                        }

                        print_cycle_counter++;
                        if (print_cycle_counter >= print_cycle_divider)
                            print_cycle_counter = 0;

                        break;
                    }

                    case RobotState::DRIVE: {
                        const bool do_print = (print_cycle_counter == 0);
                        const uint8_t action_code = line_array_module.update(do_print);

                        // Scale drive velocity by line deviation (0..max_rps).
                        // a 0..1 scale, then multiply by the chosen top speed in rps.
                        static constexpr float DRIVE_MAX_RPS = 1.3f;
                        const float drive_scale = line_array_module.driveVoltage() / 12.0f;
                        motor_module.setVelocity(drive_scale * DRIVE_MAX_RPS);
                        servo_module.setSteeringAngle(line_array_module.steeringCommand());

                        int raw_color = color_sensor.getColor();
                        int farbe = 0;
                        if (raw_color == 3)
                            farbe = 1; // RED
                        else if (raw_color == 7)
                            farbe = 2; // BLUE
                        else if (raw_color == 4)
                            farbe = 3; // YELLOW
                        else if (raw_color == 5)
                            farbe = 4; // GREEN

                        if (farbe != 0) {
                            motor_module.stop();
                            color_pause_timer.reset();
                            color_pause_timer.start();
                            robot_state = RobotState::COLOR_PAUSE;
                        }

                        print_cycle_counter++;
                        if (print_cycle_counter >= print_cycle_divider)
                            print_cycle_counter = 0;

                        break;
                    }
                    case RobotState::RETRIEVE: {
                        printPickupState();

                        break;
                    }

                    case RobotState::DELIVER: {
                        printDeliverState();

                        break;
                    }

                    case RobotState::SLEEP:
                        printSleepState();

                        break;

                    case RobotState::EMERGENCY:
                        printEmergencyState();
                        motor_module.disable();
                        servo_module.disable();
                        // the transition to the emergency state causes the execution of the commands contained
                        // in the outer else statement scope, and since do_reset_all_once is true the system undergoes a
                        // reset
                        toggle_emergency = 1;
                        break;

                    case RobotState::COLOR_PAUSE: {
                        const bool do_print = (print_cycle_counter == 0);
                        line_array_module.update(do_print);

                        int paused_time_ms = duration_cast<milliseconds>(color_pause_timer.elapsed_time()).count();

                        if (paused_time_ms < 3000) {
                            motor_module.stop();
                            servo_module.setSteeringAngle(line_array_module.steeringCommand());
                        } else {
                            static constexpr float DRIVE_MAX_RPS = 1.5f;
                            const float drive_scale = line_array_module.driveVoltage() / 12.0f;
                            motor_module.setVelocity(drive_scale * DRIVE_MAX_RPS);
                            servo_module.setSteeringAngle(line_array_module.steeringCommand());

                            // --- CHECK IF WE CLEARED THE COLOR ZONE ---
                            int raw_color = color_sensor.getColor();
                            // If it is NOT Red, Blue, Yellow, or Green...
                            if (raw_color != 3 && raw_color != 7 && raw_color != 4 && raw_color != 5) {
                                color_pause_timer.stop();
                                robot_state = RobotState::DRIVE;
                            }
                        }

                        if (do_print) {
                            printf("Color: %s\n", ColorSensor::getColorString(color_sensor.getColor()));
                        }

                        print_cycle_counter++;
                        if (print_cycle_counter >= print_cycle_divider)
                            print_cycle_counter = 0;

                        break;
                    }

                    default:
                        break; // do nothing
                }

                // toggling the user led
                user_led = !user_led;

                // --- code that runs every cycle at the end goes here ---

                // printf("US Sensor in cm: %f\n", ultrasonic_module.distanceCm());

                // read timer and make the main thread sleep for the remaining time span (non blocking)
                int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
                if (main_task_period_ms - main_task_elapsed_time_ms < 0)
                    printMainLoopOverrunWarning();
                else
                    thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
            }
        }*/
    }
}
void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task) {
        do_reset_all_once = true;
    }
}
