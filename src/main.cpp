#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "DebounceIn.h"
#include "modules/Fahrmechnismuss/color_sensor_module.h"
#include "modules/Fahrmechnismuss/debug_print.h"
#include "modules/Fahrmechnismuss/line_array_module.h"
#include "modules/Fahrmechnismuss/motor_module.h"
#include "modules/Fahrmechnismuss/servo_module.h"
#include "modules/ultrasonic_module.h"
#include "modules/greifmechanismuss/abladen_module.h"
#include "modules/greifmechanismuss/aufladen_module.h"
#include "modules/greifmechanismuss/lagern_module.h"

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
    const int main_task_period_ms = 20;  // main loop period in ms (50 Hz)
    const int print_period_ms     = 250; // print interval in ms
    Timer main_task_timer;

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

    /* MODULE OBJECTS*/
    LineArrayModule line_array_module;
    UltrasonicModule ultrasonic_module;
    ColorSensorModule color_sensor_module;
    ServoModule servo_module;
    MotorModule motor_module;

    int print_cycle_counter = 0;
    const int print_cycle_divider = print_period_ms / main_task_period_ms;

    /* ROBOT STATES DECLARATION*/
    enum RobotState {
        INITIAL,
        READY,
        DRIVE,
        RETRIEVE,
        PICKUP,
        DELIVER,
        SLEEP,
        EMERGENCY
    } robot_state = RobotState::INITIAL;

    // Emergency toggle for the while loop
    // the loop will run as long as no emergeny has been met
    int toggle_emergency = 0;

    // --- code that runs every cycle at the start goes here ---

    // start timer
    main_task_timer.start();

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
                color_sensor_module.update();

                robot_state = RobotState::READY;
                break;

            case RobotState::READY:
                printReadyState();

                if (do_execute_main_task) {
                    robot_state = RobotState::DRIVE;
                    led1 = 1;

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
                        led1 = 0;
                    }
                }

                break;

            case RobotState::DRIVE: {
                const bool do_print = (print_cycle_counter == 0);
                const uint8_t action_code = line_array_module.update(do_print);

                // Scale drive velocity by line deviation (0..max_rps).
                // a 0..1 scale, then multiply by the chosen top speed in rps.
                static constexpr float DRIVE_MAX_RPS = 0.75f; // tune as needed max at 1.5
                const float drive_scale = line_array_module.driveVoltage() / 12.0f;
                motor_module.setVelocity(drive_scale * DRIVE_MAX_RPS);
                servo_module.setSteeringAngle(line_array_module.steeringCommand());

                color_sensor_module.update();

                if (do_print) {
                    printDriveStatus(color_sensor_module);
                }

                print_cycle_counter++;
                if (print_cycle_counter >= print_cycle_divider)
                    print_cycle_counter = 0;

                if (action_code == LineArrayModule::EVENT_PICKUP_HOUSE) {
                    printPickupHouseDistanceMm(PICKUP_HOUSE_DISTANCE_MM);
                    // robot_state = RobotState::PICKUP;
                } else if (action_code == LineArrayModule::EVENT_DELIVERY_HOUSE) {
                    printDeliveryHouseDistanceMm(DELIVERY_HOUSE_DISTANCE_MM);
                    // robot_state = RobotState::DELIVER;
                }
                break;
            }

            case RobotState::RETRIEVE:
                printRetrieveState();
                

                break;

            case RobotState::PICKUP:
                printPickupState();

                break;

            case RobotState::DELIVER:
                printDeliverState();

                break;

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
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}
