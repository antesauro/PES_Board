#include "mbed.h"

// pes board pin map
#include "PESBoardPinMap.h"

// drivers
#include "modules/Fahrmechanismus/color_sensor_module.h"
#include "modules/Fahrmechanismus/debug_print.h"
#include "modules/Fahrmechanismus/line_array_module.h"
#include "modules/Fahrmechanismus/motor_module.h"
#include "modules/Fahrmechanismus/servo_module.h"
#include "modules/Greifmechanismus/actuators/motor_module_Arm.h"
#include "modules/Greifmechanismus/greifmechanismus_module.h"
#include "modules/Greifmechanismus/user_button_crane_control.h"

static constexpr int PICKUP_HOUSE_DISTANCE_MM = 100;
static constexpr int DELIVERY_HOUSE_DISTANCE_MM = 50;
static constexpr float CRANE_MANUAL_UP_VELOCITY_RPS = 1.0f;
static constexpr float HOUSE_STOP_VELOCITY_THRESHOLD_RPS = 0.03f;
static constexpr int HOUSE_STOP_CONFIRM_CYCLES = 5;
static constexpr int HOUSE_STOP_TIMEOUT_MS = 1200;
static constexpr int COLOR_RETRY_TIMEOUT_MS = 1000;
static constexpr int PICKUP_COLOR_STABLE_CYCLES = 10;
static constexpr int DELIVER_COLOR_STABLE_CYCLES = 10;

bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once

void toggle_do_execute_main_fcn();

static const char *packageColorToString(int color)
{
    switch (color) {
        case 1:
            return "RED";
        case 2:
            return "BLUE";
        case 3:
            return "YELLOW";
        case 4:
            return "GREEN";
        default:
            return "UNKNOWN";
    }
}

// main runs as an own thread
int main()
{
    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    const int main_task_period_ms = 20; // main loop period in ms (50 Hz)
    const int print_period_ms = 250;    // print interval in ms
    Timer main_task_timer;

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
    ColorSensorModule color_sensor_module;
    ServoModule servo_module;
    MotorModule motor_module;
    pickup::PickupModule pickup_module;
    dropoff::DropoffModule dropoff_module;
    MotorModuleArm &crane_rope_motor = gripper_actuators::getArmMotor();
    UserButtonCraneControl user_button_crane_control(
        BUTTON1, crane_rope_motor, callback(&toggle_do_execute_main_fcn), 3000, CRANE_MANUAL_UP_VELOCITY_RPS);

    user_button_crane_control.initialize();

    bool red_done   = false;
    bool yellow_done = false;
    bool blue_done  = false;
    bool green_done = false;
    int pkg_held = 0; // 0=none, 1=red, 2=blue, 3=yellow, 4=green
    int house_event_cooldown_cycles = 0;
    const int house_event_cooldown_set_cycles = 20; // 15 * 20ms = 350ms
    int house_stop_confirm_cycles = 0;
    int house_stop_timeout_cycles_remaining = 0;
    int pickup_color_retry_cycles = 0;
    int deliver_color_retry_cycles = 0;
    int pickup_candidate_color = 0;
    int pickup_candidate_count = 0;
    int deliver_candidate_color = 0;
    int deliver_candidate_count = 0;

    float startup_rotation = 0.0f; // Motor rotation variable for the startup sequence
    float distance_traveled = 0.0f;

    int print_cycle_counter = 0;
    const int print_cycle_divider = print_period_ms / main_task_period_ms;

    /* ROBOT STATES DECLARATION*/
    enum RobotState {
        INITIAL,
        READY,
        START,
        DRIVE,
        PICKUP_WAIT,
        RETRIEVE,
        PICKUP,
        DELIVER_WAIT,
        DELIVER,
        SLEEP,
        EMERGENCY,
        DEPART
    } robot_state = RobotState::INITIAL;

    // Emergency toggle for the while loop
    // the loop will run as long as no emergency has been met
    int toggle_emergency = 0;

    // --- code that runs every cycle at the start goes here ---

    // start timer
    main_task_timer.start();
    const int house_stop_timeout_cycles = HOUSE_STOP_TIMEOUT_MS / main_task_period_ms;
    const int color_retry_timeout_cycles = COLOR_RETRY_TIMEOUT_MS / main_task_period_ms;

    while (!toggle_emergency) {
        main_task_timer.reset();

        user_button_crane_control.update();

        // If the button is pressed while driving, instantly force the robot back to READY and trigger a reset!
        if (!do_execute_main_task && robot_state != RobotState::INITIAL && robot_state != RobotState::READY) {
            do_reset_all_once = true;
            robot_state = RobotState::READY;
        }
        // state machine
        switch (robot_state) {
            case RobotState::INITIAL:
                printInitialState();

                // Init drive servo and motor
                motor_module.initialize();
                servo_module.initialize();
                servo_module.center();
                // Init gripper servos
                gripper_actuators::initAll();
                crane_rope_motor.enableMotors();

                color_sensor_module.update();
                //gripper_actuators::testPositionSafety();
                robot_state = RobotState::READY;
                break;

            case RobotState::READY:
                printReadyState();
                color_sensor_module.printColor(); // For testing the colour recognition
                // 1. DO THE RESET FIRST
                if (do_reset_all_once) {
                    do_reset_all_once = false;

                    // Do NOT jump to INITIAL. We just wipe the variables and stay in READY!
                    red_done   = false;
                    yellow_done = false;
                    blue_done  = false;
                    green_done = false;
                    pkg_held = 0;
                    house_event_cooldown_cycles = 0;
                    house_stop_confirm_cycles = 0;
                    house_stop_timeout_cycles_remaining = 0;
                    pickup_color_retry_cycles = 0;
                    deliver_color_retry_cycles = 0;
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    print_cycle_counter = 0;

                    // GENTLY STOP INSTEAD OF CUTTING POWER TO HARDWARE
                    motor_module.stop();
                    servo_module.center();

                    // SEND CRANE HOME SLOWLY
                    gripper_actuators::returnSlow();

                    // FORCE A DELAY SO THE ARM HAS PHYSICAL TIME TO MOVE BEFORE STARTING!
                    thread_sleep_for(1500);

                    led1 = 0;
                    distance_traveled = 0.0f;
                    startup_rotation = 0.0f;

                    break;
                }

                // 2. START THE RUN
                if (do_execute_main_task) {
                    robot_state = RobotState::START;
                    startup_rotation = motor_module.getRotation();
                    gripper_actuators::enableFastMode();
                    led1 = 1;
                }
                break;
            case RobotState::START: {
                const bool do_print = (print_cycle_counter == 0);
                line_array_module.update(do_print);

                // Add a static step counter that remembers where we are
                static int start_sequence_step = 0;

                distance_traveled = (motor_module.getRotation() - startup_rotation) * -1.0f;

                if (start_sequence_step == 0) {
                    motor_module.setVelocity(-0.2f);
                    servo_module.setSteeringAngle(0.5f);
                    if (distance_traveled >= 0.1f)
                        start_sequence_step = 1; // Move to next step
                } else if (start_sequence_step == 1) {
                    motor_module.setVelocity(-1.0f);
                    servo_module.setSteeringAngle(0.8f);
                    if (distance_traveled >= 2.2f)
                        start_sequence_step = 2;
                } else if (start_sequence_step == 2) {
                    motor_module.setVelocity(-1.0f);
                    servo_module.setSteeringAngle(0.15f);
                    if (distance_traveled >= 4.7f)
                        start_sequence_step = 3;
                } else if (start_sequence_step == 3) {
                    motor_module.setVelocity(-0.3f);
                    servo_module.setSteeringAngle(0.55f);
                    if (distance_traveled >= 5.0f)
                        start_sequence_step = 4;
                } else if (start_sequence_step == 4) {
                    robot_state = RobotState::DRIVE;
                    start_sequence_step = 0; // Reset for the next time we run the course
                }

                print_cycle_counter++;
                if (print_cycle_counter >= print_cycle_divider)
                    print_cycle_counter = 0;
            } break;

            case RobotState::DRIVE: {

                const bool do_print = (print_cycle_counter == 0);
                const uint8_t action_code = line_array_module.update(do_print);

                color_sensor_module.update();
                if (do_print) {
                    printDriveStatus(color_sensor_module);
                }

                print_cycle_counter++;
                if (print_cycle_counter >= print_cycle_divider)
                    print_cycle_counter = 0;

                if (house_event_cooldown_cycles > 0)
                    house_event_cooldown_cycles--;

                // Check for house events BEFORE setting motor:
                // when a house is detected, stop first — do not drive then stop.
                if (house_event_cooldown_cycles == 0 && action_code == LineArrayModule::EVENT_PICKUP_HOUSE &&
                    pkg_held == 0) {
                    motor_module.stop();
                    house_stop_confirm_cycles = 0;
                    house_stop_timeout_cycles_remaining = house_stop_timeout_cycles;
                    robot_state = RobotState::PICKUP_WAIT;
                    pickup_color_retry_cycles = 0;
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    printf("Pickup house detected! Color determined after stop.\n");
                    house_event_cooldown_cycles = house_event_cooldown_set_cycles;
                } else if (house_event_cooldown_cycles == 0 && action_code == LineArrayModule::EVENT_DELIVERY_HOUSE &&
                           pkg_held) {
                    motor_module.stop();
                    house_stop_confirm_cycles = 0;
                    house_stop_timeout_cycles_remaining = house_stop_timeout_cycles;
                    robot_state = RobotState::DELIVER_WAIT;
                    deliver_color_retry_cycles = 0;
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    printf("Delivery house detected! Color determined after stop.\n");
                    house_event_cooldown_cycles = house_event_cooldown_set_cycles;
                } else {
                    static constexpr float DRIVE_MAX_RPS = 1.2f;
                    const float drive_scale = line_array_module.driveVoltage() / 12.0f;
                    motor_module.setVelocity(drive_scale * DRIVE_MAX_RPS);
                    servo_module.setSteeringAngle(line_array_module.steeringCommand());
                }

                break;
            }
            case RobotState::PICKUP_WAIT:
                motor_module.stop();
                if (motor_module.getVelocity() <= HOUSE_STOP_VELOCITY_THRESHOLD_RPS &&
                    motor_module.getVelocity() >= -HOUSE_STOP_VELOCITY_THRESHOLD_RPS) {
                    house_stop_confirm_cycles++;
                } else {
                    house_stop_confirm_cycles = 0;
                }

                if (house_stop_confirm_cycles >= HOUSE_STOP_CONFIRM_CYCLES ||
                    house_stop_timeout_cycles_remaining <= 0) {
                    pickup_color_retry_cycles = 0;
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    robot_state = RobotState::PICKUP;
                } else {
                    house_stop_timeout_cycles_remaining--;
                }
                break;

            case RobotState::PICKUP: {
                printPickupState();
                motor_module.stop();
                color_sensor_module.update();
                const int color = color_sensor_module.detectedPackageColor();

                if (color != 0) {
                    if (color == pickup_candidate_color) {
                        pickup_candidate_count++;
                    } else {
                        pickup_candidate_color = color;
                        pickup_candidate_count = 1;
                    }
                } else {
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                }

                const bool pickup_color_is_stable =
                    (pickup_candidate_color != 0) && (pickup_candidate_count >= PICKUP_COLOR_STABLE_CYCLES);
                const int stable_color = pickup_color_is_stable ? pickup_candidate_color : 0;

                if (stable_color == 1 and !red_done and
                    (gripper_cfg::use_storage or pkg_held == 0)) {
                    pickup_module.pickupRed();
                    if (!gripper_cfg::use_storage) {
                        pkg_held = 1;
                    }
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    robot_state = RobotState::DEPART;
                } else if (stable_color == 2 and !blue_done and
                           (gripper_cfg::use_storage or pkg_held == 0)) {
                    pickup_module.pickupBlue();
                    if (!gripper_cfg::use_storage) {
                        pkg_held = 2;
                    }
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    robot_state = RobotState::DEPART;
                } else if (stable_color == 3 and !yellow_done and
                           (gripper_cfg::use_storage or pkg_held == 0)) {
                    pickup_module.pickupYellow();
                    if (!gripper_cfg::use_storage) {
                        pkg_held = 3;
                    }
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    robot_state = RobotState::DEPART;
                } else if (stable_color == 4 and !green_done and
                           (gripper_cfg::use_storage or pkg_held == 0)) {
                    pickup_module.pickupGreen();
                    if (!gripper_cfg::use_storage) {
                        pkg_held = 4;
                    }
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    robot_state = RobotState::DEPART;
                } else {
                    // Stable color detected but no pickup needed → continue immediately
                    if (pickup_color_is_stable) {
                        printf("Pickup: color %s detected, no pickup needed. Continuing.\n",
                               packageColorToString(pickup_candidate_color));
                        pickup_candidate_color = 0;
                        pickup_candidate_count = 0;
                        color_sensor_module.resetPackageColorHold();
                        robot_state = RobotState::DEPART;
                    } else {
                        pickup_color_retry_cycles++;
                        if (pickup_color_retry_cycles >= color_retry_timeout_cycles) {
                            printf("Pickup: no stable valid color (last=%s, cand=%s, cnt=%d). Continuing.\n",
                                   packageColorToString(color),
                                   packageColorToString(pickup_candidate_color),
                                   pickup_candidate_count);
                            pickup_candidate_color = 0;
                            pickup_candidate_count = 0;
                            color_sensor_module.resetPackageColorHold();
                            robot_state = RobotState::DEPART;
                        }
                    }
                }
                break;
            }

            case RobotState::DELIVER_WAIT:
                motor_module.stop();
                if (motor_module.getVelocity() <= HOUSE_STOP_VELOCITY_THRESHOLD_RPS &&
                    motor_module.getVelocity() >= -HOUSE_STOP_VELOCITY_THRESHOLD_RPS) {
                    house_stop_confirm_cycles++;
                } else {
                    house_stop_confirm_cycles = 0;
                }

                if (house_stop_confirm_cycles >= HOUSE_STOP_CONFIRM_CYCLES ||
                    house_stop_timeout_cycles_remaining <= 0) {
                    deliver_color_retry_cycles = 0;
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    robot_state = RobotState::DELIVER;
                } else {
                    house_stop_timeout_cycles_remaining--;
                }
                break;

            case RobotState::DELIVER: {
                printDeliverState();
                motor_module.stop();
                color_sensor_module.update();

                const int color = color_sensor_module.detectedPackageColor();

                if (color != 0) {
                    if (color == deliver_candidate_color) {
                        deliver_candidate_count++;
                    } else {
                        deliver_candidate_color = color;
                        deliver_candidate_count = 1;
                    }
                } else {
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                }

                const bool deliver_color_is_stable =
                    (deliver_candidate_color != 0) && (deliver_candidate_count >= DELIVER_COLOR_STABLE_CYCLES);
                const int stable_color = deliver_color_is_stable ? deliver_candidate_color : 0;
                bool delivered_now = false;

                if (stable_color == 1 and !red_done and
                    (gripper_cfg::use_storage or pkg_held == 1)) {
                    dropoff_module.dropoffRed();
                    red_done = true;
                    if (!gripper_cfg::use_storage) {
                        pkg_held = 0;
                    }
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    delivered_now = true;
                } else if (stable_color == 2 and !blue_done and
                           (gripper_cfg::use_storage or pkg_held == 2)) {
                    dropoff_module.dropoffBlue();
                    blue_done = true;
                    if (!gripper_cfg::use_storage) {
                        pkg_held = 0;
                    }
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    delivered_now = true;
                } else if (stable_color == 3 and !yellow_done and
                           (gripper_cfg::use_storage or pkg_held == 3)) {
                    dropoff_module.dropoffYellow();
                    yellow_done = true;
                    if (!gripper_cfg::use_storage) {
                        pkg_held = 0;
                    }
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    delivered_now = true;
                } else if (stable_color == 4 and !green_done and
                           (gripper_cfg::use_storage or pkg_held == 4)) {
                    dropoff_module.dropoffGreen();
                    green_done = true;
                    if (!gripper_cfg::use_storage) {
                        pkg_held = 0;
                    }
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    delivered_now = true;
                }

                if (delivered_now) {
                    if (red_done && blue_done && yellow_done && green_done) {
                        do_reset_all_once = true;
                        do_execute_main_task = false;
                        robot_state = RobotState::READY;
                    } else {
                        robot_state = RobotState::DEPART;
                    }
                } else {
                    // Stable color detected but no delivery needed → continue immediately
                    if (deliver_color_is_stable) {
                        printf("Deliver: color %s, no delivery needed. Continuing.\n",
                               packageColorToString(deliver_candidate_color));
                        deliver_candidate_color = 0;
                        deliver_candidate_count = 0;
                        color_sensor_module.resetPackageColorHold();
                        robot_state = RobotState::DEPART;
                    } else {
                        deliver_color_retry_cycles++;
                        if (deliver_color_retry_cycles >= color_retry_timeout_cycles) {
                            printf("Deliver: no stable matching color (last=%s, cand=%s, cnt=%d). Continuing.\n",
                                   packageColorToString(color),
                                   packageColorToString(deliver_candidate_color),
                                   deliver_candidate_count);
                            deliver_candidate_color = 0;
                            deliver_candidate_count = 0;
                            color_sensor_module.resetPackageColorHold();
                            robot_state = RobotState::DEPART;
                        }
                    }
                }
                break;
            }

            case RobotState::DEPART:

                printDepartState();
                servo_module.setSteeringAngle(0.5f);
                motor_module.setRotation(motor_module.getRotation() +
                                         0.5f); // drive forward a bit to leave the house area

                robot_state = RobotState::DRIVE;
                break;

            case RobotState::SLEEP:
                printSleepState();

                break;

            case RobotState::EMERGENCY:
                printEmergencyState();
                motor_module.disable();
                servo_module.disable();
                gripper_actuators::disableTurnServo();
                gripper_actuators::disableSteerServo();
                crane_rope_motor.disableMotors();
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
