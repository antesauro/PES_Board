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
#include "modules/ultrasonic_module.h"

static constexpr int PICKUP_HOUSE_DISTANCE_MM = 100;
static constexpr int DELIVERY_HOUSE_DISTANCE_MM = 50;
static constexpr float CRANE_MANUAL_UP_VELOCITY_RPS = 1.0f;
static constexpr float HOUSE_STOP_VELOCITY_THRESHOLD_RPS = 0.03f;
static constexpr int HOUSE_STOP_CONFIRM_CYCLES = 5;
static constexpr int HOUSE_STOP_TIMEOUT_MS = 1200;
static constexpr int COLOR_RETRY_TIMEOUT_MS = 1000;
static constexpr int PICKUP_COLOR_STABLE_CYCLES = 5;
static constexpr int DELIVER_COLOR_STABLE_CYCLES = 4;

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
    UltrasonicModule ultrasonic_module;
    ColorSensorModule color_sensor_module;
    ServoModule servo_module;
    MotorModule motor_module;
    aufnehmen::AufnehmenModule aufnehmen_module;
    abladen::AbladenModule abladen_module;
    MotorModuleArm &crane_rope_motor = gripper_actuators::getArmMotor();
    UserButtonCraneControl user_button_crane_control(
        BUTTON1, crane_rope_motor, callback(&toggle_do_execute_main_fcn), 5000, CRANE_MANUAL_UP_VELOCITY_RPS);

    user_button_crane_control.initialize();

    bool rot_abgegeben = false;
    bool gelb_abgegeben = false;
    bool blau_abgegeben = false;
    bool gruen_abgegeben = false;
    int schon_ein_paeckchen_aufgenommen = 0;
    int house_event_cooldown_cycles = 0;
    const int house_event_cooldown_set_cycles = 25; // 25 * 20ms = 500ms
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
        LOSFAHRENN
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

        ultrasonic_module.update();
        // state machine
        switch (robot_state) {
            case RobotState::INITIAL:
                printInitialState();

                // Fahr-Servo initialisieren und aktivieren
                motor_module.initialize();
                servo_module.initialize();
                servo_module.center();
                // Greifmechanismus-Servos initialisieren und aktivieren
                gripper_actuators::initializeDrehkranzServo();
                gripper_actuators::initializeLenkungServo();
                crane_rope_motor.enableMotors();

                color_sensor_module.update();

                robot_state = RobotState::READY;
                break;

            case RobotState::READY:
                printReadyState();

                if (do_execute_main_task) {
                    robot_state = RobotState::START;
                    startup_rotation = motor_module.getRotation(); // Registers initial Rotation of Drive DC Motor
                    led1 = 1;

                } else {
                    // the following code block gets executed only once
                    if (do_reset_all_once) {
                        do_reset_all_once = false;
                        // --- variables and objects that should be reset go here ---
                        // reset variables and objects
                        robot_state = RobotState::INITIAL;
                        rot_abgegeben = false;
                        gelb_abgegeben = false;
                        blau_abgegeben = false;
                        gruen_abgegeben = false;
                        schon_ein_paeckchen_aufgenommen = 0;
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
                        servo_module.disable();
                        motor_module.disable();
                        gripper_actuators::disableDrehkranzServo();
                        gripper_actuators::disableLenkungServo();
                        crane_rope_motor.disableMotors();
                        ultrasonic_module.reset();
                        led1 = 0;
                        distance_traveled = 0.0f;
                        startup_rotation = 0.0f;
                    }
                }

                break;
            case RobotState::START: {
                const bool do_print = (print_cycle_counter == 0);
                line_array_module.update(do_print);

                distance_traveled = (motor_module.getRotation() - startup_rotation) *
                                    -1.0f; // Calculate distance traveled by Drive Motor
                static constexpr float DRIVE_MAX_RPS = 0.5f;
                printf("Distance traveled: %f\n", distance_traveled);
                // First intersection encounter (noch testen mit Abstand!)
                if (distance_traveled >= 0.1f && distance_traveled < 2.5f) {
                    motor_module.setVelocity(-0.4f);      // force speed to not block
                    servo_module.setSteeringAngle(0.7f); // set turn angle for left turn
                } else if (distance_traveled >= 2.5f && distance_traveled < 3.5f) {
                    motor_module.setVelocity(-0.5f);     // force speed to not block
                    servo_module.setSteeringAngle(0.2f); // set turn angle for left turn to smooth out
                }else if( distance_traveled >= 3.5f && distance_traveled <4.0f){
                  motor_module.setVelocity(-0.5f);     // force speed to not block
                    servo_module.setSteeringAngle(0.25f); // set turn angle for left turn to smooth out
                } else {
                    // normal line follow
                    float drive_scale = line_array_module.driveVoltage() / 12.0f;
                    motor_module.setVelocity(drive_scale * DRIVE_MAX_RPS);
                    servo_module.setSteeringAngle(line_array_module.steeringCommand());
                }

                if (distance_traveled >= 4.0f) {
                    robot_state = RobotState::DRIVE;
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

                // Event-Prüfung VOR dem Motor-Setzen:
                // wenn ein Haus erkannt wird, zuerst stoppen – nicht erst fahren und dann stoppen.
                if (house_event_cooldown_cycles == 0 && action_code == LineArrayModule::EVENT_PICKUP_HOUSE) {
                    motor_module.stop();
                    house_stop_confirm_cycles = 0;
                    house_stop_timeout_cycles_remaining = house_stop_timeout_cycles;
                    robot_state = RobotState::PICKUP_WAIT;
                    pickup_color_retry_cycles = 0;
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    printf("Abholhaus erkannt! Farbe wird nach Stop bestimmt.\n");
                    house_event_cooldown_cycles = house_event_cooldown_set_cycles;
                } else if (house_event_cooldown_cycles == 0 && action_code == LineArrayModule::EVENT_DELIVERY_HOUSE &&
                           fabsf(line_array_module.steeringCommand() - 0.5f) < 0.15f) {
                    motor_module.stop();
                    house_stop_confirm_cycles = 0;
                    house_stop_timeout_cycles_remaining = house_stop_timeout_cycles;
                    robot_state = RobotState::DELIVER_WAIT;
                    deliver_color_retry_cycles = 0;
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    printf("Lieferhaus erkannt! Farbe wird nach Stop bestimmt.\n");
                    house_event_cooldown_cycles = house_event_cooldown_set_cycles;
                } else {
                    static constexpr float DRIVE_MAX_RPS = 1.0f;
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
                const int farbe = color_sensor_module.detectedPackageColor();

                if (farbe != 0) {
                    if (farbe == pickup_candidate_color) {
                        pickup_candidate_count++;
                    } else {
                        pickup_candidate_color = farbe;
                        pickup_candidate_count = 1;
                    }
                } else {
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                }

                const bool pickup_color_is_stable =
                    (pickup_candidate_color != 0) && (pickup_candidate_count >= PICKUP_COLOR_STABLE_CYCLES);
                const int stable_farbe = pickup_color_is_stable ? pickup_candidate_color : 0;

                if (stable_farbe == 1 and !rot_abgegeben and
                    (gripper_cfg::lager or schon_ein_paeckchen_aufgenommen == 0)) {
                    aufnehmen_module.aufnehmenRot();
                    if (!gripper_cfg::lager) {
                        schon_ein_paeckchen_aufgenommen = 1;
                    }
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    robot_state = RobotState::LOSFAHRENN;
                } else if (stable_farbe == 2 and !blau_abgegeben and
                           (gripper_cfg::lager or schon_ein_paeckchen_aufgenommen == 0)) {
                    aufnehmen_module.aufnehmenBlau();
                    if (!gripper_cfg::lager) {
                        schon_ein_paeckchen_aufgenommen = 2;
                    }
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    robot_state = RobotState::LOSFAHRENN;
                } else if (stable_farbe == 3 and !gelb_abgegeben and
                           (gripper_cfg::lager or schon_ein_paeckchen_aufgenommen == 0)) {
                    aufnehmen_module.aufnehmenGelb();
                    if (!gripper_cfg::lager) {
                        schon_ein_paeckchen_aufgenommen = 3;
                    }
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    robot_state = RobotState::LOSFAHRENN;
                } else if (stable_farbe == 4 and !gruen_abgegeben and
                           (gripper_cfg::lager or schon_ein_paeckchen_aufgenommen == 0)) {
                    aufnehmen_module.aufnehmenGruen();
                    if (!gripper_cfg::lager) {
                        schon_ein_paeckchen_aufgenommen = 4;
                    }
                    pickup_candidate_color = 0;
                    pickup_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    robot_state = RobotState::LOSFAHRENN;
                } else {
                    pickup_color_retry_cycles++;
                    if (pickup_color_retry_cycles >= color_retry_timeout_cycles) {
                        printf("Pickup: keine stabile gueltige Farbe erkannt (letzte=%s, kandidat=%s, count=%d). Fahre "
                               "weiter.\n",
                               packageColorToString(farbe),
                               packageColorToString(pickup_candidate_color),
                               pickup_candidate_count);
                        pickup_candidate_color = 0;
                        pickup_candidate_count = 0;
                        color_sensor_module.resetPackageColorHold();
                        robot_state = RobotState::LOSFAHRENN;
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

                const int farbe = color_sensor_module.detectedPackageColor();

                if (farbe != 0) {
                    if (farbe == deliver_candidate_color) {
                        deliver_candidate_count++;
                    } else {
                        deliver_candidate_color = farbe;
                        deliver_candidate_count = 1;
                    }
                } else {
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                }

                const bool deliver_color_is_stable =
                    (deliver_candidate_color != 0) && (deliver_candidate_count >= DELIVER_COLOR_STABLE_CYCLES);
                const int stable_farbe = deliver_color_is_stable ? deliver_candidate_color : 0;
                bool delivered_now = false;

                if (stable_farbe == 1 and !rot_abgegeben and
                    (gripper_cfg::lager or schon_ein_paeckchen_aufgenommen == 1)) {
                    abladen_module.abladenRot();
                    rot_abgegeben = true;
                    if (!gripper_cfg::lager) {
                        schon_ein_paeckchen_aufgenommen = 0;
                    }
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    delivered_now = true;
                } else if (stable_farbe == 2 and !blau_abgegeben and
                           (gripper_cfg::lager or schon_ein_paeckchen_aufgenommen == 2)) {
                    abladen_module.abladenBlau();
                    blau_abgegeben = true;
                    if (!gripper_cfg::lager) {
                        schon_ein_paeckchen_aufgenommen = 0;
                    }
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    delivered_now = true;
                } else if (stable_farbe == 3 and !gelb_abgegeben and
                           (gripper_cfg::lager or schon_ein_paeckchen_aufgenommen == 3)) {
                    abladen_module.abladenGelb();
                    gelb_abgegeben = true;
                    if (!gripper_cfg::lager) {
                        schon_ein_paeckchen_aufgenommen = 0;
                    }
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    delivered_now = true;
                } else if (stable_farbe == 4 and !gruen_abgegeben and
                           (gripper_cfg::lager or schon_ein_paeckchen_aufgenommen == 4)) {
                    abladen_module.abladenGruen();
                    gruen_abgegeben = true;
                    if (!gripper_cfg::lager) {
                        schon_ein_paeckchen_aufgenommen = 0;
                    }
                    deliver_candidate_color = 0;
                    deliver_candidate_count = 0;
                    color_sensor_module.resetPackageColorHold();
                    delivered_now = true;
                }

                if (delivered_now) {
                    if (rot_abgegeben && blau_abgegeben && gelb_abgegeben && gruen_abgegeben) {
                        robot_state = RobotState::INITIAL;
                        toggle_do_execute_main_fcn(); // stop main task execution after final completed dropoff
                    } else {
                        robot_state = RobotState::LOSFAHRENN;
                    }
                } else {
                    deliver_color_retry_cycles++;
                    if (deliver_color_retry_cycles >= color_retry_timeout_cycles) {
                        printf("Deliver: keine stabile passende Farbe erkannt (letzte=%s, kandidat=%s, count=%d). "
                               "Fahre weiter.\n",
                               packageColorToString(farbe),
                               packageColorToString(deliver_candidate_color),
                               deliver_candidate_count);
                        deliver_candidate_color = 0;
                        deliver_candidate_count = 0;
                        color_sensor_module.resetPackageColorHold();
                        robot_state = RobotState::LOSFAHRENN;
                    }
                }
                break;
            }

            case RobotState::LOSFAHRENN:

                printReadyState();
                motor_module.setVelocity(-0.5f);
                servo_module.setSteeringAngle(0.5f);
                thread_sleep_for(500);
                motor_module.stop();
                robot_state = RobotState::DRIVE;
                break;

            case RobotState::SLEEP:
                printSleepState();

                break;

            case RobotState::EMERGENCY:
                printEmergencyState();
                motor_module.disable();
                servo_module.disable();
                gripper_actuators::disableDrehkranzServo();
                gripper_actuators::disableLenkungServo();
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
