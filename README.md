# Over Under

./src/chassis.cpp
=================

**Last commit:** fix: orientation (2023-10-21)

./src/chassis.hpp
=================

**Last commit:** fix: orientation (2023-10-21)

./src/config.cpp
================

**Last commit:** ref: update lvgl (2023-11-20)

    
    /**
     * This file just defines the global variables declared in config.hpp.
     * Please edit config.hpp instead of this file for port changes.
     */
    
    #include "config.hpp"
    #include "pros/rotation.hpp"
    #define SHARED(type, name) std::shared_ptr<type> name
    
    SHARED(pros::Rotation,
           odom_left_sensor) = std::make_shared<pros::Rotation>(ODOM_LEFT_PORT);
    
    SHARED(pros::Rotation,
           odom_right_sensor) = std::make_shared<pros::Rotation>(ODOM_RIGHT_PORT);
    
    SHARED(pros::Rotation,
           odom_middle_sensor) = std::make_shared<pros::Rotation>(ODOM_MIDDLE_PORT);
    
    OdomSensor odom_left = {
        .sensor = odom_left_sensor,
        .offset = 0,
    };
    
    OdomSensor odom_right = {
        .sensor = odom_right_sensor,
        .offset = 0,
    };
    
    OdomSensor odom_middle = {
        .sensor = odom_middle_sensor,
        .offset = 0,
    };
    
    SHARED(pros::Motor, drive_left_front) =
        std::make_shared<pros::Motor>(-DRIVE_LEFT_FRONT, pros::v5::MotorGear::blue);
    SHARED(pros::Motor, drive_left_back) =
        std::make_shared<pros::Motor>(-DRIVE_LEFT_BACK, pros::v5::MotorGear::blue);
    SHARED(pros::Motor, drive_left_pto) =
        std::make_shared<pros::Motor>(DRIVE_LEFT_PTO, pros::v5::MotorGear::blue);
    SHARED(pros::Motor, drive_right_front) =
        std::make_shared<pros::Motor>(DRIVE_RIGHT_FRONT, pros::v5::MotorGear::blue);
    SHARED(pros::Motor, drive_right_back) =
        std::make_shared<pros::Motor>(DRIVE_RIGHT_BACK, pros::v5::MotorGear::blue);
    SHARED(pros::Motor, drive_right_pto) =
        std::make_shared<pros::Motor>(-DRIVE_RIGHT_PTO, pros::v5::MotorGear::blue);
    
    std::vector<pros::Motor> drive_left_v = {*drive_left_front, *drive_left_back};
    
    std::vector<pros::Motor> drive_right_v = {*drive_right_front,
                                              *drive_right_back};
    
    SHARED(pros::Motor, catapult_motor) =
        std::make_shared<pros::Motor>(CATAPULT_PORT, pros::v5::MotorGear::red);
    
    SHARED(pros::Rotation,
           catapult_position) = std::make_shared<pros::Rotation>(CATAPULT_ROT_PORT);
    
    // pros::MotorGroup drive_left(drive_left_v);
    
    // pros::MotorGroup drive_left({*drive_bottom_right, *drive_bottom_right});
    // pros::MotorGroup drive_right({*drive_right_front, *drive_right_back});
    
    // auto drive_left = pros::MotorGroup({*drive_left_front, *drive_left_back});
    // SHARED(pros::MotorGroup, drive_right) = std::make_shared<pros::MotorGroup>(
    
    #undef SHARED

./src/config.hpp
================

**Last commit:** fix: ignore deprecation warning from LVGL (2023-11-20)

    
    #pragma once
    #include "main.h"
    
    // ODOMETRY
    #define ODOMETRY_WHEEL_DIAMETER 2.75 // inches
    #define ODOM_LEFT_PORT 8
    #define ODOM_MIDDLE_PORT 9
    #define ODOM_RIGHT_PORT 10
    
    // DRIVETRAIN
    #define DRIVE_LEFT_FRONT 1
    #define DRIVE_LEFT_BACK 2
    #define DRIVE_LEFT_PTO 3
    #define DRIVE_RIGHT_FRONT 4
    #define DRIVE_RIGHT_BACK 5
    #define DRIVE_RIGHT_PTO 6
    
    // CATAPULT
    #define CATAPULT_PORT 7
    #define CATAPULT_ROT_PORT 11
    
    /*************************
     * VARIABLE DECLARATIONS *
     *************************/
    
    // useful macro for quickly defining a shared_ptr
    #define SHARED(type, name) std::shared_ptr<type> name
    
    //// Odometry
    
    struct OdomSensor {
      SHARED(pros::Rotation, sensor);
      double offset;
    };
    
    // odometry sensors
    // no need to use shared pointers
    extern OdomSensor odom_left;
    extern OdomSensor odom_right;
    extern OdomSensor odom_middle;
    
    //// Drivetrain
    extern SHARED(pros::Motor, drive_left_front);
    extern SHARED(pros::Motor, drive_left_back);
    extern SHARED(pros::Motor, drive_left_pto);
    extern SHARED(pros::Motor, drive_right_front);
    extern SHARED(pros::Motor, drive_right_back);
    extern SHARED(pros::Motor, drive_right_pto);
    
    extern pros::MotorGroup drive_left;
    extern pros::MotorGroup drive_right;
    
    //// Catapult
    extern SHARED(pros::Motor, catapult_motor);
    extern SHARED(pros::Rotation, catapult_position);
    
    #undef SHARED

./src/main.cpp
==============

**Last commit:** fix: ignore deprecation warning from LVGL (2023-11-20)

    
    #include "main.h"
    #include "config.hpp"
    #include "pros/misc.h"
    #include "subsystems/subsystems.hpp"
    
    /**
     * Runs initialization code. This occurs as soon as the program is started.
     *
     * All other competition modes are blocked by initialize; it is recommended
     * to keep execution time for this mode under a few seconds.
     */
    void initialize() { catapult::initialize(); }
    
    /**
     * Runs while the robot is in the disabled state of Field Management System or
     * the VEX Competition Switch, following either autonomous or opcontrol. When
     * the robot is enabled, this task will exit.
     */
    void disabled() {}
    
    /**
     * Runs after initialize(), and before autonomous when connected to the Field
     * Management System or the VEX Competition Switch. This is intended for
     * competition-specific initialization routines, such as an autonomous selector
     * on the LCD.
     *
     * This task will exit when the robot is enabled and autonomous or opcontrol
     * starts.
     */
    void competition_initialize() {}
    
    /**
     * Runs the user autonomous code. This function will be started in its own task
     * with the default priority and stack size whenever the robot is enabled via
     * the Field Management System or the VEX Competition Switch in the autonomous
     * mode. Alternatively, this function may be called in initialize or opcontrol
     * for non-competition testing purposes.
     *
     * If the robot is disabled or communications is lost, the autonomous task
     * will be stopped. Re-enabling the robot will restart the task, not re-start it
     * from where it left off.
     */
    void autonomous() {}
    
    /**
     * Runs the operator control code. This function will be started in its own task
     * with the default priority and stack size whenever the robot is enabled via
     * the Field Management System or the VEX Competition Switch in the operator
     * control mode.
     *
     * If no competition control is connected, this function will run immediately
     * following initialize().
     *
     * If the robot is disabled or communications is lost, the
     * operator control task will be stopped. Re-enabling the robot will restart the
     * task, not resume it from where it left off.
     */
    void opcontrol() {
      pros::Controller master(pros::E_CONTROLLER_MASTER);
    
      while (true) {
        // get the joystick values
        int leftJoystick = master.get_analog(ANALOG_LEFT_Y);
        int rightJoystick = master.get_analog(ANALOG_RIGHT_Y);
    
        // update drive
        drive_left_back->move(leftJoystick);
        drive_left_front->move(leftJoystick);
        drive_left_pto->move(leftJoystick);
        drive_right_back->move(rightJoystick);
        drive_right_front->move(rightJoystick);
        drive_right_pto->move(rightJoystick);
    
        // if single press, fire once
        if (master.get_digital_new_press(DIGITAL_R1)) {
          catapult::fire();
        }
        // if held, rapid fire
        else if (master.get_digital(DIGITAL_R1)) {
          catapult::rapidFire = true;
        }
        // if released, stop rapid fire
        else {
          catapult::rapidFire = false;
        }
    
        pros::delay(10);
      }
    }

./src/algorithms/PID.cpp
========================

**Last commit:** feat: catapult (2023-11-20)

    
    #include "PID.hpp"
    #include <cstdio>
    
    PIDController::PIDController(double kP, double kI, double kD)
        : _kP(kP), _kI(kI), _kD(kD), debug(false) {}
    
    PIDController::PIDController(double kP, double kI, double kD, bool debug)
        : _kP(kP), _kI(kI), _kD(kD), debug(debug) {}
    
    double PIDController::update(double error) {
      _integral += error;
      double derivative = error - _previousError;
    
      double kPOutput = _kP * error;
      double kIOutput = _kI * _integral;
      double kDOutput = _kD * derivative;
    
      if (debug)
        printf("err: %f, kP: %f, kI: %f, kD: %f\n", error, kPOutput, kIOutput,
               kDOutput);
    
      double output = kPOutput + kIOutput + kDOutput;
    
      _previousError = error;
      return output;
    }
    
    void PIDController::reset() {
      _previousError = 0;
      _integral = 0;
    
      if (debug)
        printf("PID reset");
    }

./src/algorithms/PID.hpp
========================

**Last commit:** feat: catapult (2023-11-20)

    
    #pragma once
    
    class PIDController {
    public:
      PIDController(double kP, double kI, double kD);
      PIDController(double kP, double kI, double kD, bool debug);
      double update(double error);
      void reset();
    
    private:
      bool debug;
      double _kP, _kI, _kD;
      double _previousError = 0;
      double _integral = 0;
    };

./src/odom/odom.cpp
===================

**Last commit:** ref: odom init (2023-08-27)

    
    #include "./odom.hpp"
    #include "../config.hpp"
    #include "main.h"
    
    // Task to update the odom
    pros::Task *odomTask = nullptr;
    pros::Mutex odom::mutex;
    
    struct {
      double left, right, center, theta;
    } prevSensors = {0, 0, 0, 0};
    
    struct {
      double left, right, theta;
    } resetValues = {0, 0, 0};
    
    odom::RobotPosition state = {0, 0, 0};
    
    void odom::update() {
      // lock mutex
      mutex.take();
    
      // skip runs when all sensors are not initialized
    
      // 1. Store the current encoder values
      auto left = odom_left.sensor->get_position();
      auto right = odom_right.sensor->get_position();
      auto center = odom_middle.sensor->get_position();
    
      // 2. Calculate delta values
      auto dL = left - prevSensors.left;
      auto dR = right - prevSensors.right;
      auto dC = center - prevSensors.center;
    
      // 3. Update the previous values
      prevSensors.left = left;
      prevSensors.right = right;
      prevSensors.center = center;
    
      // 4. total change since last reset
      // auto deltaLr = left - resetValues.left;
      // auto deltaRr = right - resetValues.right;
    
      // 5. Calculate new orientation
      auto newTheta = resetValues.theta +
                      (left - right) / (odom_left.offset + odom_right.offset);
    
      printf("newTheta: %f\n", newTheta);
    
      // 6. Calculate change in orientation
      auto dTheta = newTheta - state.theta;
    
      // 7. Calculate local offset for dTheta = 0
      RobotPosition localOffset = {0, 0, 0};
    
      if (dTheta == 0) {
        localOffset.x = dC;
        localOffset.y = dR;
      } else {
        // 8. Otherwise, calculate local offset with formula.
        localOffset.x = 2 * sin(dTheta / 2) * (dC / dTheta + (odom_middle.offset));
        localOffset.y = 2 * sin(dTheta / 2) * (dR / dTheta + (odom_right.offset));
      }
    
      // 9. Calculate the average orientation
      auto thetam = state.theta + dTheta / 2;
    
      // 10. Calculate the global offset
      RobotPosition globalOffset = {0, 0, 0};
      // int globalOffsetX = 0;
      // int globalOffsetY = 0;
    
      // convert local offset to polar coordinates
      double r =
          sqrt(localOffset.x * localOffset.x + localOffset.y * localOffset.y);
      double theta = atan2(localOffset.y, localOffset.x);
    
      // subtract thetam from the angle component
      theta -= thetam;
    
      // convert back to Cartesian coordinates
      globalOffset.x = r * cos(theta);
      globalOffset.y = r * sin(theta);
    
      // 11. Update the global position
      state.x += globalOffset.x;
      state.y += globalOffset.y;
    
      state.theta = newTheta;
    
      // unlock mutex
      mutex.give();
    }
    
    void odom::init() {
      if (odomTask != nullptr) {
        std::cout << "WARNING: odom::init() called when odomTask is not null"
                  << std::endl;
        return;
      }
    
      odomTask = new pros::Task([]() {
        while (true) {
          update();
          pros::delay(10);
        }
      });
    }
    
    void odom::reset(odom::RobotPosition startState) {
      // aquire mutex
      mutex.take();
    
      // stop task
      if (odomTask != nullptr) {
        odomTask->remove();
        odomTask = nullptr;
      }
    
      // reset encoders
      odom_left.sensor->reset();
      odom_right.sensor->reset();
      odom_middle.sensor->reset();
    
      // reset state
      // state = startState;
      state.x = startState.x;
      state.y = startState.y;
      state.theta = startState.theta;
      resetValues.theta = startState.theta;
    
      // reset prevSensors
      prevSensors = {0, 0, 0, 0};
    
      // restart task
      init();
    
      // release mutex
      mutex.give();
    }
    
    void odom::reset() {
      // default to 0, 0, 0
      reset({0, 0, 0});
    }
    
    odom::RobotPosition odom::getPosition(bool degrees) {
      // aquire mutex
      mutex.take();
    
      // get the state
      RobotPosition returnState =
          degrees ? RobotPosition(state.x, state.y, state.theta * (180 / M_PI))
                  : state;
    
      // release mutex
      mutex.give();
    
      // return the state
      return returnState;
    }
    
    odom::RobotPosition odom::getPosition() { return getPosition(false); }

./src/odom/odom.hpp
===================

**Last commit:** feat: finish odom (2023-08-27)

    
    #pragma once
    #include "main.h"
    
    namespace odom {
    
    struct RobotPosition {
      double x;
      double y;
      double theta;
    
      int getDegrees() { return (int)(theta * 180 / M_PI); }
      RobotPosition(double x, double y, double theta) : x(x), y(y), theta(theta) {}
    };
    
    /**
     * Returns the current robot location, by default in radians.
     */
    RobotPosition getPosition(bool degrees);
    RobotPosition getPosition();
    
    /**
     * Updates the odoemtry position
     */
    void update();
    
    /**
     * Resets the odometry to a given position.
     */
    void reset(RobotPosition startState);
    void reset();
    
    /**
     * Initializes the odometry task.
     */
    void init();
    
    /**
     * The odometry mutex. Use whenever you are reading values.
     */
    extern pros::Mutex mutex;
    
    } // namespace odom

./src/screen/field.cpp
======================

**Last commit:** feat: catapult (2023-11-20)

    
    #include "../odom/odom.hpp"
    #include "screen.hpp"
    
    // The width of the field in inches (for use in converting inches to pixels)
    const double FIELD_WIDTH_INCHES = 147.8377757;
    
    // the width/height of the robot indicator in pixels
    const int ROBOT_INDICATOR_SIZE = 15;
    
    // load the field and robot image
    LV_IMG_DECLARE(FIELD_BACKGROUND);
    LV_IMG_DECLARE(ROBOT_IMAGE);
    
    /**
     * Converts inches to pixels.
     *
     * @param[in] inches The number of inches to convert.
     * @param[in] imageWidth The width of the image in pixels.
     * @return The number of pixels that corresponds to the given number of inches.
     */
    inline double inchesToPixels(double inches, double imageWidth) {
      return inches / FIELD_WIDTH_INCHES * imageWidth;
    }
    
    void screen::FieldWindow::init() {
      // create the images
      fieldImage = lv_img_create(this->lvObj);
      lv_img_set_src(fieldImage, &FIELD_BACKGROUND);
    
      robotImage = lv_img_create(this->lvObj);
      lv_img_set_src(robotImage, &ROBOT_IMAGE);
    
      // set the size of the images
      lv_obj_set_size(fieldImage, width, height);
      lv_obj_set_size(robotImage, ROBOT_INDICATOR_SIZE, ROBOT_INDICATOR_SIZE);
    
      // set the position of the robot image
      lv_obj_set_pos(robotImage, 0, 0);
      lv_obj_set_pos(robotImage, 0, 0);
    }
    
    void screen::FieldWindow::tick() {
      // get the robot's position
      auto position = odom::getPosition();
    
      // convert the position to pixels
      double x = inchesToPixels(position.x, width);
      double y = inchesToPixels(position.y, height);
    
      // set the position of the robot image
      lv_obj_set_pos(robotImage, x - ROBOT_INDICATOR_SIZE / 2.0,
                     height - y - ROBOT_INDICATOR_SIZE / 2.0);
    }

./src/screen/screen.hpp
=======================

**Last commit:** fix: orientation (2023-10-21)

    
    #pragma once
    #include "liblvgl/core/lv_obj.h"
    #include "liblvgl/widgets/lv_img.h"
    #include "main.h"
    
    namespace screen {
    
    /**
     * Class that represents a window on the screen.
     */
    class Window {
    public:
      Window(int width, int height);
      Window(int width, int height, lv_obj_t *parent);
      ~Window();
    
      // delete the copy constructor and assignment operator
      Window(const Window &) = delete;
      Window &operator=(const Window &) = delete;
    
      /**
       * Gets the width of the window.
       *
       * @return The width of the window.
       */
      int getWidth() const;
    
      /**
       * Gets the height of the window.
       *
       * @return The height of the window.
       */
      int getHeight() const;
    
      /**
       * Gets the LVGL object that represents this window.
       *
       * @return The LVGL object that represents this window.
       */
      lv_obj_t *getLvObj();
    
      /**
       * Gets if this window is active
       */
      bool isActive() const;
    
      /**
       * Sets if this window is active
       */
      void setActive(bool active);
    
      /**
       * Toggles the active state of this window
       * @return the new active state
       */
      bool toggleActive();
    
      /**
       * Gets the pros task handle for this window.
       * nullptr if the task is not running.
       */
      pros::Task *getTask() const;
    
      /**
       * Starts the task for this window.
       */
      void startTask();
    
      /**
       * Initializes the window.
       */
      virtual void init();
    
    protected:
      /**
       * The width of the window.
       */
      int width;
    
      /**
       * The height of the window.
       */
      int height;
    
      /**
       * The LVGL object that represents this window.
       */
      lv_obj_t *lvObj;
    
      /**
       * The pros task handle for this window.
       */
      pros::Task *task;
    
      /**
       * If this window is active
       */
      bool active;
    
      /**
       * The task function for this window.
       */
      virtual void tick();
    };
    
    /**
     * Field window class.
     */
    class FieldWindow : public Window {
    public:
      using Window::Window;
    
      // overridden functions
      void init() override;
    
    protected:
      // overridden functions
      void tick() override;
    
    private:
      lv_obj_t *fieldImage;
      lv_obj_t *robotImage;
    };
    
    } // namespace screen

./src/screen/window.cpp
=======================

**Last commit:** fix: it builds! (2023-10-18)

    
    #include "liblvgl/core/lv_disp.h"
    #include "main.h"
    #include "screen.hpp"
    
    using namespace screen;
    
    // refresh rate of the screen in milliseconds
    const int SCREEN_REFRESH_RATE = 20;
    
    /**
     * Main constructor for a window.
     */
    Window::Window(int width, int height, lv_obj_t *parent) {
      this->width = width;
      this->height = height;
      this->active = true;
      this->lvObj = lv_obj_create(parent);
      lv_obj_set_size(this->lvObj, width, height);
    }
    
    /**
     * Creates a new window with the given width and height and uses the active
     * screen as the parent.
     */
    Window::Window(int width, int height) : Window(width, height, lv_scr_act()) {}
    
    /**
     * Frees the LVGL object that represents this window and kills the pros task.
     */
    Window::~Window() {
      lv_obj_del(this->lvObj);
    
      if (this->task != nullptr) {
        this->task->remove();
        delete this->task;
      }
    }
    
    void Window::startTask() {
      if (this->task != nullptr) {
        // error - task already started
        throw "Task already started";
      }
    
      this->task = new pros::Task(
          [this]() {
            while (true) {
              this->tick();
              pros::delay(SCREEN_REFRESH_RATE);
            }
          },
          "Screen Window");
    
      // this->task->set_priority(SCREEN_TASK_PRIORITY);
    }
    
    /**
     * Gets the width of the window.
     *
     * @return The width of the window.
     */
    int Window::getWidth() const { return this->width; }
    
    /**
     * Gets the height of the window.
     *
     * @return The height of the window.
     */
    int Window::getHeight() const { return this->height; }
    
    /**
     * Gets the LVGL object that represents this window.
     *
     * @return The LVGL object that represents this window.
     */
    lv_obj_t *Window::getLvObj() { return this->lvObj; }
    
    /**
     * Gets if this window is active
     */
    bool Window::isActive() const { return this->active; }
    
    /**
     * Sets if this window is active
     */
    void Window::setActive(bool active) { this->active = active; }
    
    /**
     * Toggles the active state of this window
     * @return the new active state
     */
    bool Window::toggleActive() {
      this->active = !this->active;
      return this->active;
    }
    
    /**
     * Gets the pros task handle for this window.
     * nullptr if the task is not running.
     */
    pros::Task *Window::getTask() const { return this->task; }
    
    // stub
    void Window::tick(){};

./src/subsystems/catapult.cpp
=============================

**Last commit:** feat: catapult (2023-11-20)

    
    #include "../config.hpp"
    #include "subsystems.hpp"
    #include <memory>
    
    // simple PD controller for the catapult
    std::shared_ptr<PIDController> catapult::catapultPID =
        std::make_shared<PIDController>(1, 0, 0.001);
    
    catapult::CatapultState catapult::catapultState = RELOADING;
    pros::Task *catapult::catapultTask = nullptr;
    bool catapult::rapidFire = false;
    
    /**
     * A simple state machine-like function to handle catapult control.
     * NOTE: The rotational sensor is BACKWARDS, catapult DOWN = DECREASED ANGLE!
     */
    void catapult::update() {
      // if we're ready, do nothing
      // ratchet will keep us in place
      if (catapultState == READY) {
        return;
      }
    
      // otherwise, calculate error and spin
      double position = (double)catapult_position->get_angle() / 100;
      double error =
          position < CATAPULT_READY_STATE ? CATAPULT_READY_STATE - position : 360;
    
      // if error is less than the allowed error, we're ready
      // rapidFire needs to be false
      if (error < CATAPULT_ALLOWED_ERROR && catapultState != FIRING && !rapidFire) {
        catapultState = READY;
        catapult_motor->move_velocity(0);
        return;
      }
    
      // reload if error > 20deg
      if (error > 30 && catapultState == FIRING)
        catapultState = RELOADING;
    
      double output = catapultPID->update(error);
      catapult_motor->move_velocity(output);
    }
    
    void catapult::fire() {
      // if we're ready, fire
      if (catapultState == READY) {
        catapultState = FIRING;
      }
    
      printf("[warn] catapult is not ready to fire! Current state: %d\n",
             catapultState);
    }
    
    void catapult::updateLoop() {
      while (true) {
        update();
        pros::delay(10);
      }
    }
    
    void catapult::initialize() {
      if (catapultTask == nullptr)
        catapultTask = new pros::Task(updateLoop);
    }

./src/subsystems/subsystems.hpp
===============================

**Last commit:** ref: update lvgl (2023-11-20)

    
    #pragma once
    
    #include "../algorithms/PID.hpp"
    #include "pros/rtos.hpp"
    
    #include <memory>
    
    /**
     * Catapult-related subsystem functions
     */
    namespace catapult {
    
    #define CATAPULT_ALLOWED_ERROR 10
    #define CATAPULT_ZERO_ANGLE 262
    #define CATAPULT_READY_STATE CATAPULT_ZERO_ANGLE - 25
    
    enum CatapultState { READY, RELOADING, FIRING };
    extern CatapultState catapultState;
    extern bool rapidFire;
    
    extern std::shared_ptr<PIDController> catapultPID;
    extern pros::Task *catapultTask;
    
    /**
     * Ticks the catapult subsystem.
     */
    void update();
    
    /**
     * Runs the catapult subsystem.
     * Should be run in a separate pros task, as it is blocking.
     */
    void updateLoop();
    
    /**
     * Starts the catapult subsystem.
     */
    void initialize();
    
    /**
     * Fires the catapult. Queues if the catapult is not ready.
     */
    void fire();
    
    } // namespace catapult
    
    /**
     * Wings-related subsystem functions
     */
    namespace wings {}

./src/subsystems/wings.cpp
==========================

**Last commit:** feat: catapult (2023-11-20)

./scripts/htmlify.sh
====================

**Last commit:**

    
    #!/bin/bash
    
    # This script converts text files into one HTML file.
    
    # List of files to convert (glob)
    FILES=(
      # "README.md"
      "./src/*"
      "./src/**/*"
      "./scripts/*"
    )
    
    # Ext override map
    EXT_OVERRIDES=(
      "hpp:cpp"
      "sh:bash"
    )
    
    # Output file
    OUTPUT="index.html"
    
    # Header
    cat <<EOF > $OUTPUT
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset="utf-8">
      <title>My Project</title>
    
      <!-- PrismJS -->
      <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/themes/prism-coy.min.css" />
      <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/plugins/line-numbers/prism-line-numbers.min.css" />
    
      <style>
        body {
          font-family: sans-serif;
          margin: 0;
          padding: 8px;
        }
        pre {
          margin: 0;
        }
    
        code {
          font-family: monospace;
        }
    
        h1 {
          margin-top: 1rem;
          margin-bottom: 0.25rem;
        }
    
        p {
          margin-top: 0.25rem;
          margin-bottom: 0.25rem;
        }
    
        pre[class*="language-"] {
          max-height: inherit !important;
          overflow: hidden !important;
    
          box-shadow: none !important;
          border-left: none !important;
        }
    
        code[class*="language"] {
          overflow: hidden !important;
        }
      </style>
    </head>
    <body>
    EOF
    
    # Append each file to the output with a header of file path
    for FILE in ${FILES[@]}; do
      # Ignore directories
      if [ -d "$FILE" ]; then
        continue
      fi
    
      FILE_EXT="${FILE##*.}"
    
      # Convert header files to C++ files
      # if [ "$FILE_EXT" = "hpp" ]; then
      #   FILE_EXT="cpp"
      # fi
      # Check if the file extension is overridden
      for OVERRIDE in ${EXT_OVERRIDES[@]}; do
        EXT=${OVERRIDE%:*}
        NEW_EXT=${OVERRIDE#*:}
        if [ "$FILE_EXT" = "$EXT" ]; then
          FILE_EXT="$NEW_EXT"
        fi
      done
    
      # Append the title
      echo "<h1>$FILE</h1>" >> $OUTPUT
      
      # Get the last git commit message and date
      LAST_COMMIT=$(git log -1 --pretty=format:"%s (%ad)" --date=short $FILE)
      echo "<p><strong>Last commit:</strong> $LAST_COMMIT</p>" >> $OUTPUT
    
      # Append the code
      echo "<pre><code class=\"language-$FILE_EXT line-numbers\">" >> $OUTPUT
    
      cat $FILE | sed 's/&/\&amp;/g; s/</\&lt;/g; s/>/\&gt;/g; s/"/\&quot;/g; s/'"'"'/\&#39;/g' >> $OUTPUT
      echo "</code></pre>" >> $OUTPUT
    
      # cat $FILE | sed -e 's/^/    /' | sed -e 's/  / \&nbsp;/' | sed -e 's/$/<br>/' >> $OUTPUT
    done
    
    # Footer
    cat <<EOF >> $OUTPUT
      <!-- PrismJS -->
      <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/prism.min.js"></script> 
      <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/components/prism-c.min.js"></script>
      <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/components/prism-cpp.min.js"></script>
      <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/components/prism-bash.min.js"></script>
      <script src="https://cdnjs.cloudflare.com/ajax/libs/prism/9000.0.1/plugins/line-numbers/prism-line-numbers.min.js"></script>
    </body>
    </html>
    EOF
