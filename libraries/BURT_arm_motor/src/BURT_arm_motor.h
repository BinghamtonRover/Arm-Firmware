/*! @file 
 * \addtogroup Motor 
 * \{
*/

#ifndef burt_arm_motor_h
#define burt_arm_motor_h

#include <Arduino.h>
#include <BURT_arm_constants.h>
#include <TMCStepper.h>

/// How often to check if any of the motors have stalled.
#define STALL_CHECK_INTERVAL 100  // ms

/// The speed of the motors (units unknown).
#define MOTOR_SPEED 5

/// The acceleration of the motor (units unknown).
#define MOTOR_ACCELERATION 1

/// Controls a TMC 5160 stepper motor. 
/// 
/// Backed by the `TMCStepper` library, this class manages an individual stepper motor such that
/// it moves to a specific location within physical bounds with lots of redundancy. When moving,
/// the code ensures the motor never moves by more than BurtArmConstants#maxDelta steps at a time,
/// never moves lower than #minLimit, and never moves beyond #maxLimit. 
/// 
/// To use this class, simply call #setup once and use #moveTo or #moveBy as desired. 
/// 
/// This class also has safety features such as #fixPotentialStall, #calibrate, and #isFinished.
/// These are important as the design of the arm means that certain arrangements are likely to 
/// damage the arm. If one motor behaves unpredictably, the structure as a whole risks instability.
/// In the event of a failure, the safest thing to do is stop _all_ motor movement, which the code
/// does with an infinite while loop. Error messages are logged over the Serial Monitor. 
class StepperMotor { 
	private: 
		/// The chip select pin.
		/// 
		/// Used to initialize and configure the motor. If the setup goes wrong it probably 
		/// has something to do with this pin. 
		byte chipSelectPin;

		/// The enable pin. 
		/// 
		/// Enables the motor. This doesn't seem to be used outside of #setup, not even by TMC. 
		byte enablePin;

		/// The limit switch pin.
		/// 
		/// Used to evaluate whether the motor has trigged the limit switch. Used for calibration.
		byte limitSwitchPin;

		/// The Root Mean Square current to feed the motor. 
		/// 
		/// This value is passed to TMC in #setup.
		short current;

		/// The lower physical bound for this motor. 
		/// 
		/// WARNING: Moving lower than this value can cause damage to the arm. 
		float minLimit;

		/// The upper physical bound for this motor. 
		/// 
		/// WARNING: Moving higher than this value can cause damage to the arm.
		float maxLimit;

		/// The ratio of the gears in the gearbox for this motor. 
		/// 
		/// This determines how many steps are in a radian and vice-versa. See #radToSteps.
		float gearboxRatio;

		/// The time (in milliseconds) that the next stall check is scheduled for. 
		/// 
		/// The system should check for stalls every #STALL_CHECK_INTERVAL milliseconds. Failure
		/// to do so may result in one or motors stuck in place while the others keep moving. This
		/// can cause damage to the arm and will certainly throw off any calculations. 
		/// 
		/// When this time has passed, call #fixPotentialStall to check and fix stalls.
		float nextStallCheck;

		/// The TMC instance for this motor.
		/// 
		/// Most physical actions are delegated to the TMC library. 
		TMC5160Stepper driver;

		/// Converts the desired radians to a number of steps. 
		/// 
		/// The result is specific to this motor, as the #gearboxRatio changes the calculations.
		int radToSteps(float radians);

		/// Checks if this motor has stalled. 
		/// 
		/// Check this every #STALL_CHECK_INTERVAL milliseconds. Use #nextStallCheck to see when 
		/// the next stall check has been scheduled for. 
		/// 
		/// NOTE: Do **not** call #fixPotentialStall from here, as that function calls this 
		/// one to check for stalls, and will cause a stack overflow. 
		bool didStall();

	public: 
		/// The current angle of the joint this motor is powering.
		float angle;

		/// Manage a stepper motor with the given pins.
		StepperMotor(byte chipSelectPin, byte enablePin, byte limitSwitchPin, int current, float minLimit, float maxLimit, float gearboxRatio) : 
			chipSelectPin(chipSelectPin),
			enablePin(enablePin),
			limitSwitchPin(limitSwitchPin),
			current(current),
			minLimit(minLimit),
			maxLimit(maxLimit),
			gearboxRatio(gearboxRatio),
			driver(TMC5160Stepper(SPI, chipSelectPin, 0.075)) { }

		/// Initializes the motor.
		/// 
		/// If any part of the setup fails, the function will log to the serial monitor and block
		/// indefinitely. Issues in this function usually result from an issue with the #chipSelectPin
		/// or the #enablePin.
		/// 
		/// Many settings are configured here behind the scenes. For example, #MOTOR_ACCELERATION and 
		/// #MOTOR_SPEED are both used here. In the future, explore what the different options actually
		/// do and how to optimize them. 
		void setup();  

		/// Calibrates the motor by returning to its home position.
		/// 
		/// Moves backwards until it hits its limit switch, then stops and overrides its internal
		/// state (including #angle) to a known value.
		void calibrate();  // calibrates the motor

		/// Whether the motor has reached its destination.
		/// 
		/// The TMC backend knows where the motor is trying to go and where it actually is. When 
		/// the two values match, this returns true. Do not move the arm while this is false. 
		bool isFinished();

		/// Checks for potential stalls (#didStall) and fixes it (#calibrate).
		/// 
		/// For performance, this function does nothing until #nextStallCheck has passed. 
		/// 
		/// For now, this function actually hangs indefinitely if it detects a stall. This way,
		/// the other motors will not move independently which will protect the arm.
		void fixPotentialStall();

		/// Moves the motor to a specific angle, in radians. 
		/// 
		/// Differs from #moveBy in that this function receives a target rotation and rotates 
		/// until it matches (see #isFinished). Use this method in IK mode, where you know where to go. 
		void moveTo(float newAngle);

		/// Moves the motor by a given rotation, in radians. 
		/// 
		/// Differs from #moveTo in that this function receives an offset in radians and rotates 
		/// by that amount. Use this method in precision mode, where you move in small increments. 
		void moveBy(float radians);

		/// Moves the given number of steps. 
		/// 
		/// Use in debugging only. In production code, use #moveBy. This method can help determine
		/// when the conversion factor (#radToSteps) is off, or the motor is misbehaving.
		void debugMoveSteps(int steps);

		/// Checks if the limit switch is activated.
		///
		/// Assumes HIGH is active.
		bool readLimitSwitch();
};

#endif
// The following close bracket marks the file for Doxygen
/*! \} */
