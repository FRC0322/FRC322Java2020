/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *<p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	// Robot.java
	public static final DriverStation DS = DriverStation.getInstance();

	// AddressableLEDs.java
	public static final int ADDRESSABLE_LED_PORT = 1,
				ADDRESSABLE_LED_LENGTH = 60;

	// AutomaticLED.java
	public static final double AUTONOMOUS_BLINK_RATE = 0.25,
				   TELOP_BLINK_RATE = 0.5,
				   DISABLED_BLINK_RATE = 0.0,
				   DEFAULT_BLINK_RATE = 2.5;

	// ColorSensor.java
	// These are predetermined defaults.
	// We need to recalibrate these for ourselves.
	public static final double RED_R = 0.561,
				   RED_G = 0.232,
				   RED_B = 0.114,
				   GREEN_R = 0.197,
				   GREEN_G = 0.561,
				   GREEN_B = 0.240,
				   BLUE_R = 0.143,
				   BLUE_G = 0.427,
				   BLUE_B = 0.429,
				   YELLOW_R = 0.361,
				   YELLOW_G = 0.524,
				   YELLOW_B = 0.113;

	// Chassis.java
	public static final int DRIVE_LEFTFRONT = 1,
				DRIVE_LEFTREAR = 2,
				DRIVE_RIGHTFRONT = 3,
				DRIVE_RIGHTREAR = 4,
				TICKS_PER_INCH = 512;

	// Feeder.java
	public static final int FEEDER_MOTOR = 5;

	// Shooter.java
	public static final int LEFT_SHOOTER_MOTOR = 6;
	public static final int RIGHT_SHOOTER_MOTOR = 7;

	// RobotContainer.java
	public static final int DRIVE_STICK = 0,
				MANIPULATOR_STICK = 1,
				DEBUGGER_STICK = 2,
				BRAKE_BUTTON = 1,
				LOG_BUTTON = 8,
				FEEDER_BUTTON = 1,
				SHOOTER_BUTTON = 3,
				FEEDER_REVERSE_BUTTON = 2,
				SHOOTER_REVERSE_BUTTON = 4,
				VISION_MODE_BUTTON = 5,
				DRIVER_MODE_BUTTON = 6,
				LED_ON_BUTTON = 5,
				LED_OFF_BUTTON = 6,
				LED_BLINK_BUTTON = 7,
				LED_DEFAULT_BUTTON = 8,
				CAMERASERVER_FEEDS = 2;
	public static final double FEEDER_SPEED = 0.5,
				   FEEDER_REVERSE_SPEED = -(0.5),
				   SHOOTER_SPEED = 0.5,
				   SHOOTER_REVERSE_SPEED = -(0.5);

	// BasicAutonomous.java
	public static final double DEFAULT_AUTONOMOUS_DISTANCE = 24.0,
				   DEFAULT_AUTONOMOUS_HEADING = 0.0,
				   AUTONOMOUS_DISTANCE_ERROR_FACTOR = 6.0;
}
