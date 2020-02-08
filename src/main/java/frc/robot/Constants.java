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
                            LED_ON_BUTTON = 5,
                            LED_OFF_BUTTON = 6,
                            LED_DEFAULT_BUTTON = 8;
    public static final double  FEEDER_SPEED = 0.5,
                                FEEDER_REVERSE_SPEED = -(0.5),
                                SHOOTER_SPEED = 0.5,
                                SHOOTER_REVERSE_SPEED = -(0.5);
    
    // BasicAutonomous.java
    public static final double  DEFAULT_AUTONOMOUS_DISTANCE = 24.0, 
                                DEFAULT_AUTONOMOUS_HEADING = 0.0,
                                AUTONOMOUS_DISTANCE_ERROR_FACTOR = 6.0;
}
