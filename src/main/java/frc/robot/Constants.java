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
    public static final int DRIVE_LEFTFRONT = 1,
                            DRIVE_LEFTREAR = 2,
                            DRIVE_RIGHTFRONT = 3, 
                            DRIVE_RIGHTREAR = 4,
                            TICKS_PER_INCH = 512;

    public static final int FEEDER_MOTOR = 1;

    public static final DriverStation DS = DriverStation.getInstance();

    public static final int DRIVE_STICK = 0,
                            MANIPULATOR_STICK = 1,
                            DEBUGGER_STICK = 2,
                            BRAKE_BUTTON = 1,
                            LOG_BUTTON = 8;

    public static final double  DEFAULT_AUTONOMOUS_DISTANCE = 24.0, 
                                DEFAULT_AUTONOMOUS_HEADING = 0.0,
                                AUTONOMOUS_DISTANCE_ERROR_FACTOR = 6.0;
}
