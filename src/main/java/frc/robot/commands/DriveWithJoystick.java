/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Chassis;

/**
 * Have the robot drive racing game style.
 */
public class DriveWithJoystick extends CommandBase {
  private final Chassis m_chassis;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;
  private final JoystickButton m_brakeButton, m_logButton;

  /**
   * Creates a new DriveWithJoystick command.
   *
   * @param left       The control input for the left side of the drive
   * @param right      The control input for the right sight of the drive
   * @param drivetrain The drivetrain subsystem to drive
   */
  public DriveWithJoystick(DoubleSupplier left, DoubleSupplier right, Chassis chassis,
                            JoystickButton brake, JoystickButton logButton) {
    m_chassis = chassis;
    m_left = left;
    m_right = right;
    m_brakeButton = brake;
    m_logButton = logButton;
    addRequirements(m_chassis);
  }

  @Override
  public void initialize() {
    //m_chassis.setupEncoders();
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    m_chassis.drive(m_left.getAsDouble(), m_right.getAsDouble());
    if (m_brakeButton.get())
      m_chassis.brake(true);
    else
      m_chassis.brake(false);
    if (m_logButton.get())
      m_chassis.chassisLog(true);
    else
      m_chassis.chassisLog(false);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    m_chassis.drive(0, 0);
  }
}
