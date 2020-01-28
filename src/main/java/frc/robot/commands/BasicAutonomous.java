/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;
import frc.robot.Robot;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;


public class BasicAutonomous extends CommandBase {
  private final Chassis m_chassis;
  @Config
  private double distance;
  private double startingDistance;
  /**
   * Creates a new BasicAutonomous.
   */
  public BasicAutonomous(Chassis chassis) {
    m_chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingDistance = Math.min(m_chassis.leftDistanceIn(), m_chassis.rightDistanceIn());
    distance = 24.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.drive(0.5, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((startingDistance + 24.0) > Math.min(Math.abs(m_chassis.leftDistanceIn()),
      Math.abs(m_chassis.rightDistanceIn())))
        return false;
    else
      return true;
  }
}
