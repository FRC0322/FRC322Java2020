/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;

public class RunShooter extends CommandBase {
  private final Shooter m_shooter;
  private final double m_speed;
  private final JoystickButton m_logButton;
  /**
   * Creates a new RunShooter.
   */
  public RunShooter(Shooter shooter, double speed, JoystickButton log) {
    m_shooter = shooter;
    m_speed = speed;
    m_logButton = log;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.run(m_speed);
    if(m_logButton.get())
      m_shooter.shooterLog(true);
    else
      m_shooter.shooterLog(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
