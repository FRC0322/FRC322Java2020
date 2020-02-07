/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Feeder;

public class RunFeeder extends CommandBase {
  private final Feeder m_feeder;
  private final DoubleSupplier m_speed;
  private final JoystickButton m_logButton;
  /**
   * Creates a new RunFeeder.
   */
  public RunFeeder(Feeder feeder, DoubleSupplier speed, JoystickButton log) {
    m_feeder = feeder;
    m_speed = speed;
    m_logButton = log;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feeder.run(m_speed.getAsDouble());
    if(m_logButton.get())
      m_feeder.feederLog(true);
    else
      m_feeder.feederLog(false);
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
