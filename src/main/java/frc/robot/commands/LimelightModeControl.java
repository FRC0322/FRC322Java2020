/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class LimelightModeControl extends CommandBase {
  private final Limelight m_limelight;
  private final int m_mode;
  /**
   * Creates a new LimelightLEDOff.
   */
  public LimelightModeControl(Limelight limelight, int mode) {
    m_limelight = limelight;
    m_mode = mode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setCamMode(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limelight.setLightMode(m_mode);
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
