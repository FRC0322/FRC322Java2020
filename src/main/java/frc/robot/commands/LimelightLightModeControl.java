/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.CameraMode;
import frc.robot.subsystems.Limelight.LightMode;

public class LimelightLightModeControl extends CommandBase {
  private final Limelight m_limelight;
  private final LightMode m_lightMode;
  /**
   * Creates a new LimelightLightModeControl.
   */
  public LimelightLightModeControl(Limelight limelight, LightMode lightMode) {
    m_limelight = limelight;
    m_lightMode = lightMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_limelight.setLedMode(m_lightMode);
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
