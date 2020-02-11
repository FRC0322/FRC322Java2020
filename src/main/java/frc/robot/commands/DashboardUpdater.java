/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Limelight;

public class DashboardUpdater extends CommandBase {
  private final Dashboard m_dashboard;
  private final Limelight m_limelight;
  /**
   * Creates a new DashboardUpdater.
   */
  public DashboardUpdater(Dashboard dashboard, Limelight limelight) {
    m_dashboard = dashboard;
    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dashboard);
    addRequirements(m_limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.DS.isAutonomous()) {
      m_dashboard.getAutonomousTab().addNumber("Limelight Tx", () -> m_limelight.getTx());
      m_dashboard.getAutonomousTab().addNumber("Limelight Ty", () -> m_limelight.getTy());
      m_dashboard.getAutonomousTab().addNumber("Limelight Ta", () -> m_limelight.getTa());
      Shuffleboard.selectTab("Autonomous");
    }
    else if (Constants.DS.isOperatorControl()) {
      m_dashboard.getDriverTab().addNumber("Limelight Tx", () -> m_limelight.getTx());
      m_dashboard.getDriverTab().addNumber("Limelight Ty", () -> m_limelight.getTy());
      m_dashboard.getDriverTab().addNumber("Limelight Ta", () -> m_limelight.getTa());
      Shuffleboard.selectTab("Driver");
    }
    else {
      m_dashboard.getDebuggerTab().addNumber("Limelight Tx", () -> m_limelight.getTx());
      m_dashboard.getDebuggerTab().addNumber("Limelight Ty", () -> m_limelight.getTy());
      m_dashboard.getDebuggerTab().addNumber("Limelight Ta", () -> m_limelight.getTa());
      Shuffleboard.selectTab("Debugger");
    }
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
