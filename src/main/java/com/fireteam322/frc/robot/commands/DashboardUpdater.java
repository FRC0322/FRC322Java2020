/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fireteam322.frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.fireteam322.frc.robot.Constants;
import com.fireteam322.frc.robot.subsystems.Dashboard;

public class DashboardUpdater extends CommandBase {
	private final Dashboard m_dashboard;
	/**
	 * Creates a new DashboardUpdater.
	 */
	public DashboardUpdater(Dashboard dashboard) {
		m_dashboard = dashboard;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_dashboard);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (Constants.DS.isAutonomous()) {
			Shuffleboard.selectTab("Autonomous");
		}
		else if (Constants.DS.isOperatorControl()) {
			Shuffleboard.selectTab("Driver");
		}
		else {
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
