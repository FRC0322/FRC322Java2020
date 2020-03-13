/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fireteam322.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.fireteam322.frc.robot.subsystems.Feeder;
import com.fireteam322.frc.robot.subsystems.Intake;
import com.fireteam322.frc.robot.subsystems.Shooter;

public class ManipulatorLog extends CommandBase {
	/**
	 * Creates a new ManipulatorLog.
	 */
	public final Shooter m_shooter;
	public final Feeder m_feeder;
	public final Intake m_intake;
	public final JoystickButton m_logButton;

	public ManipulatorLog(Shooter shooter, Feeder feeder, Intake intake, JoystickButton logButton) {
		m_shooter = shooter;
		m_feeder = feeder;
		m_intake = intake;
		m_logButton = logButton;
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_logButton.get()) {
			m_shooter.shooterLog(true);
			m_feeder.feederLog(true);
			m_intake.intakeLog(true);
		}
		else {
			m_shooter.shooterLog(false);
			m_feeder.feederLog(false);
			m_intake.intakeLog(false);
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
