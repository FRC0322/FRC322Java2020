/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fireteam322.frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.fireteam322.frc.robot.subsystems.Shooter;

public class RunShooter extends CommandBase {
	private final Shooter m_shooter;
	private final DoubleSupplier m_speed;
	/**
	 * Creates a new RunShooter.
	 */
	public RunShooter(Shooter shooter, DoubleSupplier speed) {
		m_shooter = shooter;
		m_speed = speed;
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
		m_shooter.run(m_speed.getAsDouble());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		if (!interrupted)
			m_shooter.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
