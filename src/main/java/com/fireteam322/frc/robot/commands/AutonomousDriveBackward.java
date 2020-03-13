/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fireteam322.frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.fireteam322.frc.robot.Constants;
import com.fireteam322.frc.robot.subsystems.Chassis;

public class AutonomousDriveBackward extends CommandBase {
	private final Chassis m_chassis;

	private final DoubleSupplier m_autonSpeed, m_autonHeading;

	private double startTime;
	/**
	 * Creates a new SimpleAutonomous.
	 */
	public AutonomousDriveBackward(Chassis chassis) {
		m_chassis = chassis;
		m_autonSpeed = ()->Constants.DEFAULT_AUTONOMOUS_SPEED;
		m_autonHeading = ()->Constants.DEFAULT_AUTONOMOUS_HEADING;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_chassis);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		startTime = Timer.getFPGATimestamp();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_chassis.drive(-m_autonSpeed.getAsDouble(), m_autonHeading.getAsDouble());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_chassis.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if(Timer.getFPGATimestamp() < (startTime + Constants.DEFAULT_AUTONOMOUS_TIME)) {
			return false;
		}
		else {
			return true;
		}
	}
}
