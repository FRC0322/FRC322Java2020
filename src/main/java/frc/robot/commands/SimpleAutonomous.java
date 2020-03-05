/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;
import io.github.oblarg.oblog.annotations.Config;

public class SimpleAutonomous extends CommandBase {
	private final Chassis m_chassis;

	@Config
	private double startTime;
	/**
	 * Creates a new BasicAutonomous.
	 */
	public SimpleAutonomous(Chassis chassis) {
		m_chassis = chassis;
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
		m_chassis.drive(0.4, Constants.DEFAULT_AUTONOMOUS_HEADING);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_chassis.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if(Timer.getFPGATimestamp() < (startTime + Constants.DEFAULT_AUTONOMOUS_TIME))
			return false;
		else
			return true;
	}
}
