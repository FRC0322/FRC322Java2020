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
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import io.github.oblarg.oblog.annotations.Config;

public class ShooterAutonomous extends CommandBase {
	private final Chassis m_chassis;
	private final Feeder m_feeder;
	private final Shooter m_shooter;

	@Config
	private double startTime;
	/**
	 * Creates a new ShooterAutonomous.
	 */
	public ShooterAutonomous(Chassis chassis, Feeder feeder, Shooter shooter) {
		m_chassis = chassis;
		m_feeder = feeder;
		m_shooter = shooter;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_chassis);
		addRequirements(m_feeder);
		addRequirements(m_shooter);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		startTime = Timer.getFPGATimestamp();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (Timer.getFPGATimestamp() < (startTime + 3.0))
			m_chassis.drive(-Constants.DEFAULT_AUTONOMOUS_SPEED, Constants.DEFAULT_AUTONOMOUS_HEADING);
		else if (Timer.getFPGATimestamp() < (startTime + 3.0 + 0.5)) {
			m_chassis.drive(0.0, -Constants.DEFAULT_AUTONOMOUS_SPEED);
		}
		else if (Timer.getFPGATimestamp() < (startTime + 3.0 + 1.0 + 0.5)) {
			m_shooter.run(Constants.SHOOTER_SPEED);
		}
		else if (Timer.getFPGATimestamp() > (startTime + 3.0 + 1.0 + 0.5 + 0.75)) {
			m_shooter.run(Constants.SHOOTER_SPEED);
			m_feeder.run(Constants.FEEDER_SPEED);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_chassis.stop();
		m_shooter.stop();
		m_feeder.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if(Timer.getFPGATimestamp() < (startTime + 10.0))
			return false;
		else
			return true;
	}
}
