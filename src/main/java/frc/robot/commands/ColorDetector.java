/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;

public class ColorDetector extends CommandBase {
	private ColorSensor m_colorSensor;
	private String m_color;
	private boolean m_match;
	/**
	 * Creates a new ColorDetector.
	 */
	public ColorDetector(ColorSensor colorSensor, String color) {
		m_colorSensor = colorSensor;
		if (color != null)
			m_color = color;
		else
			m_color = "none";

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_colorSensor);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		/**
		 * if (m_color.charAt(0) == m_colorSensor.getColorString().charAt(0))
		 * 	m_match = true;
		 * else
		 * 	m_match = false;
		 */
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
