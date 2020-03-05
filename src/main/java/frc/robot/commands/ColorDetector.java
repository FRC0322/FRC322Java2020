/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensor;

public class ColorDetector extends CommandBase {
	private ColorSensor m_colorSensor;
	/**
	 * Creates a new ColorDetector.
	 */
	public ColorDetector(ColorSensor colorSensor) {
		m_colorSensor = colorSensor;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_colorSensor);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_colorSensor.setMatchStatus(false);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (Constants.DS.getGameSpecificMessage() != null && Constants.DS.getGameSpecificMessage() != "") {
			var color = Constants.DS.getGameSpecificMessage();
			if (m_colorSensor.getColorString() != null && m_colorSensor.getColorString() != "") {
				if ((color.charAt(0) == 'b' && m_colorSensor.getColorString().charAt(0) == 'r')
				    || (color.charAt(0) == 'y' && m_colorSensor.getColorString().charAt(0) == 'g')
				    || (color.charAt(0) == 'g' && m_colorSensor.getColorString().charAt(0) == 'y')
				    || (color.charAt(0) == 'r' && m_colorSensor.getColorString().charAt(0) == 'b'))
					m_colorSensor.setMatchStatus(true);
				else
					m_colorSensor.setMatchStatus(false);
			}
			else {
				m_colorSensor.setMatchStatus(false);
			}
		}
		else {
			m_colorSensor.setMatchStatus(false);
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
