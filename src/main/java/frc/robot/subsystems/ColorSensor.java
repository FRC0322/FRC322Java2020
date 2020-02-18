/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.annotations.Log;

public class ColorSensor extends SubsystemBase {
	/**
	 * Change the I2C port below to match the connection of your color sensor
	 */
	private final I2C.Port i2cPort = I2C.Port.kOnboard;

	/**
	 * A Rev Color Sensor V3 object is constructed with an I2C port as a
	 * parameter. The device will be automatically initialized with default
	 * parameters.
	 */
	private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

	/**
	 * A Rev Color Match object is used to register and detect known colors. This can
	 * be calibrated ahead of time or during operation.
	 *
	 * This object uses a simple euclidian distance to estimate the closest match
	 * with given confidence range.
	 */
	private final ColorMatch m_colorMatcher = new ColorMatch();

	private final Color kRedTarget = ColorMatch.makeColor(Constants.RED_R, Constants.RED_G, Constants.RED_B);
	private final Color kGreenTarget = ColorMatch.makeColor(Constants.GREEN_R, Constants.GREEN_G, Constants.GREEN_B);
	private final Color kBlueTarget = ColorMatch.makeColor(Constants.BLUE_R, Constants.BLUE_G, Constants.BLUE_B);
	private final Color kYellowTarget = ColorMatch.makeColor(Constants.YELLOW_R, Constants.YELLOW_G, Constants.YELLOW_B);

	private String m_colorString = "";
	/**
	 * Creates a new ColorSensor.
	 */
	public ColorSensor() {
		m_colorMatcher.addColorMatch(kRedTarget);
		m_colorMatcher.addColorMatch(kGreenTarget);
		m_colorMatcher.addColorMatch(kBlueTarget);
		m_colorMatcher.addColorMatch(kYellowTarget);
	}

	@Log.ToString(name = "Detected Color", tabName = "Driver", columnIndex = 8, rowIndex = 8)
	@Log.ToString(name = "Detected Color", tabName = "Debugger", columnIndex = 8, rowIndex = 8)
	public String getColorString() {
		if (m_colorString != null)
			return m_colorString;
		else
			return "none";
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		Color detectedColor = m_colorSensor.getColor();

		/**
		 * Run the color match algorithm on our detected color
		 */
		ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

		if (match.color == kRedTarget)
			m_colorString = "Red";
		else if (match.color == kGreenTarget)
			m_colorString = "Green";
		else if (match.color == kBlueTarget)
			m_colorString = "Blue";
		else if (match.color == kYellowTarget)
			m_colorString = "Yellow";
		else
			m_colorString = "Unknown";
	}
}
