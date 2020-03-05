/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class RobotPower extends SubsystemBase implements Loggable {
	/**
	 * Creates a new RobotPower.
	 */
	private final PowerDistributionPanel m_pdp = new PowerDistributionPanel(Constants.PDP_CHANNEL);
	public RobotPower() {
		super();

		m_pdp.resetTotalEnergy();
	}

	public double getCurrent(int channel) {
		return m_pdp.getCurrent(channel);
	}

	public double getVoltage() {
		return m_pdp.getVoltage();
	}

	public double getTotalCurrent() {
		return m_pdp.getTotalCurrent();
	}

	@Log(name = "PDP Temp", tabName = "Debugger",   columnIndex = 8, rowIndex = 8)
	public double getTemperature() {
		return m_pdp.getTemperature();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
