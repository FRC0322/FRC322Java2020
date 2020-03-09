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
	@Log
	private final PowerDistributionPanel m_pdp = new PowerDistributionPanel(Constants.PDP_CHANNEL);
	public RobotPower() {
		super();

		m_pdp.resetTotalEnergy();
	}

	/**
	 * Reset the total energy to 0.
	 */
	public void resetTotalEnergy() {
		m_pdp.resetTotalEnergy();
	}

	/**
	 * Query the input voltage of the PDP.
	 *
	 * @return The voltage of the PDP in volts
	 */
	public double getVoltage() {
		return m_pdp.getVoltage();
	}

	/**
	 * Query the temperature of the PDP.
	 *
	 * @return The temperature of the PDP in degrees Celsius
	 */
	@Log(name = "PDP Temp", tabName = "Debugger",   columnIndex = 8, rowIndex = 8)
	public double getTemperature() {
		return m_pdp.getTemperature();
	}

	/**
	 * Query the current of a single channel of the PDP.
	 *
	 * @return The current of one of the PDP channels (channels 0-15) in Amperes
	 */
	public double getCurrent(int channel) {
		return m_pdp.getCurrent(channel);
	}

	/**
	 * Query the current of all monitored PDP channels (0-15).
	 *
	 * @return The current of all the channels in Amperes
	 */
	public double getTotalCurrent() {
		return m_pdp.getTotalCurrent();
	}

	/**
	 * Query the total power drawn from the monitored PDP channels.
	 *
	 * @return the total power in Watts
	 */
	public double getTotalPower() {
		return m_pdp.getTotalPower();
	}

	/**
	 * Query the total energy drawn from the monitored PDP channels.
	 *
	 * @return the total energy in Joules
	 */
	public double getTotalEnergy() {
		return m_pdp.getTotalEnergy();
	}

	/**
	 * Clear all PDP sticky faults.
	 */
	public void clearStickyFaults() {
		m_pdp.clearStickyFaults();
	}

	@Log(name = "Overcurrent", tabName = "Debugger", columnIndex = 8, rowIndex = 7)
	public boolean currentWarning() {
		boolean warning;
		if (getTotalCurrent() <= 120.0) warning = false;
		else warning = true;
		return warning;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
