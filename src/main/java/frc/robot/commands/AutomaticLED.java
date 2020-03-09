/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.AddressableLEDs;
import frc.robot.subsystems.LED;

public class AutomaticLED extends CommandBase {
	private final LED m_led;
	private final AddressableLEDs m_addressableLEDs;
	/**
	 * Creates a new AutomaticLED.
	 */
	public AutomaticLED(LED led, AddressableLEDs addressableLEDs) {
		m_led = led;
		m_addressableLEDs = addressableLEDs;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_led);
		addRequirements(m_addressableLEDs);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_led.setRGB(Color.kWhite, Constants.DEFAULT_BLINK_RATE);
		for(var i = 0; i < m_addressableLEDs.getLength(); i++)
			m_addressableLEDs.setLED(i, Color.kWhite);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_led.automaticLEDSetter();
		m_addressableLEDs.automaticLEDSetter();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_led.setRGB(Color.kWhite, Constants.DEFAULT_BLINK_RATE);
		for(var i = 0; i < m_addressableLEDs.getLength(); i++)
			m_addressableLEDs.setLED(i, Color.kWhite);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
