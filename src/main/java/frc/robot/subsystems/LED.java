/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
	private final CANifier m_ledControlCANifier;
	private double m_red, m_blue, m_green, m_startTime, m_blinkRate;
	/**
	 * Creates a new LED.
	 */
	public LED() {
		super();
		m_ledControlCANifier = new CANifier(0);
		m_red = m_blue = m_green = 0.0;
		m_startTime = 0.0;
		m_blinkRate = 0;
	}

	public void setRGB(double redIntensity, double greenIntensity, double blueIntensity, double blinkRate) {
		if(blinkRate <= 0.1) {
			m_ledControlCANifier.setLEDOutput(redIntensity, LEDChannel.LEDChannelA);
			m_ledControlCANifier.setLEDOutput(greenIntensity, LEDChannel.LEDChannelB);
			m_ledControlCANifier.setLEDOutput(blueIntensity, LEDChannel.LEDChannelC);
		}
		else if(m_startTime == 0.0 && blinkRate > 0.1)
			m_startTime = Timer.getFPGATimestamp();
		else if(((Timer.getFPGATimestamp()) < (m_startTime + blinkRate)) && blinkRate > 0.1) {
			m_ledControlCANifier.setLEDOutput(redIntensity, LEDChannel.LEDChannelA);
			m_ledControlCANifier.setLEDOutput(greenIntensity, LEDChannel.LEDChannelB);
			m_ledControlCANifier.setLEDOutput(blueIntensity, LEDChannel.LEDChannelC);
		}
		else if((Timer.getFPGATimestamp() < (m_startTime + (blinkRate * 2))) && blinkRate > 0.1) {
			m_ledControlCANifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
			m_ledControlCANifier.setLEDOutput(0.0, LEDChannel.LEDChannelB);
			m_ledControlCANifier.setLEDOutput(0.0, LEDChannel.LEDChannelC);
		}
		else
			m_startTime = 0.0;
	}

	public void automaticLEDSetter() {
		if(Constants.DS.isDisabled()) m_blinkRate = Constants.DISABLED_BLINK_RATE;
		else if(Constants.DS.isAutonomous()) m_blinkRate = Constants.AUTONOMOUS_BLINK_RATE;
		else if(Constants.DS.isOperatorControl()) m_blinkRate = Constants.TELOP_BLINK_RATE;
		else m_blinkRate = 0.0;

		if(Constants.DS.getAlliance() == DriverStation.Alliance.Red) {
			m_red = Color.kFirstRed.red;
			m_green = Color.kFirstRed.green;
			m_blue = Color.kFirstRed.blue;
		}
		else if(Constants.DS.getAlliance() == DriverStation.Alliance.Blue) {
			m_red = Color.kFirstBlue.red;
			m_green = Color.kFirstBlue.green;
			m_blue = Color.kFirstBlue.blue;
		}
		else if(Constants.DS.getAlliance() == DriverStation.Alliance.Invalid) {
			m_red = Color.kGreen.red;
			m_green = Color.kGreen.green;
			m_blue = Color.kGreen.blue;
		}
		else {
			m_red = Color.kDarkMagenta.red;
			m_green = Color.kDarkMagenta.green;
			m_blue = Color.kDarkMagenta.blue;
		}

		setRGB(m_red, m_green, m_blue, m_blinkRate);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
