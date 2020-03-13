/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fireteam322.frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.fireteam322.frc.robot.Constants;

public class LED extends SubsystemBase {
	private final CANifier m_ledControlCANifier;
	private double m_startTime;
	/**
	 * Creates a new LED.
	 */
	public LED() {
		super();
		m_ledControlCANifier = new CANifier(0);
		m_startTime = 0.0;
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

	public void setRGB(Color color, double blinkRate) {
		if(blinkRate <= 0.1) {
			m_ledControlCANifier.setLEDOutput(color.red, LEDChannel.LEDChannelA);
			m_ledControlCANifier.setLEDOutput(color.green, LEDChannel.LEDChannelB);
			m_ledControlCANifier.setLEDOutput(color.blue, LEDChannel.LEDChannelC);
		}
		else if(m_startTime == 0.0 && blinkRate > 0.1)
			m_startTime = Timer.getFPGATimestamp();
		else if(((Timer.getFPGATimestamp()) < (m_startTime + blinkRate)) && blinkRate > 0.1) {
			m_ledControlCANifier.setLEDOutput(color.red, LEDChannel.LEDChannelA);
			m_ledControlCANifier.setLEDOutput(color.green, LEDChannel.LEDChannelB);
			m_ledControlCANifier.setLEDOutput(color.blue, LEDChannel.LEDChannelC);
		}
		else if((Timer.getFPGATimestamp() < (m_startTime + (blinkRate * 2))) && blinkRate > 0.1) {
			m_ledControlCANifier.setLEDOutput(Color.kBlack.red, LEDChannel.LEDChannelA);
			m_ledControlCANifier.setLEDOutput(Color.kBlack.green, LEDChannel.LEDChannelB);
			m_ledControlCANifier.setLEDOutput(Color.kBlack.blue, LEDChannel.LEDChannelC);
		}
		else
			m_startTime = 0.0;
	}

	public void automaticLEDSetter() {
		var blinkRate = 0.0;
		var color = Color.kBlack;
		if(Constants.DS.isDisabled()) blinkRate = Constants.DISABLED_BLINK_RATE;
		else if(Constants.DS.isAutonomous()) blinkRate = Constants.AUTONOMOUS_BLINK_RATE;
		else if(Constants.DS.isOperatorControl()) blinkRate = Constants.TELOP_BLINK_RATE;
		else blinkRate = 0.0;

		if(Constants.DS.getAlliance() == DriverStation.Alliance.Red) {
			//color = Color.kFirstRed;
			color = Color.kRed;
		}
		else if(Constants.DS.getAlliance() == DriverStation.Alliance.Blue) {
			color = Color.kFirstBlue;
		}
		else if(Constants.DS.getAlliance() == DriverStation.Alliance.Invalid) {
			color = Color.kKhaki;
		}
		else {
			color = Color.kDarkMagenta;
		}

		setRGB(color, blinkRate);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
