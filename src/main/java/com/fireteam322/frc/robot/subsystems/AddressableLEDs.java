/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fireteam322.frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.fireteam322.frc.robot.Constants;

public class AddressableLEDs extends SubsystemBase {
	private final AddressableLED m_LED;
	private final AddressableLEDBuffer m_LEDBuffer;
	private Color m_ledColor;
	private int m_rainbowFirstPixelHue;
	private double m_startTime;
	/**
	 * Creates a new AddressableLEDs.
	 */
	public AddressableLEDs() {
		super();
		m_startTime = 0.0;
		m_rainbowFirstPixelHue = 0;

		m_LED = new AddressableLED(Constants.ADDRESSABLE_LED_PORT);
		m_LED.setLength(Constants.ADDRESSABLE_LED_LENGTH);
		m_LEDBuffer = new AddressableLEDBuffer(Constants.ADDRESSABLE_LED_LENGTH);
	}

	public Color getLED(int index) {
		return m_LEDBuffer.getLED(index);
	}

	public Color8Bit getLED8Bit(int index) {
		return m_LEDBuffer.getLED8Bit(index);
	}

	public int getLength() {
		return m_LEDBuffer.getLength();
	}

	public void setLED(int index, Color color) {
		m_LEDBuffer.setLED(index, color);
	}

	public void setLED(int index, Color8Bit color) {
		m_LEDBuffer.setLED(index, color);
	}

	public void setLEDRGB(int index, int r, int g, int b) {
		m_LEDBuffer.setRGB(index, r, g, b);
	}

	public void setLEDHSV(int index, int h, int s, int v) {
		m_LEDBuffer.setHSV(index, h, s, v);
	}

	public void automaticLEDSetter() {
		var blinkRate = 0.0;
		if(Constants.DS.isDisabled()) blinkRate = Constants.DISABLED_BLINK_RATE;
		else if(Constants.DS.isAutonomous()) blinkRate = Constants.AUTONOMOUS_BLINK_RATE;
		else if(Constants.DS.isOperatorControl()) blinkRate = Constants.TELOP_BLINK_RATE;
		else blinkRate = 0.0;

		if(Constants.DS.getAlliance() == DriverStation.Alliance.Red) {
			m_ledColor = Color.kFirstRed;
		}
		else if(Constants.DS.getAlliance() == DriverStation.Alliance.Blue) {
			m_ledColor = Color.kFirstBlue;
		}
		else if(Constants.DS.getAlliance() == DriverStation.Alliance.Invalid) {
			m_ledColor = Color.kKhaki;
		}
		else {
			m_ledColor = Color.kDarkMagenta;
		}

		if (blinkRate < 0.03) {
			for (var i = 0; i < this.getLength(); i++) {
				this.setLED(i, m_ledColor);
			}
		}
		else if (m_startTime == 0.0 && blinkRate >= 0.03) {
			m_startTime = Timer.getFPGATimestamp();
		}
		else if(((Timer.getFPGATimestamp()) < (m_startTime + blinkRate)) && blinkRate >= 0.03) {
			for (var j = 0; j < this.getLength(); j++) {
				this.setLED(j, m_ledColor);
			}
		}
		else if((Timer.getFPGATimestamp() < (m_startTime + (blinkRate * 2))) && blinkRate >= 0.03) {
			for (var k = 0; k < this.getLength(); k++) {
				this.setLED(k, Color.kBlack);
			}
		}
		else
			m_startTime = 0.0;
	}

	public void rainbowLED() {
		// For every pixel
		for (var i = 0; i < this.getLength(); i++) {
			// Calculate the hue - hue is easier for rainbows because the color
			// shape is a circle so only one value needs to precess
			final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_LEDBuffer.getLength())) % 180;
			// Set the value
			this.setLEDHSV(i, hue, 255, 128);
		}
		// Increase by to make the rainbow "move"
		m_rainbowFirstPixelHue += 3;
		// Check bounds
		m_rainbowFirstPixelHue %= 180;
	}

	public void StartLED() {
		m_LED.setData(m_LEDBuffer);
		m_LED.start();
	}

	public void StopLED() {
		m_LED.stop();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
