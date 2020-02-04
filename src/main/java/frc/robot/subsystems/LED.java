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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
  private final CANifier m_ledControlCANifier;
  private double m_red, m_blue, m_green, m_startTime;
  private long m_blinkRate;
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

  public void setRGB(double redIntensity, double greenIntensity, double blueIntensity, long blinkRate) throws InterruptedException {
    if(m_startTime == 0.0)
      m_startTime = Timer.getFPGATimestamp();
    else if((m_startTime * 1000) < ((m_startTime * 1000) + blinkRate)) {
      m_ledControlCANifier.setLEDOutput(redIntensity, LEDChannel.LEDChannelA);
      m_ledControlCANifier.setLEDOutput(greenIntensity, LEDChannel.LEDChannelB);
      m_ledControlCANifier.setLEDOutput(blueIntensity, LEDChannel.LEDChannelC);
    }
    else if((m_startTime * 1000) < ((m_startTime * 1000) + (blinkRate * 2))) {
      m_ledControlCANifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
      m_ledControlCANifier.setLEDOutput(0.0, LEDChannel.LEDChannelB);
      m_ledControlCANifier.setLEDOutput(0.0, LEDChannel.LEDChannelC);
    }
    else
      m_startTime = 0.0;
  }

  public void automaticLEDSetter() throws InterruptedException {
   	if(Constants.DS.isDisabled())             m_blinkRate = 500;
   	else if(Constants.DS.isAutonomous())      m_blinkRate = 100;
   	else if(Constants.DS.isOperatorControl()) m_blinkRate = 250;
   	else                                      m_blinkRate = 0;
  
    if(Constants.DS.getAlliance() == DriverStation.Alliance.Red) {
   		m_red = 1.0;
   		m_green = 0.0;
   		m_blue = 0.0;
   	}
   	else if(Constants.DS.getAlliance() == DriverStation.Alliance.Blue) {
   		m_red = 0.0;
   		m_green = 0.0;
   		m_blue = 1.0;
   	}
   	else if(Constants.DS.getAlliance() == DriverStation.Alliance.Invalid) {
   		m_red = 1.0;
   		m_green = 0.0;
   		m_blue = 1.0;
   	}
   	else {
   		m_red = 0.0;
   		m_green = 1.0;
   		m_blue = 0.0;
   	}
  
     setRGB(m_red, m_green, m_blue, m_blinkRate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
