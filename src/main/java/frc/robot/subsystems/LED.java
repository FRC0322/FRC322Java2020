/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Robot;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.LEDChannel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private final CANifier m_ledControlCANifier;
  private double m_red, m_blue, m_green;
  private long m_blinkRate;
  /**
   * Creates a new LED.
   */
  public LED() {
    super();
    m_ledControlCANifier = new CANifier(0);
    m_red = m_blue = m_green = 0.0;
    m_blinkRate = 0;
  }

  public void setRGB(double redIntensity, double greenIntensity, double blueIntensity, long blinkRate) throws InterruptedException {
  	m_ledControlCANifier.setLEDOutput(redIntensity, LEDChannel.LEDChannelA);
    m_ledControlCANifier.setLEDOutput(greenIntensity, LEDChannel.LEDChannelB);
    m_ledControlCANifier.setLEDOutput(blueIntensity, LEDChannel.LEDChannelC);
    Thread.sleep(blinkRate);
    m_ledControlCANifier.setLEDOutput(0.0, LEDChannel.LEDChannelA);
    m_ledControlCANifier.setLEDOutput(0.0, LEDChannel.LEDChannelB);
    m_ledControlCANifier.setLEDOutput(0.0, LEDChannel.LEDChannelC);
    Thread.sleep(blinkRate);
  }

  public void automaticLEDSetter() throws InterruptedException {
   	if(Robot.DS.isDisabled())             m_blinkRate = 500;
   	else if(Robot.DS.isAutonomous())      m_blinkRate = 100;
   	else if(Robot.DS.isOperatorControl()) m_blinkRate = 250;
   	else                                  m_blinkRate = 0;
  
    if(Robot.DS.getAlliance() == DriverStation.Alliance.Red) {
   		m_red = 1.0;
   		m_green = 0.0;
   		m_blue = 0.0;
   	}
   	else if(Robot.DS.getAlliance() == DriverStation.Alliance.Blue) {
   		m_red = 0.0;
   		m_green = 0.0;
   		m_blue = 1.0;
   	}
   	else if(Robot.DS.getAlliance() == DriverStation.Alliance.Invalid) {
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
