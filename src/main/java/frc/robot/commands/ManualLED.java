/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LED;

public class ManualLED extends CommandBase {
  private final LED m_led; 
  private double m_red, m_blue, m_green, m_blinkRate;
  /**
   * Creates a new AutomaticLED.
   */
  public ManualLED(LED led, double red, double green, double blue, double blinkRate) {
    m_led = led;
    m_red = red;
    m_green = green;
    m_blue = blue;
    m_blinkRate = blinkRate;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      m_led.setRGB(1.0, 1.0, 1.0, 0.0);
    }
    catch(Exception e) {
      System.out.println("Something LED related has failed.");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      m_led.setRGB(m_red, m_blue, m_green, m_blinkRate);
    }
    catch(Exception e) {
      System.out.println("Something LED related has failed.");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      m_led.setRGB(1.0, 1.0, 1.0, 0.0);
    }
    catch(Exception e) {
      System.out.println("Something LED related has failed.");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
