/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutomaticLED;
import frc.robot.commands.BasicAutonomous;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.LimelightLEDControl;
import frc.robot.commands.LimelightModeControl;
import frc.robot.commands.RunFeeder;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.F310Controller;
import frc.robot.utilities.RumblePad2;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chassis m_chassis = new Chassis();
  private final Feeder m_feeder = new Feeder();
  private final LED m_led = new LED();
  private final Shooter m_shooter = new Shooter();
  private final Limelight m_limelight = new Limelight();
  
  private final F310Controller m_driveStick = new F310Controller(Constants.DRIVE_STICK);
  private final F310Controller m_manipulatorStick = new F310Controller(Constants.MANIPULATOR_STICK);
  private final RumblePad2 m_debuggerStick = new RumblePad2(Constants.DEBUGGER_STICK);
  
  private final JoystickButton m_brakeButton = new JoystickButton(m_driveStick, Constants.BRAKE_BUTTON);
  private final JoystickButton m_logButton = new JoystickButton(m_driveStick, Constants.LOG_BUTTON);
  //private final JoystickButton m_logButton = new JoystickButton(m_debuggerStick, Constants.LOG_BUTTON);
  private final JoystickButton m_feederButton = new JoystickButton(m_manipulatorStick, Constants.FEEDER_BUTTON);
  private final JoystickButton m_feederReverseButton = new JoystickButton(m_manipulatorStick, Constants.FEEDER_REVERSE_BUTTON);
  private final JoystickButton m_shooterButton = new JoystickButton(m_manipulatorStick, Constants.SHOOTER_BUTTON);
  private final JoystickButton m_shooterReverseButton = new JoystickButton(m_manipulatorStick, Constants.SHOOTER_REVERSE_BUTTON);
  private final JoystickButton m_manipulatorLogButton = new JoystickButton(m_driveStick, Constants.LOG_BUTTON);
  private final JoystickButton m_visionModeButton = new JoystickButton(m_driveStick, Constants.VISION_MODE_BUTTON);
  private final JoystickButton m_driverModeButton = new JoystickButton(m_driveStick, Constants.DRIVER_MODE_BUTTON);
  private final JoystickButton m_LEDOnButton = new JoystickButton(m_manipulatorStick, Constants.LED_ON_BUTTON);
  private final JoystickButton m_LEDBlinkButton = new JoystickButton(m_manipulatorStick, Constants.LED_BLINK_BUTTON);
  private final JoystickButton m_LEDOffButton = new JoystickButton(m_manipulatorStick, Constants.LED_OFF_BUTTON);
  private final JoystickButton m_LEDDefaultButton = new JoystickButton(m_manipulatorStick, Constants.LED_DEFAULT_BUTTON);
  
  private final BasicAutonomous m_autoCommand = new BasicAutonomous(m_chassis);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Assign default commands
    m_chassis.setDefaultCommand(new DriveWithJoystick(
      () -> m_driveStick.getTriggerAxis(Hand.kRight) - m_driveStick.getTriggerAxis(Hand.kLeft),
      () -> -(m_driveStick.getX(Hand.kLeft)), m_chassis, m_brakeButton, m_logButton));
  
    // Default command for debugging purposes
    //m_chassis.setDefaultCommand(new DriveWithJoystick(
    //  () -> -(m_debuggerStick.getY(Hand.kRight), () -> -(m_debuggerStick.getX(Hand.kRight)),
    //  m_chassis, m_brakeButton, m_logButton)));

    m_feeder.setDefaultCommand(new RunFeeder(m_feeder, m_manipulatorStick.getY(Hand.kLeft),
      m_manipulatorLogButton));

    //m_led.setDefaultCommand(new AutomaticLED(m_led));

    m_shooter.setDefaultCommand(new RunShooter(m_shooter, m_manipulatorStick.getY(Hand.kRight),
      m_manipulatorLogButton));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_visionModeButton.whileActiveOnce(new LimelightModeControl(m_limelight, 0));
    m_driverModeButton.whileActiveOnce(new LimelightModeControl(m_limelight, 1));

    m_LEDDefaultButton.whileActiveOnce(new LimelightLEDControl(m_limelight, 0));
    m_LEDOffButton.whileActiveOnce(new LimelightLEDControl(m_limelight, 1));
    m_LEDBlinkButton.whileActiveOnce(new LimelightLEDControl(m_limelight, 2));
    m_LEDOnButton.whileActiveOnce(new LimelightLEDControl(m_limelight, 3));

    /**
     * This block will make the feeder and shooter run on button presses rather than by joystick axis.
     * We need to figure out how fast we want the motors running first though.
     *  m_feederButton.toggleWhenActive(new RunFeeder(m_feeder, Constants.FEEDER_SPEED,
     *    m_manipulatorLogButton), true);
     *
     *  m_feederReverseButton.toggleWhenActive(new RunFeeder(m_feeder, Constants.FEEDER_REVERSE_SPEED,
     *    m_manipulatorLogButton), true);
     *
     *  m_shooterButton.toggleWhenActive(new RunShooter(m_shooter, Constants.SHOOTER_SPEED,
     *    m_manipulatorLogButton), true); 
     *
     *  m_shooterReverseButton.toggleWhenActive(new RunShooter(m_shooter, Constants.SHOOTER_REVERSE_SPEED,
     *    m_manipulatorLogButton), true); 
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand;
  }
}
