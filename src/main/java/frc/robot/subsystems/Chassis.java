/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Chassis extends SubsystemBase {
  /**
   * The Chassis subsystem incorporates the sensors and actuators attached to the robots chassis.
   * It includes the four drive motors (two on each side), quadrature encoders connected directly to each
   * front motor Talon SRX, and a navX-MXP IMU.
   */
  private final WPI_TalonSRX m_leftFrontMotor = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_leftRearMotor = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_rightFrontMotor = new WPI_TalonSRX(3);
  private final WPI_TalonSRX m_rightRearMotor = new WPI_TalonSRX(4);

  private final SpeedController m_leftMotors = 
    new SpeedControllerGroup(m_leftFrontMotor, m_leftRearMotor);
  private final SpeedController m_rightMotors = 
    new SpeedControllerGroup(m_rightFrontMotor, m_rightRearMotor);

  private final DifferentialDrive m_drive = 
    new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final AHRS m_imu = new AHRS();
  
  // This is a constant which is the number of Encoder ticks that matches one inch of Robot travel.
  private final int ticksPerInch = 512;
  /**
  * Creates a new Chassis.
  */
  public Chassis() {
    super();
    // Invert all four motors due to the way they're mounted. 
    m_leftMotors.setInverted(true);
    m_rightMotors.setInverted(true);

    // Setup the encoders
    m_leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    // Choose whether to invert the encoders on the Talon SRX's
    m_leftFrontMotor.setSensorPhase(false);
    m_rightFrontMotor.setSensorPhase(false);
  }

  /**
  * Modern racing game style driving for the Chassis.
  *
  * @param speed  Speed in range [-1.0,1.0]
  * @param rotation Rotation in range [-1.0,1.0]
  */
  public void drive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  /**
  * MotionMagic Autonomous driving for the Chassis.
  *
  * @param heading  Heading in degrees
  * @param distance Distance in inches
  */
  public void autoDriveStraight(double heading, double distance) {
    double ticks = distance * ticksPerInch;
    m_leftFrontMotor.set(ControlMode.MotionMagic, ticks, DemandType.AuxPID, heading);
    m_leftRearMotor.follow(m_leftFrontMotor, FollowerType.AuxOutput1);
    m_rightFrontMotor.follow(m_leftFrontMotor, FollowerType.AuxOutput1);
    m_rightRearMotor.follow(m_leftFrontMotor, FollowerType.AuxOutput1);
  }

  // This stops the robot
  public void stop() {
    m_drive.arcadeDrive(0.0, 0.0);
  }

  // The following are feed-through functions providing public access to Encoder ticks from the Talon SRX's

  @Log
  public int leftDistance() {
    return m_leftFrontMotor.getSelectedSensorPosition();
  }

  @Log
  public double leftDistanceIn() {
    return this.leftDistance() / this.ticksPerInch;
  }

  @Log
  public int rightDistance() {
    return m_rightFrontMotor.getSelectedSensorPosition();
  }

  @Log
  public double rightDistanceIn() {
    return this.rightDistance() / this.ticksPerInch;
  }

  // The following are feed-through functions providing public access to the IMU data.

  @Log
  public double getAngle() {
    return m_imu.getAngle();
  }

  @Log
  public float getPitch() {
    return m_imu.getPitch();
  }

  @Log
  public float getRoll() {
    return m_imu.getRoll();
  }

  @Log
  public float getYaw() {
    return m_imu.getYaw();
  }

  @Log
  public float getWorldLinearAccelX() {
    return m_imu.getWorldLinearAccelX();
  }

  @Log
  public float getWorldLinearAccelY() {
    return m_imu.getWorldLinearAccelY();
  }

  @Log
  public float getWorldLinearAccelZ() {
    return m_imu.getWorldLinearAccelZ();
  }

  @Log
  public double getRate() {
    return m_imu.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
