/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
  /**
   * The Chassis subsystem incorporates the sensors and actuators attached to the robots chassis.
   * It includes the four drive motors (two on each side) and a NavX-MXP IMU.
   */
  private final WPI_TalonSRX m_leftFrontMotor = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_leftRearMotor = new WPI_TalonSRX(2);
  private final WPI_TalonSRX m_RightFrontMotor = new WPI_TalonSRX(3);
  private final WPI_TalonSRX m_RightRearMotor = new WPI_TalonSRX(4);

  private final SpeedController m_leftMotors = 
    new SpeedControllerGroup(m_leftFrontMotor, m_leftRearMotor);
  private final SpeedController m_rightMotors = 
    new SpeedControllerGroup(m_RightFrontMotor, m_RightRearMotor);

  private final DifferentialDrive m_drive = 
    new DifferentialDrive(m_leftMotors, m_rightMotors);

  private final AHRS m_imu = new AHRS();

  /**
  * Creates a new Chassis.
  */
  public Chassis() {
    super();
    m_leftMotors.setInverted(true);
    m_rightMotors.setInverted(true);
  }

  /**
  * Racing game style driving for the Chassis.
  *
  * @param left  Speed in range [-1.0,1.0]
  * @param right Speed in range [-1.0,1.0]
  */
  public void drive(double left, double right) {
    m_drive.arcadeDrive(left, right, true);
  }

  // All of the following are feed-through functions providing public access to the IMU data.

  public double getAngle() {
    return m_imu.getAngle();
  }

  public float getPitch() {
    return m_imu.getPitch();
  }

  public float getRoll() {
    return m_imu.getRoll();
  }

  public float getYaw() {
    return m_imu.getYaw();
  }

  public float getWorldLinearAccelX() {
    return m_imu.getWorldLinearAccelX();
  }

  public float getWorldLinearAccelY() {
    return m_imu.getWorldLinearAccelY();
  }

  public float getWorldLinearAccelZ() {
    return m_imu.getWorldLinearAccelZ();
  }

  public double getRate() {
    return m_imu.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
