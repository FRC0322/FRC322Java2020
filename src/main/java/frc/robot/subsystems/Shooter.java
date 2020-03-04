/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
	//The Shooter is both our upper level ball intake and our ball output mechanism.
	private final WPI_TalonSRX m_leftShooterMotor = new WPI_TalonSRX(Constants.LEFT_SHOOTER_MOTOR);
	private final WPI_TalonSRX m_rightShooterMotor = new WPI_TalonSRX(Constants.RIGHT_SHOOTER_MOTOR);
	private final SpeedControllerGroup m_shooterMotors = new SpeedControllerGroup(m_leftShooterMotor, m_rightShooterMotor);
	/**
	 * Creates a new Shooter.
	 */
	public Shooter() {
		super();
		// Set the inversion on the shooter motors.
		m_leftShooterMotor.setInverted(false);
		m_rightShooterMotor.setInverted(false);
		m_shooterMotors.setInverted(false);

		// Set the shooter motors to Brake mode to keep the feeder from running when we don't want it to.
		m_leftShooterMotor.setNeutralMode(NeutralMode.Coast);
		m_rightShooterMotor.setNeutralMode(NeutralMode.Coast);
	}

	public void run(double speed) {
		m_shooterMotors.set(speed);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
