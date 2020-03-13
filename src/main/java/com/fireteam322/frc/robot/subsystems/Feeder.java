/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fireteam322.frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.fireteam322.frc.robot.Constants;

public class Feeder extends SubsystemBase {
	// The Feeder moves balls from the intake to the shooter.
	private final WPI_TalonSRX m_feederMotor = new WPI_TalonSRX(Constants.FEEDER_MOTOR);
	/**
	 * Creates a new Feeder.
	 */
	public Feeder() {
		super();

		// Set the inversion of the feeder motor.
		m_feederMotor.setInverted(true);

		// Set the feeder motor to Brake mode to keep balls from moving when we don't want them to.
		m_feederMotor.setNeutralMode(NeutralMode.Brake);
	}

	// This method stops the feeder.
	public void stop() {
		m_feederMotor.stopMotor();
	}

	// This method runs the feeder.
	public void run(double speed) {
		m_feederMotor.set(speed);
	}

	public void feederLog(boolean logging) {
		if (logging) {
			System.out.println("Nothing here, yet.");
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
