/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	// The Intake is our ground level ball intake.
	private final WPI_TalonSRX m_intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR);
	/**
	 * Creates a new Intake.
	 */
	public Intake() {
		super();

		// Set the inversion of the intake motor.
		m_intakeMotor.setInverted(false);

		// Set the intake motor to Coast.
		m_intakeMotor.setNeutralMode(NeutralMode.Coast);
	}

	// This method stops the intake.
	public void stop() {
		m_intakeMotor.stopMotor();
	}

	// This method runs the intake.
	public void run(double speed) {
		m_intakeMotor.set(speed);
	}

	public void intakeLog(boolean logging) {
		if (logging) {
			System.out.println("Nothing here, yet.");
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
