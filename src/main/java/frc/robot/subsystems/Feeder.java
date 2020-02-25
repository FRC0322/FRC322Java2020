/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
	//The Feeder is our ground level ball intake.
	//The we'll switch lines when we attach the Falcon 500
	//private final WPI_TalonFX m_feederMotor = new WPI_TalonFX(Constants.FEEDER_MOTOR);
	private final WPI_TalonSRX m_feederMotor = new WPI_TalonSRX(Constants.FEEDER_MOTOR);
	/**
	 * Creates a new Feeder.
	 */
	public Feeder() {
		super();

		// Set the feeder motor to Brake mode to keep the feeder from running when we don't want it to.
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

	// This method will output data from various logging methods.
	public void feederLog(boolean logging) {
		if(logging) {
			System.out.println("There's nothing to see here. (Yet.)");
		}
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
