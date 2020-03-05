/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.Limelight.CameraMode;
import frc.robot.subsystems.LimelightCamera;

public class LimelightCameraModeControl extends CommandBase {
	private final LimelightCamera m_limelightCamera;
	private final CameraMode m_cameraMode;
	/**
	 * Creates a new LimelightModeControl.
	 */
	public LimelightCameraModeControl(LimelightCamera limelightCamera, CameraMode cameraMode) {
		m_limelightCamera = limelightCamera;
		m_cameraMode = cameraMode;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_limelightCamera);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_limelightCamera.getLimelight().setCameraMode(m_cameraMode);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
