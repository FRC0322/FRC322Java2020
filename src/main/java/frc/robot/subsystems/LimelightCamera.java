/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Limelight.CameraMode;
import frc.robot.utilities.Limelight.LightMode;

public class LimelightCamera extends SubsystemBase {
	/**
	 * The Limelight subsystem incorporates the Limelight 2+ camera.
	 */
	private Limelight m_limelight;
	private HttpCamera limelightFeed;

	/**
	 * Creates a new Limelight.
	 */
	public LimelightCamera() {
		super();
		m_limelight = new Limelight();

		// Set the camera to Driver Mode
		m_limelight.setCameraMode(CameraMode.kdriver);

		// Turn off the lights
		m_limelight.setLedMode(LightMode.kforceOff);

		// Activate a CameraServer for the Limelight
		limelightFeed = new HttpCamera("limelight", "http://10.3.22.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
		CameraServer.getInstance().startAutomaticCapture(limelightFeed);
	}

	public Limelight getLimelight() {
		return m_limelight;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
