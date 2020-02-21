/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.Limelight.CameraMode;
import frc.robot.utilities.Limelight.LightMode;
import io.github.oblarg.oblog.annotations.Log;

public class LimelightCamera extends SubsystemBase {
	/**
	 * The Limelight subsystem incorporates the Limelight 2+ camera.
	 */
	private final Limelight m_limelight;
	private final HttpCamera m_limelightFeed;

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

		// Activate an HttpCamera for the Limelight
		m_limelightFeed = new HttpCamera("Limelight Camera", "http://10.3.22.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
		//CameraServer.getInstance().startAutomaticCapture(m_limelightFeed);
	}

	/**
	 * This method exists to pass the Limelight object to other classes.
	 * @return Returns a Limelight object.
	 */
	public Limelight getLimelight() {
		return m_limelight;
	}

	@Log.Dial(min = -30.0, max = 30.0, name = "Limelight TX", tabName = "Autonomous",
		  rowIndex = 0, columnIndex = 32)
	@Log.Dial(min = -30.0, max = 30.0, name = "Limelight TX", tabName = "Driver",
		  rowIndex = 32, columnIndex = 4)
	@Log.Dial(min = -30.0, max = 30.0, name = "Limelight TX", tabName = "Debugger",
		  rowIndex = 0, columnIndex = 32)
	private double getTX() {
		return m_limelight.getTX();
	}

	@Log.Dial(min = -21.0, max = 21.0, name = "Limelight TY", tabName = "Autonomous",
		  rowIndex = 4, columnIndex = 32)
	@Log.Dial(min = -21.0, max = 21.0, name = "Limelight TY", tabName = "Driver",
		  rowIndex = 32, columnIndex = 8)
	@Log.Dial(min = -21.0, max = 21.0, name = "Limelight TY", tabName = "Debugger",
		  rowIndex = 4, columnIndex = 32)
	private double getTY() {
		return m_limelight.getTY();
	}

	@Log.Dial(min = 0.0, max = 100.0, name = "Limelight TA", tabName = "Autonomous",
		  rowIndex = 0, columnIndex = 36)
	@Log.Dial(min = 0.0, max = 100.0, name = "Limelight TA", tabName = "Driver",
		  rowIndex = 0, columnIndex = 32)
	@Log.Dial(min = 0.0, max = 100.0, name = "Limelight TA", tabName = "Debugger",
		  rowIndex = 0, columnIndex = 36)
	private double getTA() {
		return m_limelight.getTA();
	}

	/**
	 * This method returns the Limelight HttpCamera feed.
	 * @return Returns an HttpCamera feed.
	 */
	@Log.CameraStream(name = "Limelight Camera", tabName = "Driver",
			  showControls = false, showCrosshairs = false, rowIndex = 8, columnIndex = 12)
	@Log.CameraStream(name = "Limelight Camera", tabName = "Debugger",
			  showControls = false, showCrosshairs = false, rowIndex = 8, columnIndex = 12)
	public HttpCamera getLimelightFeed() {
		return m_limelightFeed;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
