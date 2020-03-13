/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.fireteam322.frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class RearCamera extends SubsystemBase implements Loggable {
	private static UsbCamera rearCameraServer;
	/**
	 * Creates a new RearCamera.
	 */
	public RearCamera() {
		//Setup Camera
		rearCameraServer = CameraServer.getInstance().startAutomaticCapture();
	}

	public void setResolution(int width, int height) {
		rearCameraServer.setResolution(width, height);
	}


	/**
	 * This method returns the Limelight HttpCamera feed.
	 * @return Returns a UsbCamera feed.
	 */
	@Log.CameraStream(name = "Rear Camera", tabName = "Driver",
			  showControls = false, showCrosshairs = true, columnIndex = 6, rowIndex = 2)
	@Log.CameraStream(name = "Rear Camera", tabName = "Autonomous",
			  showControls = true, showCrosshairs = true, columnIndex = 6, rowIndex = 2)
	@Log.CameraStream(name = "Rear Camera", tabName = "Debugger",
			  showControls = true, showCrosshairs = true, columnIndex = 6, rowIndex = 2)
	public UsbCamera getCameraFeed() {
		return rearCameraServer;
	}

	public void setFPS(int fps) {
		rearCameraServer.setFPS(fps);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
