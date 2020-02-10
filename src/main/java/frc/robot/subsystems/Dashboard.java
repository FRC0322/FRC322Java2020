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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
  private HttpCamera limelightFeed;
  private ShuffleboardTab driverShuffleboardTab;
  private ShuffleboardTab autonomousShuffleboardTab;
  private ShuffleboardTab debuggerShuffleboardTab;
  /**
   * Creates a new Dashboard.
   */
  public Dashboard() {
    limelightFeed = new HttpCamera("limelight", "http://10.3.22.11:5800/stream.mjpg", HttpCameraKind.kMJPGStreamer);
    CameraServer.getInstance().startAutomaticCapture(limelightFeed);
    driverShuffleboardTab = Shuffleboard.getTab("Driver");
    autonomousShuffleboardTab = Shuffleboard.getTab("Autonomous");
    debuggerShuffleboardTab = Shuffleboard.getTab("Debugger");
  }

  public void setTab(int tab) {
    if (tab == 0)
      Shuffleboard.selectTab("Driver");
    else if (tab == 1)
      Shuffleboard.selectTab("Autonomous");
    else if (tab == 2)
      Shuffleboard.selectTab("Debugger");
  }

  public void driverTab() {}

  public void autonomousTab() {}

  public void debuggerTab() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
