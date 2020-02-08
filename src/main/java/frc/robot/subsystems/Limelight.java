/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    /**
   * The Limelight subsystem incorporates the Limelight 2+ camera.
   */
  private NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry m_tx;
  private NetworkTableEntry m_ty;
  private NetworkTableEntry m_ta;
  
  private double x,y,area;
  /**
   * Creates a new Limelight.
   */
  public Limelight() {
    super();
    // Create a network table instance for the limelight and variables for it's output.
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    m_tx = m_table.getEntry("tx");
    m_ty = m_table.getEntry("ty");
    m_ta = m_table.getEntry("ta");
  }

  public void setLightMode(int mode) {
    if (mode >= 0 && mode <= 3)
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
    else
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public void setCamMode(int mode) {
    if (mode == 0) 
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(mode);
    else
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
  }

  public void setPipeline(int pipe) {
    if (pipe >= 0 && pipe <= 9)
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipe);
    else
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
  }
  
  public void setStream(int stream) {
    if (stream >= 0 && stream <= 2)
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(stream);
    else
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
  }

  public void setSnapshot(int snap) {
    if (snap == 1)
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(snap);
    else
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //read limelight values periodically
    x = m_tx.getDouble(0.0);
    y = m_ty.getDouble(0.0);
    area = m_ta.getDouble(0.0);

    //post limelight values to the SmartDashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}
