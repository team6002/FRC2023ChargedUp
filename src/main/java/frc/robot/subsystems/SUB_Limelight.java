// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class SUB_Limelight extends SubsystemBase {
  public SUB_Limelight() {
    /*
    * 0 - Standard - Side-by-side streams if a webcam is attached to Limelight
    * 1 - PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
    * 2 - PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
    */
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(CameraConstants.kLimelightIndex);
  }

  private double[] dv = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public double[] botpose;

  @Override
  public void periodic() {
    botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(dv);

    telemetry();
  }

  public void telemetry(){
    SmartDashboard.putBoolean("DO YOU SEE ANYTHING ", hasTarget());

    if (hasTarget()) {
      SmartDashboard.putNumber("target ID", getTargetID());
      SmartDashboard.putNumber("target x", getTargetX());
      SmartDashboard.putNumber("target Y", getTargetY());
      SmartDashboard.putNumber("target Z", getTargetZ());
      SmartDashboard.putNumber("target pitch", getTargetPitch());
      SmartDashboard.putNumber("target yaw", getTargetYaw());
      SmartDashboard.putNumberArray("botpose", botpose);
    }
  }

  public boolean hasTarget(){
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1){
      return true;
    }
    else{
      return false;
    }
  }

  public double getTargetID(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
  }

  public Pose2d getRobotPoseInTargetSpace() {
    System.out.println("botpose[4]= " + botpose[4]);
    System.out.println(Rotation2d.fromDegrees(botpose[4]));
    // Pose2d pose = new Pose2d(botpose[2], -botpose[0], Rotation2d.fromDegrees(0));
    Pose2d pose = new Pose2d(-botpose[2], botpose[0], Rotation2d.fromDegrees(-botpose[4]));

    return pose;
  }

  public double getTargetX(){
    return -botpose[2];
  }

  public double getTargetY(){
    return botpose[0];
  }

  public double getTargetZ(){
    return Units.metersToInches(botpose[1]);
  }

  public double getTargetPitch(){
    return botpose[3];
  }

  public double getTargetYaw(){
    return -botpose[4];
  }

  public void useMainCamera(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(CameraConstants.kLimelightIndex);
  }

  public void useSecondaryCamera(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(CameraConstants.kDriveCamIndex);
  }
}