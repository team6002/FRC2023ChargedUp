// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LimeLightConstants;

public class SUB_LimeLight extends SubsystemBase {
  SUB_Blinkin m_blinkin;
  double[] target1 = LimeLightConstants.kTarget1Constants;
  double[] target2 = LimeLightConstants.kTarget2Constants;
  double[] target3 = LimeLightConstants.kTarget3Constants;
  double[] target4 = LimeLightConstants.kTarget4Constants;
  double[] target5 = LimeLightConstants.kTarget5Constants;
  double[] target6 = LimeLightConstants.kTarget6Constants;
  double[] target7 = LimeLightConstants.kTarget7Constants;
  double[] target8 = LimeLightConstants.kTarget8Constants;
  SUB_FiniteStateMachine m_finiteStateMachine;
  public SUB_LimeLight(SUB_Blinkin p_blinkin, SUB_FiniteStateMachine p_finiteStateMachine) {
    m_blinkin = p_blinkin;
    m_finiteStateMachine = p_finiteStateMachine;
  }
  private double[] dv = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public double[] botpose;
  public double wanted_x;
  public double wanted_y;
  public double wanted_z;
  public double wanted_pitch;

  public CommandBase exampleMethodCommand() {

    return runOnce(
        () -> {
        });
  }

  @Override
  public void periodic() {
    // botpose =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(dv);
    botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(dv);
    // if(m_finiteStateMachine.getState() == RobotState.SCORING){  
    //   if(hasTarget()){
    //     m_blinkin.setHasTarget();
    //   }
    //   else{
    //     m_blinkin.setHasNoTarget();
    //   }
    // }
    telemetry();
  }

  public void telemetry(){
    SmartDashboard.putBoolean("DO YOU SEE ANYTHING ", hasTarget());
    if(hasTarget()){
      SmartDashboard.putNumber("target ID", getTargetID());
      SmartDashboard.putNumber("target x", getTargetX());
      SmartDashboard.putNumber("target Y", getTargetY());
      SmartDashboard.putNumber("target Z", getTargetZ());
      SmartDashboard.putNumber("target pitch", getTargetPitch());
      SmartDashboard.putNumber("target yaw", getTargetYaw());
      SmartDashboard.putNumberArray("botpose", botpose);
    }
  }

  @Override
  public void simulationPeriodic() {
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
    // return Units.metersToInches(-botpose[2]);
    return -botpose[2];
  }

  public double getTargetY(){
    // return Units.metersToInches(botpose[0]);
    return botpose[0];
  }

  public double getTargetZ(){
    return Units.metersToInches(botpose[1]);
    // return botpose[1];
  }

  public double getTargetPitch(){
    return botpose[3];
  }

  public double getTargetYaw(){
    return -botpose[4];
  }

  public double getWantedX(){
    return wanted_x;
  }

  public double getWantedY(){
    return wanted_y;
  }
  
  public double getWantedZ(){
    return wanted_z;
  }

  public double getWantedPitch(){
    return wanted_pitch;
  }

  public void setWantedX(double p_wanted_x){
     wanted_x = p_wanted_x;
  }

  public void setWantedY(double p_wanted_y){
    wanted_y = p_wanted_y;
  }
  
  public void setWantedZ(double p_wanted_z){
    wanted_z = p_wanted_z;
  }

  public void setWantedPitch(double p_wanted_pitch){
    wanted_pitch = p_wanted_pitch;
  }

  
}