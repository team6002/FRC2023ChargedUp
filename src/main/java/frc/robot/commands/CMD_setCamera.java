// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Limelight;

public class CMD_setCamera extends CommandBase {
  SUB_Limelight m_cameras;
  boolean m_camera;
  public CMD_setCamera(SUB_Limelight p_cameras, boolean p_camera) {
    m_cameras = p_cameras;
    m_camera = p_camera;
  }

  @Override
  public void initialize() {
    if(m_camera){
      m_cameras.useSecondaryCamera();
    }else{
      m_cameras.useMainCamera();
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
