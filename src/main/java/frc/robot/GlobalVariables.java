// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GlobalVariables extends SubsystemBase {
  /** Creates a new GlobalVariables. */
  private boolean m_intakeState = true;// true for cone mode, false for cube mode
  private int m_dropLevel = 2;// 1 is ground, 2 is second level, 3 is third level
  private int m_stowLocation = 1;// 0 is ground, 1 is shelf
  private int m_pickMode = 0;// 0 is ground, 1 is shelf


  public GlobalVariables() {}

  public void setIntakeState(boolean p_state){
    m_intakeState = p_state;
  }

  public boolean getIntakeState(){
    return m_intakeState;
  }

  public void setDropLevel(int p_level){
    m_dropLevel = p_level;
  }
  public int getDropLevel(){
    return m_dropLevel;
  }

  public void setStowLocation(int p_Location){
    m_stowLocation = p_Location;
  }

  public int getStowLocaton(){
    return m_stowLocation;
  }

  public void setPickMode(int p_mode){
    m_pickMode = p_mode;
  }

  public int getPickMode(){
    return m_pickMode;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("IntakeMode", m_intakeState);
    SmartDashboard.putNumber("DropLevel", m_dropLevel);
    SmartDashboard.putNumber("StowLocation", m_stowLocation);
    SmartDashboard.putNumber("PickMode", m_pickMode);
  }
}
