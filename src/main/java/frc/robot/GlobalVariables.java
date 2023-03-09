// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoAlignConstants.*;

public class GlobalVariables extends SubsystemBase {
  /** Creates a new GlobalVariables. */
  private boolean m_intakeState = true;// true for cone mode, false for cube mode
  private int m_dropLevel = 2;// 1 is ground, 2 is second level, 3 is third level
  private int m_stowLocation = 1;// 0 is ground, 1 is shelf
  private int m_pickMode = -1;// 0 is groundBack, 1 is groundForwards, 2 shelfBack, 3 shelfForwards
  private int m_intakeCommandKey = -1;
  private int m_stage = 0;// 0 is intake, 2 is drop, 3 is dropped
  private int m_extendKey = -1;

  private Constants.AutoAlignConstants.AlignPosition m_AlignPosition;

  public GlobalVariables() {}

  public void setIntakeState(boolean p_state){
    m_intakeState = p_state;
  }

  public boolean getIntakeState(){
    return m_intakeState;
  }

  public void setAlignPosition(Constants.AutoAlignConstants.AlignPosition p_alignPosition){
    m_AlignPosition = p_alignPosition;
  }

  public Constants.AutoAlignConstants.AlignPosition getAlignPosition(){
    return m_AlignPosition;
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

  public int getIntakeCommandKey(){
    return m_intakeCommandKey;
  }

  public void setIntakeCommandKey(int p_command){
    m_intakeCommandKey = p_command;
  }

  public int getStage(){
    return m_stage;
  }

  public void setStage(int p_state){
    m_stage = p_state;
  }

  public int getExtendKey(){
    return m_extendKey;
  }

  public void setExtendKey(int p_extendKey){
    m_extendKey = p_extendKey;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeCommandKey", m_intakeCommandKey);
    SmartDashboard.putBoolean("IntakeMode", m_intakeState);
    SmartDashboard.putNumber("DropLevel", m_dropLevel);
    SmartDashboard.putNumber("StowLocation", m_stowLocation);
    SmartDashboard.putNumber("PickMode", m_pickMode);

    if(getAlignPosition() == AlignPosition.LEFT){
      SmartDashboard.putString("AlignPosition", "LEFT");
    }else if(getAlignPosition() == AlignPosition.MIDDLE){
      SmartDashboard.putString("AlignPosition", "MIDDLE");
    }else if(getAlignPosition() == AlignPosition.RIGHT){
      SmartDashboard.putString("AlignPosition", "RIGHT");
    }else{
      SmartDashboard.putString("AlignPosition", "Not a valid align position");
    }
  }
}
