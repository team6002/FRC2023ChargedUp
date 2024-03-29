// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SUB_Intake;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_FiniteStateMachine;

public class CMD_IntakeDrop extends CommandBase {
  SUB_Intake m_intake;
  SUB_FiniteStateMachine m_finiteStateMachine;
  GlobalVariables m_variables;


  public CMD_IntakeDrop(SUB_Intake p_intake, GlobalVariables p_variables){
    m_intake = p_intake;
    m_variables = p_variables;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {    
    m_intake.setIntakeCurrent();
    if (m_variables.getIntakeCommandKey() != -1){
      if(m_variables.getIntakeState() == GlobalConstants.kConeMode){
        m_intake.setPower(IntakeConstants.kIntakeDropCone);
      }else{
        m_intake.setPower(IntakeConstants.kIntakeShoot);
        // m_intake.setPower(IntakeConstants.kIntakeDropCube);
      }
    }else{
      //nothing TA DA
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