// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;

public class CMD_IntakeSetState extends CommandBase {
  GlobalVariables m_variables;
  boolean m_intakeState;
  public CMD_IntakeSetState(GlobalVariables p_variables, boolean p_intakeState) {
    m_variables = p_variables;
    m_intakeState = p_intakeState;
  }

  @Override
  public void initialize() {
    m_variables.setIntakeState(m_intakeState);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
