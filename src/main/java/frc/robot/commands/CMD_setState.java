// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;;

public class CMD_setState extends CommandBase {
  SUB_FiniteStateMachine m_finiteStateMachine;
  RobotState m_state;
  public CMD_setState(SUB_FiniteStateMachine p_finiteStateMachine, RobotState p_state) {
    m_finiteStateMachine = p_finiteStateMachine;
    m_state = p_state;
    addRequirements(m_finiteStateMachine);
  }

  @Override
  public void initialize() {
    m_finiteStateMachine.setState(m_state);
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
