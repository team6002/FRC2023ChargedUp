// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;

public class CMD_CheckWristSafe extends CommandBase {
  SUB_Elbow m_elbow;
  SUB_Elevator m_elevator;
  public CMD_CheckWristSafe(SUB_Elbow p_elbow, SUB_Elevator p_elevator) {
    m_elbow = p_elbow;
    m_elevator = p_elevator;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_elevator.getPosition() > ElevatorConstants.kElevatorSafety || m_elbow.getPosition() < ElbowConstants.kElbowSaftey;
  }
}
