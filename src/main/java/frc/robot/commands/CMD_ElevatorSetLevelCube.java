// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.SUB_Elevator;

/*This is for setting the elevator to the respective scoring level */
public class CMD_ElevatorSetLevelCube extends CommandBase {
  /** Creates a new CMD_ElevatorSetLevel. */
  SUB_Elevator m_elevator;
  GlobalVariables m_variables;
  double m_wantedPosition;
  public CMD_ElevatorSetLevelCube(SUB_Elevator p_elevator, GlobalVariables p_variables) {
    m_elevator = p_elevator;
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_variables.getDropLevel() == GlobalConstants.kElevator3rdLevel){
      m_wantedPosition = ElevatorConstants.kElevatorThirdCubeLevel;
    }else if (m_variables.getDropLevel() == GlobalConstants.kElevator2ndLevel){
      m_wantedPosition = ElevatorConstants.kElevatorSecondCubeLevel;
    }else if (m_variables.getDropLevel() == GlobalConstants.kElevator1stLevel){
      m_wantedPosition = ElevatorConstants.kElevatorFirstCubeLevel;
    }
    m_elevator.setReference(m_wantedPosition);
    // SmartDashboard.putNumber("ElevatorWantedPositon", m_wantedPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if (m_elevator.getPosition() >= m_wantedPosition - 3 && m_elevator.getPosition() <= m_wantedPosition + 3 ){
      return true;
    }
    return false;
  }
}
