// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;

public class CMD_SelectExtendCommand extends CommandBase {
  /** Creates a new CMD_SelectExtendMode. */
  GlobalVariables m_variables;
  public CMD_SelectExtendCommand(GlobalVariables p_variables) {
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_variables.getIntakeState() == GlobalConstants.kConeMode){
      if (m_variables.getDropLevel() == GlobalConstants.kElevator1stLevel){
        m_variables.setExtendKey(GlobalConstants.k1stLevelForwardCone);
      }
      else if (m_variables.getDropLevel() == GlobalConstants.kElevator2ndLevel){
        m_variables.setExtendKey(GlobalConstants.k2ndLevelCone);
      }
      else if (m_variables.getDropLevel() == GlobalConstants.kElevator3rdLevel){
        m_variables.setExtendKey(GlobalConstants.k3rdLevelCone);
      }
    } 
    else if (m_variables.getIntakeState() == GlobalConstants.kCubeMode){
      if (m_variables.getDropLevel() == GlobalConstants.kElevator1stLevel){
        m_variables.setExtendKey(GlobalConstants.k1stLevelForwardCube);
      }
      else if (m_variables.getDropLevel() == GlobalConstants.kElevator2ndLevel){
        m_variables.setExtendKey(GlobalConstants.k2ndLevelCube);
      }
      else if (m_variables.getDropLevel() == GlobalConstants.kElevator3rdLevel){
        m_variables.setExtendKey(GlobalConstants.k3rdLevelCube);
      }
    }
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
    return true;
  }
}
