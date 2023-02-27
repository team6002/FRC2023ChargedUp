//set your robot to run commands for picking in specfic modes
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;

public class CMD_selectIntakeMode extends CommandBase {
  /** Creates a new CMD_selectIntakeMode. */
  GlobalVariables m_variables;
  public CMD_selectIntakeMode(GlobalVariables p_variables) {
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_variables.getIntakeState() == GlobalConstants.kConeMode ){
      if (m_variables.getPickMode() == GlobalConstants.kPickBackGroundMode) {
        m_variables.setIntakeCommandKey(GlobalConstants.kGroundBackCone);
      } else if (m_variables.getPickMode() == GlobalConstants.kPickForwardsGroundMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kGroundForwardsCone);
      } else  if (m_variables.getPickMode() == GlobalConstants.kPickForwardsShelfMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kShelfForwardsCone);
      } else if (m_variables.getPickMode() == GlobalConstants.kPickBackShelfMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kShelfBackCone);
      }
    }else if (m_variables.getIntakeState() == GlobalConstants.kCubeMode){
      if (m_variables.getPickMode() == GlobalConstants.kPickBackGroundMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kGroundBackCube);
      } else if (m_variables.getPickMode() == GlobalConstants.kPickForwardsShelfMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kShelfForwardsCube);
      } else if (m_variables.getPickMode() == GlobalConstants.kPickBackShelfMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kShelfBackCube);
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
