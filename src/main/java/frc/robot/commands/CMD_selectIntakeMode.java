//set your robot to run commands for picking in specfic modes
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.SUB_Blinkin;

public class CMD_selectIntakeMode extends CommandBase {
  /** Creates a new CMD_selectIntakeMode. */
  GlobalVariables m_variables;
  SUB_Blinkin m_blinkin;
  public CMD_selectIntakeMode(GlobalVariables p_variables, SUB_Blinkin p_blinkin) {
    m_variables = p_variables;
    m_blinkin = p_blinkin;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_variables.getIntakeState() == GlobalConstants.kConeMode ){
      if (m_variables.getPickMode() == GlobalConstants.kPickBackGroundMode) {
        m_variables.setIntakeCommandKey(GlobalConstants.kGroundBackCone);
        m_blinkin.set(BlinkinConstants.kColor1Chase);
      } else if (m_variables.getPickMode() == GlobalConstants.kPickForwardsGroundMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kGroundForwardsCone);
        m_blinkin.set(BlinkinConstants.kColor1Chase);
      } else  if (m_variables.getPickMode() == GlobalConstants.kPickForwardsShelfMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kShelfForwardsCone);
        m_blinkin.set(BlinkinConstants.kColor1ShelfForwards);
      } else if (m_variables.getPickMode() == GlobalConstants.kPickBackShelfMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kShelfBackCone);
        m_blinkin.set(BlinkinConstants.kColor1ShelfBackwards);
      }
    }else if (m_variables.getIntakeState() == GlobalConstants.kCubeMode){
      if (m_variables.getPickMode() == GlobalConstants.kPickBackGroundMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kGroundBackCube);
        m_blinkin.set(BlinkinConstants.kColor2Chase);
      } else if (m_variables.getPickMode() == GlobalConstants.kPickForwardsShelfMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kShelfForwardsCube);
        m_blinkin.set(BlinkinConstants.kColor2ShelfForwards);
      } else if (m_variables.getPickMode() == GlobalConstants.kPickBackShelfMode){
        m_variables.setIntakeCommandKey(GlobalConstants.kShelfBackCube);
        m_blinkin.set(BlinkinConstants.kColor2ShelfBackwards);
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
