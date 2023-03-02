// when the elbow is in a safe location to fully retract
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elbow;

public class CMD_ElbowSafetyCheck extends CommandBase {
  /** Creates a new CMD_ElbowSafetyCheck. */
  SUB_Elbow m_elbow;
  public CMD_ElbowSafetyCheck(SUB_Elbow p_elbow) {
    m_elbow = p_elbow;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_elbow.getPosition() < 205);
  }
}
