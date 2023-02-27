// checks if the elbow has lifted
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elbow;

public class CMD_ElbowLiftCheck extends CommandBase {
  /** Creates a new CMD_ElbowLiftCheck. */
  SUB_Elbow m_elbow;
  boolean m_finished = false;
  public CMD_ElbowLiftCheck(SUB_Elbow p_elbow) {
    m_elbow = p_elbow;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_finished = (m_elbow.getPosition() < 210);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("im good");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
