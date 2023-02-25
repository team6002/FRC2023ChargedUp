// NOTE make our commands intruptable, this is just a really bad method to stop our intake
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IntakeStop extends CommandBase {
  /** Creates a new CMD_IntakeStop. */
  SUB_Intake  m_intake;
  boolean m_detected;

  public CMD_IntakeStop(SUB_Intake p_intake, boolean p_detected) {
    m_intake = p_intake;
    m_detected = p_detected;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setCubeDetected(m_detected);
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
