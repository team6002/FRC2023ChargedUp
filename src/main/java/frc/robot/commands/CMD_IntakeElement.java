// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IntakeElement extends CommandBase {
  /** Creates a new CMD_IntakeCheck. */
  SUB_Intake m_intake;
  GlobalVariables m_variables;
  boolean m_detected = false;
  boolean m_pressed = false;
  double m_timer = 0;
  CommandXboxController m_driverController;

  public CMD_IntakeElement(SUB_Intake p_intake, GlobalVariables p_variables, CommandXboxController p_driverController) {
    m_driverController = p_driverController;
    m_intake = p_intake;
    m_timer = 0;
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer = 0;
    m_pressed = false;
    m_detected = false;
    m_intake.setIntakeCurrent();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driverController.back().getAsBoolean()){
      m_pressed = true;
    }

    if (m_intake.getCurrent() >= IntakeConstants.kIntakeConeDetectedCurrent) {
      if (m_timer == 20) {
        m_detected = true;
      } else {
        m_timer += 1;
      }
    } else {
      m_detected = false;
      m_timer = 0;
    }
    if (m_variables.getIntakeCommandKey() != -1){
      if (m_variables.getIntakeState() == GlobalConstants.kConeMode){
        m_intake.setPower(IntakeConstants.kIntakeForwardPower);
      }else if (m_variables.getIntakeState() == GlobalConstants.kCubeMode){
        m_intake.setPower(-IntakeConstants.kIntakeForwardPower);
      }
    }else{
      //not nothing
      m_pressed = true;
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (m_pressed == true){
      m_variables.setStage(GlobalConstants.kIntakeStage);
    }else {
      m_variables.setStage(GlobalConstants.kExtendStage);
    }
    m_detected = false;
    m_pressed = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_detected || m_pressed;
  }
}
