// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.subsystems.SUB_Blinkin;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_celebrate extends CommandBase {
  SUB_Drivetrain m_gyro;
  SUB_Blinkin m_blinkin;
  public CMD_celebrate(SUB_Drivetrain p_gyro, SUB_Blinkin p_blinkin) {
    m_gyro = p_gyro;
    m_blinkin = p_blinkin;
  }

  @Override
  public void initialize() {
    if(Math.abs(m_gyro.getRoll()) < 5){
      m_blinkin.set(BlinkinConstants.kBlinkinBalanceCelebrate);
    }
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
