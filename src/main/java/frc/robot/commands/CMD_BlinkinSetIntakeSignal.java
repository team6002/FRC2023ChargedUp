// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.subsystems.SUB_Blinkin;
import frc.robot.GlobalVariables;

public class CMD_BlinkinSetIntakeSignal extends CommandBase {
  boolean state;

  SUB_Blinkin m_blinkin;
  GlobalVariables m_variables;

  public CMD_BlinkinSetIntakeSignal(SUB_Blinkin p_blinkin, GlobalVariables p_variables) {
    m_blinkin = p_blinkin;
    m_variables = p_variables;
  }

  @Override
  public void initialize() {
    state = m_variables.getIntakeState();

    switch(m_variables.getIntakeCommandKey()) {
      /* GROUND */
      case GlobalConstants.kGroundBackCone:
      case GlobalConstants.kPickBackGroundMode:
        m_blinkin.set((state == GlobalConstants.kConeMode) ? Constants.BlinkinConstants.kBlinkinConeGround : Constants.BlinkinConstants.kBlinkinCubeGround);
        break;
      /* DOUBLE SUBSTATION */
      case GlobalConstants.kShelfForwardsCone:
      case GlobalConstants.kShelfForwardsCube:
        m_blinkin.set((state == GlobalConstants.kConeMode) ? Constants.BlinkinConstants.kBlinkinConeShelf : Constants.BlinkinConstants.kBlinkinCubeShelf);
        break;
      /* SINGLE SUBSTATION */
      case GlobalConstants.kShelfBackCone:
      case GlobalConstants.kShelfBackCube:
        m_blinkin.set((state == GlobalConstants.kConeMode) ? Constants.BlinkinConstants.kBlinkinConeBackShelf : Constants.BlinkinConstants.kBlinkinCubeBackShelf);
        break;
      /* WHAT IZ YOU DOING */
      default:    
        m_blinkin.set(Constants.BlinkinConstants.kBlinkinUnknownIntakeState);
        break;
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
