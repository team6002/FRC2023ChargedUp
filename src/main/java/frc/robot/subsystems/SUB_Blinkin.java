// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SUB_Blinkin extends SubsystemBase {
  Spark m_blinkin;
  GlobalVariables m_variables;

  public SUB_Blinkin(GlobalVariables p_variables) {
    m_blinkin = new Spark(Constants.BlinkinConstants.kBlinkinPortId);
    m_variables = p_variables;
  }

  public void set(double pwm) {
    m_blinkin.set(pwm);
  }

  @Override
  public void periodic() {
    switch(m_variables.getIntakeCommandKey()) {
        /* CONE */
        case GlobalConstants.kGroundBackCone:
          m_blinkin.set(Constants.BlinkinConstants.kBlinkinConeGround);
          break;
        case GlobalConstants.kShelfForwardsCone:
          m_blinkin.set(Constants.BlinkinConstants.kBlinkinConeShelf);
          break;
        case GlobalConstants.kShelfBackCone:
          m_blinkin.set(Constants.BlinkinConstants.kBlinkinConeBackShelf);
          break;

        /* CUBE */
        case GlobalConstants.kPickBackGroundMode:
          m_blinkin.set(Constants.BlinkinConstants.kBlinkinCubeGround);
          break;
        case GlobalConstants.kShelfForwardsCube:
          m_blinkin.set(Constants.BlinkinConstants.kBlinkinCubeShelf);
          break;
        case GlobalConstants.kShelfBackCube:
          m_blinkin.set(Constants.BlinkinConstants.kBlinkinCubeBackShelf);
          break;

        /* UNKNOWN CASE */
        default:    
          m_blinkin.set(0);
          break;
    }
  }
}