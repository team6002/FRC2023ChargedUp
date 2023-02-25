// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.GlobalVariables;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Wrist;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_PlaceForwardsAuto extends SequentialCommandGroup {

  public CMD_PlaceForwardsAuto(SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_Elbow p_elbow, SUB_Wrist p_wrist, 
  SUB_FiniteStateMachine p_finiteStateMachine, GlobalVariables p_variables
  ) {
    addCommands(
      new CMD_setState(p_finiteStateMachine, RobotState.SCORING),
      new ParallelCommandGroup(
        new CMD_ElevatorSetPosition(p_elevator, ElevatorConstants.kElevatorThirdLevel - 15),
        new SequentialCommandGroup(
          new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowLift),
          new CMD_ElevatorSafteyCheck(p_elevator),
          new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowPrepareDrop)
      ),
        new SequentialCommandGroup(
          new CMD_CheckWristSafe(p_elbow, p_elevator),
          new CMD_WristSetPosition(p_wrist, WristConstants.kWristShelf)
        )
      ),
      new CMD_IntakeDrop(p_intake, p_variables),
      new WaitCommand(0.5),
      new CMD_IntakeOff(p_intake)
    );
  }
}
