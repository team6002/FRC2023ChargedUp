// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class AUTO_CubeRun extends SequentialCommandGroup {
  public AUTO_CubeRun(AUTO_Trajectories p_trajectories, SUB_Drivetrain p_drivetrain, SUB_Elbow p_elbow, SUB_Elevator p_elevator,
  SUB_Wrist p_wrist, SUB_FiniteStateMachine p_finiteStateMachine, GlobalVariables p_variables, SUB_Intake p_intake, CommandXboxController p_controller) {
    addCommands(
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new CMD_SetInitalOdometry(p_drivetrain, p_trajectories.CubeRunTrajectory),
          new InstantCommand(() -> p_variables.setIntakeState(Constants.GlobalConstants.kCubeMode)),
          new InstantCommand(() -> p_variables.setDropLevel(Constants.GlobalConstants.kElevator3rdLevel)),
          new CMD_PlaceForwardsCube(p_elevator, p_intake, p_elbow, p_wrist, p_finiteStateMachine, p_variables),
          new InstantCommand(() -> p_variables.setDropLevel(Constants.GlobalConstants.kElevator2ndLevel)),
          new ParallelCommandGroup(
            // new CMD_IntakeGroundBackAuto(p_elbow, p_elevator, p_intake, p_wrist, p_finiteStateMachine, p_variables),
            new SequentialCommandGroup(
              new WaitCommand(1.5),
              p_trajectories.driveTrajectory(p_trajectories.CubeRunTrajectory) 
            )
          ),
          new CMD_IntakeCheck(p_intake, p_controller),
          new CMD_SetInitalOdometry(p_drivetrain, p_trajectories.CubePlaceTrajectory),
          new ParallelDeadlineGroup(
            p_trajectories.driveTrajectory(p_trajectories.CubePlaceTrajectory)
            // new CMD_PrepPlaceAuto(p_elevator, p_elbow, p_wrist, p_intake, p_variables)
          )
        ),
        new WaitCommand(11.5)
      ),
      new CMD_IntakeDrop(p_intake, p_variables),
      new WaitCommand(.5),
      new CMD_Stow(p_elevator, p_intake, p_elbow, p_wrist, p_finiteStateMachine)
    );
  }
}