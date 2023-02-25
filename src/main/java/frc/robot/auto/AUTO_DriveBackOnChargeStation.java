// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CMD_SetInitalOdometry;
import frc.robot.subsystems.SUB_Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_DriveBackOnChargeStation extends SequentialCommandGroup {
  /** Creates a new AUTO_DriveBackOnChargeStation. */
  public AUTO_DriveBackOnChargeStation(AUTO_Trajectories p_trajectories, SUB_Drivetrain p_drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_SetInitalOdometry(p_drivetrain, p_trajectories.BackOnChargeStationTrajectory),
      p_trajectories.driveTrajectory(p_trajectories.BackOnChargeStationTrajectory)
    );
  }
}
