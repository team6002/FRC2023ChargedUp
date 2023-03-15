// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_DriveAlignScoring extends SequentialCommandGroup {
  /** Creates a new CMD_DriveAlignScoring. */
  public CMD_DriveAlignScoring(SUB_Drivetrain p_drivetrain, SUB_LimeLight p_limeLight, GlobalVariables p_variables, CommandXboxController p_driverController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_DriveAlignTagPidOdom(p_drivetrain, p_limeLight, p_variables, p_driverController),
      new CMD_DriveAlignOdometry(p_drivetrain, p_variables, p_driverController)
    );
  }
}
