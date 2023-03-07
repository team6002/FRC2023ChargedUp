// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_LimeLight;

public class CMD_DriveAlignTagPidOdom extends CommandBase {
  private SUB_Drivetrain m_drivetrain;
  private SUB_LimeLight m_limeLight;
  private GlobalVariables m_variables;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final PIDController turnController;

  private boolean end;

  private double xSpeed, ySpeed, turnSpeed;

  private Pose2d goalPose;
  private Pose2d robotOdom;

  public CMD_DriveAlignTagPidOdom(SUB_Drivetrain p_drivetrain, SUB_LimeLight p_limeLight, GlobalVariables p_variables) {
    m_drivetrain = p_drivetrain;
    m_limeLight = p_limeLight;

    xController = new ProfiledPIDController(Constants.AutoAlignConstants.driveKp,
      Constants.AutoAlignConstants.driveKi,
      Constants.AutoAlignConstants.driveKd,
      Constants.AutoAlignConstants.driveConstraints);

    yController = new ProfiledPIDController(Constants.AutoAlignConstants.driveKp,
      Constants.AutoAlignConstants.driveKi,
      Constants.AutoAlignConstants.driveKd,
      Constants.AutoAlignConstants.driveConstraints);

    turnController = new PIDController(Constants.AutoAlignConstants.turnKp,
      Constants.AutoAlignConstants.turnKi,
      Constants.AutoAlignConstants.turnKd);

    addRequirements(m_drivetrain, m_limeLight);
  }

  @Override
  public void initialize() {
    goalPose = Constants.AutoAlignConstants.goalPose.get(m_variables.getAlignPosition());
    robotOdom = m_drivetrain.getPose();

    end = false;

    if (!m_limeLight.hasTarget()) {
      System.out.println("no limelight targets found, bailing");
      end = true;
      return;
    }

    /* TODO: maybe check that the limelight distance is within valid range for accuracy */

    /* Set the goals as an offset of the robot's current odometry */
    xController.setGoal(robotOdom.getX() + (goalPose.getX() - m_limeLight.getTargetX()));
    yController.setGoal(robotOdom.getY() + (goalPose.getY() - m_limeLight.getTargetY()));
    turnController.setSetpoint(m_drivetrain.getAngle() + m_limeLight.getTargetYaw() + goalPose.getRotation().getDegrees());

    xController.setTolerance(.01);
    yController.setTolerance(.01);
    turnController.setTolerance(1);

    xController.reset(robotOdom.getX());
    yController.reset(robotOdom.getY());
    turnController.reset();

    turnController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    if (end) {
      return;
    }

    robotOdom = m_drivetrain.getPose();

    xSpeed = xController.calculate(robotOdom.getX());
    if (xController.atGoal()) {
      xSpeed = 0.0;
    }

    ySpeed = yController.calculate(robotOdom.getY());
    if (yController.atGoal()) {
      ySpeed = 0.0;
    }

    turnSpeed = MathUtil.clamp(turnController.calculate(m_drivetrain.getAngle()), -0.5, 0.5);

    SmartDashboard.putNumber("AutoAlignXSpeed: ", xSpeed);
    SmartDashboard.putNumber("AutoAlignYSpeed: ", ySpeed);
    SmartDashboard.putNumber("AutoAlignTurnSpeed: ", turnSpeed);
    SmartDashboard.putNumber("AutoAlignTurnGoal", turnController.getSetpoint());

    if (xController.atGoal() && yController.atGoal() && turnController.atSetpoint()) {
      System.out.println("At Goal " + Timer.getFPGATimestamp());
      end = true;
      return;
    }

    m_drivetrain.drive(xSpeed, ySpeed, turnSpeed, false, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0.0, 0.0, 0.0, true, false);
  }

  @Override
  public boolean isFinished() {
    if (end) {
      System.out.println("Done! " + Timer.getFPGATimestamp());
    }

    return end;
  }
}