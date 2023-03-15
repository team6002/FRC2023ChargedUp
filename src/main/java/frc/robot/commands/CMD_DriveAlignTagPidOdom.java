// lines up straight to tag
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;

public class CMD_DriveAlignTagPidOdom extends CommandBase {
  private SUB_Drivetrain m_drivetrain;
  private SUB_Limelight m_limeLight;
  private GlobalVariables m_variables;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final PIDController turnController;

  private boolean end;

  private double xSpeed, ySpeed, turnSpeed;

  private Pose2d goalPose;
  private Pose2d robotOdom;

  private CommandXboxController m_driverController;

  public CMD_DriveAlignTagPidOdom(SUB_Drivetrain p_drivetrain, SUB_Limelight p_limeLight, GlobalVariables p_variables, CommandXboxController p_driverController) {
    m_drivetrain = p_drivetrain;
    m_limeLight = p_limeLight;
    m_variables = p_variables;
    m_driverController = p_driverController;

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
    goalPose = Constants.AutoAlignConstants.goalPose.get(AutoAlignConstants.AlignPosition.MIDDLE);
    robotOdom = m_drivetrain.getPose();

    end = false;

    if (!m_limeLight.hasTarget() || (m_limeLight.getTargetX() < -1.7)) {
      System.out.println("no limelight targets found, bailing");
      // System.out.println("TARGET MISSING SWITCHING TO NOSE");
      end = true;
      return;
    }

    /* TODO: maybe check that the limelight distance is within valid range for accuracy */

    /* Set the goals as an offset of the robot's current odometry */
    xController.setGoal(robotOdom.getX() + (goalPose.getX() - m_limeLight.getTargetX()));
    yController.setGoal(robotOdom.getY() + (goalPose.getY() - m_limeLight.getTargetY()));
    turnController.setSetpoint(m_drivetrain.getAngle() + m_limeLight.getTargetYaw() + goalPose.getRotation().getDegrees());

    xController.setTolerance(AutoAlignConstants.kXTolerance);
    yController.setTolerance(AutoAlignConstants.kYTolerance);
    turnController.setTolerance(AutoAlignConstants.kTurnTolerance);

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

    if (Math.abs(m_driverController.getLeftY()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getLeftX()) > AutoAlignConstants.kAbortThreshold || Math.abs(m_driverController.getRightX()) > AutoAlignConstants.kAbortThreshold) {
      end = true;
      System.out.println("Aborted by driver");
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

    // turnSpeed = 0;
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
    // if (m_limeLight.hasTarget()) {
    //   m_drivetrain.resetOdometry(new Pose2d(m_limeLight.getTargetX(), m_limeLight.getTargetY(), robotOdom.getRotation()));
    // } 
    if (m_limeLight.getTargetX() > -1.2){
      m_drivetrain.resetOdometry(new Pose2d(m_limeLight.getTargetX(), m_limeLight.getTargetY(), robotOdom.getRotation()));
    }

  }

  @Override
  public boolean isFinished() {
    if (end) {
      System.out.println("Done! " + Timer.getFPGATimestamp());
    }

    return end;
  }
}