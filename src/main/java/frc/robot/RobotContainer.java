//NOTE THE RADIO IS LOOSING  CONNECTION
package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AUTO_BalanceStation;
import frc.robot.auto.AUTO_CubeRun;
import frc.robot.auto.AUTO_Trajectories;
import frc.robot.commands.*;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Wrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SUB_Drivetrain m_drivetrain = new SUB_Drivetrain();
  private final AUTO_Trajectories m_trajectories = new AUTO_Trajectories(m_drivetrain);
  private final SUB_Elevator m_elevator = new SUB_Elevator();
  private final SUB_Elbow m_elbow = new SUB_Elbow();
  private final SUB_Wrist m_wrist = new SUB_Wrist();
  private final SUB_Intake m_intake = new SUB_Intake();
  private final SUB_FiniteStateMachine m_finiteStateMachine = new SUB_FiniteStateMachine();
  private final GlobalVariables m_variables = new GlobalVariables();
  
  
  private final BooleanSupplier IntakeToggle = () -> m_variables.getPickMode() == 1;
  // The driver's controller

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_drivetrain.setDefaultCommand(new CMD_Drive(m_drivetrain, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.leftBumper().onTrue(new ConditionalCommand(
      new SequentialCommandGroup(
        new CMD_IntakeShelf(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine, m_variables),
        new CMD_IntakeCheck(m_intake, m_driverController),
        new CMD_HoldShelf(m_intake, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)
      ),
      new SequentialCommandGroup(
        new CMD_IntakeGroundBack(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine, m_variables),
        new CMD_IntakeCheck(m_intake, m_driverController),
        new CMD_HoldGround(m_intake, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)
      )
        ,IntakeToggle)
      );
    m_driverController.rightBumper().onTrue(new CMD_ToggleDropLevel(m_variables));
    m_driverController.a().onTrue(new CMD_PrepIntakeGroundBack(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables));
    m_driverController.b().onTrue(new CMD_PrepIntakeGroundForwards(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables));
    m_driverController.y().onTrue(new CMD_PrepIntakeShelf(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables));
    m_driverController.x().onTrue(new CMD_PlaceForwards(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine, m_variables));
    m_driverController.pov(0).onTrue(new CMD_ToggleDropLevel(m_variables));
    m_driverController.pov(90).onTrue(new CMD_ToggleIntakeState(m_variables));
    // m_driverController.start().onTrue(new CMD_HomeEverything(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine));
    m_driverController.pov(270).onTrue(new CMD_ResetGyro(m_drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AUTO_CubeRun(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables, m_intake, m_driverController);
  }

  public void zeroHeading(){
    m_drivetrain.zeroHeading();
  }

  public void SubsystemsInit(){
    m_elbow.elbowInit();
    m_elevator.elevatorInit();
    m_wrist.wristInit();
  }
}
