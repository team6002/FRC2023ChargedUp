//NOTE THE RADIO IS LOOSING  CONNECTION
package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AUTO_CubeRun;
import frc.robot.auto.AUTO_Trajectories;
import frc.robot.commands.*;
import frc.robot.subsystems.SUB_Blinkin;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Wrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final SUB_Blinkin m_blinkin = new SUB_Blinkin();
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
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

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
    m_driverController.leftBumper().onTrue(new SequentialCommandGroup(
      DeployIntakeCommand,
      new CMD_IntakeCheck(m_intake, m_driverController),
      HoldIntakeCommand
      ));
    m_driverController.rightBumper().onTrue(new CMD_ToggleDropLevel(m_variables));
    m_driverController.a().onTrue(new CMD_selectIntakeMode(m_variables, m_blinkin));
    m_driverController.b().onTrue(new CMD_ToggleIntakeState(m_variables));
    m_driverController.y().onTrue(new CMD_TogglePickMode(m_variables));
    m_driverController.x().onTrue(new SequentialCommandGroup(
      new CMD_PlaceForwards(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine, m_variables),
      new WaitCommand(.5),
      StowIntakeCommand
    ));
    m_driverController.pov(0).onTrue(new CMD_ToggleDropLevel(m_variables));
    m_driverController.pov(90).onTrue(PrepIntakeCommand);
    // m_driverController.start().onTrue(new CMD_AdjustBalanceBackwards(m_drivetrain));
    m_driverController.pov(270).onTrue(new CMD_ResetGyro(m_drivetrain));

    //operator controller commands
    m_operatorController.a().onTrue(new CMD_setDropLevel(m_variables, GlobalConstants.kElevator1stLevel));
    m_operatorController.b().onTrue(new CMD_setDropLevel(m_variables, GlobalConstants.kElevator2ndLevel));
    m_operatorController.x().onTrue(new CMD_setDropLevel(m_variables, GlobalConstants.kElevator3rdLevel));
    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> m_variables.setIntakeState(GlobalConstants.kConeMode)));
    m_operatorController.rightBumper().onTrue(new InstantCommand(() -> m_variables.setIntakeState(GlobalConstants.kCubeMode)));
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

  private int getIntakeType() {
    return m_variables.getIntakeCommandKey();
  };

  public final Command PrepIntakeCommand  = 
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kUnknownIntakeKey, new PrintCommand("I HAVE NO IDEA WHAT YOU ARE TRYING TO DO")),
      Map.entry(GlobalConstants.kGroundBackCube, new CMD_PrepIntakeGroundBack(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kGroundBackCone, new CMD_PrepIntakeGroundBack(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kGroundForwardsCone, new CMD_PrepIntakeShelfForwards(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kShelfForwardsCube, new CMD_PrepIntakeShelfForwards(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kShelfForwardsCone, new CMD_PrepIntakeShelfForwards(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kShelfBackCone, new CMD_PrepIntakeShelfBack(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kShelfBackCube, new CMD_PrepIntakeShelfBack(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine, m_variables))
    )
    ,this::getIntakeType
  );

  public final Command DeployIntakeCommand  = 
    new SelectCommand(
      Map.ofEntries(
        Map.entry(GlobalConstants.kUnknownIntakeKey, new PrintCommand("I HAVE NO IDEA WHAT YOU ARE TRYING TO DO")),
        Map.entry(GlobalConstants.kGroundBackCube, new CMD_IntakeGroundBackCube(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine, m_variables)),
        Map.entry(GlobalConstants.kGroundBackCone, new CMD_IntakeGroundBackCone(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine, m_variables)),
        Map.entry(GlobalConstants.kGroundForwardsCone, new PrintCommand("2")),
        Map.entry(GlobalConstants.kShelfForwardsCube, new CMD_IntakeShelfForwardsCube(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine, m_variables)),
        Map.entry(GlobalConstants.kShelfForwardsCone, new CMD_IntakeShelfForwardsCone(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine, m_variables)),
        Map.entry(GlobalConstants.kShelfBackCone, new CMD_IntakeShelfBackCone(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine, m_variables)),
        Map.entry(GlobalConstants.kShelfBackCube, new CMD_IntakeShelfBackCone(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine, m_variables))
        )
      ,this::getIntakeType
    );

    public final Command HoldIntakeCommand  = 
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kUnknownIntakeKey, new PrintCommand("I HAVE NO IDEA WHAT YOU ARE TRYING TO DO")),
      Map.entry(GlobalConstants.kGroundBackCube, new CMD_HoldGroundBack(m_intake, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kGroundBackCone, new CMD_HoldGroundBack(m_intake, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kGroundForwardsCone, new PrintCommand("2")),
      Map.entry(GlobalConstants.kShelfForwardsCube, new CMD_HoldShelfForwards(m_intake, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kShelfForwardsCone, new CMD_HoldShelfForwards(m_intake, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kShelfBackCone, new CMD_HoldShelfBack(m_intake, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.kShelfBackCube, new CMD_HoldShelfBack(m_intake, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables))
    )
    ,this::getIntakeType
  );

  public final Command StowIntakeCommand  = 
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kUnknownIntakeKey, new PrintCommand("I HAVE NO IDEA WHAT YOU ARE TRYING TO DO")),
      Map.entry(GlobalConstants.kGroundBackCube, new CMD_StowGround(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kGroundBackCone, new CMD_StowGround(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kGroundForwardsCone, new PrintCommand("2")),
      Map.entry(GlobalConstants.kShelfForwardsCube, new CMD_Stow(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kShelfForwardsCone, new CMD_Stow(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kShelfBackCone, new CMD_StowShelfBack(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine)),
      Map.entry(GlobalConstants.kShelfBackCube, new CMD_StowShelfBack(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine))
    )
    ,this::getIntakeType
  );  
}

