//NOTE THE RADIO IS LOOSING  CONNECTION
package frc.robot;

import java.util.Map;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;
import frc.robot.Constants.AutoAlignConstants.AlignPosition;
import frc.robot.auto.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final SUB_LimeLight m_limelight = new SUB_LimeLight();
  private final AUTO_Trajectories m_trajectories = new AUTO_Trajectories(m_drivetrain);
  private final SUB_Elevator m_elevator = new SUB_Elevator();
  private final SUB_Elbow m_elbow = new SUB_Elbow();
  private final SUB_Wrist m_wrist = new SUB_Wrist();
  private final SUB_Intake m_intake = new SUB_Intake();
  private final SUB_FiniteStateMachine m_finiteStateMachine = new SUB_FiniteStateMachine();
  private final GlobalVariables m_variables = new GlobalVariables();
  private final SUB_Blinkin m_blinkin = new SUB_Blinkin();

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
    /* ==================DRIVER CONTROLS================== */
    m_driverController.leftBumper().onTrue(CycleCommand);
    m_driverController.y().onTrue(new CMD_ToggleDropLevel(m_variables));
    m_driverController.b().onTrue(new SequentialCommandGroup(
        new CMD_ToggleIntakeState(m_variables),
        new CMD_BlinkinSetIntakeSignal(m_blinkin, m_variables)
      )
    );
    m_driverController.x().onTrue(new CMD_TogglePickMode(m_variables));
    m_driverController.a().onTrue(new CMD_DriveAlignTagPidOdom(m_drivetrain, m_limelight, m_variables, m_driverController));
    m_driverController.rightBumper().onTrue(
      new SequentialCommandGroup(
        new CMD_SetStage(m_variables, GlobalConstants.kIntakeStage),
        new CMD_selectIntakeCommand(m_variables),
        new CMD_BlinkinSetIntakeSignal(m_blinkin, m_variables),
        PrepIntakeCommand
      )
    );
    m_driverController.pov(0).onTrue(
      new SequentialCommandGroup(
        new CMD_SyncElbowPosition(m_elbow),
        new CMD_HomeEverything(m_elbow, m_elevator, m_intake, m_wrist, m_finiteStateMachine)
      )
      );
    // m_driverController.pov(0).onTrue(new CMD_ToggleDropLevel(m_variables));
    // m_driverController.start().onTrue(new CMD_DrivetrainX(m_drivetrain));
    m_driverController.pov(270).onTrue(
      new SequentialCommandGroup(
        new CMD_SyncElbowPosition(m_elbow),
        new CMD_ResetGyro(m_drivetrain)
      )
      );
    /* ==================DRIVER CONTROLS END================== */

    /* ==================OPERATOR CONTROLS================== */
    m_operatorController.povDown().onTrue(new SequentialCommandGroup(
        new CMD_ToggleIntakeState(m_variables),
        new CMD_BlinkinSetIntakeSignal(m_blinkin, m_variables)
      )
    );

    //autodrive right, bottom, numpad 1
    m_operatorController.a().onTrue(new SequentialCommandGroup(
      new CMD_setDropLevel(m_variables, GlobalConstants.kElevator1stLevel),
      new CMD_setIntakeMode(m_variables, GlobalConstants.kConeMode),
      new CMD_setAlignPosition(m_variables, AlignPosition.RIGHT)
    ));

    //autodrive middle, bottom, numpad 2
    m_operatorController.b().onTrue(new SequentialCommandGroup(
      new CMD_setDropLevel(m_variables, GlobalConstants.kElevator1stLevel),
      new CMD_setIntakeMode(m_variables, GlobalConstants.kCubeMode),
      new CMD_setAlignPosition(m_variables, AlignPosition.MIDDLE)
    ));

    //autodrive left, bottom, numpad 3
    m_operatorController.x().onTrue(new SequentialCommandGroup(
      new CMD_setDropLevel(m_variables, GlobalConstants.kElevator1stLevel),
      new CMD_setIntakeMode(m_variables, GlobalConstants.kConeMode),
      new CMD_setAlignPosition(m_variables, AlignPosition.LEFT)
    ));

    //autodrive right, 2nd level, numpad 4
    m_operatorController.rightBumper().onTrue(new SequentialCommandGroup(
      new CMD_setDropLevel(m_variables, GlobalConstants.kElevator2ndLevel),
      new CMD_setIntakeMode(m_variables, GlobalConstants.kConeMode),
      new CMD_setAlignPosition(m_variables, AlignPosition.RIGHT)
    ));

    //autodrive middle, 2nd level, numpad 5
    m_operatorController.leftBumper().onTrue(new SequentialCommandGroup(
      new CMD_setDropLevel(m_variables, GlobalConstants.kElevator2ndLevel),
      new CMD_setIntakeMode(m_variables, GlobalConstants.kCubeMode),
      new CMD_setAlignPosition(m_variables, AlignPosition.MIDDLE)
    ));

    //autodrive left, 2nd level, numpad 6
    m_operatorController.y().onTrue(new SequentialCommandGroup(
      new CMD_setDropLevel(m_variables, GlobalConstants.kElevator2ndLevel),
      new CMD_setIntakeMode(m_variables, GlobalConstants.kConeMode),
      new CMD_setAlignPosition(m_variables, AlignPosition.LEFT)
    ));

    //autodrive right, 3rd level, numpad 7
    m_operatorController.povRight().onTrue(new SequentialCommandGroup(
      new CMD_setDropLevel(m_variables, GlobalConstants.kElevator3rdLevel),
      new CMD_setIntakeMode(m_variables, GlobalConstants.kConeMode),
      new CMD_setAlignPosition(m_variables, AlignPosition.RIGHT)
    ));

    //autodrive middle, 3rd level, numpad 8
    m_operatorController.povUp().onTrue(new SequentialCommandGroup(
      new CMD_setDropLevel(m_variables, GlobalConstants.kElevator3rdLevel),
      new CMD_setIntakeMode(m_variables, GlobalConstants.kCubeMode),
      new CMD_setAlignPosition(m_variables, AlignPosition.MIDDLE)
    ));

    //autodrive left, 3rd level, numpad 9
    m_operatorController.povLeft().onTrue(new SequentialCommandGroup(
      new CMD_setDropLevel(m_variables, GlobalConstants.kElevator3rdLevel),
      new CMD_setIntakeMode(m_variables, GlobalConstants.kConeMode),
      new CMD_setAlignPosition(m_variables, AlignPosition.LEFT)
    ));
    /* ==================OPERATOR CONTROLS END================== */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandManual() {
    return 
      new AUTO_CubeRunRed(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables, m_intake, m_driverController); 
      // new AUTO_BalanceStation(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_intake, m_finiteStateMachine, m_wrist, m_variables, m_driverController);
  }

  public Command getCubeRunBlue() {
    return new AUTO_CubeRunBlue(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables, m_intake, m_driverController);
  }

  public Command getCubeRunRed() {
    return new AUTO_CubeRunRed(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables, m_intake, m_driverController);
  }

  public Command getBalanceStation() {
    return new AUTO_BalanceStation(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_intake, m_finiteStateMachine, m_wrist, m_variables, m_driverController);
  }
  
  public void zeroHeading(){
    m_drivetrain.zeroHeading();
  }

  public void setAutoKey(int p_key){
    m_variables.setAutoKey(p_key);
  }
  public void SubsystemsInit(){
    m_elbow.elbowInit();
    m_elevator.elevatorInit();
    m_wrist.wristInit();
  }

  private int getIntakeType() {
    return m_variables.getIntakeCommandKey();
  }

  private int getRobotStage(){
    return m_variables.getStage();
  }

  private boolean getIntakeState(){
    return m_variables.getIntakeState();
  }
  
  private int getAutonomousCommandKey(){
    return m_variables.getAutoKey();
  }

  private int getExtendKey(){
    return m_variables.getExtendKey();
  }
  public final Command getAutonomusCommand =
  new SelectCommand(
    Map.ofEntries(
      // Map.entry(AutoConstants.kBalanceStationKey, new AUTO_BalanceStation(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_intake, m_finiteStateMachine, m_wrist, m_variables, m_driverController)),
      // Map.entry(AutoConstants.kCubeRunKey, new AUTO_CubeRun(m_trajectories, m_drivetrain, m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_variables, m_intake, m_driverController))
    
      Map.entry(AutoConstants.kBalanceStationKey, new PrintCommand("1")),
      Map.entry(AutoConstants.kCubeRunKey, new PrintCommand("2"))
    ), 
    this::getAutonomousCommandKey
  );

  public final Command PrepIntakeCommand  = 
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kUnknownIntakeKey, new PrintCommand("I HAVE NO IDEA WHAT YOU ARE TRYING TO DO")),
      Map.entry(GlobalConstants.kGroundBackCube, new CMD_PrepIntakeGroundBack(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_intake, m_variables)),
      Map.entry(GlobalConstants.kGroundBackCone, new CMD_PrepIntakeGroundBack(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_intake, m_variables)),
      Map.entry(GlobalConstants.kGroundForwardsCone, new CMD_PrepIntakeShelfForwards(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_intake, m_variables)),
      Map.entry(GlobalConstants.kShelfForwardsCube, new CMD_PrepIntakeShelfForwards(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_intake, m_variables)),
      Map.entry(GlobalConstants.kShelfForwardsCone, new CMD_PrepIntakeShelfForwards(m_elbow, m_elevator, m_wrist, m_finiteStateMachine, m_intake, m_variables)),
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
  
  public final Command ExtendCommand  = 
  new SelectCommand(
    Map.ofEntries(
      Map.entry(GlobalConstants.kUnknownExtendKey, new PrintCommand("I HAVE NO IDEA WHAT YOU ARE TRYING TO DO")),
      Map.entry(GlobalConstants.k1stLevelForwardCone, new CMD_PlaceForwardsGroundCone(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.k1stLevelForwardCube, new CMD_PlaceForwardsGroundCube(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.k2ndLevelCone, new CMD_PlaceForwardsCone(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.k2ndLevelCube, new CMD_PlaceSecondLevelCube(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.k3rdLevelCone, new CMD_PlaceForwardsCone(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine, m_variables)),
      Map.entry(GlobalConstants.k3rdLevelCube, new CMD_PlaceForwardsCube(m_elevator, m_intake, m_elbow, m_wrist, m_finiteStateMachine, m_variables))
    )
    ,this::getExtendKey
    );
  

  public final Command CycleCommand = 
  new SelectCommand(
    Map.ofEntries(   
      Map.entry(GlobalConstants.kIntakeStage, 
        new SequentialCommandGroup(
        DeployIntakeCommand,
        new CMD_IntakeElement(m_intake, m_variables, m_driverController),
        HoldIntakeCommand,
        new CMD_SetStage(m_variables, GlobalConstants.kExtendStage)
      )
    ),
      Map.entry(GlobalConstants.kExtendStage,
        new SequentialCommandGroup(
          new CMD_SelectExtendCommand(m_variables),
          ExtendCommand,
          new CMD_SetStage(m_variables, GlobalConstants.kDropStage)
        )
      ),    
      Map.entry(GlobalConstants.kDropStage, 
        new SequentialCommandGroup(
          new CMD_IntakeDrop(m_intake, m_variables),
          new WaitCommand(.3),
          StowIntakeCommand,
          new CMD_SetStage(m_variables, GlobalConstants.kIntakeStage)
        )
      )  
    ), 
    this::getRobotStage
    );

}

