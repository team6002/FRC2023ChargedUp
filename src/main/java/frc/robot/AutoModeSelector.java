package frc.robot;

import frc.robot.auto.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

/*
 * Auto mode selector taken from Team 254 with added slight modifications.
 *
 * https://github.com/Team254/FRC-2022-Public/blob/6a24236b37f0fcb75ceb9d5dec767be58ea903c0/src/main/java/com/team254/frc2022/AutoModeSelector.java
 */
public class AutoModeSelector {
    private enum DesiredMode {
        ChargeStation,
        CubeRunRed,
        CubeRunBlue
    }

    private DesiredMode m_cachedDesiredMode = DesiredMode.ChargeStation;

    private RobotContainer m_robotContainer;
    private final SendableChooser<DesiredMode> m_modeChooser;

    private Optional<Command> m_autoMode = Optional.empty();

    public AutoModeSelector(RobotContainer p_robotContainer) {
        m_robotContainer = p_robotContainer;

        m_modeChooser = new SendableChooser<>();
        m_modeChooser.setDefaultOption("ChargeStation", DesiredMode.ChargeStation);
        m_modeChooser.addOption("CubeRunRed", DesiredMode.CubeRunRed);
        m_modeChooser.addOption("CubeRunBlue", DesiredMode.CubeRunBlue);

        SmartDashboard.putData("Auto mode", m_modeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = m_modeChooser.getSelected();

        if (desiredMode == null) {
            desiredMode = DesiredMode.ChargeStation;
        }

        if (m_cachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            m_autoMode = getAutoModeForParams(desiredMode);
        }

        m_cachedDesiredMode = desiredMode;
    }

    private Optional<Command> getAutoModeForParams(DesiredMode mode) {
        switch(mode) {
            case ChargeStation:
                return Optional.of(m_robotContainer.getBalanceStation());
            case CubeRunRed:
                return Optional.of(m_robotContainer.getCubeRunRed());
            case CubeRunBlue:
                return Optional.of(m_robotContainer.getCubeRunBlue());
            default:
                break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        m_autoMode = Optional.empty();
        m_cachedDesiredMode = DesiredMode.ChargeStation;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", m_cachedDesiredMode.name());
    }

    public Optional<Command> getAutoMode() {
        return m_autoMode;
    }

    public String getDesiredModeLabel() {
        return m_cachedDesiredMode.name();
    }
}
