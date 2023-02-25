// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_FiniteStateMachine extends SubsystemBase{

    public enum RobotState 
    {
        HOME,
        PREPINTAKE,
        INTAKE,
        STOW,
        SCORING,
        BALANCING,
    }

    private RobotState m_currentState = RobotState.HOME;

    public void setState(RobotState p_State) {
        m_currentState = p_State;
    }
    
    public RobotState getState() {
        return m_currentState;
    }
    
    public boolean isState(RobotState p_State) {
        return (m_currentState == p_State);
    }

    public RobotState getCurrentState(){
        return m_currentState;
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("Robot state", getState().toString());
    }

}