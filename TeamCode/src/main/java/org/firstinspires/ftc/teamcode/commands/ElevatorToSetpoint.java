package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ElevatorToSetpoint extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;

    private final int setpoint;

    public ElevatorToSetpoint(ElevatorSubsystem elevatorSubsystem, int setpoint){

        this.elevatorSubsystem = elevatorSubsystem;
        this.setpoint = setpoint;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setAbsolutePosition(setpoint);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetPoint();
    }
}
