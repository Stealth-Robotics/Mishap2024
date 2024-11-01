package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.BooleanSupplier;

public class ElevatorDefaultCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    int absolute_position;

    private final BooleanSupplier B2;

    public ElevatorDefaultCommand(ElevatorSubsystem elevatorSubsystem, BooleanSupplier B2) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.B2 = B2;

        addRequirements(elevatorSubsystem);


    }

    @Override
    public void execute() {


    }
}
