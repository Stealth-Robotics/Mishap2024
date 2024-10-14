package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorDefaultCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    private final DoubleSupplier rightStickY2;

    public ElevatorDefaultCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier right_stick_y2) {
        this.elevatorSubsystem = elevatorSubsystem;

        this.rightStickY2 = right_stick_y2;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setPower(rightStickY2.getAsDouble());
    }
}
