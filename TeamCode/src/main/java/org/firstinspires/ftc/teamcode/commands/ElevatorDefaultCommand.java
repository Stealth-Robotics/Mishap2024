package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class ElevatorDefaultCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final DoubleSupplier leftTrigger;
    private final DoubleSupplier rightTrigger;

    public ElevatorDefaultCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setPower(leftTrigger.getAsDouble() - rightTrigger.getAsDouble());
    }
}
