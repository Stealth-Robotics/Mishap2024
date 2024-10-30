package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ElevatorDefaultCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    private final BooleanSupplier Down2;

    private final BooleanSupplier Left2;

    private final BooleanSupplier Up2;

    private final BooleanSupplier Right2;

    int absolute_position;

    public ElevatorDefaultCommand(ElevatorSubsystem elevatorSubsystem, BooleanSupplier down2, BooleanSupplier left2, BooleanSupplier up2, BooleanSupplier right2) {
        this.elevatorSubsystem = elevatorSubsystem;

        this.Down2 = down2;

        this.Left2 = left2;

        this.Up2 = up2;

        this.Right2 = right2;

        addRequirements(elevatorSubsystem);

        if (Up2.getAsBoolean())
        {

        }
        else if (Down2.getAsBoolean())
        {

        }
        else if (Right2.getAsBoolean())
        {

        }
        else if (Left2.getAsBoolean())
        {

        }
    }

    @Override
    public void execute() {
        elevatorSubsystem.setAbsolutePosition(absolute_position);
    }
}
