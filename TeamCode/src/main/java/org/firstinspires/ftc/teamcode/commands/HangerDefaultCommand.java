package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;

import java.util.function.DoubleSupplier;

public class HangerDefaultCommand extends CommandBase {
    private final HangerSubsystem hangerSubsystem;

    private final DoubleSupplier rightStickY;

    public HangerDefaultCommand(HangerSubsystem hangerSubsystem, DoubleSupplier right_stick_y) {
        this.hangerSubsystem = hangerSubsystem;

        this.rightStickY = right_stick_y;

        addRequirements(hangerSubsystem);
    }

    @Override
    public void execute() {
        hangerSubsystem.setPower(rightStickY.getAsDouble());
    }
}
