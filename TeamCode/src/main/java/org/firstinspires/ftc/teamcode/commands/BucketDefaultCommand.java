package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;

import java.util.function.DoubleSupplier;

public class BucketDefaultCommand extends CommandBase {
    private final BucketSubsystem bucketSubsystem;

    private final DoubleSupplier leftStickY2;

    public BucketDefaultCommand(BucketSubsystem bucketSubsystem, DoubleSupplier left_stick_y2) {
        this.bucketSubsystem = bucketSubsystem;

        this.leftStickY2 = left_stick_y2;

        addRequirements(bucketSubsystem);
    }

    @Override
    public void execute() {
        bucketSubsystem.setPower(leftStickY2.getAsDouble());
    }
}