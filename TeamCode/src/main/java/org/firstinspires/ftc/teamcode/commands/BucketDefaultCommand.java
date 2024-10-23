package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BucketDefaultCommand extends CommandBase {
    private final BucketSubsystem bucketSubsystem;

    private final BooleanSupplier a2;

    private final BooleanSupplier b2;

    private final BooleanSupplier y2;

    double position;

    public BucketDefaultCommand(BucketSubsystem bucketSubsystem, BooleanSupplier A2, BooleanSupplier B2, BooleanSupplier Y2) {
        this.bucketSubsystem = bucketSubsystem;

        this.a2 = A2;
        this.b2 = B2;
        this.y2 = Y2;

        if (A2.getAsBoolean())
        {
            position = 0.0;
        }
        else if (B2.getAsBoolean())
        {
            position = 0.15;
        }
        else if (Y2.getAsBoolean())
        {
            position = 0.1;
        }

        addRequirements(bucketSubsystem);
    }

    @Override
    public void execute() {
        bucketSubsystem.setPosition(position);
    }
}