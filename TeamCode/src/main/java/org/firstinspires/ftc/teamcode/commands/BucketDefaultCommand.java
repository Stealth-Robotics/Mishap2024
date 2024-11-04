package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BucketDefaultCommand extends CommandBase {
    private final BucketSubsystem bucketSubsystem;

    private final BooleanSupplier a2;

    private final BooleanSupplier x2;

    private final BooleanSupplier y2;

    double position = 0.17;

    public BucketDefaultCommand(BucketSubsystem bucketSubsystem, BooleanSupplier A2, BooleanSupplier X2, BooleanSupplier Y2) {
        this.bucketSubsystem = bucketSubsystem;

        this.a2 = A2;
        this.x2 = X2;
        this.y2 = Y2;



        addRequirements(bucketSubsystem);
    }

    @Override
    public void execute()
    {
        if (x2.getAsBoolean())
        {
            position = 0.0;
        }
        else if (a2.getAsBoolean())
        {
            position = 0.17;
        }
        else if (y2.getAsBoolean())
        {
            position = 0.06;
        }
        bucketSubsystem.setPosition(position);
    }
}