package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

import java.util.function.BooleanSupplier;

public class ClawCommand extends CommandBase {
    private final ClawSubsystem clawSubsystem;



    double position = 0.17;

    boolean open = false;

    public ClawCommand(ClawSubsystem clawSubsystem, Boolean open) {
        this.clawSubsystem = clawSubsystem;
        this.open = open;



        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize()
    {
        if (open)
        {
            clawSubsystem.setPosition(1);
        }
        else
        {
            clawSubsystem.setPosition(-1);
        }


    }


}