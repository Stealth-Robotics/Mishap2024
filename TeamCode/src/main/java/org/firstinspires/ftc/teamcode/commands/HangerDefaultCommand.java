package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;

import java.util.function.BooleanSupplier;


public class HangerDefaultCommand extends CommandBase {
    private final HangerSubsystem hangerSubsystem;

    private final BooleanSupplier rightbutton;

    private final BooleanSupplier leftbutton;

    int position = 0;

    public HangerDefaultCommand(HangerSubsystem hangerSubsystem, BooleanSupplier right_button, BooleanSupplier left_button) {
        this.hangerSubsystem = hangerSubsystem;

        this.rightbutton = right_button;
        this.leftbutton = left_button;

        addRequirements(hangerSubsystem);
    }

    @Override
    public void execute() {
        if (leftbutton.getAsBoolean() && rightbutton.getAsBoolean())
        {
            position = 0;
        }
        else if (!leftbutton.getAsBoolean() && !rightbutton.getAsBoolean())
        {
            position = 0;
        }
        else if (rightbutton.getAsBoolean())
        {
            position = -100;
        }
        else if (leftbutton.getAsBoolean())
        {
            position = 100;
        }


        hangerSubsystem.setPosition(position);
    }
}
