package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmDefaultCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;

    private final DoubleSupplier leftStickY2;
    
    double Dposition;

    boolean go_to_zero = false;
    private final BooleanSupplier upbutton;

    public ArmDefaultCommand(ArmSubsystem armSubsystem, DoubleSupplier left_stick_y2, BooleanSupplier up_button) {
        this.armSubsystem = armSubsystem;
        this.upbutton = up_button;
        this.leftStickY2 = left_stick_y2;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        Dposition = leftStickY2.getAsDouble() * 100;


        //armSubsystem.setPosition(((int) Dposition));

        if (upbutton.getAsBoolean())
        {
            go_to_zero = true;
        }
        else
        {
            go_to_zero = false;
        }

        if (go_to_zero)
        {
            armSubsystem.setAbsolutePosition(0);
        }
        else
        {
            armSubsystem.setPosition(((int) Dposition));
        }
    }
}
