package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

public class ArmDefaultCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;

    private final DoubleSupplier leftStickY2;
    
    double Dposition;
    
    int Iposition;

    public ArmDefaultCommand(ArmSubsystem armSubsystem, DoubleSupplier left_stick_y2) {
        this.armSubsystem = armSubsystem;

        this.leftStickY2 = left_stick_y2;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        Dposition = leftStickY2.getAsDouble() * 500;
        


        armSubsystem.setPosition(((int) Dposition));
    }
}
