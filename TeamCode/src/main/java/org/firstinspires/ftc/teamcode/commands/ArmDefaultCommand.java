package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArmDefaultCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;


    
    double Dposition;

    int go_to_position = 0;
    private final BooleanSupplier up2;
    private final BooleanSupplier left2;

    private final BooleanSupplier down2;

    private final BooleanSupplier right2;

    private final BooleanSupplier rightBumper2;

    private final BooleanSupplier leftBumper2;

    boolean disable_drive = false;

    public ArmDefaultCommand(ArmSubsystem armSubsystem, BooleanSupplier Down2, BooleanSupplier Left2, BooleanSupplier Up2, BooleanSupplier Right2, BooleanSupplier Left_bumper2, BooleanSupplier Right_bumper2) {
        this.armSubsystem = armSubsystem;
        this.up2 = Up2;
        this.left2 = Left2;
        this.down2 = Down2;
        this.right2 = Right2;
        this.rightBumper2 = Right_bumper2;
        this.leftBumper2 = Left_bumper2;


        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {



        //armSubsystem.setPosition(((int) Dposition));

        if (leftBumper2.getAsBoolean() && !disable_drive)
        {
            go_to_position = 0;
        }
        else if (rightBumper2.getAsBoolean() && !disable_drive)
        {
            go_to_position = -3400;
        }
        else if (left2.getAsBoolean() && !disable_drive)
        {
            go_to_position = -300;
            disable_drive = true;
        }
        else if ((up2.getAsBoolean() || right2.getAsBoolean()) && !disable_drive)
        {
            go_to_position = 0;
            disable_drive = true;
        }
        else if (down2.getAsBoolean())
        {
            disable_drive = true;
        }
        armSubsystem.setAbsolutePosition(go_to_position);

    }
}
