package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeDefaultCommand extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    private final DoubleSupplier rightTrigger2;

    private final DoubleSupplier leftTrigger2;

    int rightzero = 0;
    int leftzero = 0;
    double hold = 0.0;

    public IntakeDefaultCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier left_trigger_2, DoubleSupplier right_trigger_2) {
        this.intakeSubsystem = intakeSubsystem;

        this.rightTrigger2 = right_trigger_2;

        this.leftTrigger2 = left_trigger_2;


        addRequirements(intakeSubsystem);



    }

    @Override
    public void execute() {
        if (rightzero == 0 && rightTrigger2.getAsDouble() <= 0.05)
        {
            hold = 0.1;
            rightzero = 1;
        }
        else if (leftzero == 0 && leftTrigger2.getAsDouble() <= 0.05)
        {
            hold = 0.0;
            leftzero = 1;
        }
        if (leftTrigger2.getAsDouble() >= 0.05)
        {
            leftzero = 0;
        }
        if (rightTrigger2.getAsDouble() >= 0.05)
        {
            rightzero = 0;
        }
        intakeSubsystem.setPower(rightTrigger2.getAsDouble() - leftTrigger2.getAsDouble() + hold);
    }
}
