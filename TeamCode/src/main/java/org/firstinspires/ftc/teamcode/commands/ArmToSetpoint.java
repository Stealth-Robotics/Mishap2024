package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ArmToSetpoint extends CommandBase {

    private final int setpoint;

    private final ArmSubsystem armSubsystem;
    public ArmToSetpoint(ArmSubsystem armSubsystem, int setpoint){

        this.setpoint = setpoint;
        this.armSubsystem = armSubsystem;

    }

    @Override
    public void initialize() {
        armSubsystem.setAbsolutePosition(setpoint);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.atSetPoint();
    }
}
