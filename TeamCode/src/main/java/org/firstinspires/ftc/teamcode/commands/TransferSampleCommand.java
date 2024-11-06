package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class TransferSampleCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsysterm;
    private final BucketSubsystem bucketSubsystem;

    public TransferSampleCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsysterm, BucketSubsystem bucketSubsystem) {
        // Create instances of all the hardware we are doing to use

        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsysterm = armSubsysterm;
        this.bucketSubsystem = bucketSubsystem;

        // Register which sub-systems this Command uses with the Scheduler
        addRequirements(elevatorSubsystem, armSubsysterm, bucketSubsystem);
    }


    @Override
    public void initialize() {
        // Call once per execute - do entire series of command in this
        // Move elevatgor to transfer height
      //  elevatorSubsystem.setPosition(0.7);
        // pivot arm forward
        // move bucket to trasnport position
    }

    @Override
    public boolean isFinished() {
        // Will return as finished since one time operation done in init()
        return true;
    }

    @Override
    public void execute() {
        // Nothing needed here for this command
    }
}
