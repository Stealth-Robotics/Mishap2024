package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

/*
Glenn
Example of "Command" code that uses multiple sub-systems (pieces of hardware).
In this case it is the outline to do the sample transfer steps from intake to getting ready to drop in bucket.

This is not meant to be robot-ready code but instead for testing and learning purposes.
 */

public class TransferSampleCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsysterm;
    private final BucketSubsystem bucketSubsystem;

    private final Telemetry telemetry;

    public TransferSampleCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsysterm, BucketSubsystem bucketSubsystem, Telemetry telemetry) {
        // make sure we have instance of telemetry
        this.telemetry = telemetry;
        telemetry.addData("TransferCmd", "Created the command class");

        // Create instances of all the hardware we are doing to use

        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsysterm = armSubsysterm;
        this.bucketSubsystem = bucketSubsystem;

        // Register which sub-systems this Command uses with the Scheduler
        addRequirements(elevatorSubsystem, armSubsysterm, bucketSubsystem);
    }


    @Override
    public void initialize() {
        // Call once per execute - do entire series of commands here

        telemetry.addData("TransferCmd", "*** Start Sequence ***");

        // Move elevator to transfer height
        telemetry.addData("TransferCmd", "Move Elevator to Xfer height");
        //  elevatorSubsystem.setPosition(0.7);

        // pivot arm forward
        telemetry.addData("TransferCmd", "Move Pivot Arm forward");

        // move bucket to transport position
        telemetry.addData("TransferCmd", "Move bucket to xfer position");

        telemetry.addData("TransferCmd", "*** End Sequence ***");
    }

    @Override
    public boolean isFinished() {
        // Will return as finished since one time operation done in init()
        return true;
    }

    @Override
    public void execute() {
        // Nothing needed here for this command as we did everything in init()
    }
}
