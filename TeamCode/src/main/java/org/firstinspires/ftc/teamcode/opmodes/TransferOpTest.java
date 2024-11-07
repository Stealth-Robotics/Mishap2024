package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.BucketDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.HangerDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.TransferSampleCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

/* This is example code from Glenn to illustrate one way to create a command that uses
multiple sub-systems (pieces of hardware). In this case it is the outline to do the sample
transfer steps from intake to getting ready to drop in bucket.

This is not meant to be robot-ready code but instead for testing and learning purposes.
 */
@TeleOp(name = "transfer test")
public class TransferOpTest extends StealthOpMode {

    // Subsystems

    // First step is declare the hardware subsystem that we are using
    // Each subsystem wraps up the physical hardware
    // To test the sample transfer, we need the arm, elevator and bucket.

    ElevatorSubsystem elevator; // motor 2 exp hub 3
    ArmSubsystem arm; // motor 1 exp hub 3
    BucketSubsystem bucket;
    IntakeSubsystem intake;

    // Commands - these are the operations we want the robot to perform
    // Here we are using one command to perform all the steps of the transfer operation
    TransferSampleCommand transferSampleCommand;

    // Game controllers - only need the mech secondary controller but inc both for fun
    GamepadEx driveGamepad;
    GamepadEx mechGamepad;

    @Override
    public void initialize() {
        telemetry.addData("entered init", "yes");
        // Setup all of your subsystems by creating one instance of each subsystem class
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap, telemetry);
        bucket = new BucketSubsystem(hardwareMap, telemetry);

        // Register all the sub-systems with the scheduler here. Required
        register(elevator, intake, arm, bucket);

        driveGamepad = new GamepadEx(gamepad1);

        mechGamepad = new GamepadEx(gamepad2);

        /*
        Create the commands that drive the HW sub-systems
        Most of our commands only target one sub-system
        But for this case, we have command that operates on multiple sub-systems to do
        all the steps of the transfer. (Alternative, we could also use a command group)
        */
        transferSampleCommand = new TransferSampleCommand(elevator, arm, bucket, telemetry);

         // Get the "Y" button on secondary pad
        Button transferButton = mechGamepad.getGamepadButton(GamepadKeys.Button.Y);

        // Finally, we bind the transfer command to one of our control surfaces
        // in this case the "Y" button on the secondary controller
        // When it is pressed, we want the transfer command to be put on the scheduler to be executed
        transferButton.whenPressed(transferSampleCommand);

     }
}