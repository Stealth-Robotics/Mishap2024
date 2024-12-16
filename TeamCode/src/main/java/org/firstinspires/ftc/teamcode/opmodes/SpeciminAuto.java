package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmToSetpoint;
import org.firstinspires.ftc.teamcode.commands.DriveBackwardInches;
import org.firstinspires.ftc.teamcode.commands.ElevatorToSetpoint;
import org.firstinspires.ftc.teamcode.commands.StrafeRightInches;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name="SpeciminAuto", group="red auto", preselectTeleOp = "BLUE | Tele-Op")
public class SpeciminAuto extends StealthOpMode {

    SimpleMecanumDriveSubsystem drive; // Ports are front left: 0, back left: 1, front right: 2, back right: 3, all on Control hub.
    ElevatorSubsystem elevator; // motor 2 exp hub 3

    IntakeSubsystem intake; // servo 0

    //CameraSubsystem camera; (No camera on robot)

    ArmSubsystem arm; // motor 1 exp hub 3

    BucketSubsystem bucket; // servo hub 1 expansion hub 3

    HangerSubsystem hanger; // motor 0 exp hub 3

    ClawSubsystem claw;









    @Override
    public void initialize() {
        drive = new SimpleMecanumDriveSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap, telemetry);
        claw = new ClawSubsystem(hardwareMap, telemetry);
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        bucket = new BucketSubsystem(hardwareMap, telemetry);

        new InstantCommand(() -> bucket.setPosition(0.17));
        arm.resetMotor();
        claw.setPosition(-1);


    }
    /*
    Move right
    move elevator up
    move right a bit
    move elevator down a little
    releace side gripper
    move left a bit
    move elevator down
    move backwards and/or right
     */

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                /*new DriveBackwardInches(telemetry, drive,-14.0),
                new ArmToSetpoint(arm,-3195),
                new InstantCommand(() -> claw.setPosition(1)),
                new ArmToSetpoint(arm, 0),
                new DriveBackwardInches(telemetry, drive, 10.0),
                new StrafeRightInches(telemetry, drive, 25.0)*/



                new StrafeRightInches(telemetry,drive,24),
                new ElevatorToSetpoint(elevator, -1300),
                new StrafeRightInches(telemetry, drive, 2),
                new ElevatorToSetpoint(elevator, -900),
                new InstantCommand(()-> claw.setPosition(1)),
                new StrafeRightInches(telemetry, drive, -2),
                new ElevatorToSetpoint(elevator, 0),
                new StrafeRightInches(telemetry,drive,-24),
                new DriveBackwardInches(telemetry,drive, 50)


        );
    }
}