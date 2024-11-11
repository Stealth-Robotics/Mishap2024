package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name="Auto", group="red auto", preselectTeleOp = "BLUE | Tele-Op")
public class Basic_Auto extends StealthOpMode {

    SimpleMecanumDriveSubsystem drive; // Ports are front left: 0, back left: 1, front right: 2, back right: 3, all on Control hub.
    ElevatorSubsystem elevator; // motor 2 exp hub 3

    IntakeSubsystem intake; // servo 0

    //CameraSubsystem camera; (No camera on robot)

    ArmSubsystem arm; // motor 1 exp hub 3

    BucketSubsystem bucket; // servo hub 1 expansion hub 3

    HangerSubsystem hanger; // motor 0 exp hub 3









    @Override
    public void initialize() {
        drive = new SimpleMecanumDriveSubsystem(hardwareMap);


    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> drive.drive(0.0,-1.0,0.0)),
                new WaitCommand(1000),
                new InstantCommand(() -> drive.drive( 0.0, 0.0, 0.0))

        );
    }
}