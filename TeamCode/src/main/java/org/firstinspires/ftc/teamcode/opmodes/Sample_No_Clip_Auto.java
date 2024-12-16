package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.ArmToSetpoint;
import org.firstinspires.ftc.teamcode.commands.DriveBackwardInches;
import org.firstinspires.ftc.teamcode.commands.ElevatorToSetpoint;
import org.firstinspires.ftc.teamcode.commands.TurnToDegrees;
import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name="Sample_No_Clip", group="red auto", preselectTeleOp = "BLUE | Tele-Op")
public class Sample_No_Clip_Auto extends StealthOpMode {

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
                new DriveBackwardInches(telemetry, drive,-15.0),
                new TurnToDegrees(telemetry, drive, 45),

                new WaitCommand(500),
                // add the bucket and elevator set up here
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new ElevatorToSetpoint(elevator, -200),
                        new ArmToSetpoint(arm, -850),
                        new InstantCommand(() -> bucket.setPosition(0.06))
                ),

                    new WaitCommand(500),

                    new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new ElevatorToSetpoint(elevator, -3150)

                ),


            new DriveBackwardInches(telemetry, drive, 5),
            new TurnToDegrees(telemetry, drive, 45),
            new DriveBackwardInches(telemetry, drive, 10),
            new TurnToDegrees(telemetry, drive, -90)

        );
    }
}