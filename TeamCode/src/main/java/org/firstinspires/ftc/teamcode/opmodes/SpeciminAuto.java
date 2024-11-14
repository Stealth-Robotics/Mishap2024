package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmToSetpoint;
import org.firstinspires.ftc.teamcode.commands.DriveBackwardInches;
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

        arm.resetMotor();
        claw.setPosition(-1);


    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new DriveBackwardInches(telemetry, drive,-16.0),
                new ArmToSetpoint(arm,-2850),
                new InstantCommand(() -> claw.setPosition(1)),
                new WaitCommand(3000),
                new DriveBackwardInches(telemetry, drive, 10.0),
                new ArmToSetpoint(arm,0),
                new StrafeRightInches(telemetry, drive, 25.0)
        );
    }
}