package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.DefaultMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class Teleop extends StealthOpMode {

    // Subsystems
    SimpleMecanumDriveSubsystem drive;
    ElevatorSubsystem elevator;

    IntakeSubsystem intake;

    CameraSubsystem camera;

    // Game controllers
    GamepadEx driveGamepad;
    GamepadEx mechGamepad;


    @Override
    public void initialize() {
        // Setup and register all of your subsystems here
        drive = new SimpleMecanumDriveSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        camera = new CameraSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        register(drive, elevator, camera, intake);

        driveGamepad = new GamepadEx(gamepad1);
        mechGamepad = new GamepadEx(gamepad2);

        // Automatically reset the elevator all the way down when we init
//        schedule(new ResetElevatorCommand(elevator));

        // A subsystem's default command runs all the time. Great for drivetrains and such.
        drive.setDefaultCommand(
                new DefaultMecanumDriveCommand(
                        drive,
                        () -> driveGamepad.gamepad.left_stick_y,
                        () -> driveGamepad.gamepad.left_stick_x,
                        () -> driveGamepad.gamepad.right_stick_x
                )
        );

        elevator.setDefaultCommand(
                new ElevatorDefaultCommand (
                        elevator,
                        () -> mechGamepad.gamepad.right_stick_y
                )
        );
        intake.setDefaultCommand(
                new IntakeDefaultCommand(
                        intake,
                        () -> mechGamepad.gamepad.left_trigger,
                        () -> mechGamepad.gamepad.right_trigger
                )
        );

//        Button button = mechGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> elevator.setTargetLocation(0.0)),
//                        new InstantCommand(() -> arm.scorePosition())
//                )
//        );
//        Button button1 = mechGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> elevator.setTargetLocation(1)),
//                        new InstantCommand(() -> arm.intakePosition())
//                )
//        );

//        mechGamepad.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new ResetElevatorCommand(elevator));
//        driveGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> drive.resetHeading()));
//
//        mechGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(() -> lever.toggle()));
//        mechGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(() -> arm.toggle()));
//        mechGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(() -> airplane.toggle()));
    }

    /**
     * Ideally your red vs. blue opmodes are nothing more than this. Keep code shared between
     * them, and take different actions based on the alliance color.
     *
     * @see org.stealthrobotics.library.Alliance
     */

    @SuppressWarnings("unused")
    @TeleOp(name = "RED | Tele-Op", group = "Red")
    public static class RedTeleop extends Teleop {
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "BLUE | Tele-Op", group = "Blue")
    public static class BlueTeleop extends Teleop {
    }

    @Override
    public double getFinalHeading() {
        return drive.getHeading();
    }

}