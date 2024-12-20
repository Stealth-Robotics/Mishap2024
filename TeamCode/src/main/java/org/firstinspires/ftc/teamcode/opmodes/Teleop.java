package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.commands.ArmDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToSetpoint;
import org.firstinspires.ftc.teamcode.commands.BucketDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.DefaultMecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorReset;
import org.firstinspires.ftc.teamcode.commands.ElevatorToSetpoint;
import org.firstinspires.ftc.teamcode.commands.HangerDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.function.BooleanSupplier;

@TeleOp(name = "teleop")
public class Teleop extends StealthOpMode {

    // Subsystems
    SimpleMecanumDriveSubsystem drive; // Ports are front left: 0, back left: 1, front right: 2, back right: 3, all on Control hub.
    ElevatorSubsystem elevator; // motor 2 exp hub 3



    IntakeSubsystem intake; // servo 0

    //CameraSubsystem camera; (No camera on robot)

    ArmSubsystem arm; // motor 1 exp hub 3

    BucketSubsystem bucket; // servo hub 1 expansion hub 3

    HangerSubsystem hanger; // motor 0 exp hub 3

    ClawSubsystem claw; // servo hub 2 expansion hub 3 & servo hub 3 expansion hub



    // Game controllers
    GamepadEx driveGamepad;
    GamepadEx mechGamepad;


    @Override
    public void initialize() {
        // Setup and register all of your subsystems here
        drive = new SimpleMecanumDriveSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        claw = new ClawSubsystem(hardwareMap, telemetry);
        //camera = new CameraSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap, telemetry);
        bucket = new BucketSubsystem(hardwareMap, telemetry);
        hanger = new HangerSubsystem(hardwareMap, telemetry);
        register(elevator, intake, arm, bucket, hanger, claw);
        arm.resetMotor();
        elevator.innitresetMotor();
        driveGamepad = new GamepadEx(gamepad1);
        mechGamepad = new GamepadEx(gamepad2);

        /*for(int i=0;i<1000;i++)
        {
            //  Jim was here
            this.telemetry.addData("RM SWITCH", arm.checkSwitch());
            //telemetry.addData("arm switch:"+switchArm.toString());
            telemetry.update();
            //           arm.resetMotor(0.3);
        }
        this.telemetry.addData("RM SWITCH DONE", 0);
        telemetry.update();
*/


        //while (!arm.checkSwitch() && !opModeIsActive())
        {
 //  Jim was here
        //            arm.resetMotor(0.3);
        }

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
                        () -> mechGamepad.getGamepadButton(GamepadKeys.Button.B).get()
                )
        );
        /*
        driveGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> arm.setDisableDrive()),
                        new InstantCommand(() -> bucket.setDisableDrive()),
                        new InstantCommand(() -> elevator.setDisableDrive())
                )
        );
*/
        driveGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new InstantCommand(() -> elevator.setPosition(100)),
                        new ClawCommand(claw, false),
                        new ElevatorToSetpoint(elevator, 0)

                ));
        driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new ElevatorToSetpoint(elevator, -1500)

                ));
        driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new ElevatorToSetpoint(elevator, -300)

                ));
        mechGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new ElevatorToSetpoint(elevator, -3150)

                ));
        mechGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new ElevatorToSetpoint(elevator, -1750)
                ));


        mechGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new ElevatorToSetpoint(elevator, -200),
                        new ArmToSetpoint(arm, -850),
                        new InstantCommand(() -> bucket.setPosition(0.06))
                ));

        driveGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> claw.setPosition(-1))

        );
        driveGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> claw.setPosition(1))
        );
        driveGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ArmToSetpoint(arm, 0)
        );
        driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new ElevatorToSetpoint(elevator, 0),
                        new InstantCommand(() -> bucket.setPosition(0.17)),
                        new ArmToSetpoint(arm, -2805) //TODO make this pick up specimins
                )
        );
        driveGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new ElevatorToSetpoint(elevator, 0),
                        new InstantCommand(() -> bucket.setPosition(0.17)),
                        new ArmToSetpoint(arm, -3195)
                )
        );
        mechGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ElevatorReset(elevator));



        intake.setDefaultCommand(
                new IntakeDefaultCommand(
                        intake,
                        () -> mechGamepad.gamepad.left_trigger,
                        () -> mechGamepad.gamepad.right_trigger
                )
        );
        arm.setDefaultCommand(
                new ArmDefaultCommand(
                        arm
                )
        );


        mechGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new ElevatorToSetpoint(elevator, 0),
                        new InstantCommand(() -> bucket.setPosition(0.17)),
                        new ArmToSetpoint(arm, -3400)
                )
        );
        mechGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                        new ArmToSetpoint(arm, 0),
                        new InstantCommand(() -> bucket.setPosition(0.17)),
                        new ElevatorToSetpoint(elevator, 0)


                )
        );
        mechGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(

                        new InstantCommand(() -> bucket.setPosition(0.17)),
                        new ArmToSetpoint(arm, 0)

                )
        );
        mechGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(

                        new InstantCommand(() -> bucket.setPosition(0.0))


                )
        );


        hanger.setDefaultCommand(
                new HangerDefaultCommand(
                        hanger,
                        () -> driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get(),
                        () -> driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()

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

    /*@Override
    public double getFinalHeading() {
        return drive.getHeading();
    }
*/
}