package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.stealthrobotics.library.AutoToTeleStorage;

/**
 * This is the most basic Mecanum subsystem you can have, and provides simple methods to drive and stop.
 */
public class SimpleMecanumDriveSubsystem extends SubsystemBase {
    public final DcMotor leftFront;
    public final DcMotor leftBack;
    public final DcMotor rightFront;
    public final DcMotor rightBack;
    final IMU imu;
    boolean fieldcentric = false;
    boolean slowMode = false;
    double headingOffset = 0.0;

    public SimpleMecanumDriveSubsystem(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );

        imu.initialize(parameters);

    }

    /**
     * Drive using gamepad inputs. This includes compensating for imperfect strafing, and adjusting
     * inputs based on stick directions. Better versions would include field-centric driving,
     * deadbands, and more.
     */
    public void driveTeleop(double leftSickY, double leftStickX, double rightStickX) {
        // This code is pulled from Game Manual 0
        // https://gm0.org/en/latest/docs/software/mecanum-drive.html

        double y = -leftSickY; // Remember, this is reversed!
        double x = -leftStickX * 1.1; // Counteract imperfect strafing
        double rotation = rightStickX;
        if (fieldcentric) {
            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -getHeading();
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            x = rotX;
            y = rotY;
        }
        y = y * Math.abs(y);
        x = x * Math.abs(x);
        rotation = rotation * Math.abs(rotation);

        drive(y * 0.85, x * 0.85, rotation * 0.85); //TODO Make Slower for presision

    }

    public void toggleFieldCentric() {
        fieldcentric = !fieldcentric;
    }

//    public double getHeading() {
//        return -imu.getAngularOrientation().firstAngle;
//    }

    // The actual heading from the IMU, only adjusted so that positive is clockwise
    public double getRawHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    // The heading we'll use to drive the bot, adjusted for an offset which we can set any time
    // we want to correct for gyro drift as we drive.
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;
    }

    // Adjust our heading so the front of the bot is forward, no matter how we've drifted over time.
    public void resetHeading() {
        headingOffset = getRawHeading();
        AutoToTeleStorage.finalAutoHeading = 0;
    }
    public void drive(double y, double x, double rotation) {
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
        double leftFrontDrivePower = (y + x + rotation) / denominator;
        double leftRearDrivePower = (y - x + rotation) / denominator;
        double rightFrontDrivePower = (y - x - rotation) / denominator;
        double rightRearDrivePower = (y + x - rotation) / denominator;

        if(slowMode){
            leftFrontDrivePower *= 0.5;
            leftRearDrivePower *= 0.5;
            rightFrontDrivePower *= 0.5;
            rightRearDrivePower *= 0.5;
        }

        leftFront.setPower(leftFrontDrivePower);
        leftBack.setPower(leftRearDrivePower);
        rightFront.setPower(rightFrontDrivePower);
        rightBack.setPower(rightRearDrivePower);
    }

    public void stop(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public int getTicks(){
        return leftFront.getCurrentPosition();
    }

    @Override
    public void periodic() {

    }
    public void toggleSlowMode(){
        slowMode = !slowMode;
    }
}