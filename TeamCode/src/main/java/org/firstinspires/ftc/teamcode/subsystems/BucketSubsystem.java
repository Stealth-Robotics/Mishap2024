package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BucketSubsystem extends SubsystemBase {
    private final Servo bucketServo;

    private final Telemetry telemetry;

    public BucketSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        bucketServo = hardwareMap.get(Servo.class, "bucket");

        this.telemetry = telemetry;
    }

    public void setPosition(double position)
    {
        bucketServo.setPosition(position);
    }

    @Override
    public void periodic() {
        telemetry.addData("bucket", bucketServo.getPosition());
    }

}