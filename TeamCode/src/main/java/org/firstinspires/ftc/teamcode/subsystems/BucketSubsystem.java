package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class BucketSubsystem extends SubsystemBase {
    private final Servo bucketMotor;

    public BucketSubsystem(HardwareMap hardwareMap) {
        bucketMotor = hardwareMap.get(Servo.class, "bucket");
    }

    public void setPosition(double position) {
        bucketMotor.setPosition(position);
    }
}