package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BucketSubsystem extends SubsystemBase {
    private final DcMotor bucketMotor;

    public BucketSubsystem(HardwareMap hardwareMap) {
        bucketMotor = hardwareMap.get(DcMotor.class, "arm");
        bucketMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setPower(double power) {
        bucketMotor.setPower(power);
    }
}