package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangerSubsystem extends SubsystemBase {
    private final DcMotor hangerMotor;

    public HangerSubsystem(HardwareMap hardwareMap) {
        hangerMotor = hardwareMap.get(DcMotor.class, "hanger");
        hangerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setPower(double power) {
        hangerMotor.setPower(power);
    }
}
