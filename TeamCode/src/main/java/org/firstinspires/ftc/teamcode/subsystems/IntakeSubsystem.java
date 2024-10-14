package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final CRServo intakeServo;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intake");


    }

    public void setPower(double power) {
        intakeServo.setPower(power);
    }
}
