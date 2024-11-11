package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem extends SubsystemBase {
    private final Servo clawServo;

    private final Telemetry telemetry;

    boolean disableDrive = false;

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo = hardwareMap.get(Servo.class, "claw");

        this.telemetry = telemetry;
    }

    public void setPosition(double position)
    {
        if (!disableDrive)
        {
            clawServo.setPosition(position);

        }
    }

    public void setDisableDrive() {disableDrive = !disableDrive;}

    @Override
    public void periodic() {
        telemetry.addData("bucket", clawServo.getPosition());
    }

}