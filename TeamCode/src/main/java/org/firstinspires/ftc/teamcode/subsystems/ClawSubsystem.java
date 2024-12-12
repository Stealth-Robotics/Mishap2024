package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem extends SubsystemBase {
    private final Servo clawServo;

    private final Servo clawServo2;

    private final Telemetry telemetry;

    boolean disableDrive = false;

    public ClawSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServo2 = hardwareMap.get(Servo.class, "claw2");


        this.telemetry = telemetry;

        clawServo.setPosition(0);
        clawServo2.setPosition(0);
    }

    public void setPosition(double position)
    {
        if (!disableDrive)
        {
            clawServo2.setPosition(position);
            clawServo.setPosition(position);
        }
    }

    public void setDisableDrive() {disableDrive = !disableDrive;}

    @Override
    public void periodic() {
        telemetry.addData("claw", clawServo.getPosition());
    }

}