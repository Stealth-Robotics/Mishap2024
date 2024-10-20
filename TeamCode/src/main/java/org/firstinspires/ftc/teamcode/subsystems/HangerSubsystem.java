package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HangerSubsystem extends SubsystemBase {
    private final MotorEx hangerMotor;
    private final Telemetry telemetry;

    public HangerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        hangerMotor = hardwareMap.get(MotorEx.class, "hanger");
        //hangerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangerMotor.setRunMode(MotorEx.RunMode.PositionControl);
        hangerMotor.getCurrentPosition();
        this.telemetry = telemetry;
    }

    public void setPower(double power) {
        hangerMotor.set(power);
    }

    @Override
    public void periodic() {
        telemetry.addData("hook:", hangerMotor.getCurrentPosition());

    }
}
