package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HangerSubsystem extends SubsystemBase {
    private final DcMotorEx hangerMotor;
    private final Telemetry telemetry;

    int targetPosition = 0;

    private final PIDController control = new PIDController(0.02, 0, 0);

    public HangerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        hangerMotor = hardwareMap.get(DcMotorEx.class, "hanger");
        hangerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hangerMotor.getCurrentPosition();
        control.setTolerance(5);
        this.telemetry = telemetry;
        
    }

    public void setPosition(int position) {

        if (hangerMotor.getCurrentPosition() + position > 0) {
            targetPosition = 0;
        }
        else if (hangerMotor.getCurrentPosition() + position < -7800) {
            targetPosition = -7800;
        }
        else
        {
            targetPosition = hangerMotor.getCurrentPosition() + position;
        }



        control.setSetPoint(targetPosition);
        telemetry.addData("position:", position);
    }

    @Override
    public void periodic() {
        double calc = control.calculate(hangerMotor.getCurrentPosition());
        hangerMotor.setPower(calc);


        telemetry.addData("calc:", calc);
        telemetry.addData("hook:", hangerMotor.getCurrentPosition());
        telemetry.addData("target:", targetPosition);
    }
}
