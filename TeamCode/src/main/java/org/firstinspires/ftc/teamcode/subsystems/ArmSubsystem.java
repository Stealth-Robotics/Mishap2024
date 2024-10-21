package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotorEx armMotor;

    private final Telemetry telemetry;

    int targetPosition = 0;

    private final PIDController control = new PIDController(0.1, 0, 0);

    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.getCurrentPosition();
        control.setTolerance(5);
        this.telemetry = telemetry;

    }

    public void setPosition(int position) {

        if (armMotor.getCurrentPosition() + position > 0) {
            targetPosition = 0;
        }
        else if (armMotor.getCurrentPosition() + position < -7800) {
            targetPosition = -7800;
        }
        else
        {
            targetPosition = armMotor.getCurrentPosition() + position;
        }



        control.setSetPoint(targetPosition);
        telemetry.addData("A position:", position);
    }

    @Override
    public void periodic() {
        double calc = control.calculate(armMotor.getCurrentPosition());
        armMotor.setPower(calc);


        telemetry.addData("A calc:", calc);
        telemetry.addData("arm:", armMotor.getCurrentPosition());
        telemetry.addData("A target:", targetPosition);
    }
}
