package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem extends SubsystemBase {
    private final DcMotorEx armMotor;

    private final DigitalChannel armKill; // digital port 1 expansion hub 3

    private final Telemetry telemetry;

    int targetPosition;

    private final PIDController control = new PIDController(0.1, 0, 0);

    public ArmSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        this.telemetry = telemetry;
        armKill = hardwareMap.get(DigitalChannel.class, "armKill");
        //armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.getCurrentPosition();
        control.setTolerance(100);

        this.telemetry.addData("innit!", 0);

    }

    public void setPosition(int position) {

        if (armMotor.getCurrentPosition() + position > 0) {
            targetPosition = 0;
        }
        else if (armMotor.getCurrentPosition() + position < -3500) {
            targetPosition = -3500;
        }
        else
        {
            targetPosition = armMotor.getCurrentPosition() + position;
        }

        control.setSetPoint(targetPosition);
        telemetry.addData("A position:", position);
    }

    public void setAbsolutePosition(int position) {
        control.setSetPoint(position);
    }

    public boolean checkSwitch(){
        return armKill.getState();
    }

    public void resetMotor(double power){
        armMotor.setPower(power);
    }


    @Override
    public void periodic() {
        double calc = control.calculate(armMotor.getCurrentPosition());
        armMotor.setPower(calc);


        telemetry.addData("A calc:", calc);
        telemetry.addData("arm:", armMotor.getCurrentPosition());
        telemetry.addData("A target:", control.getSetPoint());
        telemetry.addData("killswitch", armKill.getState());
    }
}
