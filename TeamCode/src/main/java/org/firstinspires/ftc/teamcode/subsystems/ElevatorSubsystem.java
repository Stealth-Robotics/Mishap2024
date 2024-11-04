package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElevatorSubsystem extends SubsystemBase {
    private final DcMotorEx elevatorMotor;

    private final DigitalChannel elevatorKill; // digital port 2 expansion hub 3

    private final Telemetry telemetry;

    int targetPosition;

    private final PIDController control = new PIDController(0.01, 0, 0);

    boolean resetOnce = false;

    boolean disableDrive = false;

    public ElevatorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevator");
        this.telemetry = telemetry;
        elevatorKill = hardwareMap.get(DigitalChannel.class, "armKill");
        //armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        control.setTolerance(40);
        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        this.telemetry.addData("innit!", 0);

    }

    public void setPosition(int position) {

      /*  if (elevatorMotor.getCurrentPosition() + position > 0) {
            targetPosition = 0;
        }
        else if (elevatorMotor.getCurrentPosition() + position < -3400) {
            targetPosition = -3400;
        }
        else
        {
            targetPosition = elevatorMotor.getCurrentPosition() + position;
        }
*/
        control.setSetPoint(targetPosition);
        telemetry.addData("A position:", position);
    }

    public void setAbsolutePosition(int position) {
        if (!disableDrive)
        {
            control.setSetPoint(position);
        }
    }

    public boolean atSetPoint(){
        return control.atSetPoint();
    }

    public boolean checkSwitch(){
        return elevatorKill.getState();
    }

    public void setDisableDrive() {disableDrive = !disableDrive;}

    public void innitresetMotor(){
        if (checkSwitch() && !resetOnce)
        {
            elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            control.setSetPoint(0);
            resetOnce = true;
        }
        else if (!checkSwitch() && !resetOnce)
        {
            elevatorMotor.setPower(0.3);
        }
    }

    public void resetMotor(){
        if (checkSwitch())
        {
            elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            control.setSetPoint(0);
        }
        else if (!checkSwitch())
        {
            elevatorMotor.setPower(0.5);
        }
    }

    public void setPower(double power){
        elevatorMotor.setPower(power);
    }


    @Override
    public void periodic() {
        double calc = control.calculate(elevatorMotor.getCurrentPosition());
        elevatorMotor.setPower(calc);




        telemetry.addData("E calc:", calc);
        telemetry.addData("elevator:", elevatorMotor.getCurrentPosition());
        telemetry.addData("E target:", control.getSetPoint());
        //telemetry.addData("killswitch", Kill.getState());
    }
}