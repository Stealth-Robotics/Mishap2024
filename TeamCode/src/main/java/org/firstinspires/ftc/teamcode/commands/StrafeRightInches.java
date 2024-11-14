package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;

public class StrafeRightInches extends CommandBase {

    private final double inches;
    final SimpleMecanumDriveSubsystem drive;

    final Telemetry telemetry;
    public StrafeRightInches(Telemetry telemetry, SimpleMecanumDriveSubsystem drive, double inches){

        this.inches = inches;
        this.drive = drive;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        telemetry.addData("Inches",this.inches);
        telemetry.addData("reset",0);
        telemetry.update();
        this.drive.resetOdom();
    }

    @Override
    public boolean isFinished() {
        telemetry.addData("Auto Inches",this.inches);
        telemetry.addData("Auto Pos",this.drive.getOdomY());
        telemetry.update();
        // left is negative, right is positive, both are Y for otos
        if(this.inches<0) {
            drive.drive(.0,-.4,0);
            if(this.drive.getOdomY()<this.inches) {
                drive.drive(0,0,0);
                return true;
            }
        }
        else {
            drive.drive(.0,.4,0);
            if(this.drive.getOdomY()>this.inches) {
                drive.drive(0,0,0);
                return true;
            }
        }

        return false;
    }
}
