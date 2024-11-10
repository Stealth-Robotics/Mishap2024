package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;

public class DriveForwardInches extends CommandBase {

    private final double inches;
    final SimpleMecanumDriveSubsystem drive;

    final Telemetry telemetry;
    public DriveForwardInches(Telemetry telemetry, SimpleMecanumDriveSubsystem drive, double inches){

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
        telemetry.addData("Auto Pos",this.drive.getOdomX());
        telemetry.update();

        if(this.inches<0) {
            drive.drive(.4,0,0);
            if(this.drive.getOdomX()<this.inches) {
                drive.drive(0,0,0);
                return true;
            }
        }
        else {
            drive.drive(-.4,0,0);
            if(this.drive.getOdomX()>this.inches) {
                drive.drive(0,0,0);
                return true;
            }
        }

        return false;
    }
}
