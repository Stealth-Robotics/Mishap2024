package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDriveSubsystem;

public class TurnToDegrees extends CommandBase {

    private final double degrees;
    final SimpleMecanumDriveSubsystem drive;

    final Telemetry telemetry;
    public TurnToDegrees(Telemetry telemetry, SimpleMecanumDriveSubsystem drive, double degrees){

        this.degrees = degrees;
        this.drive = drive;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize() {
        telemetry.addData("Degrees",this.degrees);
        telemetry.addData("reset",0);
        telemetry.update();
        this.drive.resetOdom();
    }

    @Override
    public boolean isFinished() {
        telemetry.addData("Auto Degrees",this.degrees);
        telemetry.addData("Auto Pos",this.drive.getOdomHeading());
        telemetry.update();

        if(this.degrees<0) {
            drive.drive(0,0,0.6);
            if(this.drive.getOdomHeading()<this.degrees) {
                drive.drive(0,0,0);
                return true;
            }
        }
        else {
            drive.drive(0,0,-0.6);
            if(this.drive.getOdomHeading()>this.degrees) {
                drive.drive(0,0,0);
                return true;
            }
        }

        return false;
    }
}
