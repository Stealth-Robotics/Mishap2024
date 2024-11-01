package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ElevatorReset extends CommandBase {

    private final ElevatorSubsystem elevatorSubsystem;

    
    


    public ElevatorReset(ElevatorSubsystem elevatorSubsystem){

        this.elevatorSubsystem = elevatorSubsystem;
        
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize(){
        elevatorSubsystem.resetMotor();
    }

}