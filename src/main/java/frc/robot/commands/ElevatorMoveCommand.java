package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorMoveCommand extends Command {
    private final ElevatorSystem elevator;
    private double targetHeight;

    public ElevatorMoveCommand(ElevatorSystem elevator){
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.moveToHeight(targetHeight);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return  false; //elevator.isAtHeight(targetHeight);
    }

    public void setTargetHeight(double height) {
        targetHeight = height;
    }

    public boolean getIsNear(double height) {
        return elevator.isAtHeight(height);
    }
}

