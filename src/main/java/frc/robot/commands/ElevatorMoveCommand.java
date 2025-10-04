package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorMoveCommand extends Command {
    private final ElevatorSystem elevator;
    private double targetHeight;
    private boolean setNewTarget;

    public ElevatorMoveCommand(ElevatorSystem elevator){
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        setNewTarget = true;
        targetHeight = 0;
    }

    @Override
    public void execute() {
        if (setNewTarget) {
            elevator.moveToHeight(targetHeight);
            setNewTarget = false;
        }

        SmartDashboard.putBoolean("ElevatorAtTargetHeight", getIsNear(targetHeight));
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false; //elevator.isAtHeight(targetHeight);
    }

    public void setTargetHeight(double height) {
        targetHeight = height;
        setNewTarget = true;
    }

    public boolean getIsNear(double height) {
        return elevator.isAtHeight(height);
    }
}

