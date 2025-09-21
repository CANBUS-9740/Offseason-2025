package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorMoveCommand extends Command {
    private final ElevatorSystem elevator;
    private final double targetHeight;

    public ElevatorMoveCommand(ElevatorSystem elevator, double targetHeight){
        this.elevator = elevator;
        this.targetHeight = targetHeight;

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
        return elevator.isAtHeight(targetHeight);
    }
}

