package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ResetElevator extends Command {
    private final Elevator elevator;
    public ResetElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
    @Override
    public void initialize() {
        elevator.move(-0.2);
    }
    @Override
    public boolean isFinished(){
        return elevator.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
        elevator.ResetEncoder();
    }

}
