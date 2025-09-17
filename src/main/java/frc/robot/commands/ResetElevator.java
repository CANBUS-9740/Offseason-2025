package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class ResetElevator extends Command {
    private final ElevatorSystem elevatorSystem;

    public ResetElevator(ElevatorSystem elevatorSystem) {
        this.elevatorSystem = elevatorSystem;

        addRequirements(elevatorSystem);
    }

    @Override
    public void initialize() {
        elevatorSystem.move(-0.2);
    }

    @Override
    public boolean isFinished(){
        return elevatorSystem.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSystem.stop();
        Timer.delay(0.05);
        elevatorSystem.ResetEncoder();
    }

}
