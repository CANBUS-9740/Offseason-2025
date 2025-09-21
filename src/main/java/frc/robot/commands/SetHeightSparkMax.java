package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class SetHeightSparkMax extends Command {

    private final double heightMeters;
    private final ElevatorSystem elevatorSystem;

    public SetHeightSparkMax(double height) {
        elevatorSystem = new ElevatorSystem();
        this.heightMeters = height;

        addRequirements(elevatorSystem);
    }

    @Override
    public void initialize() {
        elevatorSystem.moveToSetPoint(heightMeters);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {return false;}

    @Override
    public void end(boolean interrupted) {
        elevatorSystem.stop();
    }
}
