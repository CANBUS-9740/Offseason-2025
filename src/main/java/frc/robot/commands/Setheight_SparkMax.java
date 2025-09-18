package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class Setheight_SparkMax extends Command {

    private final double hight;
    private final ElevatorSystem elevatorSystem;

    public Setheight_SparkMax(double height) {
        elevatorSystem = new ElevatorSystem();
        this.hight = height;

        addRequirements(elevatorSystem);
    }

    @Override
    public void initialize() {
        elevatorSystem.moveToSetPointSpark(hight);
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
