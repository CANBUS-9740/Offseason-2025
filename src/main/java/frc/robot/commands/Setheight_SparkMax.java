package frc.robot.commands;

import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
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
    public void execute() {
        elevatorSystem.movetosetpointSPARK(this.hight);
    }
    @Override
    public boolean isFinished() {return false;}
    @Override
    public void end(boolean interrupted) {}
}
