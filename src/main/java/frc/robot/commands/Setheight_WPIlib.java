package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ElevatorSystem;

public class Setheight_WPIlib extends Command {
    private double height,processvariable,output;
    private final ElevatorSystem elevatorSystem;

    //PID values
    private static final double KP = RobotMap.KP;
    private static final double KI = RobotMap.KI;
    private static final double KD = RobotMap.KD;
    private final PIDController pid;


    public Setheight_WPIlib(ElevatorSystem elevatorSystem, double height) {
        this.elevatorSystem = elevatorSystem;
        this.pid = new PIDController(KP, KI, KD);

        addRequirements(elevatorSystem);

    }

    @Override
    public void initialize() {
        pid.reset();

    }

    @Override
    public void execute() {
        processvariable= elevatorSystem.getPositionMeters();
        output = pid.calculate(processvariable);
        elevatorSystem.move(output);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSystem.stop();
    }
}
