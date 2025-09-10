package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class Setheight extends Command {
    private double height,processvariable,output;
    private final Elevator elevator;

    //PID values
    private static final double KP = 0.05;
    private static final double KI = 0;
    private static final double KD = 0.1;
    private final PIDController controller;


    public Setheight(Elevator elevator, double height) {
        this.elevator = elevator;
        this.controller = new PIDController(KP, KI, KD);
        this.controller.setSetpoint(height);
        addRequirements(elevator);

    }

    @Override
    public void initialize() {
        controller.reset();

    }

    @Override
    public void execute() {
        processvariable= elevator.getDistancePassedMeters();
        output = controller.calculate(processvariable);
        elevator.move(output);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
