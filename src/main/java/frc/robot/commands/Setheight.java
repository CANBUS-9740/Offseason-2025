package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

public class Setheight extends Command {
    private double height;
    private final Elevator elevator;
    private double NR;//Number of Revolutions
    private double CH;//Current Height
    private double pow;

    public Setheight(Elevator elevator, double height) {
        this.elevator = elevator;
        this.height = height;
    }

    @Override
    public void initialize() {
        NR = this.height / RobotMap.CIRCUMFERENCE;
        CH = RobotMap.CIRCUMFERENCE * elevator.getmotorPosition();
        if (height < CH) {
            elevator.move(0.3);
        } else if (height > NR) {
            elevator.move(-0.3);
        } else {
            elevator.stop();
        }
    }

    @Override
    public void execute() {
        if (CH > this.height + 10) {
            initialize();
        }
    }

    @Override
    public boolean isFinished() {
        return CH > this.height - 10 && CH < this.height + 10;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
