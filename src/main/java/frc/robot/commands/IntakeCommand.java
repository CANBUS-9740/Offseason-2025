package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class IntakeCommand extends Command {
    private final ShooterSystem shooterSystem;
    private double startdist;//in CM

    public IntakeCommand(ShooterSystem shooterSystem) {
        this.shooterSystem = shooterSystem;
        addRequirements(shooterSystem);
    }
    @Override
    public void initialize() {startdist=shooterSystem.getDistance();}

    @Override
    public void execute() {
        shooterSystem.motorMove(-0.25);
    }

    @Override
    public boolean isFinished() {
        return shooterSystem.getDistance()<=startdist/2;
    }

    @Override
    public void end(boolean wasInterrupted) {
        shooterSystem.stop();
    }

}

