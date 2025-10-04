package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class ShootCommand extends Command {

    private final ShooterSystem shooterSystem;

    public ShootCommand(ShooterSystem shooterSystem) {
        this.shooterSystem = shooterSystem;
        addRequirements(shooterSystem);

    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooterSystem.motorMove(0.5);
    }

    @Override
    public boolean isFinished() {
        return !shooterSystem.hasCoral();
    }

    @Override
    public void end(boolean wasInterrupted) {
        shooterSystem.stop();
    }

}
