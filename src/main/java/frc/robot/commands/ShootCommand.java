package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends Command {
    private final Shooter shooter;

    public ShootCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);

    }
    @Override
    public void initialize() {
        shooter.motormove(0.5);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return !(shooter.hasCoral());
    }

    @Override
    public void end(boolean wasInterrupted) {
        shooter.stop();
    }

}
