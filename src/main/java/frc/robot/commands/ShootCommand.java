package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class ShootCommand extends Command {

    private static final double POST_SEE_DELAY_SEC = 0.3;

    private final ShooterSystem shooterSystem;
    private final Timer timer;

    public ShootCommand(ShooterSystem shooterSystem) {
        this.shooterSystem = shooterSystem;
        timer = new Timer();
        addRequirements(shooterSystem);

    }
    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        shooterSystem.motorMove(5);
        if (!shooterSystem.hasCoral()) {
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(POST_SEE_DELAY_SEC);
    }

    @Override
    public void end(boolean wasInterrupted) {
        shooterSystem.stop();
        timer.stop();
    }

}
