package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSystem;

public class IntakeCommand extends Command {

    private static final double POST_SEE_DELAY_SEC = 0.1;

    private final ShooterSystem shooterSystem;
    private final Timer timer;

    public IntakeCommand(ShooterSystem shooterSystem) {
        this.shooterSystem = shooterSystem;
        this.timer = new Timer();
        addRequirements(shooterSystem);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        shooterSystem.motorMove(RobotMap.SHOOTER_MOTOR_INTAKE_SPEED);
        if (shooterSystem.hasCoral()) {
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

