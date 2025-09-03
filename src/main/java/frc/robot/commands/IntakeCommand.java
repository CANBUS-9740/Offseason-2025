package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {
    private final Shooter shooter;

//teest
    public IntakeCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }


    @Override
    public void initialize() {
        shooter.motormove(0.25);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return shooter.hasCoral();
    }

    @Override
    public void end(boolean wasInterrupted) {
        shooter.stop();
    }

}

