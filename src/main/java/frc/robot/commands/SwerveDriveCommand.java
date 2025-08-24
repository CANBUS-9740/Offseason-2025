package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;

public class SwerveDriveCommand extends Command {
    private Swerve swerveSub;
    private XboxController xboxController;

    public SwerveDriveCommand(Swerve swerveSub, XboxController xboxController) {
        this.swerveSub = swerveSub;
        this.xboxController = xboxController;

        addRequirements(swerveSub);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double x = MathUtil.applyDeadband(xboxController.getLeftY(), 0.1) * RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS * 2;
        double y = MathUtil.applyDeadband(xboxController.getLeftX(), 0.1) * RobotMap.SWERVE_DRIVE_MAX_SPEED_MPS * 2;
        double rotation = MathUtil.applyDeadband(xboxController.getRightX(), 0.1) * 10;

        swerveSub.drive(new ChassisSpeeds(x, y, rotation));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
