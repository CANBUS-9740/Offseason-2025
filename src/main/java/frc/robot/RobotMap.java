package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class RobotMap {
    private RobotMap() {}

    public static final double STAND_SELECTION_HEADING_MARGIN = 5;
    public static final double STAND_SELECTION_GENERAL_ORIENTATION_MARGIN = 45;

    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getKrakenX60(1);
    public static final double SWERVE_DRIVE_MAX_SPEED_MPS = Units.radiansToRotations(SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec) / 6.12 * 0.05;
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2.8, 1.2, Math.PI, Math.PI);
}
