package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class RobotMap {

    private RobotMap() {}

    public static final double STAND_SELECTION_HEADING_MARGIN = 5;
    public static final double STAND_SELECTION_GENERAL_ORIENTATION_MARGIN = 45;

    //Swerve System
    public static final DCMotor SWERVE_DRIVE_MOTOR = DCMotor.getKrakenX60(1);
    public static final double SWERVE_DRIVE_MAX_SPEED_MPS = Units.radiansToRotations(SWERVE_DRIVE_MOTOR.freeSpeedRadPerSec) / 6.12 * 0.05;
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(2, 2.5, Math.PI, Math.PI);
    public static final PIDConstants SWERVE_PATH_DRIVE_PID = new PIDConstants(5, 0, 0);
    public static final PIDConstants SWERVE_PATH_ROTATE_PID = new PIDConstants(3, 0, 0);

    //Shooter system
    public static final int SHOOTER_MOTOR_ID = 46;
    public static final int SHOOTER_IR_PROXIMITY_SENSOR_ID = 1;
    public static final double SHOOTER_MOTOR_GEAR_RATIO = 3;
    public static final double SHOOTER_MOTOR_OUTTAKE_DEFAULT_SPEED = 1;
    public static final double SHOOTER_MOTOR_OUTTAKE_LOWER_SPEED = 0.5;
    public static final double SHOOTER_MOTOR_INTAKE_SPEED = 0.35;
    public static final int ULTRASONIC_SENSOR_PING_PORT = 6;
    public static final int ULTRASONIC_SENSOR_ECHO_PORT = 5;

    //Elevator system
    public static final double ELEVATOR_VELOCITY_RPM_THRESHOLD = 100;
    public static final int ELEVATOR_MASTER_MOTOR = 45;
    public static final int ELEVATOR_FOLLOW_MOTOR = 44;
    public static final double ELEVATOR_P = 0.8;
    public static final double ELEVATOR_I = 0;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_FF_VOLTAGE = 0.3375;
    public static final double CIRCUMFERNCE_MM = 0.717 / 2.52;
    public static final double GEAR_RATIO_ELEVATOR = 20;
    public static final double ROBOT_HEIGHT_M = 0;
    public static final double SOURCE_HEIGHT = 1.35;
    public static final double ELEVATOR_MIN_HEIGHT_M = 0;
    public static final double ELEVATOR_MAX_HEIGHT_M = 1.5;
    public static final double ELEVATOR_PARKING_HEIGHT_M = 0;
    public static final double ELEVATOR_L1_HEIGHT_M = 0.036920;
    public static final double ELEVATOR_L2_HEIGHT_M = 0.333636;
    public static final double ELEVATOR_L3_HEIGHT_M = 0.697081;
    public static final double ELEVATOR_L4_HEIGHT_M = 1.312589
            ;
    public static final TrapezoidProfile.Constraints ELEVATOR_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(3.5, 8);
}
