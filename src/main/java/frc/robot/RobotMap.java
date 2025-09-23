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

    public static final double STAND_SELECTION_HEADING_MARGIN = 5;
    public static final double STAND_SELECTION_GENERAL_ORIENTATION_MARGIN = 45;

    public static final int SHOOTER_MOTOR_ID = 0;
    public static final int SHOOTER_IR_PROXIMITY_SENSOR_ID = 1;
    public static final double SHOOTER_MOTOR_GEAR_RATIO = 3;


    // add constants here
    // public static final type NAME = value;
    //Elevator system
    public static final int ELEVATOR_MASTER_MOTOR = 0;
    public static final int ELEVATOR_FOLLOW_MOTOR = 1;
    public static final double ELEVATOR_P = 0;
    public static final double ELEVATOR_I = 0;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_FF = 0;
    public static final double CIRCUMFERNCE_MM = 0.020803;
    public static final double GEAR_RATIO_ELEVATOR = 15;
    public static final double ROBOT_HEIGHT_M = 0;
    public static final double SOURCE_HEIGHT = 1.35;
    public static final double ELEVATOR_L1_HEIGHT_M = 0;
    public static final double ELEVATOR_L2_HEIGHT_M = 0.46;
    public static final double ELEVATOR_L3_HEIGHT_M = 0.81;
    public static final double ELEVATOR_L4_HEIGHT_M = 1.21;

}
