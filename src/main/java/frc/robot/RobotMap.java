package frc.robot;

public class RobotMap {
    public static final int LEFT_MOTORID = 0;
    public static final int RIGHT_MOTORID = 1;
    public static final int ELEVATOR_LIMITSWITCH = 2;
    public static final double CIRCUMFERENCE = 0.1;
    public static final double ELEVATOR_MAX = 10.0;

    //PID VALUES
    public static final double KP = 0.05;
    public static final double KI = 0;
    public static final double KD = 0.1;
    public static final double ELEVATOR_FF = 0.2;

    private RobotMap() {
    }

    // add constants here
    // public static final type NAME = value;
}
