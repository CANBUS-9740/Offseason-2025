package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
    private final SparkMax motor;
    private final Ultrasonic proximity;

    public ShooterSystem() {
        motor = new SparkMax(RobotMap.SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        proximity = new Ultrasonic(RobotMap.ULTRASONIC_SENSOR_PING_PORT,RobotMap.ULTRASONIC_SENSOR_ECHO_PORT);
        Ultrasonic.setAutomaticMode(true);

        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.
                positionConversionFactor(1 / RobotMap.SHOOTER_MOTOR_GEAR_RATIO).
                velocityConversionFactor(1 / RobotMap.SHOOTER_MOTOR_GEAR_RATIO);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.smartCurrentLimit(0, 0);//test the limit

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public double getDistanceMeters(){
        return proximity.getRangeMM() / 1000.0;
    }

    public void motorMove(double power){
        motor.set(power);
    }

    public void stop(){
        motor.stopMotor();
    }

    public boolean hasCoral() {
        return getDistanceMeters() <= 0.1;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ShooterSonic", getDistanceMeters());
        SmartDashboard.putBoolean("HasCoral", hasCoral());
    }
}
