package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
    private final SparkMax shoot;
    private final DigitalInput proximity;

    public ShooterSystem() {
        shoot = new SparkMax(RobotMap.SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        proximity = new DigitalInput(RobotMap.SHOOTER_IR_PROXIMITY_SENSOR_ID);

        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.
                positionConversionFactor(1 / RobotMap.SHOOTER_MOTOR_GEAR_RATIO).
                velocityConversionFactor(1 / RobotMap.SHOOTER_MOTOR_GEAR_RATIO);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.smartCurrentLimit(0, 0);//test the limit

        shoot.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    public boolean hasCoral() {
        return proximity.get();
    }

    public void motorMove(double power){
        shoot.set(power);
    }

    public void stop(){
        shoot.stopMotor();
    }
}
