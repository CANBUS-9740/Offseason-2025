package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
    private final SparkMax shoot;
    private final DigitalInput proximity;

    public ShooterSystem() {
        shoot = new SparkMax(RobotMap.SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        proximity = new DigitalInput(RobotMap.IR_PROXIMITY_SENSOR);

        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.
                positionConversionFactor(1 / RobotMap.GEAR_RATIO).
                velocityConversionFactor(1 / RobotMap.GEAR_RATIO);
    }

    public boolean hasCoral() {
        return proximity.get();
    }

    public void MotorMove(double power){
        shoot.set(power);
    }



    public void stop(){
        shoot.stopMotor();
    }
}
