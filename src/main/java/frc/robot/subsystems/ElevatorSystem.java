package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ElevatorSystem extends SubsystemBase {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private double targetHeight;


    public ElevatorSystem() {
        motor = new SparkMax(RobotMap.ELEVATOR_MOTOR, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkMaxConfig configMaster = new SparkMaxConfig();
        configMaster.closedLoop
                .p(RobotMap.ELEVATOR_P)
                .i(RobotMap.ELEVATOR_I)
                .d(RobotMap.ELEVATOR_D)
                .velocityFF(RobotMap.ELEVATOR_FF);
        configMaster.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        configMaster.encoder.positionConversionFactor(1 / RobotMap.GEAR_RATIO_ELEVATOR);
        motor.configure(configMaster, SparkBase.ResetMode.kResetSafeParameters , SparkBase.PersistMode.kNoPersistParameters);

        SparkMaxConfig configFollow = new SparkMaxConfig();
        configFollow.follow(motor).inverted(true);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getHeight(){
        return encoder.getPosition() * RobotMap.CIRCUMFERNCE_MM;
    }

    public void moveToHeight(double targetHeight){
        motor.getClosedLoopController().setReference(targetHeight / RobotMap.CIRCUMFERNCE_MM, SparkBase.ControlType.kPosition);
    }

    public boolean isAtHeight(double targetHeight){
        return MathUtil.isNear(targetHeight,getHeight(), 0.1);
    }

}
