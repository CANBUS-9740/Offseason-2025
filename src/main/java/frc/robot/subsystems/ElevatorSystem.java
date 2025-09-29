package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ElevatorSystem extends SubsystemBase {
    private final SparkMax motorMaster;
    private final SparkMax motorFollow;
    private final AbsoluteEncoder encoder;


    public ElevatorSystem() {
        motorMaster = new SparkMax(RobotMap.ELEVATOR_MASTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        motorFollow = new SparkMax(RobotMap.ELEVATOR_FOLLOW_MOTOR, SparkLowLevel.MotorType.kBrushless);
        encoder = motorMaster.getAbsoluteEncoder();

        SparkMaxConfig configMaster = new SparkMaxConfig();
        configMaster.closedLoop
                .p(RobotMap.ELEVATOR_P)
                .i(RobotMap.ELEVATOR_I)
                .d(RobotMap.ELEVATOR_D)
                .velocityFF(RobotMap.ELEVATOR_FF);
        configMaster.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        configMaster.encoder.positionConversionFactor(1 / RobotMap.GEAR_RATIO_ELEVATOR);
        motorMaster.configure(configMaster, SparkBase.ResetMode.kResetSafeParameters , SparkBase.PersistMode.kNoPersistParameters);

        SparkMaxConfig configFollow = new SparkMaxConfig();
        configFollow.follow(motorMaster).inverted(true);
        motorFollow.configure(configFollow, SparkBase.ResetMode.kResetSafeParameters , SparkBase.PersistMode.kNoPersistParameters);
    }

    public void stop() {
        motorMaster.stopMotor();
    }

    public double getHeightM(){
        return encoder.getPosition() * RobotMap.CIRCUMFERENCE_M / 10;
    }

    public void moveToHeight(double targetHeightM){
        motorMaster.getClosedLoopController().setReference(targetHeightM / RobotMap.CIRCUMFERENCE_M, SparkBase.ControlType.kPosition);
    }

    public boolean isAtHeight(double targetHeightM){
        return MathUtil.isNear(targetHeightM,getHeightM(), 0.1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ElevatorHeightM", getHeightM());
        SmartDashboard.putNumber("encoderPosition", encoder.getPosition());
    }
}
