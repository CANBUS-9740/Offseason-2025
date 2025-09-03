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
    private final SparkMax motorMaster;
    private final SparkMax motorFollow;
    private final RelativeEncoder encoder;
    private double targetHeight;


    public ElevatorSystem() {
        motorMaster = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
        motorFollow = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
        encoder = motorMaster.getEncoder();

        SparkMaxConfig configMaster = new SparkMaxConfig();
        configMaster.closedLoop
                .p(RobotMap.ELEVATOR_P)
                .i(RobotMap.ELEVATOR_I)
                .d(RobotMap.ELEVATOR_D);
        configMaster.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        configMaster.encoder.positionConversionFactor(1 / RobotMap.GEAR_RATIO_ELEVATOR);
        motorMaster.configure(configMaster, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        SparkMaxConfig configFollow = new SparkMaxConfig();
        configFollow.follow(motorMaster).inverted(true);
        motorFollow.configure(configFollow, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public void stop() {
        motorMaster.stopMotor();
    }

    public double getHeight(){
        return encoder.getPosition() * RobotMap.CIRCUMFERNCE_MM;
        //המשך חישוב?
    }

    public void moveToHeight(double targetHeight){
        motorMaster.getClosedLoopController().setReference(targetHeight / RobotMap.CIRCUMFERNCE_MM, SparkBase.ControlType.kPosition);
    }

    public boolean isAtHeight(double targetHeight){
        return MathUtil.isNear(targetHeight,getHeight(), 0.1);
    }

}
