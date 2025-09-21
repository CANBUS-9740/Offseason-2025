package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ElevatorSystem extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final DigitalInput limitswitch;
    private final RelativeEncoder leftEncoder;
    public final SparkClosedLoopController sparkPIDcontroller;

    public ElevatorSystem() {
        leftMotor = new SparkMax(RobotMap.LEFT_MOTORID, SparkLowLevel.MotorType.kBrushless);
        rightMotor = new SparkMax(RobotMap.RIGHT_MOTORID, SparkLowLevel.MotorType.kBrushless);
        limitswitch = new DigitalInput(RobotMap.ELEVATOR_LIMITSWITCH);
        leftEncoder = leftMotor.getEncoder();

        sparkPIDcontroller = leftMotor.getClosedLoopController();
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
                .p(RobotMap.KP)
                .i(RobotMap.KI)
                .d(RobotMap.KD)
                .velocityFF(RobotMap.ELEVATOR_FF)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        leftMotor.configure(config,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);

        SparkMaxConfig configrRight = new SparkMaxConfig();
        configrRight.follow(leftMotor,true);
        rightMotor.configure(configrRight,SparkBase.ResetMode.kResetSafeParameters,SparkBase.PersistMode.kPersistParameters);

        rightMotor.configure(configrRight, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    public void move(double pow) {
        leftMotor.set(RobotMap.ELEVATOR_FF);
    }

    public void stop() {
        leftMotor.stopMotor();
    }

    public double getPositionMeters() {
        return leftEncoder.getPosition() * RobotMap.CIRCUMFERENCE;
    }

    public boolean getLimitSwitch() {
        return limitswitch.get();
    }

    public void ResetEncoder(){
        leftEncoder.setPosition(0);
    }

    public void moveToSetPoint(double setpoint){
        sparkPIDcontroller.setReference(setpoint / RobotMap.CIRCUMFERENCE, SparkBase.ControlType.kPosition);

    }



}
