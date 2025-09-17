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
    private final DigitalInput Limitswitch;
    private final RelativeEncoder encoder;
    public final SparkClosedLoopController sparkPIDcontroller;




    public ElevatorSystem() {
        leftMotor = new SparkMax(RobotMap.LEFT_MOTORID, SparkLowLevel.MotorType.kBrushless);
        rightMotor = new SparkMax(RobotMap.RIGHT_MOTORID, SparkLowLevel.MotorType.kBrushless);
        Limitswitch = new DigitalInput(RobotMap.ELEVATOR_LIMITSWITCH);
        encoder = leftMotor.getEncoder();

        sparkPIDcontroller = leftMotor.getClosedLoopController();
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
                .p(RobotMap.KP)
                .i(RobotMap.KI)
                .d(RobotMap.KD)
                .velocityFF(RobotMap.ELEVATOR_FF)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);



        SparkMaxConfig configright = new SparkMaxConfig();
        configright.follow(leftMotor,true);

        rightMotor.configure(configright, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }




    public void move(double pow) {
        leftMotor.set(pow + RobotMap.ELEVATOR_FF);
    }

    public void stop() {
        leftMotor.stopMotor();
    }

    public double getPositionMeters() {
        return encoder.getPosition() * RobotMap.CIRCUMFERENCE;
    }

    public boolean getLimitSwitch() {
        return Limitswitch.get();
    }

    public void ResetEncoder(){
        encoder.setPosition(0);
    }

    public void moveToSetPointSpark(double setpoint){
        sparkPIDcontroller.setReference(setpoint / RobotMap.CIRCUMFERENCE, SparkBase.ControlType.kPosition);

    }



}
