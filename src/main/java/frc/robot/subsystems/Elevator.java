package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final DigitalInput Limitswitch;
    private RelativeEncoder encoder;


    private double outputKS=0.2;

    public Elevator() {
        leftMotor = new SparkMax(RobotMap.LEFTM, SparkLowLevel.MotorType.kBrushless);
        rightMotor = new SparkMax(RobotMap.RIGHTM, SparkLowLevel.MotorType.kBrushless);
        Limitswitch = new DigitalInput(RobotMap.LIMITER);
        encoder = leftMotor.getEncoder();

    }

    public void move(double pow) {
        leftMotor.set(pow+outputKS);
        rightMotor.set(-pow-outputKS);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public double getDistancePassedMeters() {
        return encoder.getPosition()*RobotMap.CIRCUMFERENCE;
    }

    public boolean getLimitSwitch() {
        return Limitswitch.get();
    }
    public void ResetEncoder(){
        encoder.setPosition(0);
    }

}
