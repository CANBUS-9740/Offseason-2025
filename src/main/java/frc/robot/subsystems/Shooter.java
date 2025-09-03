package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {
    private final SparkMax shoot;
    private final DigitalInput proximity;

    public Shooter() {
        shoot=new SparkMax(RobotMap.SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        proximity=new DigitalInput(RobotMap.IR_PROXIMITY_SENSOR);
    }

    public boolean hasCoral() {
        return proximity.get();
    }

    public void motormove(double power){
        shoot.set(power);
    }



    public void stop(){
        shoot.stopMotor();
    }
}
