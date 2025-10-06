package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ElevatorSystem extends SubsystemBase {

    private final SparkMax masterMotor;
    private final SparkMax followMotor;
    private final RelativeEncoder encoder;
    private final DigitalInput lowerLimit;

    SparkMaxConfig configMaster;

    public ElevatorSystem() {
        masterMotor = new SparkMax(RobotMap.ELEVATOR_MASTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        followMotor = new SparkMax(RobotMap.ELEVATOR_FOLLOW_MOTOR, SparkLowLevel.MotorType.kBrushless);
        encoder = masterMotor.getEncoder();
        lowerLimit = new DigitalInput(0);

        SparkMaxConfig configMaster = new SparkMaxConfig();
        this.configMaster = configMaster;
        configMaster.inverted(true);
        configMaster.closedLoop
                .p(RobotMap.ELEVATOR_P)
                .i(RobotMap.ELEVATOR_I)
                .d(RobotMap.ELEVATOR_D);
        configMaster.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        configMaster.encoder
                .positionConversionFactor(1 / RobotMap.GEAR_RATIO_ELEVATOR)
                .velocityConversionFactor(1 / RobotMap.GEAR_RATIO_ELEVATOR);
        configMaster.idleMode(SparkBaseConfig.IdleMode.kBrake);
        masterMotor.configure(configMaster, SparkBase.ResetMode.kResetSafeParameters , SparkBase.PersistMode.kNoPersistParameters);

        SparkMaxConfig configFollow = new SparkMaxConfig();
        configFollow.follow(masterMotor, true);
        configFollow.idleMode(SparkBaseConfig.IdleMode.kBrake);
        followMotor.configure(configFollow, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        encoder.setPosition(0);
    }

    public void stop() {
        masterMotor.stopMotor();
    }

    public double getHeightMeters(){
        return encoder.getPosition() * RobotMap.CIRCUMFERNCE_MM;
    }

    public void moveToHeight(double targetHeight) {
        SmartDashboard.putNumber("ElevatorSetPointMeters", targetHeight);
        SmartDashboard.putNumber("ElevatorSetPointRots", targetHeight / RobotMap.CIRCUMFERNCE_MM);

        masterMotor.getClosedLoopController().setReference(targetHeight / RobotMap.CIRCUMFERNCE_MM, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, RobotMap.ELEVATOR_FF_VOLTAGE, SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    public boolean isAtHeight(double targetHeight){
        return MathUtil.isNear(targetHeight, getHeightMeters(), 0.02) && Math.abs(encoder.getVelocity()) < RobotMap.ELEVATOR_VELOCITY_RPM_THRESHOLD;
    }

    public boolean getLowerLimit() {
        return !lowerLimit.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ElevatorHeightMeters", getHeightMeters());
        SmartDashboard.putNumber("ElevatorHeightRots", encoder.getPosition());
        SmartDashboard.putNumber("ElevatorMasterCurrent", masterMotor.getOutputCurrent());
        SmartDashboard.putNumber("ElevatorFollowerCurrent", followMotor.getOutputCurrent());
        SmartDashboard.putBoolean("ElevatorLowerLimit", getLowerLimit());
    }

    public void setKp(double kp) {
        configMaster.closedLoop.p(kp);
        masterMotor.configure(configMaster, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }
}
