package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorMoveCommand extends Command {
    private final ElevatorSystem elevator;
    private double targetHeight;
    private boolean setNewTarget;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;

    public ElevatorMoveCommand(ElevatorSystem elevator){
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        setNewTarget = true;
        targetHeight = RobotMap.ELEVATOR_PARKING_HEIGHT_M;
    }

    @Override
    public void execute() {
        if (setNewTarget) {
            setNewTarget = false;

            if (targetHeight == RobotMap.ELEVATOR_PARKING_HEIGHT_M && getIsNear()) {
                elevator.stop();
            } else {
                motionProfile = new TrapezoidProfile(RobotMap.ARM_JOINT_MOTION_PROFILE_CONSTRAINTS);
                motionProfileGoal = new TrapezoidProfile.State(targetHeight, 0);
                motionProfileSetPoint = new TrapezoidProfile.State(elevator.getHeightMeters(), 0);

                motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);
                elevator.moveToHeight(motionProfileSetPoint.position);
            }
        }

        SmartDashboard.putBoolean("ElevatorAtTargetHeight", getIsNear());
        SmartDashboard.putNumber("ElevatorTargetHeightMeters", targetHeight);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false; //elevator.isAtHeight(targetHeight);
    }

    public void setTargetHeight(double height) {
        if (height > RobotMap.ELEVATOR_MIN_HEIGHT_M && height < RobotMap.ELEVATOR_MAX_HEIGHT_M) {
            targetHeight = height;
            setNewTarget = true;
        }
    }

    public boolean getIsNear() {
        if (setNewTarget) {
            return false;
        } else {
            return elevator.isAtHeight(targetHeight);
        }
    }
}

