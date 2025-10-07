package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
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

    public ElevatorMoveCommand(ElevatorSystem elevator) {
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
            System.out.println("nono");

            if (targetHeight == RobotMap.ELEVATOR_PARKING_HEIGHT_M && getIsNear()) {
                elevator.stop();
                System.out.println("parkingMode");
            } else {
                motionProfile = new TrapezoidProfile(RobotMap.ELEVATOR_MOTION_PROFILE_CONSTRAINTS);
                motionProfileGoal = new TrapezoidProfile.State(targetHeight, 0);
                motionProfileSetPoint = new TrapezoidProfile.State(elevator.getHeightMeters(), 0);
            }
        }

        if (elevator.getLowerLimit()) {
            elevator.setEncoderHeight(0);
        }

        SmartDashboard.putBoolean("ElevatorAtTargetHeight", getIsNear());
        SmartDashboard.putNumber("ElevatorTargetHeightMeters", targetHeight);

        if (getIsNear()) {
            if (MathUtil.isNear(RobotMap.ELEVATOR_PARKING_HEIGHT_M, targetHeight, 0.01)) {
                // we've reached position, but it is position 0 (floor). So stop holding
                elevator.stop();
            } else {
                // if we've reached our position, just set the PID to the setpoint and let it and the FF hold the elevator in position.
                elevator.moveToHeight(targetHeight);
                System.out.println("stopi");
            }
        } else {
            // if we are still on our way to the target height, continue running the motion profile
            System.out.println("go");
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);
            elevator.moveToHeight(motionProfileSetPoint.position);
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void setTargetHeight(double height) {
        if (height >= RobotMap.ELEVATOR_MIN_HEIGHT_M && height <= RobotMap.ELEVATOR_MAX_HEIGHT_M) {
            targetHeight = height;
            System.out.println("no biatch");
            setNewTarget = true;
        } else {
            DriverStation.reportWarning("Request to set height out of range", true);
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

