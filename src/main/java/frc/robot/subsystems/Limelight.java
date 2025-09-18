package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import java.util.Optional;

public class Limelight extends SubsystemBase {
    private String name;

    public Limelight(String name){
        this.name = name;
    }

    public Optional<LimelightHelpers.PoseEstimate> getPose(){
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if(!LimelightHelpers.getTV(name)){
            SmartDashboard.putBoolean("hasAprilTag", false);
            SmartDashboard.putString("HasAprilTagReason", "No Fiducial");
            return Optional.empty();
        }

        SmartDashboard.putBoolean("hasAprilTag", true);
        return Optional.of(poseEstimate);
    }
}
