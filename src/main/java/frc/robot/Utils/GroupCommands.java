package frc.robot.Utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CoralReef;
import frc.robot.RobotMap;
import frc.robot.commands.*;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.Swerve;

import java.util.Set;


public class GroupCommands {

     /*
    group commands:

        -coral on reef:
        swerve position to reef
        enum for podium, first second and third stage.
        optional-shoot the coral
        constants: stage heights

        -get from source:
        swerve position to source
        elevator to source height, intake
        constants: source height
     */

    private final ElevatorSystem elevatorSystem;
    private final ShooterSystem shooterSystem;
    private final Swerve swerve;
    private final GameField gameField;
    private final PathPlanner pathPlanner;
    private final XboxController controller;


    public GroupCommands(XboxController xboxController) {
        elevatorSystem = new ElevatorSystem();
        shooterSystem = new ShooterSystem();
        gameField = new GameField();
        pathPlanner = new PathPlanner();
        swerve = new Swerve();
        this.controller = xboxController;
    }

    public Command coralOnReefStage(CoralReef reef, GameField.ReefStand reefStand, GameField.ReefStandSide reefStandSide ) {
        double elevatorHeight;

        switch (reef) {
            case PODIUM:
                elevatorHeight = RobotMap.ELEVATOR_L1_HEIGHT_M;
                break;
            case FIRST_STAGE:
                elevatorHeight = RobotMap.ELEVATOR_L2_HEIGHT_M;
                break;
            case SECOND_STAGE:
                elevatorHeight = RobotMap.ELEVATOR_L3_HEIGHT_M;
                break;
            case THIRD_STAGE:
                elevatorHeight = RobotMap.ELEVATOR_L4_HEIGHT_M;
                break;
            default:
                elevatorHeight = RobotMap.ELEVATOR_L1_HEIGHT_M;
                break;
        }

        return Commands.defer(() -> {

            if(shooterSystem.hasCoral()){
                return  Commands.none();
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToPoseReef(reefStand, reefStandSide),
                    new ElevatorMoveCommand(elevatorSystem, elevatorHeight),
                    new ShootCommand(shooterSystem)
            );
        }, Set.of(elevatorSystem, shooterSystem));
    }


    public Command GetCoralFromSource(GameField.SourceStand sourceStand, GameField.SourceStandSide sourceStandSide) {
        return Commands.defer(() -> {

            if(!shooterSystem.hasCoral()){
                return  Commands.none();
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToPoseSource(sourceStand,sourceStandSide),
                    new ElevatorMoveCommand(elevatorSystem, RobotMap.SOURCE_HEIGHT),
                    new IntakeCommand(shooterSystem)
            );
        }, Set.of(elevatorSystem, shooterSystem));
    }

    public Command swerveDrive(){
        return new SwerveDriveCommand(swerve,controller);
    }

    public Command swerveFieldDrive(){
        return new SwerveFieldDriveCommand(swerve,controller);
    }
}
