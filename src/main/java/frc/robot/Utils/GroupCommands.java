package frc.robot.Utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

    public final ElevatorSystem elevatorSystem;
    public final ShooterSystem shooterSystem;
    private final Swerve swerve;
    private final GameField gameField;
    private final PathPlanner pathPlanner;
    private final CommandXboxController controller;


    public GroupCommands(CommandXboxController xboxController, Swerve swerve) {
        elevatorSystem = new ElevatorSystem();
        shooterSystem = new ShooterSystem();
        gameField = new GameField();
        pathPlanner = new PathPlanner(swerve);

        this.swerve = swerve;
        this.controller = xboxController;
    }

    public Command resetCommand() {
        return Commands.defer(Commands::none, Set.of(swerve, shooterSystem, elevatorSystem));
    }

    public Command shootCommand() {
        return new ShootCommand(shooterSystem);
    }

    public Command intakeCommand() {
        return new IntakeCommand(shooterSystem);
    }

    public Command coralOnReefStage(CoralReef stage, GameField.ReefStand reefStand, GameField.ReefStandSide reefStandSide ) {
        double elevatorHeight = getStageHeight(stage);

        return Commands.defer(() -> {

            if(shooterSystem.hasCoral()){
                return  Commands.none();
            } else if (GameField.getDistanceToMeters(swerve.getPose(), gameField.getPoseForReefStand(reefStand, reefStandSide)) < 0.3) {
                return new SequentialCommandGroup(
                        pathPlanner.goToPoseReef(reefStand, reefStandSide),
                        new ElevatorMoveCommand(elevatorSystem, elevatorHeight),
                        new ShootCommand(shooterSystem)
                );
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToReefPathFind(reefStand, reefStandSide),
                    new ElevatorMoveCommand(elevatorSystem, elevatorHeight),
                    new ShootCommand(shooterSystem)
            );
        }, Set.of(elevatorSystem, shooterSystem));
    }

    public Command coralOnClosestReefStage(CoralReef stage) {
        double elevatorHeight = getStageHeight(stage);

        return Commands.defer(() -> {

            if(shooterSystem.hasCoral() || !pathPlanner.closestReefIsPresent()){
                return  Commands.none();
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToClosestReef(),
                    new ElevatorMoveCommand(elevatorSystem, elevatorHeight),
                    new ShootCommand(shooterSystem)
            );
        }, Set.of(elevatorSystem, shooterSystem));
    }

    public Command GetCoralFromSource(GameField.SourceStand sourceStand, GameField.SourceStandSide sourceStandSide) {
        return Commands.defer(() -> {

            if(!shooterSystem.hasCoral()){
                return  Commands.none();
            } else if (GameField.getDistanceToMeters(swerve.getPose(), gameField.getPoseForSource(sourceStand, sourceStandSide)) < 0.3) {
                return new SequentialCommandGroup(
                        pathPlanner.goToPoseSource(sourceStand,sourceStandSide),
                        new ElevatorMoveCommand(elevatorSystem, RobotMap.SOURCE_HEIGHT),
                        new IntakeCommand(shooterSystem)
                );
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToSourcePathFind(sourceStand,sourceStandSide),
                    new ElevatorMoveCommand(elevatorSystem, RobotMap.SOURCE_HEIGHT),
                    new IntakeCommand(shooterSystem)
            );
        }, Set.of(elevatorSystem, shooterSystem));
    }

    public Command getCoralFromClosestSource() {
        return Commands.defer(() -> {

            if(!shooterSystem.hasCoral() || !pathPlanner.closestSourceIsPresent()){
                return  Commands.none();
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToClosestSource(),
                    new ElevatorMoveCommand(elevatorSystem, RobotMap.SOURCE_HEIGHT),
                    new IntakeCommand(shooterSystem)
            );
        }, Set.of(elevatorSystem, shooterSystem));
    }

    public SwerveDriveCommand swerveDrive(boolean fieldDrive){
        return new SwerveDriveCommand(swerve, controller, fieldDrive);
    }

    private double getStageHeight(CoralReef stage) {
        double elevatorHeight;

        switch (stage) {
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

        return elevatorHeight;
    }

    public void update() {
        pathPlanner.update();
    }

}
