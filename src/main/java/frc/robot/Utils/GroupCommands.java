package frc.robot.Utils;

import edu.wpi.first.wpilibj2.command.*;
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
    public final ElevatorMoveCommand elevatorMoveCommand;


    public GroupCommands(CommandXboxController xboxController, Swerve swerve) {
        elevatorSystem = new ElevatorSystem();
        shooterSystem = new ShooterSystem();
        gameField = new GameField();
        pathPlanner = new PathPlanner(swerve);
        elevatorMoveCommand = new ElevatorMoveCommand(elevatorSystem);

        this.swerve = swerve;
        this.controller = xboxController;
    }

    public Command resetCommand() {
        return Commands.defer(() -> new InstantCommand(() -> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)), Set.of(swerve, shooterSystem));
    }

    public Command shootCommand(double power) {
        return new ShootCommand(shooterSystem, power);
    }

    public Command intakeCommand() {
        return new IntakeCommand(shooterSystem);
    }

    public Command coralOnReefStage(CoralReef stage, GameField.ReefStand reefStand, GameField.ReefStandSide reefStandSide ) {
        double elevatorHeight = getStageHeight(stage);
        double shooterSpeed;

        if (elevatorHeight == RobotMap.ELEVATOR_L1_HEIGHT_M) {
            shooterSpeed = RobotMap.SHOOTER_MOTOR_OUTTAKE_LOWER_SPEED;
        } else {
            shooterSpeed = RobotMap.SHOOTER_MOTOR_OUTTAKE_DEFAULT_SPEED;
        }

        return Commands.defer(() -> {

            if(!shooterSystem.hasCoral()){
                return  Commands.none();
            } else if (GameField.getDistanceToMeters(swerve.getPose(), gameField.getPoseForReefStand(reefStand, reefStandSide)) < 0.3) {
                return new SequentialCommandGroup(
                        pathPlanner.goToPreTargetReefPose(reefStand, reefStandSide),
                        new ParallelCommandGroup(
                                pathPlanner.goToPoseReef(reefStand, reefStandSide),
                                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                                new InstantCommand(() -> System.out.println("biatch")),
                                Commands.waitUntil(elevatorMoveCommand::getIsNear)
                        ),
                        new ShootCommand(shooterSystem, shooterSpeed),
                        new ParallelCommandGroup(
                                pathPlanner.goToPreTargetReefPose(reefStand, reefStandSide),
                                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                                Commands.waitUntil(elevatorMoveCommand::getIsNear)
                        )
                );
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToPreTargetReefPosePathFind(reefStand, reefStandSide),
                    new ParallelCommandGroup(
                            pathPlanner.goToReefPathFind(reefStand, reefStandSide),
                            new InstantCommand(() -> System.out.println("biatch " + elevatorHeight)),
                            new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                            Commands.waitUntil(elevatorMoveCommand::getIsNear)
                    ),
                    new ShootCommand(shooterSystem, shooterSpeed),
                    new ParallelCommandGroup(
                            pathPlanner.goToPreTargetReefPose(reefStand, reefStandSide),
                            new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                            Commands.waitUntil(elevatorMoveCommand::getIsNear)
                    )
            );
        }, Set.of(shooterSystem, swerve));
    }

    public Command coralOnClosestReefStageLeft(CoralReef stage) {
        double elevatorHeight = getStageHeight(stage);
        double shooterSpeed;

        if (elevatorHeight == RobotMap.ELEVATOR_L1_HEIGHT_M) {
            shooterSpeed = RobotMap.SHOOTER_MOTOR_OUTTAKE_LOWER_SPEED;
        } else {
            shooterSpeed = RobotMap.SHOOTER_MOTOR_OUTTAKE_DEFAULT_SPEED;
        }

        return Commands.defer(() -> {

            if(!shooterSystem.hasCoral() || pathPlanner.closestReefIsPresent(GameField.ReefStandSide.LEFT)){
                return Commands.none();
            } else if (elevatorHeight == RobotMap.ELEVATOR_L4_HEIGHT_M) {
                return new SequentialCommandGroup(
                        pathPlanner.goToPreTargetClosestReefPose(GameField.ReefStandSide.LEFT),
                        new ParallelCommandGroup(
                                pathPlanner.goToClosestReef(GameField.ReefStandSide.LEFT),
                                new InstantCommand(()-> System.out.println("gigi")),
                                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                                Commands.waitUntil(elevatorMoveCommand::getIsNear)
                        ),
                        new ShootCommand(shooterSystem, shooterSpeed),
                        new ParallelCommandGroup(
                                pathPlanner.goToPreTargetClosestReefPose(GameField.ReefStandSide.LEFT),
                                new SequentialCommandGroup(
                                        new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_L3_HEIGHT_M)),
                                        Commands.waitUntil(elevatorMoveCommand::getIsNear),
                                        new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_L2_HEIGHT_M)),
                                        Commands.waitUntil(elevatorMoveCommand::getIsNear),
                                        new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                                        Commands.waitUntil(elevatorMoveCommand::getIsNear)
                                )
                        )

                );
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToPreTargetClosestReefPose(GameField.ReefStandSide.LEFT),
                    new ParallelCommandGroup(
                            pathPlanner.goToClosestReef(GameField.ReefStandSide.LEFT),
                            new InstantCommand(()-> System.out.println("gigi")),
                            new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                            Commands.waitUntil(elevatorMoveCommand::getIsNear)
                    ),
                    new ShootCommand(shooterSystem, shooterSpeed),
                    new ParallelCommandGroup(
                            pathPlanner.goToPreTargetClosestReefPose(GameField.ReefStandSide.LEFT),
                            new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                            Commands.waitUntil(elevatorMoveCommand::getIsNear)
                    )

            );
        }, Set.of(shooterSystem, swerve));
    }

    public Command coralOnClosestReefStageRight(CoralReef stage) {

        double elevatorHeight = getStageHeight(stage);
        double shooterSpeed;

        if (elevatorHeight == RobotMap.ELEVATOR_L1_HEIGHT_M) {
            shooterSpeed = RobotMap.SHOOTER_MOTOR_OUTTAKE_LOWER_SPEED;
        } else {
            shooterSpeed = RobotMap.SHOOTER_MOTOR_OUTTAKE_DEFAULT_SPEED;
        }

        return Commands.defer(() -> {

            if(!shooterSystem.hasCoral() || pathPlanner.closestReefIsPresent(GameField.ReefStandSide.RIGHT)){
                return  Commands.none();
            } else if (elevatorHeight == RobotMap.ELEVATOR_L4_HEIGHT_M) {
                return new SequentialCommandGroup(
                        pathPlanner.goToPreTargetClosestReefPose(GameField.ReefStandSide.LEFT),
                        new ParallelCommandGroup(
                                pathPlanner.goToClosestReef(GameField.ReefStandSide.LEFT),
                                new InstantCommand(()-> System.out.println("gigi")),
                                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                                Commands.waitUntil(elevatorMoveCommand::getIsNear)
                        ),
                        new ShootCommand(shooterSystem, shooterSpeed),
                        new ParallelCommandGroup(
                                pathPlanner.goToPreTargetClosestReefPose(GameField.ReefStandSide.LEFT),
                                new SequentialCommandGroup(
                                        new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_L3_HEIGHT_M)),
                                        Commands.waitUntil(elevatorMoveCommand::getIsNear),
                                        new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_L2_HEIGHT_M)),
                                        Commands.waitUntil(elevatorMoveCommand::getIsNear),
                                        new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                                        Commands.waitUntil(elevatorMoveCommand::getIsNear)
                                )
                        )

                );
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToPreTargetClosestReefPose(GameField.ReefStandSide.RIGHT),
                    new ParallelCommandGroup(
                            pathPlanner.goToClosestReef(GameField.ReefStandSide.RIGHT),
                            new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                            Commands.waitUntil(elevatorMoveCommand::getIsNear)
                    ),
                    new ShootCommand(shooterSystem, shooterSpeed),
                    new ParallelCommandGroup(
                            pathPlanner.goToPreTargetClosestReefPose(GameField.ReefStandSide.RIGHT),
                            new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                            Commands.waitUntil(elevatorMoveCommand::getIsNear)
                    )

            );
        }, Set.of(shooterSystem, swerve));
    }

    public Command GetCoralFromSource(GameField.SourceStand sourceStand, GameField.SourceStandSide sourceStandSide) {
        return Commands.defer(() -> {

            if(shooterSystem.hasCoral()){
                return  Commands.none();
            } else if (GameField.getDistanceToMeters(swerve.getPose(), gameField.getPoseForSource(sourceStand, sourceStandSide)) < 0.3) {
                return new SequentialCommandGroup(
                        pathPlanner.goToPoseSource(sourceStand,sourceStandSide),
                        new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.SOURCE_HEIGHT)),
                        Commands.waitUntil(elevatorMoveCommand::getIsNear),
                        new IntakeCommand(shooterSystem),
                        new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                        Commands.waitUntil(elevatorMoveCommand::getIsNear)
                );
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToSourcePathFind(sourceStand,sourceStandSide),
                    new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.SOURCE_HEIGHT)),
                    Commands.waitUntil(elevatorMoveCommand::getIsNear),
                    new IntakeCommand(shooterSystem),
                    new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                    Commands.waitUntil(elevatorMoveCommand::getIsNear)
            );
        }, Set.of(shooterSystem, swerve));
    }

    public Command getCoralFromClosestSource(GameField.SourceStandSide sourceStandSide) {
        return Commands.defer(() -> {

            if(shooterSystem.hasCoral() || !pathPlanner.closestSourceIsPresent(sourceStandSide)){
                return  Commands.none();
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToClosestSource(sourceStandSide),
                    new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.SOURCE_HEIGHT)),
                    Commands.waitUntil(elevatorMoveCommand::getIsNear),
                    new IntakeCommand(shooterSystem),
                    new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                    Commands.waitUntil(elevatorMoveCommand::getIsNear)
            );
        }, Set.of(shooterSystem, swerve));

    }

    //Teleop command groups

    public Command teleopCoralOnReef(double elevatorHeight){
        double shooterSpeed;

        if (elevatorHeight == RobotMap.ELEVATOR_L1_HEIGHT_M) {
            shooterSpeed = RobotMap.SHOOTER_MOTOR_OUTTAKE_LOWER_SPEED;
        } else {
            shooterSpeed = RobotMap.SHOOTER_MOTOR_OUTTAKE_DEFAULT_SPEED;
        }

        return new SequentialCommandGroup(
                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                Commands.waitUntil(elevatorMoveCommand::getIsNear),
                Commands.waitUntil(controller.a().onTrue(new ShootCommand(shooterSystem, shooterSpeed))),
                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                Commands.waitUntil(elevatorMoveCommand::getIsNear)

        );
    }
    public Command teleopGetCoralFromSource(){
        return new SequentialCommandGroup(
                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.SOURCE_HEIGHT)),
                Commands.waitUntil(elevatorMoveCommand::getIsNear),
                Commands.waitUntil(controller.b().onTrue(new IntakeCommand(shooterSystem))),
                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_PARKING_HEIGHT_M)),
                Commands.waitUntil(elevatorMoveCommand::getIsNear)
        );
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
                elevatorHeight = RobotMap.ELEVATOR_PARKING_HEIGHT_M;
                break;
        }

        return elevatorHeight;
    }

    public void update() {
        pathPlanner.update();
    }



    public Command goToHeightCommand(double height) {
        return new InstantCommand(() -> elevatorMoveCommand.setTargetHeight(height));
    }
}
