package frc.robot.Utils;

import edu.wpi.first.wpilibj.XboxController;
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

    private ElevatorSystem elevatorSystem;
    private ShooterSystem shooterSystem;
    private final Swerve swerve;
    private final GameField gameField;
    private final PathPlanner pathPlanner;
    private final CommandXboxController controller;
    private final ElevatorMoveCommand elevatorMoveCommand;


    public GroupCommands(CommandXboxController xboxController, Swerve swerve) {
        elevatorSystem = new ElevatorSystem();
        shooterSystem = new ShooterSystem();
        gameField = new GameField();
        pathPlanner = new PathPlanner(swerve);
        elevatorMoveCommand = new ElevatorMoveCommand(elevatorSystem);

        this.swerve = swerve;
        this.controller = xboxController;
    }

    public Command coralOnReefStage(CoralReef stage, GameField.ReefStand reefStand, GameField.ReefStandSide reefStandSide ) {
        double elevatorHeight = getStageHeight(stage);

        return Commands.defer(() -> {

            if(shooterSystem.hasCoral()){
                return  Commands.none();
            } else if (GameField.getDistanceToMeters(swerve.getPose(), gameField.getPoseForReefStand(reefStand, reefStandSide)) < 0.3) {
                return new SequentialCommandGroup(
                        pathPlanner.goToPreTargetReefPose(reefStand, reefStandSide),
                        new ParallelCommandGroup(
                                pathPlanner.goToClosestReef(),
                                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                                Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(elevatorHeight))
                        ),
                        new ShootCommand(shooterSystem),
                        new ParallelCommandGroup(
                                pathPlanner.goToPreTargetClosestReefPose(),
                                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_L1_HEIGHT_M)),
                                Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(RobotMap.ELEVATOR_L1_HEIGHT_M))
                        )
                );
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToPreTargetReefPose(reefStand, reefStandSide),
                    new ParallelCommandGroup(
                            pathPlanner.goToClosestReef(),
                            new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                            Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(elevatorHeight))
                    ),
                    new ShootCommand(shooterSystem),
                    new ParallelCommandGroup(
                            pathPlanner.goToPreTargetClosestReefPose(),
                            new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_L1_HEIGHT_M)),
                            Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(RobotMap.ELEVATOR_L1_HEIGHT_M))
                    )
            );
        }, Set.of(elevatorSystem, shooterSystem, swerve));
    }

    public Command coralOnClosestReefStage(CoralReef stage) {
        double elevatorHeight = getStageHeight(stage);

        return Commands.defer(() -> {

            if(shooterSystem.hasCoral() || !pathPlanner.closestReefIsPresent()){
                return  Commands.none();
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToPreTargetClosestReefPose(),
                    new ParallelCommandGroup(
                            pathPlanner.goToClosestReef(),
                            new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                            Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(elevatorHeight))
                    ),
                    new ShootCommand(shooterSystem),
                    new ParallelCommandGroup(
                            pathPlanner.goToPreTargetClosestReefPose(),
                            new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_L1_HEIGHT_M)),
                            Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(RobotMap.ELEVATOR_L1_HEIGHT_M))
                    )

            );
        }, Set.of(elevatorSystem, shooterSystem, swerve));
    }

    public Command GetCoralFromSource(GameField.SourceStand sourceStand, GameField.SourceStandSide sourceStandSide) {
        return Commands.defer(() -> {

            if(!shooterSystem.hasCoral()){
                return  Commands.none();
            } else if (GameField.getDistanceToMeters(swerve.getPose(), gameField.getPoseForSource(sourceStand, sourceStandSide)) < 0.3) {
                return new SequentialCommandGroup(
                        pathPlanner.goToPoseSource(sourceStand,sourceStandSide),
                        new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.SOURCE_HEIGHT)),
                        Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(RobotMap.SOURCE_HEIGHT)),
                        new IntakeCommand(shooterSystem)
                );
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToSourcePathFind(sourceStand,sourceStandSide),
                    new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.SOURCE_HEIGHT)),
                    Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(RobotMap.SOURCE_HEIGHT)),
                    new IntakeCommand(shooterSystem)
            );
        }, Set.of(elevatorSystem, shooterSystem, swerve));
    }

    public Command getCoralFromClosestSource() {
        return Commands.defer(() -> {

            if(!shooterSystem.hasCoral() || !pathPlanner.closestSourceIsPresent()){
                return  Commands.none();
            }

            return new SequentialCommandGroup(
                    pathPlanner.goToClosestSource(),
                    new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.SOURCE_HEIGHT)),
                    Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(RobotMap.SOURCE_HEIGHT)),
                    new IntakeCommand(shooterSystem)
            );
        }, Set.of(elevatorSystem, shooterSystem, swerve));

    }

    //Teleop command groups

    public Command teleopCoralOnReef(double elevatorHeight){
        return new SequentialCommandGroup(
                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(elevatorHeight)),
                Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(elevatorHeight)),
                Commands.waitUntil(controller.a().onTrue(new ShootCommand(shooterSystem))),
                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_L1_HEIGHT_M)),
                Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(RobotMap.ELEVATOR_L1_HEIGHT_M))

        );
    }
    public Command teleopGetCoralFromSource(double elevatorHeight){
        return new SequentialCommandGroup(
                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.SOURCE_HEIGHT)),
                Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(RobotMap.SOURCE_HEIGHT)),
                Commands.waitUntil(controller.b().onTrue(new IntakeCommand(shooterSystem))),
                new InstantCommand(()-> elevatorMoveCommand.setTargetHeight(RobotMap.ELEVATOR_L1_HEIGHT_M)),
                Commands.waitUntil(()-> elevatorMoveCommand.getIsNear(RobotMap.ELEVATOR_L1_HEIGHT_M))
        );
    }


    public SwerveDriveCommand swerveDrive(boolean fieldDrive){
        return new SwerveDriveCommand(swerve,controller, fieldDrive);
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
