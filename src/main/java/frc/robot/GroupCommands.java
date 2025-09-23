package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.ShooterSystem;

import java.util.Optional;
import java.util.Set;

import static frc.robot.CoralReef.FIRST_STAGE;
import static frc.robot.CoralReef.PODIUM;

public class GroupCommands {

     /*
    group commands:

        -coral on reef:
        enum for podium, first second and third stage.
        optional-shoot the coral
        constants: stage heights

        -get from source:
        elevator to source height, intake
        constants: source height
     */

    public Command coralOnReefStage(CoralReef reef) {
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
            return new SequentialCommandGroup(
                    new ElevatorMoveCommand(new ElevatorSystem(), elevatorHeight),
                    new ShootCommand(new ShooterSystem())
            );
        }, Set.of(new ElevatorSystem(), new ShooterSystem()));
    }


    public Command GetCoralFromSource() {
        return Commands.defer(() -> {

            return new SequentialCommandGroup(
                    new ElevatorMoveCommand(new ElevatorSystem(), RobotMap.SOURCE_HEIGHT),
                    new IntakeCommand(new ShooterSystem())
            );
        }, Set.of(new ElevatorSystem(), new ShooterSystem()));
    }
}
