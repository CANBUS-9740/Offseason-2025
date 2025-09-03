package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
    private XboxController controller;
    private Shooter shooter;

    @Override
    public void robotInit() {
        shooter = new Shooter();
        controller = new XboxController(0);

        //init for intake command that activates on an A button press
        JoystickButton aButton = new JoystickButton(controller,XboxController.Button.kA.value);
        aButton.onTrue(new IntakeCommand(shooter));
        //init for shoot command that activates on a B button press
        JoystickButton bButton = new JoystickButton(controller,XboxController.Button.kB.value);
        aButton.onTrue(new ShootCommand(shooter));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }
}
