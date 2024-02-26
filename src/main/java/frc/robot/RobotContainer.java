// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberMove;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.WristMove;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Wrist;
import static frc.robot.Constants.OperatorConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private static final TankDrive tankDrive = new TankDrive();
  private static final Climber climber = new Climber();
  private static final Wrist wrist = new Wrist();
  private static final Intake intake = new Intake();
  private static final Shooter shooter = new Shooter();

  // The driver's controllers are defined here...
  private static final CommandXboxController chassisController = new CommandXboxController(controllersPort[0]);
  private static final CommandXboxController mechController = new CommandXboxController(controllersPort[1]);

  //The driver's triggers are defined here...
  private static final Trigger take = chassisController.x();
  private static final Trigger climberUp = chassisController.povUp();
  private static final Trigger climberDown = chassisController.povDown();
  private static final Trigger saveWrist = chassisController.povRight();
  private static final Trigger engadeWrist = chassisController.povLeft();
  private static final Trigger shoot = mechController.x();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    tankDrive.setDefaultCommand(new DriveCommand(tankDrive, () -> chassisController.getLeftY(), () -> chassisController.getRightX()));
    wrist.setDefaultCommand(new WristMove(wrist, () -> mechController.getRightX()));
    climber.setDefaultCommand(new ClimberMove(climber, () -> mechController.getLeftY()));
    
    // Configure the trigger bindings
    configureBindings();
  }

    /** Use this method to define your trigger->command mappings. */
  private void configureBindings()
  {
    take.onTrue(new InstantCommand(() -> intake.take(1), intake));
    climberUp.onTrue(new InstantCommand(() -> climber.activate(1), climber));
    climberDown.onTrue(new InstantCommand(() -> climber.activate(-1), climber));
    saveWrist.onTrue(new InstantCommand(() -> wrist.moveWrist(0.3), wrist).withTimeout(0.5));
    engadeWrist.onTrue(new InstantCommand(() -> wrist.moveWrist(-0.3), wrist).withTimeout(0.3));
    shoot.onTrue(new ShootCommand(shooter, intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return null;
  }
}
