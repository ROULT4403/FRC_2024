// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private final TankDrive tankDrive = new TankDrive();
  private final Climber climber = new Climber();
  private final Wrist wrist = new Wrist();
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  // The driver's controllers are defined here...
  private final CommandXboxController chassisController(controllrsPort[0]);
  private final CommandXboxController mechController(controllrsPort[1]);

  private final Trigger x = mechController.x();
  private final Trigger y = mechController.y();
  private final Trigger shoot = mechController.rightTrigger();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    tankDrive.setDefaultCommand(new DriveCommand(tankDrive, chassisController.getLeftY(), chassisController.getRightX()));
    wrist.setDefaultCommand(new WristMove(wrist, mechController.getRightX()));
    climber.setDefaultCommand(new ClimberMove(climber, mechController.getLeftY()));
    
    // Configure the trigger bindings
    configureBindings();
  }

    /** Use this method to define your trigger->command mappings. */
  private void configureBindings()
  {
    x.onTrue(Commands.take);
    y.onTrue(Commands.outtake);

    shoot.onTrue(Commands.manualShoot);
    shoot.onTrue(Commands.stopShoot);
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
