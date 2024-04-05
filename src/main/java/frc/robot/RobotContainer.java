// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.AutoConfig;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Wrist;
import static frc.robot.Constants.OperatorConstants.*;
import com.pathplanner.lib.auto.AutoBuilder;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // The robot's subsystems and commands are defined here...
  private final SendableChooser<Command> autoChooser;

  public static final TankDrive tankDrive = new TankDrive();
  public static final Climber climber = new Climber();
  public static final Wrist wrist = new Wrist();
  public static final Intake intake = new Intake();
  public static final shooter shooter = new shooter();
  public static final AutoConfig autoConfig = new AutoConfig();

  // The driver's controllers are defined here...
  private static final CommandXboxController chassisController = new CommandXboxController(controllersPort[0]);
  public static final CommandXboxController mechController = new CommandXboxController(controllersPort[1]);

  //The driver's triggers are defined here...
  private static final Trigger take = chassisController.a().or(chassisController.b());
  private static final Trigger climberUp = chassisController.povUp();
  private static final Trigger climberDown = chassisController.povDown();
  private static final Trigger saveWrist = chassisController.x();
  private static final Trigger lowWrist = chassisController.y();
  private static final Trigger shoot = mechController.rightTrigger();
  private static final Trigger outtake = mechController.leftTrigger();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
  autoConfig.setNamedCommands();
  autoChooser = AutoBuilder.buildAutoChooser();
  SmartDashboard.putData("Auto Chooser", autoChooser);
  tankDrive.setDefaultCommand(new RunCommand(() -> tankDrive.drive(-chassisController.getLeftY(), chassisController.getRightX()), tankDrive));
  configureBindings();
 
  }

    /** Use this method to define your trigger->command mappings. */
  private void configureBindings()
  {

    take.whileTrue(intake.intakeCommand(.3));
    outtake.whileTrue(intake.intakeCommand(-1));
    climberUp.whileTrue(climber.climbUp(1.0).until(climber::climberUpSwitch));
    climberDown.whileTrue(climber.climbDown(-.6));
    shoot.whileTrue(shooter.shootCommand(1));
    saveWrist.whileTrue(wrist.wristCommand(.3));
    lowWrist.whileTrue(wrist.wristCommand(-.3));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }
}
