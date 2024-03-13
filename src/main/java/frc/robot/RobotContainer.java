// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Wrist;
import static frc.robot.Constants.OperatorConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;


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

  private static final TankDrive tankDrive = new TankDrive();
  private static final Climber climber = new Climber();
  private static final Wrist wrist = new Wrist();
  private static final Intake intake = new Intake();
  private static final Shooter shooter = new Shooter();

  // The driver's controllers are defined here...
  private static final CommandXboxController chassisController = new CommandXboxController(controllersPort[0]);
  private static final CommandXboxController mechController = new CommandXboxController(controllersPort[1]);

  //The driver's triggers are defined here...
  private static final Trigger take = chassisController.a().or(chassisController.b());

  //private static final Trigger climberUp = chassisController.povUp();
  //private static final Trigger climberDown = chassisController.povDown();

  private static final Trigger saveWrist = chassisController.x();
  private static final Trigger engadeWrist = chassisController.y();
  private static final Trigger shoot = mechController.rightTrigger();
  private static final Trigger outtake = mechController.leftTrigger();
  private static final Trigger shooting = mechController.x().or(mechController.y());
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    //Named Commands
    NamedCommands.registerCommand("ShootOn",new InstantCommand(() ->shooter.shoot(1), shooter));
    NamedCommands.registerCommand("ShootOff",new InstantCommand(() ->shooter.shoot(0), shooter));

    NamedCommands.registerCommand("IntakeOut",new InstantCommand(() -> intake.activate(-1), intake));
    NamedCommands.registerCommand("IntakeIn",new InstantCommand(() -> intake.activate(1), intake));

    NamedCommands.registerCommand("IntakeOff",new InstantCommand(() -> intake.activate(0), intake));

    NamedCommands.registerCommand("WristDown", new InstantCommand(()-> wrist.moveWrist(-0.2), wrist).withTimeout(0.3));
    NamedCommands.registerCommand("WristUp", new InstantCommand(()-> wrist.moveWrist(0.2), wrist).withTimeout(0.3));




    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    tankDrive.setDefaultCommand(new RunCommand(() -> tankDrive.drive(-chassisController.getLeftY(), chassisController.getRightX()), tankDrive));

    // Configure the trigger bindings
    configureBindings();
  }

    /** Use this method to define your trigger->command mappings. */
  private void configureBindings()
  {
    take.onTrue(new InstantCommand(() -> intake.activate(0.5), intake));

    //climberUp.onTrue(new InstantCommand(() -> climber.activate(0.1), climber));
    //climberDown.onTrue(new InstantCommand(() -> climber.activate(-0.1), climber));
    // THIS THE GOOD ONE climberUp.onTrue(new InstantCommand(() -> climber.climb(0.7), climber));
     // THIS THE GOOD ONEclimberDown.onTrue(new InstantCommand(() -> climber.climb(-0.7), climber));

    saveWrist.onTrue(new InstantCommand(() -> wrist.moveWrist(0.3), wrist));
    engadeWrist.onTrue(new InstantCommand(() -> wrist.moveWrist(-0.3), wrist));
    shoot.whileTrue(new InstantCommand(() -> shooter.shoot(1), shooter));
    outtake.whileTrue(new InstantCommand(() -> intake.activate(-1), intake));
    shooting.onTrue(new SequentialCommandGroup(new RunCommand(()-> shooter.shoot(1), shooter),
    new RunCommand(()-> intake.activate(-1), intake)));


    take.onFalse(new InstantCommand(() -> intake.activate(0), intake));
    
    //climberUp.onFalse(new InstantCommand(() -> climber.activate(0), climber));
    //climberDown.onFalse(new InstantCommand(() -> climber.activate(0), climber));
     // THIS THE GOOD ONEclimberUp.onFalse(new InstantCommand(() -> climber.climb(0), climber));
     // THIS THE GOOD ONEclimberDown.onFalse(new InstantCommand(() -> climber.climb(0), climber));

    saveWrist.onFalse(new InstantCommand(() -> wrist.moveWrist(0), wrist));
    engadeWrist.onFalse(new InstantCommand(() -> wrist.moveWrist(0), wrist));
    shoot.whileFalse(new InstantCommand(() -> shooter.shoot(0), shooter));
    outtake.whileFalse(new InstantCommand(() -> intake.activate(0), intake));
    shooting.onFalse(new SequentialCommandGroup(new RunCommand(()-> shooter.shoot(0), shooter),
    new RunCommand(()-> intake.activate(0), intake)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    //PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");
    //return AutoBuilder.followPath(path);
    return autoChooser.getSelected();
  }
}
