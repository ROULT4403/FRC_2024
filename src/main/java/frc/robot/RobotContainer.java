// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
<<<<<<< Updated upstream
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
=======
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
  public static final shooter shooter = new shooter();
=======
  public static final Shooter shooter = new Shooter();
  private static final AutoConfig autoConfig = new AutoConfig();

>>>>>>> Stashed changes

  // The driver's controllers are defined here...
  private static final CommandXboxController chassisController = new CommandXboxController(controllersPort[0]);
  private static final CommandXboxController mechController = new CommandXboxController(controllersPort[1]);

  //The driver's triggers are defined here...
  private static final Trigger take = chassisController.a().or(chassisController.b());

  //private static final Trigger climberUp = chassisController.povUp();
  //private static final Trigger climberDown = chassisController.povDown();

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
<<<<<<< Updated upstream
<<<<<<< Updated upstream
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
=======
    take.onTrue(new RunCommand(() -> intake.activate(0.3), intake));

    climberUp.onTrue(new RunCommand(() -> climber.climb(0.6), climber).withTimeout(.05));
    climberDown.onTrue(new RunCommand(() -> climber.climb(-0.6), climber).withTimeout(.1));

    saveWrist.onTrue(new RunCommand(() -> wrist.moveWrist(0.3), wrist));
    engadeWrist.onTrue(new RunCommand(() -> wrist.moveWrist(-0.3), wrist));
    shoot.whileTrue(new RunCommand(() -> shooter.shoot(.9), shooter));
    outtake.whileTrue(new RunCommand(() -> intake.activate(-1), intake));
    shooting.onTrue(new SequentialCommandGroup(new RunCommand(()-> shooter.shoot(.9), shooter),
>>>>>>> Stashed changes
    new RunCommand(()-> intake.activate(-1), intake)));


    take.onFalse(new InstantCommand(() -> intake.activate(0), intake));
    
<<<<<<< Updated upstream
    //climberUp.onFalse(new InstantCommand(() -> climber.activate(0), climber));
    //climberDown.onFalse(new InstantCommand(() -> climber.activate(0), climber));
     // THIS THE GOOD ONEclimberUp.onFalse(new InstantCommand(() -> climber.climb(0), climber));
     // THIS THE GOOD ONEclimberDown.onFalse(new InstantCommand(() -> climber.climb(0), climber));
=======
   
    climberUp.onFalse(new InstantCommand(() -> climber.climb(0), climber));
    climberDown.onFalse(new InstantCommand(() -> climber.climb(0), climber));
>>>>>>> Stashed changes

    saveWrist.onFalse(new InstantCommand(() -> wrist.moveWrist(0), wrist));
    engadeWrist.onFalse(new InstantCommand(() -> wrist.moveWrist(0), wrist));
    shoot.whileFalse(new InstantCommand(() -> shooter.shoot(0), shooter));
    outtake.whileFalse(new InstantCommand(() -> intake.activate(0), intake));
    shooting.onFalse(new SequentialCommandGroup(new RunCommand(()-> shooter.shoot(0), shooter),
    new RunCommand(()-> intake.activate(0), intake)));
=======
    take.whileTrue(intake.intakeCommand(.3));
    outtake.whileTrue(intake.intakeCommand(-1));
    climberUp.whileTrue(climber.climbUp());
    climberDown.whileTrue(climber.climbDown());
    shoot.whileTrue(shooter.shootCommand(1));
    saveWrist.whileTrue(wrist.wristCommand(.3));
    lowWrist.whileTrue(wrist.wristCommand(-.3));


>>>>>>> Stashed changes
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
