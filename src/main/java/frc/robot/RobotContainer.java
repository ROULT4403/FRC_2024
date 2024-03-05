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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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

  private static final Trigger climberUp = chassisController.povUp();
  private static final Trigger climberDown = chassisController.povDown();

  private static final Trigger saveWrist = chassisController.x();
  private static final Trigger engadeWrist = chassisController.y();
  private static final Trigger shoot = mechController.rightTrigger();
  private static final Trigger outtake = mechController.leftTrigger();
  private static final Trigger shooting = mechController.x().or(mechController.y());

  SendableChooser<Command> chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
  {
    tankDrive.setDefaultCommand(new RunCommand(() -> tankDrive.drive(-chassisController.getLeftY(), chassisController.getRightX()), tankDrive));
    
    // Configure the trigger bindings
    configureBindings();

    chooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", chooser);
  }

    /** Use this method to define your trigger->command mappings. */
  private void configureBindings()
  {
    take.onTrue(new InstantCommand(() -> intake.activate(0.5), intake));

    //climberUp.onTrue(new InstantCommand(() -> climber.activate(0.1), climber));
    //climberDown.onTrue(new InstantCommand(() -> climber.activate(-0.1), climber));
    climberUp.onTrue(new InstantCommand(() -> climber.climb(0.7), climber));
    climberDown.onTrue(new InstantCommand(() -> climber.climb(-0.7), climber));

    saveWrist.onTrue(new InstantCommand(() -> wrist.moveWrist(0.3), wrist));
    engadeWrist.onTrue(new InstantCommand(() -> wrist.moveWrist(-0.3), wrist));
    shoot.whileTrue(new InstantCommand(() -> shooter.shoot(1), shooter));
    outtake.whileTrue(new InstantCommand(() -> intake.activate(-1), intake));
    shooting.onTrue(new SequentialCommandGroup(new RunCommand(()-> shooter.shoot(1), shooter),
    new RunCommand(()-> intake.activate(-1), intake)));


    take.onFalse(new InstantCommand(() -> intake.activate(0), intake));
    
    //climberUp.onFalse(new InstantCommand(() -> climber.activate(0), climber));
    //climberDown.onFalse(new InstantCommand(() -> climber.activate(0), climber));
    climberUp.onFalse(new InstantCommand(() -> climber.climb(0), climber));
    climberDown.onFalse(new InstantCommand(() -> climber.climb(0), climber));

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
    // Blue Lineal Auto
    /*return new SequentialCommandGroup(
        new RunCommand(()-> shooter.shoot(1), shooter).withTimeout(1),
        new RunCommand(()-> intake.activate(-1), intake).withTimeout(0.8),
        
        new InstantCommand(()-> shooter.shoot(0), shooter),
        new InstantCommand(()-> intake.activate(0), intake),   

        new InstantCommand(()-> wrist.moveWrist(-0.5), wrist).withTimeout(0.3), 
        new RunCommand(()-> tankDrive.drive(-0.7, 0), tankDrive).withTimeout(0.8).alongWith(
          new RunCommand(() -> intake.activate(0.5), intake).withTimeout(2)),
        new InstantCommand(() -> intake.activate(0), intake),
        new InstantCommand(() -> wrist.moveWrist(0.3), wrist).withTimeout(0.4),  

        new RunCommand(()-> tankDrive.drive(0.8, 0), tankDrive).withTimeout(0.7),  
        new RunCommand(()-> tankDrive.drive(0, 0), tankDrive).withTimeout(0.3),  
        new RunCommand(()-> shooter.shoot(1), shooter).withTimeout(1),     
        new RunCommand(()-> intake.activate(-1), intake).withTimeout(1),
        new InstantCommand(() -> shooter.shoot(0), shooter),
        new InstantCommand(()-> intake.activate(0), intake),

        new RunCommand(()->tankDrive.drive(-0.7, 0), tankDrive).withTimeout(0.6),
        new RunCommand(() -> tankDrive.drive(0, -0.5), tankDrive).withTimeout(0.275),
        new RunCommand(() -> tankDrive.drive(-0.7, 0), tankDrive).withTimeout(0.2),
        new RunCommand(() -> tankDrive.drive(0, 0), tankDrive).withTimeout(0.3),

        new InstantCommand(()-> wrist.moveWrist(-0.5), wrist).withTimeout(0.3),
        new RunCommand(()-> intake.activate(0.5), intake).withTimeout(2),
        new InstantCommand(() -> intake.activate(0), intake));*/

        // Red Lineal Auto
    /*return new SequentialCommandGroup(
        new RunCommand(()-> shooter.shoot(1), shooter).withTimeout(1),
        new RunCommand(()-> intake.activate(-1), intake).withTimeout(0.8),
        
        new InstantCommand(()-> shooter.shoot(0), shooter),
        new InstantCommand(()-> intake.activate(0), intake),   

        new InstantCommand(()-> wrist.moveWrist(-0.5), wrist).withTimeout(0.3), 
        new RunCommand(()-> tankDrive.drive(-0.7, 0), tankDrive).withTimeout(0.8).alongWith(
          new RunCommand(() -> intake.activate(0.5), intake).withTimeout(2)),
        new InstantCommand(() -> intake.activate(0), intake),
        new InstantCommand(() -> wrist.moveWrist(0.3), wrist).withTimeout(0.4),  

        new RunCommand(()-> tankDrive.drive(0.8, 0), tankDrive).withTimeout(1.1),  
        new RunCommand(()-> shooter.shoot(1), shooter).withTimeout(1),     
        new RunCommand(()-> intake.activate(-1), intake).withTimeout(1),
        new InstantCommand(() -> shooter.shoot(0), shooter),
        new InstantCommand(()-> intake.activate(0), intake));*/


        // Red-LEFT Side Auto
       /*  return new SequentialCommandGroup(
        new RunCommand(()-> shooter.shoot(1), shooter).withTimeout(1),
        new RunCommand(()-> intake.activate(-0.8), intake).withTimeout(0.8),
     
        new InstantCommand(()-> shooter.shoot(0), shooter),
        new InstantCommand(()-> intake.activate(0), intake),

        new RunCommand(()-> tankDrive.drive(-0.6, 0), tankDrive).withTimeout(0.4),
        new RunCommand(()-> tankDrive.drive(0, 0.5), tankDrive).withTimeout(0.45),
        new InstantCommand(()-> wrist.moveWrist(-0.5), wrist).withTimeout(0.3),
        new RunCommand(()-> tankDrive.drive(-0.6, 0), tankDrive).withTimeout(1).alongWith(
          new InstantCommand(() -> intake.activate(0.5), intake).withTimeout(0.9)),
        new InstantCommand(() -> intake.activate(0), intake),
        new InstantCommand(()-> wrist.moveWrist(0.5), wrist).withTimeout(0.3),
        new RunCommand(()-> tankDrive.drive(0.6, 0), tankDrive).withTimeout(1),
        new RunCommand(()-> tankDrive.drive(0,-0.5), tankDrive).withTimeout(0.40),
        new RunCommand(()-> tankDrive.drive(0.5, 0), tankDrive).withTimeout(0.3),
        new RunCommand(()-> shooter.shoot(1), shooter).withTimeout(1),
        new RunCommand(()-> intake.activate(-0.8),intake).withTimeout(0.8));*/

        // Blue-Left Side Auto
        /*return new SequentialCommandGroup(
        new RunCommand(()-> shooter.shoot(1), shooter).withTimeout(1),
        new RunCommand(()-> intake.activate(-0.8), intake).withTimeout(0.8),
     
        new InstantCommand(()-> shooter.shoot(0), shooter),
        new InstantCommand(()-> intake.activate(0), intake),

        new RunCommand(()-> tankDrive.drive(-0.6, 0), tankDrive).withTimeout(0.4),
        new RunCommand(()-> tankDrive.drive(0, -0.5), tankDrive).withTimeout(0.45),
        new InstantCommand(()-> wrist.moveWrist(-0.5), wrist).withTimeout(0.3),
        new RunCommand(()-> tankDrive.drive(-0.6, 0), tankDrive).withTimeout(1).alongWith(
          new InstantCommand(() -> intake.activate(0.5), intake).withTimeout(0.9)),
        new InstantCommand(() -> intake.activate(0), intake),
        new InstantCommand(()-> wrist.moveWrist(0.5), wrist).withTimeout(0.3),
        new RunCommand(()-> tankDrive.drive(0.6, 0), tankDrive).withTimeout(1),
        new RunCommand(()-> tankDrive.drive(0,0.5), tankDrive).withTimeout(0.40),
        new RunCommand(()-> tankDrive.drive(0.5, 0), tankDrive).withTimeout(0.3),
        new RunCommand(()-> shooter.shoot(1), shooter).withTimeout(1),
        new RunCommand(()-> intake.activate(-0.8),intake).withTimeout(0.8),
        new InstantCommand(()-> shooter.shoot(0), shooter),
        new InstantCommand(()-> intake.activate(0), intake));*/
        //Red Side Playoffs
        /*return new SequentialCommandGroup(
        new RunCommand(()-> shooter.shoot(1), shooter).withTimeout(1),
        new RunCommand(()-> intake.activate(-1), intake).withTimeout(0.8),
        
        new InstantCommand(()-> shooter.shoot(0), shooter),
        new InstantCommand(()-> intake.activate(0), intake),

        new WaitCommand(7),
        
        new RunCommand(() -> tankDrive.drive(-0.7, 0), tankDrive).withTimeout(0.4));*/

        // Playoffs
        /*return new SequentialCommandGroup(
        new RunCommand(()-> shooter.shoot(1), shooter).withTimeout(1),
        new RunCommand(()-> intake.activate(-1), intake).withTimeout(0.8),
        
        new InstantCommand(()-> shooter.shoot(0), shooter),
        new InstantCommand(()-> intake.activate(0), intake),

        new WaitCommand(11),

        new RunCommand(() -> tankDrive.drive(-0.7, 0), tankDrive).withTimeout(0.25),
        new RunCommand(() -> tankDrive.drive(0, -0.5), tankDrive).withTimeout(0.25),
        new RunCommand(() -> tankDrive.drive(-0.7,0 ), tankDrive).withTimeout(0.5),
        new RunCommand(() -> tankDrive.drive(0,0), tankDrive).withTimeout(10));*/

        return null;
  }
}
