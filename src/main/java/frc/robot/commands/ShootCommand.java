// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommand extends SequentialCommandGroup
{
  // The shoot command's parameters are defined here...
  private Shooter shooter = new Shooter();
  private Intake intake = new Intake();

  /** Creates a new ShootCommand. */
  public ShootCommand(Shooter shooterSubsystem, Intake intakeSubsystem)
  {
    this.shooter = shooterSubsystem;
    this.intake = intakeSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands
    (
      // Starts shooting process
      new RunCommand(() -> shooter.shoot(1), shooter).withTimeout(1),
      new InstantCommand(() -> intake.outtake(-1), intake).withTimeout(0.3),
      // Stops the shooting process
      new ParallelCommandGroup(
        new RunCommand(() -> shooter.shoot(1), shooter),
        new InstantCommand(() -> intake.outtake(-1), intake))
    );
  }
}
