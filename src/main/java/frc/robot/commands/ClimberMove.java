// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberMove extends Command
{
  // The climber move command's parameters are defined here...
  private final Climber climber;
  private final Supplier<Double> joystick;
  private final SlewRateLimiter limiter;

  /** Creates a new ClimberMove. */
  public ClimberMove
  (Climber climber, Supplier<Double> joystick, SlewRateLimiter limiter)
  {
    // Gives value to the paramenters
    this.climber = climber;
    this.joystick = joystick;
    this.limiter = new SlewRateLimiter(climberDriveMaxSpeed);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // Gets the speeds
    double speed = joystick.get();

    // Applies deadband
    speed = Math.abs(speed) > climberDeadband ? speed : 0;

    // Makes a smoother move
    speed = limiter.calculate(speed) * climberDriveMaxSpeed;

    // Sends the speeds to the climber
    climber.activate(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    climber.activate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
