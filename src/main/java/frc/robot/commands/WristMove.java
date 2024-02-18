// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;
import static frc.robot.Constants.WristConstants.*;

public class WristMove extends Command
{
  // The wrist move command's parameters are defined here...
  private final Wrist wrist;
  private final Supplier<Double> joystick;
  private final SlewRateLimiter limiter;

  /** Creates a new WristCommand. */
  public WristMove(Wrist wrist, Supplier<Double> joystick)
  {
    // Gives value to the paramenters
    this.wrist = wrist;
    this.joystick = joystick;
    this.limiter = new SlewRateLimiter(wristDriveMaxRot);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
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
    speed = Math.abs(speed) > wristDeadband ? speed : 0;

    // Makes a smoother move
    speed = limiter.calculate(speed) * wristDriveMaxRot;

    // Sends the speeds to the wrist
    wrist.manualWrist(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    // Stops the wrist
    wrist.manualWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
