// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDrive;

import static frc.robot.Constants.TankDriveConstants.*;

public class DriveCommand extends Command
{
  // The drive command's parameters are defined here...
  private final TankDrive tankDrive;
  private final Supplier <Double> speedJoystick, rotJoystick;
  private final SlewRateLimiter speedLimiter, rotLimiter;

  /** Creates a new DriveCommand. */
  public DriveCommand
  (TankDrive tankDrive, Supplier<Double> speed, Supplier<Double> rot)
  {
    // Gives value to the paramenters
    this.tankDrive = tankDrive;
    this.speedJoystick = speed;
    this.rotJoystick = rot;
    this.speedLimiter = new SlewRateLimiter(chassisDriveMaxSpeed);
    this.rotLimiter = new SlewRateLimiter(chassisDriveMaxRot);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tankDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // Gets the speeds
    double speed = speedJoystick.get();
    double rot = rotJoystick.get();

    // Makes a smoother move
    speed = speedLimiter.calculate(speed) * chassisDriveMaxSpeed;
    rot = rotLimiter.calculate(rot) * chassisDriveMaxRot;

    // Sends the speeds to the tank drive
    tankDrive.drive(speed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    // Stops the chassis
    tankDrive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
