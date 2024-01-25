// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TankDrive;

public class TankDrivecommand extends Command {
  private final TankDrive tank;
  private double speed, rot;
  private double speedJoystick, rotJoystick;

  /** Creates a new TankDrivecommand. */
  public TankDrivecommand(TankDrive tank, double speedJoystick, double rotJoystick) {
    this.tank = tank;
    this.speedJoystick = speedJoystick;
    this.rotJoystick = rotJoystick;

    addRequirements(tank);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = MathUtil.applyDeadband(speedJoystick, 0);
    rot = MathUtil.applyDeadband(rotJoystick, 0);

    tank.drive(speed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
