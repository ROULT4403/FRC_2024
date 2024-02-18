// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import static frc.robot.Constants.WristConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/** Creates all the follow mech commands */
public class Commands
{
    // The mech's subsystems are defined here...
    private final Wrist wrist = new Wrist();
    private final Shooter shooter = new Shooter();
    private final Intake intake = new Intake();
    private final Climber climber = new Climber();

    // The wrist's commands are defined here...
    public InstantCommand saveWristCommand = new InstantCommand(() -> wrist.setGoal(saveWrist), wrist);
    public InstantCommand engandeWristCommand = new InstantCommand(() -> wrist.setGoal(engandeWrist), wrist);

    // The shoot commands are defined here...
    public InstantCommand closeShootCommand = new InstantCommand(() -> shooter.setSetpoint(closeShoot), shooter);
    public InstantCommand largeShootCommand = new InstantCommand(() -> shooter.setSetpoint(largeShoot), shooter);

    // The intake commands are defined here...
    public RunCommand take = new RunCommand(() -> intake.take(1), intake);
    public RunCommand outtake = new RunCommand(() -> intake.outtake(-1), intake);

    // The manual shoot commands are defined here...
    public RunCommand manualShoot = new RunCommand(() -> shooter.manualShooter(1), shooter);
    public RunCommand stopShoot = new RunCommand(() -> shooter.manualShooter(0), shooter);

    // The climber's commands are defined here...
    public InstantCommand upClimberCommand = new InstantCommand(() -> climber.setGoal(upClimber), climber);
    public InstantCommand downClimberCommand = new InstantCommand(() -> climber.activate(-1), climber);

    // The semi-automatic routines will be defined here...
}
