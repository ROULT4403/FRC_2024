// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AutoConfig extends SubsystemBase {
  /** Creates a new Pathplanner. */
  public AutoConfig() {}
  public void setNamedCommands(){
    //Named Commands
    NamedCommands.registerCommand("i_OFF",new InstantCommand(() -> RobotContainer.intake.activate(0), RobotContainer.intake).withTimeout(.1));
    NamedCommands.registerCommand("i_IN",new InstantCommand(() -> RobotContainer.intake.activate(.5), RobotContainer.intake).withTimeout(.1));
    NamedCommands.registerCommand("ShootOff",new InstantCommand(() ->RobotContainer.shooter.shoot(0), RobotContainer.shooter).withTimeout(.1));
    NamedCommands.registerCommand("WristDown", new RunCommand(()-> RobotContainer.wrist.moveWrist(-.3), RobotContainer.wrist).withTimeout(0.3));
    NamedCommands.registerCommand("IntakeOut",new InstantCommand(() -> RobotContainer.intake.activate(-.5), RobotContainer.intake).withTimeout(.1));
    NamedCommands.registerCommand("ShootOn",new InstantCommand(() ->RobotContainer.shooter.shoot(1), RobotContainer.shooter).withTimeout(.1));
    NamedCommands.registerCommand("Wrist Up", new RunCommand(()-> RobotContainer.wrist.moveWrist(0.3), RobotContainer.wrist).withTimeout(0.3));
    NamedCommands.registerCommand("Wrist Off", new RunCommand(()-> RobotContainer.wrist.moveWrist(0.0), RobotContainer.wrist).withTimeout(0.1));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
