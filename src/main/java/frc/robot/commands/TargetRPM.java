// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TargetRPM extends PIDCommand {
  /** Creates a new ArmPID. */
  public TargetRPM(shooter shooter, double setpoint) {
    super(
        // The controller that the command will use
        //new PIDController(62.192, 0, 0),
       new PIDController(2, 0.0, 0.0),
        // This should return the measurement
        shooter::getMeasurement,
        // This should return the setpoint (can also be a constant)
        setpoint,
        // This uses the output
        (output) -> {
          shooter.shoot(output);// Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(shooter);
    getController().setTolerance((100));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
