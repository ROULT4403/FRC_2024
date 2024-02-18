// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import static frc.robot.Constants.ElectronicConstants.*;
import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends ProfiledPIDSubsystem
{
  // The climber's motor controllers are defined here...
  private final TalonFX leftClimber = new TalonFX(talonFXIDs[1]);
  private final TalonFX rightClimber = new TalonFX(talonFXIDs[2]);

  // The climber's limit switch is defined here...
  private final DigitalInput limitSwitch = new DigitalInput(limitSwitchChannel);

  // The Feedforward used by the climber
  private final ElevatorFeedforward climberFeedforward = new ElevatorFeedforward(climberS, climberG, climberV);

  /** Creates a new Climber. */
  public Climber()
  {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            climberP,
            climberI,
            climberD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(climberMaxV, climberMaxA)));

    // The motor's mode is defined here...
    leftClimber.setNeutralMode(falconBrakeMode);
    rightClimber.setNeutralMode(falconBrakeMode);

    // The motor's inversion is defined here...
    leftClimber.setInverted(clockWise);
    rightClimber.setInverted(clockWise);

    // The subsystem's tolerance is defined here...
    getController().setTolerance(climberTolerance);
  }

  /** Use to set the climber's output... */
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint)
  {
    // Use the output (and optionally the setpoint) here
    double feedforward = climberFeedforward.calculate(setpoint.position, setpoint.velocity);

    leftClimber.set(output + feedforward);
    rightClimber.set(output + feedforward);
  }

  /** Use to get the climber's position... */
  @Override
  public double getMeasurement()
  {
    // Return the process variable measurement here
    double leftDistance = leftClimber.getPosition().getValueAsDouble() * climberDistanceConversion;
    double rightDistance = rightClimber.getPosition().getValueAsDouble() * climberDistanceConversion;
    return (leftDistance + rightDistance) / 2;
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Pos", getMeasurement());
  }

  /*Activates the climber. */
  public void activate(double output)
  {
    // The climber will stops after touchng the limit switch
    if (limitSwitch.get() == false)
    {
      leftClimber.set(output);
      rightClimber.set(output);
    }
    else
    {
      leftClimber.stopMotor();
      rightClimber.stopMotor();
    }
  }
}
