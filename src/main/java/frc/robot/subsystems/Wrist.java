// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import static frc.robot.Constants.ElectronicConstants.*;
import static frc.robot.Constants.WristConstants.*;

public class Wrist extends ProfiledPIDSubsystem
{
  // The wrist's motor controller is defined here...
  private final CANSparkMax wrist = new CANSparkMax(sparkMaxIDs[6], neoMotorType);

  // The wrist's encoder is defined here...
  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(encoderChannels[2]);

  // The Feedforward used by the wrist
  private final ArmFeedforward wristFeedforward = new ArmFeedforward(wristS, wristG, wristV);

  /** Creates a new WristPID. */
  public Wrist()
  {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            wristP,
            wristI,
            wristD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(wristMaxV, wristMaxA)));

    // The motor's mode is defined here...
    wrist.setIdleMode(neoBrakeMode);

    // The motor's inversion is defined here...
    wrist.setInverted(clockWise);

    // The encoders' distance per rotation are defined here...
    wristEncoder.setDistancePerRotation(wristDistancePerRotation);

    // The subsystem's tolerance is defined here...
    getController().setTolerance(wristTolerance);
  }

  /** Use to set the wrist's output... */
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint)
  {
    // Use the output and feedforward to activate the motor
    double feedforward = wristFeedforward.calculate(setpoint.position, setpoint.velocity);

    wrist.set(output + feedforward);
  }

  /** Use to get the wrist's position... */
  @Override
  public double getMeasurement()
  {
    // Return the process variable measurement here
    return wristEncoder.getDistance();
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Pos", getMeasurement());
  }

  /** Use to move manually... */
  public void manualWrist(double output)
  {
    wrist.set(output);
  }
}
