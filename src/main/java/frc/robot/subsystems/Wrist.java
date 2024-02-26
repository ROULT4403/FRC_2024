// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElectronicConstants.*;
import static frc.robot.Constants.WristConstants.*;

public class Wrist extends SubsystemBase
{
  // The wrist's motor controller is defined here...
  private final CANSparkMax wrist = new CANSparkMax(sparkMaxIDs[6], neoMotorType);

  // The wrist's encoder is defined here...
  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(encoderChannels[2]);

  /** Creates a new WristPID. */
  public Wrist()
  {
    // The motor's mode is defined here...
    wrist.setIdleMode(neoBrakeMode);

    // The motor's inversion is defined here...
    wrist.setInverted(clockWise);

    // The encoders' distance per rotation are defined here...
    wristEncoder.setDistancePerRotation(wristDistancePerRotation);
  }

  /** Use to set the wrist's output... */
  public void moveWrist(double output)
  {
    wrist.set(output);
  }

  /** Use to get the wrist's position... */
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
}
