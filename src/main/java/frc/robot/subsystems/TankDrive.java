// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.TankDriveConstants.*;
import static frc.robot.Constants.ElectronicConstants.*;

public class TankDrive extends SubsystemBase
{
  // The tank drive's motor controllers are defined here...
  private final CANSparkMax leaderLeft = new CANSparkMax(sparkMaxIDs[0], neoMotorType);
  private final CANSparkMax followLeft = new CANSparkMax(sparkMaxIDs[1], neoMotorType);
  private final CANSparkMax leaderRight = new CANSparkMax(sparkMaxIDs[2], neoMotorType);
  private final CANSparkMax followRight = new CANSparkMax(sparkMaxIDs[3], neoMotorType);

  // The tank drive's encoders/sensors are defined here...
  private final DutyCycleEncoder leftEncoder = new DutyCycleEncoder(encoderChannels[0]);
  private final DutyCycleEncoder rightEncoder = new DutyCycleEncoder(encoderChannels[1]);
  private final AHRS navx = new AHRS(navxPort);

  // The differential drive is defined here...
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leaderLeft, leaderRight);

  /** Creates a new Tank Drive. */
  public TankDrive()
  {
    // The motors' modes are defined here...
    leaderLeft.setIdleMode(neoBrakeMode);
    followLeft.setIdleMode(neoBrakeMode);
    leaderRight.setIdleMode(neoBrakeMode);
    followRight.setIdleMode(neoBrakeMode);

    // The motor groups are defined here...
    followLeft.follow(leaderLeft);
    followRight.follow(leaderRight);

    // The encoders' distance per rotation are defined here...
    leftEncoder.setDistancePerRotation(chassisDistancePerRotation);
    rightEncoder.setDistancePerRotation(chassisDistancePerRotation);

    // The deadband is defined here...
    differentialDrive.setDeadband(chassisDeadband);
  }

  /** Use to drive the chassis... */
  public void drive(double speed, double rot)
  {
    differentialDrive.arcadeDrive(speed, rot);
  }

  /** Use to get the chassis' distance... */
  public double getMeasurement()
  {
    // Return the process variable measurement here...
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    return (leftDistance + rightDistance) / 2;
  }

  /** Use to get the chassis' direction... */
  public double getDirection()
  {
    return navx.getRotation2d().getDegrees();
  }

  /** Use to reset all encoders/sensors */
  public void reset()
  {
    navx.reset();

    leftEncoder.reset();
    rightEncoder.reset();
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance", getMeasurement());
  }
}
