// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElectronicConstants.*;

public class Shooter extends SubsystemBase
{
  // The shooter's motor controllers are defined here...
  private final CANSparkMax leftShooter = new CANSparkMax(sparkMaxIDs[4], neoMotorType);
  private final CANSparkMax rightShooter = new CANSparkMax(sparkMaxIDs[5], neoMotorType);

  // The shooter's encoders are defined here...
  private final RelativeEncoder leftEncoder = leftShooter.getEncoder();
  private final RelativeEncoder rightEncoder = rightShooter.getEncoder();

  /** Creates a new Shooter. */
  public Shooter()
  {
    // The motors' modes are defined here...
    leftShooter.setIdleMode(neoCoastMode);
    rightShooter.setIdleMode(neoCoastMode);

    // The motors' inversion are defined here...
    leftShooter.setInverted(clockWise);
    rightShooter.setInverted(counterClockWise);
  }

  /** Use to set the shooter's output... */
  public void shoot(double output)
  {
    leftShooter.set(output);
    rightShooter.set(output);
  }

  /** Use to get the shooter's velocity... */
  public double getMeasurement()
  {
    // Return the process variable measurement here...
    double leftVelocity = leftEncoder.getVelocity();
    double righVelocity = rightEncoder.getVelocity();
    return (leftVelocity + righVelocity) / 2;
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Vel", getMeasurement());
  }
}
