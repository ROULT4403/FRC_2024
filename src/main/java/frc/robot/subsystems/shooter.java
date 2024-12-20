// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;

import static frc.robot.Constants.ElectronicConstants.*;

public class shooter extends SubsystemBase
{
  // The shooter's motor controllers are defined here...
  private final CANSparkMax leftShooter = new CANSparkMax(sparkMaxIDs[4], neoMotorType);
  private final CANSparkMax rightShooter = new CANSparkMax(sparkMaxIDs[5], neoMotorType);

  // The shooter's encoders are defined here...
  private final RelativeEncoder leftEncoder = leftShooter.getEncoder();
  private final RelativeEncoder rightEncoder = rightShooter.getEncoder();


  /** Creates a new Shooter. */
  public shooter()
  {
    // The motors' modes are defined here...
    leftShooter.setIdleMode(neoCoastMode);
    rightShooter.setIdleMode(neoCoastMode);
    // The motors' inversion are defined here...
    leftShooter.setInverted(counterClockWise);
    rightShooter.setInverted(clockWise);
  }

  /** Use to set the shooter's output... */
  public void shoot(double output)
  {
    //leftShooter.set(output);
    //rightShooter.set(output);
    leftShooter.setSmartCurrentLimit(40);
    rightShooter.setSmartCurrentLimit(40);


    leftShooter.set(output);
    rightShooter.set(output);
  }
 public Command shootCommand(double output){

          return startEnd(() -> shoot(output), () ->shoot(0.0));

  }
public void rumbleShooter(XboxController controller){
  if (getMeasurement()>= 3500){
    controller.setRumble(RumbleType.kRightRumble, .6);
  }
  else{
    controller.setRumble(RumbleType.kRightRumble, 0);

  }}
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
    rumbleShooter(RobotContainer.rumbleMechController);

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Vel", getMeasurement());

  }
}
