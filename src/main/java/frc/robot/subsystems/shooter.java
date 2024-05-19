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
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import static frc.robot.Constants.ElectronicConstants.*;

public class shooter extends SubsystemBase
{
  // The shooter's motor controllers are defined here...
  private final VictorSPX leftShooter = new VictorSPX(0); //Needs the IDs
  private final VictorSPX rightShooter = new VictorSPX(0); //Needs the IDs

  // The shooter's encoders are defined here...
  private final Encoder leftEncoder = new Encoder(0, 0); //Needs the IDs
  private final Encoder rightEncoder = new Encoder(0, 0); //Needs the IDs


  /** Creates a new Shooter. */
  public shooter()
  {
    // The motors' modes are defined here...
    leftShooter.setNeutralMode(NeutralMode.Brake);
    rightShooter.setNeutralMode(NeutralMode.Brake);
    // The motors' inversion are defined here...
    leftShooter.setInverted(counterClockWise);
    rightShooter.setInverted(clockWise);
  }

  /** Use to set the shooter's output... */
  public void shoot(double output)
  {
    //leftShooter.set(output);
    //rightShooter.set(output);

    //Didn´t find a way to limit current on a VICTOR SPX
;
    leftShooter.set(ControlMode.PercentOutput, output);
    rightShooter.set(ControlMode.PercentOutput, output);
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
    double leftVelocity = leftShooter.getSelectedSensorVelocity();
    double righVelocity = rightShooter.getSelectedSensorVelocity();
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
