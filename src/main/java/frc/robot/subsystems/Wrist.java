// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerMeterPerSecond;
import static edu.wpi.first.units.Units.Radian;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.MutableMeasure.mutable;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.*;


import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;

import static frc.robot.Constants.ElectronicConstants.*;
import static frc.robot.Constants.WristConstants.*;

public class Wrist extends SubsystemBase
{
  // The wrist's motor controller is defined here...
  private final CANSparkMax wrist = new CANSparkMax(3, neoMotorType);

  // The wrist's encoder is defined here...
  private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(encoderChannels[4]);


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

  public void rumbleAction(CommandXboxController chassis,XboxController rumblemechcontroller){
  if (!chassis.x().getAsBoolean()){
    rumblemechcontroller.setRumble(RumbleType.kLeftRumble, 0);
  }
  else{
    rumblemechcontroller.setRumble(RumbleType.kLeftRumble, 1);

  }
}

  /** Use to get the wrist's position... */
  public double getMeasurement()
  {
    // Return the process variable measurement here
    return wristEncoder.getDistance();
  }

public void wristCommand(double speed){
  wrist.set(speed);
}

 
public void rumbleAction(CommandXboxController chassis,CommandXboxController mech){
  if (!chassis.x().getAsBoolean()){
    mech.getHID().setRumble(RumbleType.kLeftRumble, 0);
  }
  else{
    mech.getHID().setRumble(RumbleType.kLeftRumble, 1);

  }
}
  @Override
  public void periodic()
  {


rumbleAction(RobotContainer.chassisController, RobotContainer.mechController);
    
    SmartDashboard.putNumber("Wrist Pos", getMeasurement());

    }
  }
