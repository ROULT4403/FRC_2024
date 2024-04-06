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
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.MutableMeasure.mutable;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.ElectronicConstants.*;
import static frc.robot.Constants.WristConstants.*;

import javax.swing.text.Position;

public class PIDWrist extends PIDSubsystem
{
  // The wrist's motor controller is defined here...
  private final CANSparkMax wrist = new CANSparkMax(3, neoMotorType);
private final ArmFeedforward feedforward = new ArmFeedforward(0.37485, 7.0038, 0.11421, 1.0256);
  // The wrist's encoder is defined here...
private final Encoder wristEncoder = new Encoder(2,3);  //MutableMeasure variables for sysID
private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
private final MutableMeasure<Angle> m_Position = mutable(Radians.of(0));
private final MutableMeasure<Velocity<Angle>> m_wristVelocity = mutable(DegreesPerSecond.of(0));


private SysIdRoutine m_sysIdRoutine = 
new SysIdRoutine(new SysIdRoutine.Config(Volts.of(.25).per(Seconds.of(1)), Volts.of(1), null), 
new SysIdRoutine.Mechanism((Measure<Voltage> volts)-> {wrist.setVoltage(volts.in(Volts));}, 
log -> {
  log.motor("wrist-motor").voltage(m_appliedVoltage.mut_replace((wrist.getBusVoltage() * wrist.getAppliedOutput()) , Volts))
  .angularPosition(m_Position.mut_replace(wristEncoder.getDistance(), Radians))
  .angularVelocity(m_wristVelocity.mut_replace(wristEncoder.getRate(), RadiansPerSecond));},this));


  /** Creates a new WristPID. */
  public PIDWrist()
  {
    super(new PIDController(62.192,0,0));
    // The motor's mode is defined here...
    wrist.setIdleMode(neoBrakeMode);
    // The motor's inversion is defined here...
    wrist.setInverted(clockWise);
    // The encoders' distance per rotation are defined here...
    wristEncoder.setDistancePerPulse((2 * Math.PI)/8192);
    getController().setTolerance(.005);
    setSetpoint(-.36);
  }

  

  /** Use to set the wrist's output... */
  public void moveWrist(double output)
  {
    wrist.set(output);
  }
  public Command wristCommand(double output){
          return startEnd(() -> moveWrist(output), () ->moveWrist(0.0));}
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_sysIdRoutine.quasistatic(direction);
  }
    public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_sysIdRoutine.dynamic(direction);
  }
  @Override
  public void useOutput(double output,double setpoint){
    wrist.setVoltage(output);
  }
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
    SmartDashboard.putNumber("Wrist Position", wristEncoder.getDistance());
  }
}
