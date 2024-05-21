// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class shooter extends SubsystemBase
{
  // The shooter's motor controllers are defined here...
  private final VictorSPX leftShooter = new VictorSPX(0);
  private final VictorSPX rightShooter = new VictorSPX(1);

  // The shooter's encoders are defined here...


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



    leftShooter.set(ControlMode.PercentOutput,output);
    rightShooter.set(ControlMode.PercentOutput, output);
  }
 public Command shootCommand(double output){

          return startEnd(() -> shoot(output), () ->shoot(0.0));

  }

  /** Use to get the shooter's velocity... */


  @Override
  public void periodic()
  {
  }
}
