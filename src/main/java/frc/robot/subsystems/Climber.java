// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElectronicConstants.*;
import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends SubsystemBase
{
  // The climber's motor controllers are defined here...
  private final TalonFX leftClimber = new TalonFX(talonFXIDs[1]);
  private final TalonFX rightClimber = new TalonFX(talonFXIDs[2]);

  // The climber's limit switch is defined here...
  private final DigitalInput upperSwitch = new DigitalInput(limitSwitchChannel[0]);
  private final DigitalInput lowerSwitch = new DigitalInput(limitSwitchChannel[1]);
  private Double speedMultiplier = 1.0;
  /** Creates a new Climber. */
  public Climber()
  {
    // The motor's mode is defined here...
    leftClimber.setNeutralMode(falconBrakeMode);
    rightClimber.setNeutralMode(falconBrakeMode);

    // The motor's inversion is defined here...
    leftClimber.setInverted(clockWise);
    rightClimber.setInverted(counterClockWise);

    resetEncoders();
  }

  public void climb(double output)
  {
    if(upperSwitch.get() && output > 0 )//&& getMeasurement()>=.00485
    {
      leftClimber.set(0);
      rightClimber.set(0);
    }
    else if(lowerSwitch.get() && output < 0 ) // && getMeasurement()<0
    {
      leftClimber.set(0);
      rightClimber.set(0);
    }
    else if(getMeasurement()>=.0035 && output>0){
      leftClimber.set(output/2);
      rightClimber.set(output/2);
    }

    else {
      leftClimber.set(output);
      rightClimber.set(output);
    }
    

    }
  

  /** Use to get the climber's position... */
  public double getMeasurement()
  {
    // Return the process variable measurement here
    double leftDistance = (leftClimber.getPosition().getValueAsDouble() / climberPulse) * climberDistanceConversion;
    double rightDistance = (rightClimber.getPosition().getValueAsDouble() / climberPulse) * climberDistanceConversion;
    return (leftDistance + rightDistance) / 2;
  }

  public void resetEncoders()
  {
    leftClimber.setPosition(0);
    rightClimber.setPosition(0);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Pos", getMeasurement());
    SmartDashboard.putBoolean("UpperSwitch Status", upperSwitch.get());
    SmartDashboard.putBoolean("LowerSwitch Status", lowerSwitch.get());
    if(getMeasurement()>=.0035){
      speedMultiplier = .4;

    }
    else{
      speedMultiplier = 1.0;
    }
  }
}
