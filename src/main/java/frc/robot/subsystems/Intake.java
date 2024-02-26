// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Extras.Rev2mDistanceSensor;

import static frc.robot.Constants.ElectronicConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase
{
  // The intake's motor controllers are defined here...
  private final TalonFX intake = new TalonFX(talonFXIDs[0]);
  
  // The intake's color sensor functions are defined here...
  private final Rev2mDistanceSensor distanceSensor = new Rev2mDistanceSensor(distanceSensorPort);

  /** Creates a new Intake. */
  public Intake()
  {
    // The motor's mode is defined here...
    intake.setNeutralMode(falconBrakeMode);

    // The motor's inversion is defined here...
    intake.setInverted(clockWise);
  }

  /** Activates the intake to take the game piece. */
  public void take(double output)
  {
    // The intake will stops after reaching the proximity limit
    if (distanceSensor.getRange() <= limitProximity)
    {
      intake.set(output);
    }
    else
    {
      intake.stopMotor();
    }
  }

  /** Activates the outtake to take the game piece. */
  public void outtake(double output)
  {
    intake.set(output);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance", distanceSensor.getRange());
  }
}
