// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;

import static frc.robot.Constants.ElectronicConstants.*;

public class Intake extends SubsystemBase
{
  // The intake's motor controllers are defined here...
  private final TalonFX intake = new TalonFX(talonFXIDs[0]);
  private Boolean sensorActive = true;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  /** Creates a new Intake. */
  public Intake()
  {
    // The motor's mode is defined here...
    intake.setNeutralMode(falconBrakeMode);

    // The motor's inversion is defined here...
    intake.setInverted(clockWise);
  }

  /** Activates the intake to take the game piece. */
  public void activate(double output)
  {
    intake.set(output);
  }
    public Command intakeCommand(double output) {
      return startEnd(() -> activate(output), () ->activate(0.0)).until(this::colorgetMeasurement);

}
public Command autoIntakeCommand(double output) {
  return startEnd(() -> activate(output), () ->activate(output)).until(this::colorgetMeasurement);

}
public void sensorControl(){
  sensorActive = !sensorActive;
}


    public Command outtakeCommand(double output)
    {
      return startEnd(() -> activate(output), () ->activate(0.0));
    }

  @Override
  public void periodic()
  {
    if(colorgetMeasurement() && RobotContainer.chassisController.b().getAsBoolean()){
      RobotContainer.chassisController.getHID().setRumble(RumbleType.kBothRumble, .6);
    }
    else{RobotContainer.chassisController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Note Proximity", colorgetMeasurement());
  }

  public Boolean colorgetMeasurement()
  {
    int proximity = colorSensor.getProximity();
    if(proximity>300 && sensorActive == true){
      return true;
    }
    else return false;



  }
  
}