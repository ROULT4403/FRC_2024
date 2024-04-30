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
import frc.robot.RobotContainer;

import static frc.robot.Constants.ElectronicConstants.*;

public class Intake extends SubsystemBase {
  // The intake motor controller is defined here.
  private final TalonFX intake = new TalonFX(talonFXIDs[0]);

  // A color sensor is ussd to detect if the note is taken completely by the intake.
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  // The boolean variable for the sensor is defined.
  private Boolean sensorActive = true;


  // The constructor class is called to initialize various settings.
  public Intake()
  {
    // The motor's mode is defined here...
    intake.setNeutralMode(falconBrakeMode);

    // The motor's inversion is defined here...
    intake.setInverted(clockWise);
  }


  // Activates the intake to take the game piece. 
  public void activate(double output) {
    intake.set(output);
  }
    
  // This command deactivates the intake when the sensor detects a note in a certain proximity 
  public Command intakeCommand(double output) {
    return startEnd(() -> activate(output), () ->activate(0.0)).until(this::colorgetMeasurement);
  }

  // A copy of the previous command, except that it is used during autonomous period.
  public Command autoIntakeCommand(double output) {
    return startEnd(() -> activate(output), () ->activate(output)).until(this::colorgetMeasurement);
  }

  // Method to change the status of the sensor
  public void sensorControl() {
    sensorActive = !sensorActive;
  }

  // Activates the intake to eject the note
  public Command outtakeCommand(double output) {
    return startEnd(() -> activate(output), () ->activate(0.0));
  }

  // This method is used to get the proximity of the note from the color sensor and know if the note is caught by the intake.
  public Boolean colorgetMeasurement(){

    int proximity = colorSensor.getProximity();

    // If the proximity is near enough and the sensor is active, it will send out that the note is in the intake
    if(proximity>300 && sensorActive == true){
      return true;
    }

    else return false;
  }


  @Override
  public void periodic(){

    // If the note is in the intake, and the respective button for the intake is pressed, the control will vibrate.
    if(colorgetMeasurement() && RobotContainer.chassisController.b().getAsBoolean()){
      RobotContainer.chassisController.getHID().setRumble(RumbleType.kBothRumble, .6);
    }

    else{ 
      RobotContainer.chassisController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    // Proximity of the note from the sensor.
    SmartDashboard.putBoolean("Note Proximity", colorgetMeasurement());
  }  
}