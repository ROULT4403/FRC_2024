// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.Wrist;

public class ArmPID extends PIDCommand {

  /*
   * This command turns the wrist to a specified angle, so we can do amp.
   * 
   * @param wrist Subsystem to use
   * @param targetAngle Position to be achieved
  */

  public ArmPID(Wrist wrist, double targetAngle) {
    super( 
       new PIDController(1.3, 0, 0),

        // Gets the angle in which the wrist is positioned.
        wrist::getMeasurement,

        // It returns a constant setpoint.
        targetAngle,
      
        // This uses the output to move the wrist.
        (output) -> {
          wrist.moveWrist(output);     
        });
    
    //Adds the subsystem required to perform the action.
    addRequirements(wrist);

    //We set the tolerance to this value because we don't really want almost any kind of margin of error.
    getController().setTolerance((-.00001));
  }

}