// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  public final TalonFX intake = new TalonFX(0);

  public Intake() {
    intake.setInverted(false);
    intake.setNeutralMode(NeutralModeValue.Brake);
  }
 
  public void activate(boolean input1, boolean input2){
    intake.set(1*05);

    if(input1 == true)
    {
      intake.set(1*0.5);
    }
    else if(input2 == true)
    {
      intake.set(-1 * 0.5);
    }
    else
    {
      intake.set(0);
    }
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}