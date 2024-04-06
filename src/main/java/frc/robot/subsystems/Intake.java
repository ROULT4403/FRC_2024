// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElectronicConstants.*;

public class Intake extends SubsystemBase
{
  // The intake's motor controllers are defined here...
  private final TalonFX intake = new TalonFX(talonFXIDs[0]);

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
      return startEnd(() -> activate(output), () ->activate(0.0));

}

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }
}
