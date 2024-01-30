// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ElecConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
  private final CANSparkMax shooter1 = new CANSparkMax(ElecConstants.shooter1ID, MotorType.kBrushless);
  private final CANSparkMax shooter2 = new CANSparkMax(ElecConstants.shooter2ID, MotorType.kBrushless);
  /** Creates a new shooter. */
  public shooter() {
    shooter1.restoreFactoryDefaults();
    shooter2.restoreFactoryDefaults();
    shooter1.setIdleMode(IdleMode.kCoast);
    shooter2.setIdleMode(IdleMode.kCoast);
    shooter1.setInverted(ElecConstants.shooterinverted[0]);
    shooter2.setInverted(ElecConstants.shooterinverted[1]);
  }

public void activate(double input) {
  shooter1.set(input);
  shooter2.set(input);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}