// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankDrive extends SubsystemBase {
  private final CANSparkMax m_right_up = new CANSparkMax(Constants.ID_motors.m_right_up, MotorType.kBrushless);
  private final CANSparkMax m_right_down = new CANSparkMax(Constants.ID_motors.m_right_down, MotorType.kBrushless);  
  
  private final CANSparkMax m_left_up = new CANSparkMax(Constants.ID_motors.m_left_up, MotorType.kBrushless);
  private final CANSparkMax m_left_down = new CANSparkMax(Constants.ID_motors.m_left_down, MotorType.kBrushless);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_right_up, m_left_up);

  private final AHRS navX = new AHRS(SPI.Port.kMXP);

  private final DutyCycleEncoder encoder_left = new DutyCycleEncoder(3);
  private final DutyCycleEncoder encoder_right = new DutyCycleEncoder(4);


  /** Creates a new TankDrive. */
  public TankDrive() {
    //Motores drive
    m_right_down.setInverted(true);
    m_right_up.setInverted(true);

    m_right_down.follow(m_right_up);
    m_left_down.follow(m_left_up); 

    m_drive.setDeadband(0.052);

    //Encoders  absolutos
    encoder_left.reset();
    encoder_right.reset();

    m_left_down.setIdleMode(IdleMode.kBrake);                  //-- Mod motors --//
    m_right_down.setIdleMode(IdleMode.kBrake);                 //---------------//
    m_left_up.setIdleMode(IdleMode.kBrake);                    //---------------//
    m_right_up.setIdleMode(IdleMode.kBrake);                   //---------------//
     
    encoder_left.setDistancePerRotation(Constants.OperatorConstants.calculatedistance);
    encoder_right.setDistancePerRotation(Constants.OperatorConstants.calculatedistance);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Eco_right",encoder_right.getDistance());
    SmartDashboard.putNumber("Eco_left", encoder_left.getDistance());

    SmartDashboard.putBoolean("Conect_eco_right", encoder_right.isConnected());
    SmartDashboard.putBoolean("Concet_eco_left", encoder_left.isConnected());

    SmartDashboard.putNumber("Distance", getDistance());
  }

  public void drive(double speed, double rot){
    m_drive.arcadeDrive(speed*1.5, rot*0.75);
  }

  public double getDistance(){
    return (-encoder_left.getDistance()+ encoder_right.getDistance())/2;
  }
}