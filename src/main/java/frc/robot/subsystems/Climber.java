// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElectronicConstants.*;
import static frc.robot.Constants.ClimberConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends SubsystemBase{
  // The climber's motor controllers are defined here...
  private final VictorSPX leftClimber = new VictorSPX(0); //Needs the IDs
  private final VictorSPX rightClimber = new VictorSPX(0); //Needs the IDs

  // The climber's limit switch is defined here...
  private final DigitalInput upperSwitch = new DigitalInput(limitSwitchChannel[0]);
  private final DigitalInput lowerSwitch = new DigitalInput(limitSwitchChannel[1]);
  private double speedMultiplier = 1;
  public Integer commandSpeed = 0;



  /** Creates a new Climber. */
  public Climber()
  {
    // The motor's mode is defined here...
    leftClimber.setNeutralMode(NeutralMode.Brake);
    rightClimber.setNeutralMode(NeutralMode.Brake);

    // The motor's inversion is defined here...
    leftClimber.setInverted(clockWise); 
  }




  /** Activates climber mechanism, stopping until the limit switch is pressed or the */
      public Command climbUp(double input) {
        setClimbDirection(1);

      return startEnd(() -> climb(input), () ->climb(0.0)).until(upperSwitch::get);

}
  public Command climbDown(double input) {
    setClimbDirection(-1);

    return startEnd(() -> climb(input), () ->climb(0.0)).until(lowerSwitch::get);
  }
  public void setClimbDirection(Integer direction){
    commandSpeed = direction;

  }

  public void climb(double speed)
  {

    leftClimber.set(ControlMode.Velocity, speed);
    rightClimber.set(ControlMode.Velocity, speed);
  }
 

  /** Use to get the climber's position... */


  @Override
  public void periodic()
  {
    SmartDashboard.putBoolean("UpperSwitch Status", upperSwitch.get());
    SmartDashboard.putBoolean("LowerSwitch Status", lowerSwitch.get());

  }}

