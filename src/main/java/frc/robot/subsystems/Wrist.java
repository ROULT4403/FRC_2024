package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase{

    private final TalonFX wrist = new TalonFX(5);

public Wrist() {}

public void activate(double input3){
  wrist.set(input3);
}


public double getMeasurement()
  {
    return ((wrist.getPosition().getValueAsDouble()/ (2048*(1/50))) * (2 * Math.PI));
  }

@Override
  public void periodic()
  {
    SmartDashboard.putNumber("WRIST", getMeasurement());
  }
}