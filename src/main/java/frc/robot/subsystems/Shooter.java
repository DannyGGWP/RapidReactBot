// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;




/** Add your docs here. */
public class Shooter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private Solenoid m_shootySolenoid;

  public Shooter(){
    m_shootySolenoid=new Solenoid (Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kShootySolenoidIndex);
  }
  @Override 
  public void periodic(){}
@Override
public void simulationPeriodic() {
  // TODO Auto-generated method stub
  super.simulationPeriodic();
  

}

public void shootyShoot(boolean shotta){
  m_shootySolenoid.set(shotta);
}
}
