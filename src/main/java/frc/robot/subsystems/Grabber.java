// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {

  private ColorSensorV3 m_colorSensor;
  private ColorMatch m_colorMatcher = new ColorMatch();

  /** Creates a new Grabber. */
  public Grabber() {
    m_colorSensor = new ColorSensorV3(Port.kOnboard);
    m_colorMatcher.addColorMatch(Color.kBlue);
    m_colorMatcher.addColorMatch(Color.kRed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Color", stringColor());
  }

  public String stringColor() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    SmartDashboard.putNumber("Detected Color Red", detectedColor.red);
    SmartDashboard.putNumber("Detected Color Green", detectedColor.green);
    SmartDashboard.putNumber("Detected Color Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    if (match.color == Color.kRed) {
      return "Red";
    } else {
      return "Blue";
    }
  }
}
