// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer robotContainer = new RobotContainer();
  private Command baseCommand = robotContainer.get_baseCommand();
  private Command autoCommand = robotContainer.get_autoCommand();
  private Command intakeCommand = robotContainer.get_intakeCommand();
  private UsbCamera camera = CameraServer.startAutomaticCapture();

  // private Command armCommand;

  private void setCamera(){
    camera.setBrightness(0); // 亮度
    camera.setFPS(90); //跟新綠 每秒幾張照片
    camera.setExposureManual(0); 
    camera.setExposureAuto();
    camera.setResolution(320, 240); //解析度
  }

  @Override
  public void robotInit() {
    setCamera();
  }

  @Override
  public void robotPeriodic() {
    robotContainer.set_distance();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    baseCommand.schedule();
    intakeCommand.schedule();
  }

  @Override
  public void teleopPeriodic() {
    robotContainer.set_light();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
