// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.BaseSubsystem;


// 出去後平衡

public class AutoCommandV1 extends CommandBase {
  private final BaseSubsystem basesubsystem;
  private final Timer time = new Timer();
  private double angle;
  private double time_now;
  private boolean start_balance = true;

  public AutoCommandV1(BaseSubsystem _basesubsystem) {
    this.basesubsystem = _basesubsystem;
    addRequirements(basesubsystem);
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
    start_balance = true;
  }

  @Override
  public void execute() {
    time_now = time.get();
    angle = basesubsystem.getbalance_angle();
    double output = 0.0;
    if(time_now < 4)
    {
      output  = 0.45;
    }
    else{
      if(start_balance){
      output = -0.55;
      if(Math.abs(angle)>14)
      start_balance = false;
      }
      else
      output = basesubsystem.get_balancePID();
    }
    basesubsystem.manualdrive(output, 0);
  }

  @Override
  public void end(boolean interrupted) {
    basesubsystem.manualdrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
