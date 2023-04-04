package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.BaseSubsystem;


public class AutoChargeStationCommand extends CommandBase {
  private final BaseSubsystem basesubsystem;
  private final ArmSubsystem armsubsystem;
  private final Timer time = new Timer();
  private final ArmCommand outCommand;
  private final ArmCommand inCommand;
  private final boolean in = false;
  private final boolean out = true;
  private Mode mode;
  private double angle;
  private double time_now;
  private boolean put_high = false;
  private boolean move_forward = false;
  private boolean start_balance = true;

  public AutoChargeStationCommand(BaseSubsystem _basesubsystem, ArmSubsystem _armsubsystem, ArmCommand _outcommand, ArmCommand _incommand) {
    this.basesubsystem = _basesubsystem;
    this.armsubsystem = _armsubsystem;
    this.outCommand = _outcommand;
    this.inCommand = _incommand;
    addRequirements(basesubsystem, armsubsystem);
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
    basesubsystem.resetGyro();
    basesubsystem.resetencoder();
    put_high = false;
    move_forward = false;
    start_balance = true;
    basesubsystem.set_balanceState(true);
  }

  @Override
  public void execute() {
    time_now = time.get();
    mode = RobotContainer.now_mode;
    angle = basesubsystem.getbalance_angle();
    double output = 0.0;
    if(!put_high){
      if(time_now < 3.3){
        outCommand.schedule();
        if(time_now < 2.7){
          armsubsystem.intake_setSpeed(0.07, mode, in);
        }
        else{
          armsubsystem.intake_setSpeed(0.4, mode, out);
        }
      }
      else{
        armsubsystem.intake_setSpeed(0);
        inCommand.schedule();
        if(time_now > 5){
          put_high = true;
          time.reset();
          time.start();
        }
      }
      output = 0;
    }
    if(put_high && !move_forward){
      if(time_now < 4){
        output  = 0.45;
      }
      else{
        if(start_balance){
        output = -0.55;
        if(Math.abs(angle)>14.1)
        start_balance = false;
        }
        else{
          output = basesubsystem.get_balancePID();
        }
    }
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
