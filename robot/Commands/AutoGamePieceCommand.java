

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoMode;
import frc.robot.Constants.Mode;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.BaseSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;


// 自瞄自放 2cone
public class AutoGamePieceCommand extends CommandBase {

  private final BaseSubsystem baseSubsystem;
  private final ArmSubsystem armSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final Timer time = new Timer();
  private final ArmCommand outCommand1;
  private final ArmCommand outCommand2;
  private final ArmCommand outCommand3;
  private final ArmCommand inCommand;
  private final boolean in = false;
  private final boolean out = true;
  private final AutoMode automode;
  public final double error_distance;
  public final double PID_distance;
  public final double first_angle;
  public final double second_angle;
  private Mode mode;
  private boolean put_high = false;
  private boolean move500 = false;
  private boolean turn180 = false;
  private boolean getTarget = false;
  private boolean resetDistance = false;
  private double move = 0;
  private double turn = 0;


  public AutoGamePieceCommand(ArmSubsystem _armSubsystem,BaseSubsystem _BaseSubsystem, LimelightSubsystem _limelightSubsystem, ArmCommand _outCommnad1,ArmCommand _outCommand2, ArmCommand _outCommand3, ArmCommand _inCommand, AutoMode _automode) {
    this.baseSubsystem = _BaseSubsystem;
    this.limelightSubsystem = _limelightSubsystem;
    this.outCommand1 = _outCommnad1;
    this.outCommand2 = _outCommand2;
    this.outCommand3 = _outCommand3;
    this.inCommand = _inCommand;
    this.armSubsystem = _armSubsystem;
    this.automode = _automode;
    this.error_distance = automode.error_distance;
    this.PID_distance = automode.PID_distance;
    this.first_angle = automode.first_angle;
    this.second_angle = automode.second_angle;
  }

  @Override
  public void initialize() {
    time.reset();
    time.start();
    baseSubsystem.resetGyro();
    baseSubsystem.resetencoder();
    turn180 = false;
    put_high = false;
    move500 = false;
    getTarget = false;
    resetDistance = false;
  }

  @Override
  public void execute() {
    double time_now = time.get();
    mode = RobotContainer.now_mode;
    if(!put_high){
      if(time_now < 3.3){
        outCommand1.schedule();
        if(time_now < 2.7)
        armSubsystem.intake_setSpeed(0.07, mode, in);
        else
         armSubsystem.intake_setSpeed(0.4, mode, out);
        }
      else{
       armSubsystem.intake_setSpeed(0);
       inCommand.schedule();
        if(time_now > 5)
          put_high = true;
      }
    }
    if(put_high && !move500){
      double error = error_distance - baseSubsystem.getBasePosition();
      SmartDashboard.putNumber("error", error);
      if(Math.abs(error) > 12){
        move = baseSubsystem.get_movePID(PID_distance);
        turn = baseSubsystem.get_turnPID(first_angle);
        outCommand2.schedule();
        armSubsystem.intake_setSpeed(mode, in);
      }
      else{
        move500 = true;
        inCommand.schedule();
        armSubsystem.intake_setSpeed(0);
      }
    }
    if(move500 && !turn180){
      move = 0;
      turn = baseSubsystem.get_turnPID(180);
      double error = 175 - baseSubsystem.getAngle();
      if(Math.abs(error) < 5){
        turn180 = true;
        outCommand3.schedule();
        time.reset();
        time.start();
      }
    }
    if(turn180 && !getTarget)
    {
      move = 0.65;
      turn = baseSubsystem.get_turnPID(second_angle);
      armSubsystem.intake_setSpeed(0.07, mode, in);
      if(limelightSubsystem.get_y_offset()!= 0 && time_now  > 1.5){
        getTarget = true;
      }
    }
    if(getTarget){
      move = limelightSubsystem.get_movePID();
      turn = limelightSubsystem.get_turnPID();
      double error = limelightSubsystem.getDistance() - limelightSubsystem.getGoalDistance();
      if( Math.abs(error) < 10 && !resetDistance){
        resetDistance = true;
        time.reset();
        time.start();
      }
      if(resetDistance)
      {
        if(time_now > 2.8)
          armSubsystem.intake_setSpeed(mode, out);
      }
    }
    baseSubsystem.manualdrive(move, turn);
    SmartDashboard.putNumber("move", move);
    SmartDashboard.putNumber("turn", turn);


  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
