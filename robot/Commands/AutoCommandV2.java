package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.BaseSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;


//放兩個
public class AutoCommandV2 extends CommandBase {
  private final BaseSubsystem basesubsystem;
  private final ArmCommand inCommand;
  private final ArmCommand outCommand1;
  private final ArmCommand outCommand2;
  private final LimelightSubsystem limelightSubsystem;
  private final ArmSubsystem armSubsystem;
  private final Timer time = new Timer();
  private final boolean in = false;
  private final boolean out = true;
  private Mode mode;
  private double angle;
  private double time_now;
  private boolean start_balance = true;
  private double output = 0;
  private boolean intakeReverse;
  

  public AutoCommandV2(BaseSubsystem _basesubsystem, LimelightSubsystem _LimelightSubsystem, ArmSubsystem _armSubsystem, ArmCommand _inCommand, ArmCommand _outCommand1, ArmCommand _outCommand2) {
    this.basesubsystem = _basesubsystem;
    this.inCommand = _inCommand;
    this.outCommand1 = _outCommand1;
    this.outCommand2 = _outCommand2;
    this.limelightSubsystem = _LimelightSubsystem;
    this.armSubsystem = _armSubsystem;
    addRequirements(basesubsystem, armSubsystem, limelightSubsystem);
  } 

  @Override
  public void initialize() {
    time.reset();
    time.start();
    outCommand1.schedule();
  }

  @Override
  public void execute() {
    output = 0.0;
    time_now = time.get();
    mode = RobotContainer.now_mode;

    angle = basesubsystem.getbalance_angle();
    if(time_now < 5)
    {
      output  = limelightSubsystem.get_movePID();
      if(time_now < 4.5)
        armSubsystem.intake_setSpeed(0.1, mode, in);
      else
        armSubsystem.intake_setSpeed(mode, out);
    }
    else if(time_now > 5 && time_now < 10 ){
      output = -0.55;
      if(time_now < 8){
        inCommand.schedule();
        armSubsystem.intake_setSpeed(0);
      }
      else{
        outCommand2.schedule();
        armSubsystem.intake_setSpeed(mode, in);
      }
    }
    
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
