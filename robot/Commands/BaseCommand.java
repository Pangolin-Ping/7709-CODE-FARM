package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.BaseSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class BaseCommand extends CommandBase {

  private BaseSubsystem baseSubsystem;
  private LimelightSubsystem limelightSubsystem;

  private double move;
  private double turn;
  private Joystick stick;
  private JoystickButton aim_button;
  private JoystickButton balance_button;

  public BaseCommand(
    BaseSubsystem _basesubsystem,
    LimelightSubsystem _limelightsubsystem,
    ArmSubsystem _ArmSubsystem,
    Joystick _stick,
    JoystickButton _aim_button,
    JoystickButton _balance_button) 
    {
    this.stick = _stick;
    this.baseSubsystem = _basesubsystem;
    this.limelightSubsystem = _limelightsubsystem;
    this.aim_button = _aim_button;
    this.balance_button = _balance_button;
    addRequirements(baseSubsystem);
  }

  @Override
  public void initialize() {
    baseSubsystem.balance_state = true;
  }

  @Override
  public void execute() {
    double distance = limelightSubsystem.getDistance();

    if(balance_button.getAsBoolean()){
      move = baseSubsystem.get_balancePID();
      turn = 0;
    }
    else if(aim_button.getAsBoolean() && limelightSubsystem.get_y_offset() != 0){
      move = (distance < 500 && distance > 50)?limelightSubsystem.get_movePID():0;
      turn = (distance < 500 && distance > 50)?limelightSubsystem.get_turnPID():0;
      SmartDashboard.putNumber("movePID", move);
      SmartDashboard.putNumber("turnPID", turn);
    }
    else{
      baseSubsystem.balance_state = true;
      move = -stick.getY()*0.5;//75
      turn = -stick.getZ()*0.5;//58

    }
    baseSubsystem.manualdrive(move, turn);
    
    SmartDashboard.putNumber("move", move);
    SmartDashboard.putNumber("turn", turn);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
