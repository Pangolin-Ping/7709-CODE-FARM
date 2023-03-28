package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;
import frc.robot.Subsystems.ArmSubsystem;

public class IntakeCommand extends CommandBase {
  
  private final ArmSubsystem armSubsystem;
  private Mode mode = Mode.primitive_cone;
  private final JoystickButton inButton;
  private final JoystickButton outButton;

  public IntakeCommand(ArmSubsystem _armSubsystem, Joystick _controlPanel) {
    this.armSubsystem = _armSubsystem;
    this.inButton = new JoystickButton(_controlPanel, 5);
    this.outButton = new JoystickButton(_controlPanel, 6);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mode = RobotContainer.now_mode;
    armSubsystem.Intake(mode,inButton, outButton);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
