package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Mode;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  private final ArmSubsystem armsubsystem;
  private final double arm_setpoint;
  private final double arm_front_setpoint;
  private final double arm_rear_setpoint;
  private final double max_output_arm = Constants.max_output_arm;
  private final double max_output_front = Constants.max_output_front;
  private final double max_output_rear = Constants.max_output_rear;
  private final double error_range_arm = Constants.error_range_arm;
  private final double error_range_front = Constants.error_range_front;
  private final double error_range_rear = Constants.error_ragne_rear;
  private final PIDController front_PIDController = Constants.front_PIDController;
  private final PIDController rear_PIDController = Constants.rear_PIDController;
  private final PIDController arm_brake_PIDController = Constants.arm_brake_PIDController;
  private final PIDController front_brake_PIDController = Constants.front_brake_PIDController;
  private final PIDController rear_brake_PIDController = Constants.rear_brake_PIDController;
  private final PIDController arm_PIDController;
  private final Mode mode;
  private double arm_measurment;
  private double front_measurment;
  private double raer_measurment;
  private double arm_output;
  private double front_output;
  private double raer_output;
  
  private double cos_output;
  private final double cos_zeroPoint = 8.54174804687;

  public void SmartDashboard_putData(){
    SmartDashboard.putNumber("arm_goal", mode.parameter.arm_setpoint);
    SmartDashboard.putNumber("front_goal", mode.parameter.arm_front_setpoint);
    SmartDashboard.putNumber("rear_goal", mode.parameter.arm_rear_setpoint);
    SmartDashboard.putString("mode", mode.name);
  }

  public ArmCommand(ArmSubsystem subsystem, Mode _mode) {
    this.armsubsystem = subsystem;
    this.mode = _mode;
    this.arm_PIDController =mode.outPIDController;
    this.arm_setpoint = mode.parameter.arm_setpoint;
    this.arm_front_setpoint = mode.parameter.arm_front_setpoint;
    this.arm_rear_setpoint = mode.parameter.arm_rear_setpoint;
    addRequirements(armsubsystem);
  }

  @Override
  public void initialize() {
    RobotContainer.now_mode = mode;
    SmartDashboard.putString("modeName", mode.name);
  }

  @Override
  public void execute() {
    arm_measurment = armsubsystem.get_Arm_position();
    front_measurment = armsubsystem.get_front_position();
    raer_measurment = armsubsystem.get_rear_position();
    cos_output = 0.11 * Math.cos((arm_measurment - cos_zeroPoint)*5.625*2*Math.PI/360);
    arm_output = cos_output + armsubsystem.calculate_output(arm_setpoint, arm_measurment, max_output_arm, arm_PIDController,arm_brake_PIDController, error_range_arm)*((mode.name == "primitive_cone" || mode.name == "primitive_cube")? 0.4:1);
    front_output = armsubsystem.calculate_output(arm_front_setpoint, front_measurment, max_output_front,front_PIDController, front_brake_PIDController, error_range_front);
    raer_output = armsubsystem.calculate_output(arm_rear_setpoint, raer_measurment, max_output_rear,rear_brake_PIDController, rear_PIDController, error_range_rear);
    armsubsystem.Arm(arm_output, front_output, raer_output);
  }

  @Override
  public void end(boolean interrupted) {
    // armsubsystem.stop_intake(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
