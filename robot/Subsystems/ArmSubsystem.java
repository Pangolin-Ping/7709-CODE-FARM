package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Mode;

public class ArmSubsystem extends SubsystemBase {
  
  // Motor
  private final CANSparkMax arm_motor = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax arm_motor_follow = new CANSparkMax(6, MotorType.kBrushless);

  private final CANSparkMax arm_rear_motor = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax arm_front_motor = new CANSparkMax(8, MotorType.kBrushless);
  private final CANSparkMax intake_motor = new CANSparkMax(9, MotorType.kBrushless);

  // Encoder
  private final RelativeEncoder arm_front_encoder = arm_front_motor.getEncoder();
  private final RelativeEncoder arm_rear_encoder = arm_rear_motor.getEncoder();
  private final RelativeEncoder arm_encoder = arm_motor.getEncoder();
  private final RelativeEncoder arm_follow_encoder = arm_motor_follow.getEncoder();

  // potentiometer
  private final AnalogInput potentiometer_arm = new AnalogInput(0);
  private final AnalogInput potentiometer_rear = new AnalogInput(1);

  // setting parameter
  private final double zero_po_arm = 4043; //4358
  private final double zero_po_rear = 1329;  //3671
  private final double convert_rate_arm = 0.11798345;
  private final double convert_rate_rear = 0.11798345;
  private double arm_position = 0;
  private double arm_rear_position = 0;
  private double arm_front_position = 0;
  private double arm_follow_position = 0;

  /**
   * 限制輸出不超過max_output
   * @param value  需被限制的值
   * @param max_output  最大輸出的值(取絕對值)
   */
  public double constraint(double value ,double max_output){
    if(value > max_output)
      value = max_output;
    else if(value < -max_output)
      value = -max_output;
    return value;
  }

  /**
   * 給予手臂的控制器輸出
   * 
   * @param arm_output 手臂輸出
   * @param front_output 王亦友說fliping mechanism
   * @param raer_output 手腕輸出
   */
  public void Arm(double arm_output, double front_output, double raer_output){
    arm_motor.set(arm_output);
    arm_motor_follow.set(arm_output);
    arm_front_motor.set(front_output);
    arm_rear_motor.set(raer_output);
    SmartDashboard.putNumber("arm_output", arm_output);
    SmartDashboard.putNumber("front_output", front_output);
    SmartDashboard.putNumber("rear_output", raer_output);
    SmartDashboard.putNumber("arm_follow_output", arm_output);
  }


  /**
   * 控制intake的吸入與吐出
   * 
   * @param intakeSpeed intake輸出
   * @param inButton 吸入的按鈕
   * @param outButton 吐出的按鈕
   * @param reverse 反轉旋轉方向(根據mode不同調整)
   */
  public void Intake(Mode mode, JoystickButton inButton, JoystickButton outButton){
    boolean reverse = mode.parameter.intakeReverse;
    double intakeSpeed = mode.parameter.intake_speed;

    double intakeReverse = (reverse)? -1:1;
    if(inButton.getAsBoolean()){
      intake_setSpeed(intakeReverse * intakeSpeed);
    }
    else if(outButton.getAsBoolean()){
      intake_setSpeed(-intakeReverse * intakeSpeed);
    }
    else{
      intake_setSpeed(0.05 * intakeReverse);
    }
  }


  public void intake_setSpeed(double speed){
    intake_motor.set(speed);
  }

  public void intake_setSpeed(Mode mode,boolean reverse){
    double intakeSpeed = mode.parameter.intake_speed;
    double intakeReverse = (mode.parameter.intakeReverse)? -1:1;
    intake_setSpeed(intakeReverse*intakeSpeed*((reverse)? -1:1));
  }

  
  public void intake_setSpeed(double intakeSpeed,Mode mode,boolean reverse){
    double intakeReverse = (mode.parameter.intakeReverse)? -1:1;
    intake_setSpeed(intakeReverse*intakeSpeed*((reverse)? -1:1));
  }
  


  /**
   * 根據我們需要計算的物件的參數ex:手臂、手腕等等，計算PID的值。
   * 
   * @param setpoint 目標位置
   * @param measurment 目前位置
   * @param max_output 最大輸出
   * @param pidController 主要的PID
   * @param brake_Controller 煞車的PID
   * @param error_range 只要error介於error區間，輸出會切換至煞車PID
   * @return
   */
  public double calculate_output(double setpoint,double measurment ,double max_output,PIDController pidController, PIDController brake_Controller, double error_range){
    double error = Math.abs(setpoint - measurment);
    double output;
    if(error < error_range)
      output = brake_Controller.calculate(measurment, setpoint);
    else
      output = constraint(pidController.calculate(measurment, setpoint), max_output);
    return output;
  }
  
  /**
   * 馬達的所有初始化
   */
  private void reset_motor(){
    arm_motor.restoreFactoryDefaults();
    arm_rear_motor.restoreFactoryDefaults();
    intake_motor.restoreFactoryDefaults();
    arm_motor_follow.restoreFactoryDefaults();
    arm_motor_follow.setInverted(true);
    arm_motor.setInverted(false);
    arm_rear_motor.setInverted(true);
    intake_motor.setInverted(false);
    arm_motor_follow.setIdleMode(IdleMode.kBrake);
    arm_motor.setIdleMode(IdleMode.kBrake);
    arm_rear_motor.setIdleMode(IdleMode.kBrake);
    intake_motor.setIdleMode(IdleMode.kBrake);
    arm_encoder.setPosition((zero_po_arm-potentiometer_arm.getValue())*convert_rate_arm);
    arm_rear_encoder.setPosition((potentiometer_rear.getValue()-zero_po_rear)*convert_rate_rear);
  }


  private void armFrontReset(){
    arm_front_motor.restoreFactoryDefaults();
    arm_front_motor.setInverted(true);
    arm_front_motor.setIdleMode(IdleMode.kBrake);
    arm_front_encoder.setPosition(0);
  }

  public ArmSubsystem() {
    reset_motor();
    armFrontReset();
  }


  public double get_Arm_position(){
    return arm_position;
  }

  public double get_front_position(){
    return arm_front_position;
  }

  public double get_rear_position(){
    return arm_rear_position;
  }



  
  @Override
  public void periodic() {
    arm_position = arm_encoder.getPosition();
    arm_front_position = arm_front_encoder.getPosition();
    arm_rear_position = arm_rear_encoder.getPosition();
    arm_follow_position = arm_follow_encoder.getPosition();

    SmartDashboard.putNumber("arm_follow_position", arm_follow_position);
    SmartDashboard.putNumber("arm_position", arm_position);
    SmartDashboard.putNumber("arm_front_position",arm_front_position);
    SmartDashboard.putNumber("arm_rear_position", arm_rear_position);
    SmartDashboard.putNumber("potentiometer_arm", potentiometer_arm.getValue());
    SmartDashboard.putNumber("potentiometer_rear", potentiometer_rear.getValue()); 
    SmartDashboard.putNumber("intakeSpeed", intake_motor.getEncoder().getVelocity());
  }
}
