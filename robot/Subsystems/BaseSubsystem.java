package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
// import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BaseSubsystem extends SubsystemBase {

  // MotorController
  private final CANSparkMax left1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax left2 = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax right1 = new CANSparkMax(1,MotorType.kBrushless);
  private final CANSparkMax right2 = new CANSparkMax(2, MotorType.kBrushless);

  // MotorGroup
  private final MotorControllerGroup left = new MotorControllerGroup(left1, left2);
  private final MotorControllerGroup right = new MotorControllerGroup(right1, right2);
  
  // Base
  private final DifferentialDrive drive = new DifferentialDrive(left, right);

  // Encoder
  private final RelativeEncoder left_encoder = left1.getEncoder();
  private final RelativeEncoder right_encoder = right1.getEncoder();

  //Gyro & balanceAngle
  private final AHRS gyro = new AHRS(Port.kMXP);
  private double balance_angle;
  private double angle;

  //PIDController
  private final PIDController brake_PIDController = new PIDController(0.3, 0, 0);
  private final PIDController balance_PIDController = new PIDController(0.1, 0, 0);
  private final PIDController tunrPIDController = new PIDController(0.01, 0, 0);
  private final PIDController movePIDController = new PIDController(0.004, 0, 0);

  //position
  private double left_position;

  // balance_state
  private boolean balance_state = true;

  // brake goal
  private double brake_goal = left_position;


  public void reset_balance_state(){
    balance_state = true;
  }

  /**
   *  就開車的，請先確保搖桿0號位為羅技搖桿的模式而不是GamePad模式(去DS看)
   * @param move 前進輸出
   * @param turn 選轉輸出 (搖桿往右為順時鐘選轉)
   */
  public void manualdrive(double move, double turn){
    drive.arcadeDrive(move, turn);
  }

  
  public final double get_turnPID(double goalAngle){
    return constraint(tunrPIDController.calculate(getAngle(), goalAngle), 0.5);
  }
  
  public final double get_movePID(double goalDistance){
    return constraint(movePIDController.calculate(left_position, goalDistance), 0.65);
  }

  public void resetencoder(){
    left_encoder.setPosition(0);
    right_encoder.setPosition(0);
  }

  public double getbalance_angle(){
    return balance_angle;
  }
  public double getAngle(){
    return angle;
  }
  
  /**
   * 限制輸入的最大值
   * @param value 被限制的值
   * @param max_output 最大輸出
   * @return 限制後的輸出
   */
  public double constraint(double value, double max_output){
    if(value > max_output)
      value = max_output;
    else if(value < -max_output)
      value = -max_output;
    return value;
  }

  /**
   * 計算平衡過程中底盤輸出
   * @return 底盤輸出
   */
  public double get_balancePID(){
    double output;
    if(Math.abs(balance_angle)<14){
      if(balance_state){
        brake_goal = left_position;
        balance_state = false;
      }
      output = brake_PIDController.calculate(left_position, brake_goal);
    }
    else{
      balance_state = true;
      output = constraint(balance_PIDController.calculate(balance_angle,0), 0.35);
    }
    return output;
  }

  /**
   * 由於每次調用時都要重置紀錄煞車的位置，在放開按鈕後執行。
   * @param state
   */
  public void set_balanceState(boolean state){
    balance_state = state;
  }

    
  
  /**
   * MotorController ininialize
   */
  private void resetMotorContorller(){
    left1.restoreFactoryDefaults();
    left2.restoreFactoryDefaults();
    right1.restoreFactoryDefaults();
    right2.restoreFactoryDefaults();
    left1.setIdleMode(IdleMode.kBrake);
    left2.setIdleMode(IdleMode.kBrake);
    right1.setIdleMode(IdleMode.kBrake);
    right2.setIdleMode(IdleMode.kBrake);
    left1.setInverted(false);
    left2.setInverted(false);
    right1.setInverted(true);
    right2.setInverted(true);
    left1.setOpenLoopRampRate(0.8);
    left2.setOpenLoopRampRate(0.8);
    right1.setOpenLoopRampRate(0.8);
    right2.setOpenLoopRampRate(0.8);
  }

  public void resetGyro(){
    gyro.reset();
  }

  public double getBasePosition(){
    return left_position;
  }

  public BaseSubsystem() {
    resetMotorContorller();
    resetencoder();
    resetGyro();
  }

  @Override
  public void periodic() {
    left_position = left_encoder.getPosition()*Units.inchesToMeters(6)*Math.PI/8.04*100;
    // right_position = right_encoder.getPosition();
    angle = -gyro.getAngle();
    balance_angle = gyro.getRoll();
    SmartDashboard.putNumber("left_encoder", left_position);
    SmartDashboard.putNumber("gyroRollAngle", balance_angle);
    SmartDashboard.putNumber("Angle", angle);
    // SmartDashboard.putNumber("right_encoder", right_position);
   }
}
