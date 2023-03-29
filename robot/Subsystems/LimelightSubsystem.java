
package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Mode;

public class LimelightSubsystem extends SubsystemBase {

  private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry ty = limelight.getEntry("ty");
  private final NetworkTableEntry tx = limelight.getEntry("tx");
  private NetworkTableEntry pipeline = limelight.getEntry("pipeline");
  private final Solenoid cube = new Solenoid(PneumaticsModuleType.CTREPCM, 0); // green true cube
  private final Solenoid cone = new Solenoid(PneumaticsModuleType.CTREPCM, 1); // red false cone
  

  private double x_offset;
  private double y_offset;
  private final double limelight_h = 71.4;
  private int pipe_number = 1;
  private double distance = 160;
  private final PIDController move_Controller = new PIDController(0.01, 0, 0);
  private final PIDController turn_Controller = new PIDController(0.06, 0, 0);
  private double goal_distance;


  public void set_pipe(Mode mode){
      pipeline.setNumber(mode.target.pipe_number);
  }

  public void set_distance(Mode mode){
    double target_h = mode.target.target_h;
    distance = (target_h-limelight_h)/Math.tan(Math.toRadians(y_offset-1));
  }

  public void set_goalDistance(Mode mode){
    goal_distance = mode.parameter.goal_distance;
  }

  public double getDistance(){
    return distance;
  }

  public double getGoalDistance(){
    return goal_distance;
  }


  private double constraints(double value, double max_output){
    if(value > max_output)
     return max_output;
    else if(value < -max_output)
      return -max_output;
    else
      return value;
  }


  public LimelightSubsystem() {
    pipeline.setNumber(pipe_number);
  }

  public double get_movePID(){
    if(y_offset != 0)
    return -constraints(move_Controller.calculate(distance,goal_distance),0.5);
    else
    return 0;
  }
  public double get_turnPID(){
    return constraints(turn_Controller.calculate(x_offset,0),0.5);
  }
  
  

  public void set_light(boolean light_state){
    cube.set(light_state);
    cone.set(!light_state);
  }

  public double get_y_offset(){
    return y_offset;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pipe_number", pipe_number);
    x_offset = tx.getDouble(0);
    y_offset = ty.getDouble(0);
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("x_offset", x_offset);
    SmartDashboard.putNumber("y_offset", y_offset);
    SmartDashboard.putString("Pipeline", pipeline.getName());
  }
}
