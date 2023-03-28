// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

    public final static double max_output_arm = 0.3;
    public final static double max_output_front = 0.2;
    public final static double max_output_rear = 0.25;
    public final static double error_range_arm = 0;
    public final static double error_range_front = 0.1;
    public final static double error_ragne_rear = 1;
    private final static double tape_upper_h = Units.inchesToMeters(45.875)*100;
    private final static double tape_lower_h = Units.inchesToMeters(26.125)*100;
    private final static double cube_tag_grid_h = Units.inchesToMeters(18.25)*100;
    private final static double arpil_tag_double_h = Units.inchesToMeters(27.375)*100; //69.5
    public final static PIDController front_PIDController = new PIDController(0.1, 0, 0);
    public final static PIDController rear_PIDController = new PIDController(0.1, 0, 0);
    public final static PIDController arm_brake_PIDController = new PIDController(0.04, 0, 0);
    public final static PIDController front_brake_PIDController = new PIDController(0.1, 0, 0);
    public final static PIDController rear_brake_PIDController = new PIDController(0.2, 0, 0);


    //不同模式下的手臂PIDController
    public static final class Arm_OutPIDController {
        // grid
        public static final PIDController cone_upper = new PIDController(0.015, 0.00, 0.);
        public static final PIDController cube_upper = new PIDController(0.015, 0.00, 0.);
        public static final PIDController cone_middle = new PIDController(0.063, 0, 0.00);
        public static final PIDController cube_middle = new PIDController(0.063, 0, 0.00);
        public static final PIDController cone_lower = new PIDController(0.02, 0, 0.00);
        public static final PIDController cube_lower = new PIDController(0.02, 0, 0.00);

        // double substation
        public static final PIDController cone_doubleSubstation = new PIDController(0.063, 0, 0.00);
        public static final PIDController cube_doubleSubstation = new PIDController(0.063, 0, 0.00);

        // single substation
        public static final PIDController cone_singleSubstation = new PIDController(0.035, 0, 0.00);
        public static final PIDController cube_singleSubstation = new PIDController(0.035, 0, 0.00);

        // ground
        public static final PIDController cone_ground = new PIDController(0.02, 0, 0.00);
        public static final PIDController cube_ground = new PIDController(0.02, 0, 0.00);

        //primitive 
        public static final PIDController primitive = new PIDController(0.02, 0, 0.01);
    }
    

    // 存放每個模式下的手臂各部件的位置
    public static enum Parameter{
        double_substation_cone(17,10,14.4548,0.5,0,false),
        double_substation_cube( 16,13.3,14.4548,0.5,0,true),
        single_substation_cone(6.5,15,12,0.35,0,false),
        single_substation_cube( 9,15,12,0.35,0,true),
        ground_cone(-5,11,10,0.5,0,false),
        ground_cube(-6,15,13,0.5,0,true),
        upper_cone(49.7,15.5,18.4548,0.5,130,false),
        upper_cube(50.2,20,19.4548,0.5,130,true),
        middle_cone(17,13,16.4548,0.3,107,false),
        middle_cube(18,13,16.4548,0.5,143,true),
        lower_cone(3,10,11.5,0.5,0,false),
        lower_cube(3,14,14.5,0.5,0,true),
        primitive_cone(0,10,0,0.4,0,false),
        primitive_cube(0,10,0,0.5,0,true);
        public final double arm_setpoint;
        public final double arm_front_setpoint; 
        public final double arm_rear_setpoint; // 手腕
        public final double intake_speed;
        public final double goal_distance;
        public final boolean intakeReverse;
        Parameter(double _arm_setpoint,double _arm_front_setpoint,double _arm_rear_setpoint,double _intake_speed, double _goal_distance, boolean _intakeReverse){
        this.arm_setpoint = _arm_setpoint;
        this.arm_rear_setpoint = _arm_rear_setpoint;
        this.arm_front_setpoint = _arm_front_setpoint;
        this.intake_speed = _intake_speed;
        this.goal_distance = _goal_distance;
        this.intakeReverse = _intakeReverse;
        }
    }

    //40.3473

    //本來要搞視覺辨識看來是沒料
    //目前沒有功用
    public enum Target{
        cube_grid(1,cube_tag_grid_h,"cube_tag_grid"), //green
        upper_tape(0,tape_upper_h,"tape_upper_h"), // red
        lower_tape(2,tape_lower_h,"tape_lower_h"),
        cube_double(1,arpil_tag_double_h,"cube_tag_double"),
        None(1,0,"None");
        public final int pipe_number;
        public final double target_h;
        public final String target_name;
        Target(int _pipe_number, double _target_h,String _target_name){
          this.pipe_number = _pipe_number;
          this.target_h = _target_h;
          this.target_name = _target_name;
        }
    }

      // 所有的模式
      public enum Mode{
        double_subsystem_cone(Parameter.double_substation_cone,Target.None,"double_subsystem_cone", Arm_OutPIDController.cone_doubleSubstation),
        double_subsystem_cube(Parameter.double_substation_cube,Target.None,"double_subsystem_cube", Arm_OutPIDController.cube_doubleSubstation),
        single_subsystem_cone(Parameter.single_substation_cone,Target.None,"single_subsystem_cone", Arm_OutPIDController.cone_singleSubstation),
        single_subsystem_cube(Parameter.single_substation_cube,Target.None,"single_subsystem_cube", Arm_OutPIDController.cube_singleSubstation),
        ground_cone(Parameter.ground_cone,Target.None,"ground_cone",Arm_OutPIDController.cone_ground),
        ground_cube(Parameter.ground_cube,Target.None,"ground_cube",Arm_OutPIDController.cube_ground),
        upper_cone(Parameter.upper_cone,Target.None,"upper_cone",Arm_OutPIDController.cone_upper),
        upper_cube(Parameter.upper_cube,Target.None,"upper_cube",Arm_OutPIDController.cube_upper),
        middle_cone(Parameter.middle_cone,Target.lower_tape,"middle_cone",Arm_OutPIDController.cone_middle),
        middle_cube(Parameter.middle_cube,Target.cube_grid,"middle_cube",Arm_OutPIDController.cube_middle),
        lower_cone(Parameter.lower_cone,Target.lower_tape,"lower_cone", Arm_OutPIDController.cone_lower),
        lower_cube(Parameter.lower_cube,Target.cube_grid,"lower_cube", Arm_OutPIDController.cube_lower),
        primitive_cone(Parameter.primitive_cone,Target.None,"primitive_cone", Arm_OutPIDController.primitive),
        primitive_cube(Parameter.primitive_cube,Target.None,"primitive_cube",Arm_OutPIDController.primitive);
        public final String name;
        public final Parameter parameter;
        public final Target target;
        public final PIDController outPIDController;

        Mode(Parameter _Parameter, Target _target, String _name, PIDController _outPIDController){
        this.parameter = _Parameter;
        this.target = _target; 
        this.name = _name;
        this.outPIDController = _outPIDController;
        }
    }
}
