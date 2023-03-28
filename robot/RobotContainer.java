// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.AutoCommandV1;
import frc.robot.Commands.AutoCommandV3;
import frc.robot.Commands.BaseCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.BaseSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import static frc.robot.Constants.Mode;

public class RobotContainer {
  

  //Subsystem
  private final BaseSubsystem baseSubsystem = new BaseSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  //Joystick
  private final Joystick drive_stick = new Joystick(0);
  private final Joystick control_panel = new Joystick(1);
  
  //drive Button
  private final JoystickButton balance_button = new JoystickButton(drive_stick, 3);
  private final JoystickButton aim_button = new JoystickButton(drive_stick, 1);

  //control button
  private final JoystickButton gamePiece_button = new JoystickButton(control_panel,4);
  private final JoystickButton upper_button = new JoystickButton(control_panel,1);
  private final JoystickButton middle_button = new JoystickButton(control_panel,2);
  private final JoystickButton lower_button = new JoystickButton(control_panel,3);
  private final JoystickButton doubleSubstation_button = new JoystickButton(control_panel,8);
  private final JoystickButton singleSubstation_button = new JoystickButton(control_panel,7);
  private final JoystickButton ground_button = new JoystickButton(control_panel,9);
  private final JoystickButton primitive_button = new JoystickButton(control_panel, 10);

  //Command
  private final BaseCommand baseCommand = new BaseCommand(baseSubsystem,limelightSubsystem,armSubsystem, drive_stick, aim_button, balance_button);
  private final IntakeCommand intakeCommnad = new IntakeCommand(armSubsystem, control_panel);

  // grid cone
  private final ArmCommand upper_cone_ArmCommand = new ArmCommand(armSubsystem, Mode.upper_cone);
  private final ArmCommand middle_cone_ArmCommand = new ArmCommand(armSubsystem, Mode.middle_cone);
  private final ArmCommand lower_cone_ArmCommand = new ArmCommand(armSubsystem, Mode.lower_cone);

  //grid cube
  private final ArmCommand upper_cube_ArmCommand = new ArmCommand(armSubsystem, Mode.upper_cube);
  private final ArmCommand middle_cube_ArmCommand = new ArmCommand(armSubsystem, Mode.middle_cube);
  private final ArmCommand lower_cube_ArmCommand = new ArmCommand(armSubsystem, Mode.lower_cube);

  //other cube
  private final ArmCommand double_cube_ArmCommand = new ArmCommand(armSubsystem, Mode.double_subsystem_cube);
  private final ArmCommand ground_cube_ArmCommand = new ArmCommand(armSubsystem, Mode.ground_cube);

  //other cone
  private final ArmCommand double_cone_ArmCommand = new ArmCommand(armSubsystem, Mode.double_subsystem_cone);
  private final ArmCommand ground_cone_ArmCommand = new ArmCommand(armSubsystem, Mode.ground_cone);

  //
  private final ArmCommand single_cone_ArmCommand = new ArmCommand(armSubsystem, Mode.single_subsystem_cone);
  private final ArmCommand single_cube_ArmCommand = new ArmCommand(armSubsystem, Mode.single_subsystem_cube);
  
  // primitive Commnad 
  public final ArmCommand primitiveCone_ArmCommand = new ArmCommand(armSubsystem, Mode.primitive_cone);
  public final ArmCommand primitiveCube_ArmCommand = new ArmCommand(armSubsystem, Mode.primitive_cube);


  // auto Command
  private final AutoCommandV1  autoCommandV1 = new AutoCommandV1(baseSubsystem);
  private final AutoCommandV3 autoCommandV3 = new AutoCommandV3(armSubsystem,baseSubsystem, limelightSubsystem,upper_cone_ArmCommand, ground_cone_ArmCommand,middle_cone_ArmCommand ,primitiveCone_ArmCommand);
  // private final ArmCommand inCommand = upper_cone_ArmCommand;
  // private final ArmCommand outCommand = upper_cone_ArmCommand;
  // private final AutoCommandV2 autoCommandV2 = new AutoCommandV2(baseSubsystem, limelightSubsystem, inCommand, outCommand);


  public static Mode now_mode = Mode.primitive_cone;

  public void set_light(){

    limelightSubsystem.set_light(gamePiece_button.getAsBoolean());
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command get_baseCommand(){
    return baseCommand;
  }

  public Command get_autoCommand(){
    return autoCommandV3;
  }

  public Command get_intakeCommand(){
    return intakeCommnad;
  }

  public boolean get_gamePiece(){
    return (gamePiece_button.getAsBoolean());
  }

  public void set_distance(){
    limelightSubsystem.set_distance(now_mode);
    limelightSubsystem.set_pipe(now_mode);
    limelightSubsystem.set_goalDistance(now_mode);
  }


  private void configureBindings() {
    upper_button.and(gamePiece_button).onTrue(upper_cone_ArmCommand);
    upper_button.and(gamePiece_button.negate()).onTrue(upper_cube_ArmCommand);
    middle_button.and(gamePiece_button).onTrue(middle_cone_ArmCommand);
    middle_button.and(gamePiece_button.negate()).onTrue(middle_cube_ArmCommand);
    lower_button.and(gamePiece_button).onTrue(lower_cone_ArmCommand);
    lower_button.and(gamePiece_button.negate()).onTrue(lower_cube_ArmCommand);
    doubleSubstation_button.and(gamePiece_button).onTrue(double_cone_ArmCommand);
    doubleSubstation_button.and(gamePiece_button.negate()).onTrue(double_cube_ArmCommand);
    ground_button.and(gamePiece_button).onTrue(ground_cone_ArmCommand);
    ground_button.and(gamePiece_button.negate()).onTrue(ground_cube_ArmCommand);
    primitive_button.and(gamePiece_button).onTrue(primitiveCone_ArmCommand);
    primitive_button.and(gamePiece_button.negate()).onTrue(primitiveCube_ArmCommand);
    singleSubstation_button.and(gamePiece_button).onTrue(single_cone_ArmCommand);
    singleSubstation_button.and(gamePiece_button.negate()).onTrue(single_cube_ArmCommand);
  }
}
