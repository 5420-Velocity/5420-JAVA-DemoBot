/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.robot.helpers.*;
import frc.robot.helpers.RobotOrientation.Side;
import frc.robot.helpers.console.logMode;
import frc.robot.helpers.controllers.*;
import frc.robot.commands.*;


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  private static int logInterval = 0;

  /**
   * All Input and Output Devices below should be in the
   *  Excel Sheet in the link below.
   *
   * @link https://docs.google.com/spreadsheets/d/1qpUWBg1E4hRL2MkAI9xdoiQkqGg9Q8PkZpfA0f9DALU/edit
   */

  public static DifferentialDrive m_drive;
  public static WPI_TalonSRX left1, left2, left3; // Left Side Motors
  public static WPI_TalonSRX right1, right2, right3; // Right Side Motors
  public static WPI_TalonSRX test;
  public static DoubleSolenoid transSol; // Put Solenoid to the Close State

  public static DigitalInput ballLoaded, ballUpperLimit, ballLowerLimit;
  public static Solenoid liftSol; // Put Solenoid to the Open State
  public static Encoder leftEncoder, rightEncoder;
  public static VictorSP ballIntake, wiffleAim, wiffleShoot2, wiffleShoot;

  public static Compressor compressor;
  public static CommandGroup autoCommand;

  public static char[] gameData;

  // Used to Store the Table Entries for the Network Table.
  //  Stored as an Associative Array to the NetowrkTable Entry
  // Map<String, NetworkTableEntry> tableIndex = new HashMap<String, NetworkTableEntry>();

  public static NetworkTableEntryStore tableIndex;

  public static int DRIVERR = 0;
  public static int DRIVERL = 1;
  public static int OPERATOR = 2;
  public static int CTRL_LOG_INTERVAL = 40*4;

  public static boolean liftOnce = false;

  @Override
  public void robotInit() {
    // Init User Input Controls
    new OI();

    // Generate new Save with USB as Default, Save when Disabled
    // Save.defaultMode = // Save.Mode.kUSBFirst;
    // Save.getInstance().ignoreInDisabled = false; // Log Everything

    // Save.getInstance().writeComment("Robot Log Started.");

    // LEFT SIDE Control
    Robot.left1 = new WPI_TalonSRX(52);

    Robot.left2 = new WPI_TalonSRX(54);
    Robot.left2.follow(Robot.left1);

    Robot.left3 = new WPI_TalonSRX(53);
    Robot.left3.follow(Robot.left1);

    // Right SIDE Control
    Robot.right1 = new WPI_TalonSRX(56);

    Robot.right2 = new WPI_TalonSRX(57);
    Robot.right2.follow(Robot.right1);

    Robot.right3 = new WPI_TalonSRX(55);
    Robot.right3.follow(Robot.right1);

    // Build a full Differental Drive
    Robot.m_drive = new DifferentialDrive(Robot.left1, Robot.right1);

   

    leftEncoder = new Encoder(4, 5);
    rightEncoder = new Encoder(6, 7);


    transSol = new DoubleSolenoid(4, 5);
    liftSol = new Solenoid (2);

    ballLoaded = new DigitalInput(9);
    ballUpperLimit = new DigitalInput(10);
    ballLowerLimit = new DigitalInput(11);

    wiffleShoot2 = new VictorSP(5);
    wiffleAim = new VictorSP(3);
    wiffleShoot = new VictorSP(2);
    ballIntake = new VictorSP(4);

    compressor = new Compressor(0);

    SmartDashboard.setDefaultNumber("Test", 0);
    SmartDashboard.setDefaultNumber("AutoDelay", 0);
    SmartDashboard.setDefaultBoolean("Sol", false);

    m_drive.setExpiration(0.2); // Default is 0.1
    //m_drive.setSafetyEnabled(true); // Disable Watchdog auto stop

    // Save.getInstance().push("Test", false);

    console.allowLog = logMode.kOff;

    // Setup Auto CTRL
    OI.Apply();
  }

  @Override
  public void robotPeriodic() {

    // Save.getInstance().push("t", System.currentTimeMillis());
    // Save.getInstance().push("batt", RobotController.getBatteryVoltage());

    Robot.logInterval++;
    if(logInterval % CTRL_LOG_INTERVAL == 0){
      Logger.pushCtrlValues("Driver", OI.driverR);
      Logger.pushCtrlValues("Operator", OI.operator);

      Robot.logInterval = 0;
    }
    

    // Push Sensor Values
    OI.leftEncoder.setNumber(Robot.leftEncoder.get());
    OI.rightEncoder.setNumber(Robot.rightEncoder.get());
    OI.ballSwitch.setBoolean(Robot.ballLoaded.get());
    OI.ballUppwerLimit.setBoolean(ballUpperLimit.get());
    OI.ballLowerLimit.setBoolean(ballLowerLimit.get());

    //OI.boschEncoder.setNumber(Robot.bTest.encoderGet());

    if(OI.reset.getBoolean(false)){
      OI.reset.setBoolean(false);
    }

    if(OI.resetEncoder.getBoolean(false)){
      Robot.leftEncoder.reset();
      Robot.rightEncoder.reset();
      OI.resetEncoder.setBoolean(false);
    }

    if(OI.debugLogEnabled.getBoolean(false) == false){
      console.allowLog = logMode.kOff;
    }
    else {
      console.allowLog = logMode.kAll;
    }

    // Save.getInstance().sync();
  }

  @Override
  public void disabledInit(){

  }

  @Override
  public void disabledPeriodic(){

  }

  @Override
  public void testInit() {
    // Enable Compressor Control
    compressor.setClosedLoopControl(true);

    // Drive the Robot Forward for a Set Point of 10,000 Ticks using the Right Encoder
    console.out(logMode.kDebug, "Testing Init Start");

    // Drive the Robot using an Encoder to a set point.
    Scheduler.getInstance().add(new AutoDriveEncoder(Robot.m_drive, Robot.leftEncoder, 0.5, 0, 10000));

    // Using the Given Drive, Turn to the Object until the value is within a margin.
    //Scheduler.getInstance().add(new Limelight_turn(Robot.m_drive, Robot.limelightMain));

    // Drive and follow the object detected in the LL program.
    //Scheduler.getInstance().add(new LimelightFollow(Robot.m_drive, Robot.limelightMain, 0));

    console.out(logMode.kDebug, "Testing Init Finish");
  }

  @Override
  public void testPeriodic() {

    Scheduler.getInstance().run();

  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {

    Scheduler.getInstance().run();

    if(Robot.autoCommand.isCompleted()){
      this.teleopPeriodic(); // Run Robot Telelop Code
    }

  }

  @Override
  public void teleopInit() {

    // Stop Auto Commands

    if(Robot.autoCommand != null){
      Robot.autoCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {

    Scheduler.getInstance().run();

    if(OI.driverR.getRawButton(LogitechMap_X.BUTTON_A)){
      // Add Code to run the robot.
      //Scheduler.getInstance().add();

    }
    else {
      // Add code to Cancel the Program
    }

    /////////////////
    /// SIDE CTRL ///
    /////////////////
    // Update the Side Value, Swapping Sides
    if(OI.directionSwitch.get()){
      RobotOrientation.getInstance().flipSide();
    }
    
    //////////////////
    //  BALL CTRL   //
    //////////////////
    // Ball Intake Control using Buttons
    if(OI.operator.getRawAxis(LogitechMap_X.AXIS_RT) > 0.2){
      // Ball In
      if(Robot.ballLoaded.get() == false){
        ballIntake.set(-0.5);
      }
      
      else {
        console.log("Ball Loaded, Auto Stop");
        // Ball Off
        ballIntake.set(0);
      }
    }
    else if(OI.operator.getRawButton(LogitechMap_X.BUTTON_RB)){
      // Ball Out
      ballIntake.set(0.8);
    }
    else {
      // Ball Off
      ballIntake.set(0);
    }

    //////////////////
    //  WIFFLE CTRL //
    //////////////////

    //Shoot
    if(OI.operator.getRawButton(LogitechMap_X.BUTTON_LB)){
      wiffleShoot.set(0.87);
      wiffleShoot2.set(-0.87);
    }
    else{
      wiffleShoot.set(0);
      wiffleShoot2.set(0);
    }

    //Aim
    if(OI.operator.getRawAxis(LogitechMap_X.AXIS_LEFT_Y) >= 0.2){
      wiffleAim.set(0.5);
    }

    else if(OI.operator.getRawAxis(LogitechMap_X.AXIS_LEFT_Y) <= -0.2){
      wiffleAim.set(-0.5);
    }
    else{
      wiffleAim.set(0);
    }

    ///////////////////
    ///  Ball lift  ///
    ///////////////////

    if(OI.operator.getRawAxis(LogitechMap_X.AXIS_RIGHT_Y) < 0.8){
      liftSol.set(false);
    }
    else{
      liftSol.set(true);
    }

    //////////////////
    //  SHIFT CTRL  //
    //////////////////
    if(OI.transButtonHigh.get()){
      transSol.set(Value.kForward); // High Gear
      OI.driveShift.setString("HIGH");
    }
    else if(OI.transButtonLow.get()){
      transSol.set(Value.kReverse); // LowGear
      OI.driveShift.setString("LOW");
    }
    
    //////////////////
    //  DRIVE CTRL  //
    //////////////////
    double DRIVE_Y = (OI.driverR.getRawAxis(Aviator.AXIS_Y));
    double DRIVE_X = (-OI.driverL.getRawAxis(Aviator.AXIS_Y));
    DRIVE_Y = RobotOrientation.getInstance().fix(DRIVE_Y, Side.kSideA);
    DRIVE_X = RobotOrientation.getInstance().fix(DRIVE_X, Side.kSideB);

    if(OI.driveSlowForward.get()){
      // Button Mode Forward
      console.out(logMode.kDebug, "Slow Forward");
      DRIVE_Y = 0.6;
      DRIVE_X = 0;
    }
    else if(OI.driveSlowReverse.get()){
      // Button Mode Reverse
      console.out(logMode.kDebug, "Slow Reverse");
      DRIVE_Y = -0.6;
      DRIVE_X = 0;
    }
    else if(OI.driveSlowLeft.get()){
      // Button Mode Reverse
      console.out(logMode.kDebug, "Slow Left");
      DRIVE_Y = 0;
      DRIVE_X = 0.6;
    }
    else if(OI.driveSlowRight.get()){
      // Button Mode Reverse
      console.out(logMode.kDebug, "Slow Right");
      DRIVE_Y = 0;
      DRIVE_X = -0.6;
    }
    else {
      // Joystick Mode
      DRIVE_Y = DRIVE_Y*0.90;
      DRIVE_X = DRIVE_X*0.90;
      m_drive.feed();
    }

    ////////////////////////
    //////// SIDE A ////////
    ////////////////////////
    if(RobotOrientation.getInstance().getSide() == Side.kSideA) {

      OI.cameraView.setNumber(1);
      OI.cameraViewText.setString("HATCH");
    }
    ////////////////////////
    //////// SIDE B ////////
    ////////////////////////
    else {

      OI.cameraView.setNumber(0);
      OI.cameraViewText.setString("BALL");
    }
    Robot.m_drive.tankDrive( DRIVE_Y, DRIVE_X );
  }
  

  /**
   * Is Value between the given input.
   * 
   * @param Input
   * @param trueZone
   * @return
   */
  public boolean between(double Input, double trueZone){
    return (Math.abs(Input) < trueZone);
  }

}
