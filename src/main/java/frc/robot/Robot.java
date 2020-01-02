/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.kauailabs.navx.frc.AHRS;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);
  private AHRS mNavX = new AHRS(I2C.Port.kOnboard);
  
  // Declare a new talon with CAN ID 11 for manipulator wheel motor control 
  TalonSRX manipTalon = new TalonSRX(11);

  Joystick drive = new Joystick(0);
  Joystick spin = new Joystick(1);

  static int lfSteerAdd = 1;
  static int lfDriveAdd = 2;
  static int lbSteerAdd = 3;
  static int lbDriveAdd = 4;
  static int rbSteerAdd = 5;
  static int rbDriveAdd = 6;
  static int rfSteerAdd = 7;
  static int rfDriveAdd = 8;
  static int manipTalonAdd = 11;
  static int armControlAdd = 12;

  static double l_r = 0.6425d;  // L/R 
  static double w_r = 0.7663d;  // W/R 
  static double pi_inv = 0.3183d;  // 1/pi
  static double degToRad = 0.01745d;  // 2 pi / 360

  int zeroOffsetLF = -10;
  int zeroOffsetLB = 10;
  int zeroOffsetRB = -2;
  int zeroOffsetRF = 2;
  
  boolean motorEnable;
  boolean fieldOriented;

  boolean button1;
  boolean button2;
  boolean button3;
  boolean button4;
  boolean button5;
  boolean button6;
  
  boolean _lastButton1 = false;
  boolean _lastButton2 = false;
  boolean _lastButton3 = false;
  boolean _lastButton4 = false;
  boolean _lastButton5 = false;
  boolean _lastButton6 = false;

  public static PracticeSwModule LFModule; 
  public static PracticeSwModule LBModule;
  public static PracticeSwModule RBModule;
  public static PracticeSwModule RFModule;

  public static ArmControl armControl;


  double str = 0;   //strafe
  double fwd = 0;   // forward
  double rcw = 0;   // rotate clockwise
  double padPos = -1;   // D Pad value
  double prevPadPos = -1;

  double Yaw;
  double yawZero;
 
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  // Instantiate 4 PracticeSwModule objects
   LFModule = new PracticeSwModule(lfSteerAdd, lfDriveAdd, zeroOffsetLF); 
   LBModule = new PracticeSwModule(lbSteerAdd, lbDriveAdd, zeroOffsetLB);
   RBModule = new PracticeSwModule(rbSteerAdd, rbDriveAdd, zeroOffsetRB);
   RFModule = new PracticeSwModule(rfSteerAdd, rfDriveAdd, zeroOffsetRF);
 
   armControl = new ArmControl(armControlAdd);

   fieldOriented = true;  // Normally leave at true.  Normal play mode will be field oriented
   
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /*
  *  This function is called prior to the first execution of teleopPeriodic
  */
  @Override
  public void teleopInit() {
    fieldOriented = true;
    armControl.SetTargetPos(357680);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
        
    Yaw = mNavX.getAngle() - yawZero;
    Yaw %= 360;
    if (Yaw < 0) Yaw += 360;

    // returns the angle of the POV -1 is center (unpressed), 0 is up, 90 is right, 180 is down, 270 is left
    prevPadPos = padPos;
    padPos  = drive.getPOV();   
    //* System.out.println("POV: " + padPos); 

    /* Save button state for on press detect */
    _lastButton1 = button1;
    _lastButton2 = button2;
    _lastButton3 = button3;
    _lastButton4 = button4;
    _lastButton5 = button5;
    _lastButton6 = button6;
    
    // read the buttons
    button1 = drive.getRawButton(1);	// Trigger
    button2 = drive.getRawButton(2);	// Back lower middle
    button3 = drive.getRawButton(3);  // Back lower left
    button4 = drive.getRawButton(4);  // Back lower right
    button5 = drive.getRawButton(5);  // Back  upper left
    button6 = drive.getRawButton(6);  // Back upper right
    
    //
     if (! _lastButton5 && button5) 
    {   yawZero = mNavX.getAngle();
        yawZero %= 360;
        if(yawZero < 360) yawZero += 360;
    }

    if (! _lastButton4 && button4) fieldOriented = true;
    if (! _lastButton3 && button3) fieldOriented = false; 
        
    // Get joystick values
    str = drive.getRawAxis(0); 
    fwd = - drive.getRawAxis(1);  // invert the y axis
    //rcw = drive.getRawAxis(2);
    rcw = spin.getRawAxis(2);

   // Deadband joystick values
    if (Math.abs(str) < 0.03) str = 0;
    if (Math.abs(fwd) < 0.03) fwd = 0;
    if (Math.abs(rcw) < 0.01) rcw = 0;
  
    if ((str == 0) && (fwd == 0) && (rcw == 0))  // if both joysticks have returned to center
    {
        // just set the wheel speed to 0 without changing the wheel angles
        RFModule.SetTargetSpeed(0);
        LFModule.SetTargetSpeed(0);
        LBModule.SetTargetSpeed(0);
        RBModule.SetTargetSpeed(0);

        // Disable closed looop to clear out integral error.  Allow coasting to a stop
        RFModule.DisableSpeedClosedLoop();
        LFModule.DisableSpeedClosedLoop();
        LBModule.DisableSpeedClosedLoop();
        RBModule.DisableSpeedClosedLoop();

        // if we want to stop more quickly, set the speed back to 0 with closed loop but with no residual error term
        //speedRF.SetTargetSpeed(0);
        //speedLF.SetTargetSpeed(0);
        //speedLB.SetTargetSpeed(0);
        //speedRB.SetTargetSpeed(0);

    
        Yaw = mNavX.getAngle() - yawZero;
        Yaw %= 360;
        if (Yaw < 0) Yaw += 360;
        SmartDashboard.putNumber("Yaw:  ", Yaw);
    }
    else
    {
       if (str < -0.03) 
        str = str + 0.01;
       else if (str > 0.03)
        str = str - 0.01;

      if (fwd < -0.03) 
        fwd = fwd + 0.01;
       else if (fwd > 0.03)
        fwd = fwd - 0.01;

      if (rcw < -0.01) 
      rcw = rcw + 0.01;
       else if (rcw > 0.01)
       rcw = rcw - 0.01;  

      // Square the control inputs to make less sensitive to small control changes while preserving max stick value = 1
        str = str * Math.abs(str);
        fwd = fwd * Math.abs(fwd);
        rcw = rcw * rcw * rcw;

        // code to make the control field centric
        Yaw = mNavX.getAngle() - yawZero;
        Yaw %= 360;
        if (Yaw < 0) Yaw += 360;
        SmartDashboard.putNumber("Yaw:  ", Yaw);
        
        if (fieldOriented)
        {
            double pYaw = degToRad * Yaw;
            double _cosYaw = Math.cos(pYaw);
            double _sinYaw = Math.sin(pYaw);
            double temp = fwd * _cosYaw + str * _sinYaw;
            str = -fwd * _sinYaw + str * _cosYaw;
            fwd = temp;
            //System.out.println("pYaw: " + pYaw);
        }
         
        double a = str - (rcw * l_r);
        double b = str + (rcw * l_r);
        double c = fwd - (rcw * w_r);
        double d = fwd + (rcw * w_r);

        double warf = Math.atan2(b, c) * pi_inv;     // returns wheel angle in the range (-1 to 1) rather than (-pi to pi)
        if (b == 0 && c == 0) warf = 0;
        double walf = Math.atan2(b, d) * pi_inv;     // returns wheel angle in the range (-1 to 1) rather than (-pi to pi)
        if (b == 0 && d == 0) walf = 0;
        double walb = Math.atan2(a, d) * pi_inv;     // returns wheel angle in the range (-1 to 1) rather than (-pi to pi)
        if (a == 0 && d == 0) walb = 0;
        double warb = Math.atan2(a, c) * pi_inv;     // returns wheel angle in the range (-1 to 1) rather than (-pi to pi)
        if (a == 0 && c == 0) warb = 0;

        // square the wheel speed to make it less sensitive to small movements of the joysticks to allow modules to turn to the correct 
        // angles before the wheels start to rotate
        double wsrf = Math.sqrt(b * b + c * c);  // wheel speed passed as value between (0 to 1)
        double wslf = Math.sqrt(b * b + d * d);  // wheel speed passed as value between (0 to 1)
        double wslb = Math.sqrt(a * a + d * d);  // wheel speed passed as value between (0 to 1)
        double wsrb = Math.sqrt(a * a + c * c);  // wheel speed passed as value between (0 to 1)

        // normalize the speeds
        double max = wsrf;
        if (wslf > max) max = wslf;
        if (wslb > max) max = wslb;
        if (wsrb > max) max = wsrb;

        if (max > 1)
        {
            wsrf = wsrf / max;
            wslf = wslf / max;
            wslb = wslb / max;
            wsrb = wsrb / max;
        }
        // Set the wheel speeds and angles with closed loop control

        RFModule.SetTargetVector(warf, wsrf);
        LFModule.SetTargetVector(walf, wslf);
        LBModule.SetTargetVector(walb, wslb);
        RBModule.SetTargetVector(warb, wsrb);
    }
    // Code to control arm and manipulator 

    // If button is pressed, determine what value is passed and set new target position

          if ((padPos == 0) && (prevPadPos == -1)) armControl.SetTargetPos(357680);   // Set Carry position was 327680
    else if ((padPos == 90) && (prevPadPos == -1)) armControl.SetTargetPos(224800);   // Set Front Switch score position
    else if ((padPos == 180) && (prevPadPos == -1)) armControl.SetTargetPos(5000);     // Set Pickup or Vault position
    else if ((padPos == 270) && (prevPadPos == -1)) armControl.SetTargetPos(420000);   // Set Rear Switch score position
        
    // Enter code to control manipulator here
    // while the right trigger switch is pressed, run intake at gather speed
    if (button1) manipTalon.set(ControlMode.PercentOutput, -0.75);
    // run intake at hold speeda after gathering a gamepiece
    if(!button1 && _lastButton1) manipTalon.set(ControlMode.PercentOutput, -0.11); 
    // while right bumper switch is pressed, run intake at eject speed 
    if (button2) manipTalon.set(ControlMode.PercentOutput, 0.65); 
    // set intake to off after ejecting gamepiece
    if(!button2 && _lastButton2) manipTalon.set(ControlMode.PercentOutput, 0.0); 

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}