/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmControl {

    private TalonSRX _talon;

    int _targetPosition = 0;
    
    int _armSensorRange = 4096;
    
    /* The CTRE Mag Encoder is 4096 units per rotation    */
    /* Analog encoder resolution is 1024 units per revolution */
    /* Encoder turns at motor speed 4096 * 250  (50:1 for gear box, 5:1 for chain reduction) */

    static int kTimeoutMs = 30;
    
    public ArmControl(int talonAdd)
    {
       _talon = new TalonSRX(talonAdd);

        /* first choose the sensor */
        
        _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        _talon.setSensorPhase(false);

        /* set closed loop gains in slot0 */
        _talon.config_kP(0, 0.1f); /* tweak this first, a little bit of overshoot is okay */
        _talon.config_kI(0, 0.0f);
        _talon.config_kD(0, 0.2f);
        _talon.config_kF(0, 0f); /* For position servo kF is rarely used. Leave zero */

        /* use slot0 for closed-looping */
        _talon.selectProfileSlot(0, 0);

        /* set the peak and nominal outputs, 1.0 means full */
        _talon.configNominalOutputForward(0.0f, kTimeoutMs);
        _talon.configNominalOutputReverse(0.0f, kTimeoutMs);
        _talon.configPeakOutputForward(+0.8f, kTimeoutMs);  // was 1.0f	
        _talon.configPeakOutputReverse(-0.5f, kTimeoutMs);  // was 1.0f

        /* how much error is allowed?  This defaults to 0. */
        _talon.configAllowableClosedloopError(0, 15, kTimeoutMs);

        /* put in a ramp to prevent the user from flipping their mechanism in open loop mode */
        _talon.configClosedloopRamp(0, kTimeoutMs);
        _talon.configOpenloopRamp(1, kTimeoutMs);

        /*set target to the current position */
        ResetTargetPosition();
    }

    public void ZeroSensor()
    {
        _talon.setSelectedSensorPosition(357680, 0, kTimeoutMs); /* 357680 was 0.  set start position at zero, using relative positions */
    }

    public void ResetTargetPosition()
    {
        // _targetPosition = _talon.GetSelectedSensorPosition(0);
        _talon.setSelectedSensorPosition(357680, 0, 0); //Was _targetPosition  Sets current and desired positions to be equal so we don't move unexpectedly at startup

        //Thread.sleep(100);  //wait to make sure SetPosition takes effect
    
    }

    public void GetArmPos()
    {
        _targetPosition = _talon.getSelectedSensorPosition(0);

    }

    public void SetTargetPos(int targetPos)
    {
        _targetPosition = targetPos;
        EnableClosedLoop();
    }

        void EnableClosedLoop()
    {
        /* Make sure we're in closed-loop mode and update the target position */
        _talon.set(ControlMode.Position, _targetPosition);
    }
}