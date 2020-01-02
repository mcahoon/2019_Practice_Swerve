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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PracticeSwModule {

    private TalonSRX driveMotor;
    private TalonSRX steerMotor;
		
	int _targetPosition = 0;
	int _targetSpeed = 0;

	/* The CTRE Mag Encoder is 4096 units per rotation    */
	/* The NEO built in encoder is 46 units per revolution */
	/* Absolute analog encoder resolution is 1024 units per revolution */
	int _steerSensorRange = 1024;
	int _speedSensorRange = 4096;
	private int ZeroOffset = 0;		// Module offset in encoder units.  Positive is CW. 

	public PracticeSwModule(int steerMotorAdd, int driveMotorAdd, int zeroOffset) 
	{
		driveMotor = new TalonSRX(driveMotorAdd);
		steerMotor = new TalonSRX(steerMotorAdd);
        ZeroOffset = zeroOffset;
        
        steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        steerMotor.setSensorPhase(false);
        
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        driveMotor.setSensorPhase(false);

		/* set steerMotor closed loop gains in slot0 */
		steerMotor.config_kP(0, 16.0f);
		steerMotor.config_kI(0, 0.00013f);
		steerMotor.config_kD(0, 0f);
        steerMotor.config_kF(0, 0f); /* For position servo kF is rarely used. Leave zero */
        
        /* set driveMotor closed loop gains in slot0  */
                  // For MiniCIM
        driveMotor.config_kP(0, 0.05f); /* tweak this first, a little bit of overshoot is okay was 0.02f  */
        driveMotor.config_kI(0, 0.0001f);  // was 0.0001f
        driveMotor.config_kD(0, 1.0f);    // was 0.0f
        driveMotor.config_kF(0, 0.030f); /* For position servo kF is rarely used. Leave zero  0.000f */
      
        /* use slot0 for closed-looping */
        steerMotor.selectProfileSlot(0, 0);
        driveMotor.selectProfileSlot(0, 0);
   
	/*set target to the current position */
		ResetTargetPosition();

	}
// Methods	
	
	public void SetTargetVector(double commandedAngle, double commandedSpeed)
	{
        SetTargetAngle(commandedAngle);
        SetTargetSpeed(commandedSpeed);

	}

	public void SetTargetAngle(double commandedAngle)
	{
		// translates commandedAngle (-1, 1) to (0, +sensorRange) so that 0 angle is midrange
		int targetAngle = (int)(((_steerSensorRange / 2) * commandedAngle) + (_steerSensorRange / 2));
		// add zero offset value
		targetAngle += ZeroOffset;
		// Calculates the minimum change in direction/wheel rotation
		_targetPosition = servo(targetAngle, steerMotor.getSelectedSensorPosition(0), _steerSensorRange);
		// replaces EnableSteerClosedLoop();
		/* Make sure we're in closed-loop mode and update the target position */
		steerMotor.set(ControlMode.Position, _targetPosition);
	}

	int servo(int targetAngPosition, int currentPosition, int sensorRange)
	{
		int targetPosition;

		/*Calculate where in the rotation you are */
		int currentAngPos = currentPosition % sensorRange;

		/*Calculate the distance needed to travel CW to the target */
		int upDistance = targetAngPosition - currentAngPos;

		/*If the target is a lesser sensor value than your current position,
		 * the forward distance will come out negative so you have
		 * to increment by a rotation.  */
		if (targetAngPosition < currentAngPos)
			upDistance += sensorRange;

		/*calculate the distance back to the target*/
		int downDistance = upDistance - sensorRange;    //Distance to rotate CCW direction

		/*use whichever distance is less, then add it to your
		 * current position to get your new target */
        
		if (Math.abs(upDistance) < Math.abs(downDistance))
			targetPosition = currentAngPos + upDistance;
		else
			targetPosition = currentAngPos + downDistance;
// Determine if reduced steering change can be done with reversing drive motor rotation and reversing the steering angle
		int delta = currentAngPos - targetPosition;

		if (Math.abs(delta) > (256))
		{
			if (delta > (256))
				targetPosition += (512);
			else if (delta < -(256))
				{
					targetPosition -= (512);
				}

				driveMotor.setInverted(true);
			
		}
		else
		{
			driveMotor.setInverted(false);
        }  
		
		targetPosition += currentPosition - currentAngPos;
		return targetPosition;
	}

	public void SetTargetSpeed(double commandedSpeed)
	{
    
        _targetSpeed = (int)(_speedSensorRange * commandedSpeed * 7); // originally 10.  For 775Pro use 15
                                                                      // For MiniCIM with use 7
        
        EnableSpeedClosedLoop();
        SmartDashboard.putNumber("SetPoint", _targetSpeed);
       SmartDashboard.putNumber("ProcessVariable", driveMotor.getSelectedSensorVelocity());

	}
    
    public void EnableSpeedClosedLoop()
	{
		/* Make sure we're in closed-loop mode and update the target speed */
        driveMotor.set(ControlMode.Velocity, _targetSpeed);
		
    }
    
	public void DisableSpeedClosedLoop()
	{
        driveMotor.set(ControlMode.Disabled, 0);
		
	}

	void ResetTargetPosition()
        {
            _targetPosition = steerMotor.getSelectedSensorPosition(0);
            steerMotor.setSelectedSensorPosition(_targetPosition, 0, 0);
            //Sets current and desired positions to be equal so we don't move unexpectedly at startup
        }

}
