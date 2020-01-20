/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveManual extends CommandBase {
  /**
   * Creates a new DriveManual.
   */
  public static double OutputOldR;
  public static double OutputOldL;
  public static double driveWeight = 0.85;
  public static double turnWeight = 0.85;
  private final DoubleSupplier rTrigger;
  private final DoubleSupplier lTrigger;
  private final DoubleSupplier leftX;
  private final DoubleSupplier rightX;
  private final BooleanSupplier xblb;
  private final Drivetrain drivetrain;
  public DriveManual(BooleanSupplier m_xblb, DoubleSupplier m_rTrigger, DoubleSupplier m_lTrigger, DoubleSupplier m_leftX, DoubleSupplier m_rightX, Drivetrain m_drivetrain) {
	// Use addRequirements() here to declare subsystem dependencies.
	xblb = m_xblb;
	drivetrain = m_drivetrain;
	rTrigger = m_rTrigger;
	lTrigger = m_lTrigger;
	leftX = m_leftX;
	rightX = m_rightX;
	addRequirements(drivetrain);
	
  }

	//A method to limit an input double to the range -1.0 to 1.0
	public double limit(double prelimNumber){
		if(prelimNumber >= 1.0){
			return 1.0;
					
		}else if(prelimNumber <= -1.0){
			
			return -1.0;
		}else if(prelimNumber < 1.0 && prelimNumber >-1.0){
			
			return prelimNumber;
		}else{
			
			return 0;
		}
		
	}
	double GetPositionFilteredL(double RawValueReadFromHw){
		  double FilteredPosition = 0.09516*RawValueReadFromHw+0.9048*OutputOldL;
		  OutputOldL = FilteredPosition;
		  return FilteredPosition;
	} 
	double GetPositionFilteredR(double RawValueReadFromHw){
		  double FilteredPosition = 0.09516*RawValueReadFromHw+0.9048*OutputOldR;
		  OutputOldR = FilteredPosition;
		  return FilteredPosition;
	} 
	
	//get xAxis value of Xbox joystick; argument is stick side
	public double getStickHorizontal(char side){
		if(side == 'r'){
			return limit(rightX.getAsDouble());
		
		}else if(side == 'l'){
			return limit(leftX.getAsDouble());
			
		}else{
			return 0;
		}
	}
	//get Trigger values; arguement is trigger side
	public double getTriggerValue(char side){
		if(side == 'r'){
			
			return rTrigger.getAsDouble();
		
		}else if(side == 'l'){
			return lTrigger.getAsDouble();
			
		}else{
			return 0;
			
		}
	}
	//Calculates right speed based on controller output
		public double XBControllerR(double lStick, double rTrigger, double lTrigger) {
			//speed of left side = amount Accelerator is pushed down minus
			//amount Deccelerator is pushed down - lateral input from left Joystick
			// if(rTrigger >= lTrigger){
			// }
			// return driveWeight * limit(rTrigger - lTrigger + lStick);
			return driveWeight * limit(rTrigger - lTrigger - lStick);

		}
		
		//Calculates left speed based on Controller output
		public double XBControllerL(double lStick, double rTrigger, double lTrigger){
			//speed of left side = amount Accelerator is pushed down minus
			//amount Deccelerator is pushed down + lateral input from left Joystick
			// if(rTrigger >= lTrigger){
			// }
			// return driveWeight * limit(rTrigger - lTrigger - lStick);
			return driveWeight * limit(rTrigger - lTrigger + lStick);
			
		}
		//Sets the speed for both sides using XBController methods
		public void setSpeeds(double lStick, double rTrigger, double lTrigger){
			
			if (Math.abs(OutputOldR - XBControllerR(lStick, rTrigger, lTrigger)) > .2) 
				drivetrain.setPowerRight((double)XBControllerR(lStick, rTrigger, lTrigger));
			else {
				drivetrain.setPowerRight((double)XBControllerR(lStick, rTrigger, lTrigger));
			}
				if (Math.abs(OutputOldL - XBControllerL(lStick, rTrigger, lTrigger)) > .2) {
				drivetrain.setPowerLeft((double)XBControllerL(lStick, rTrigger, lTrigger));
			}
			else {
				drivetrain.setPowerLeft((double)XBControllerL(lStick, rTrigger, lTrigger));
			}		
		}
		
	//Whenever this command is called, setspeeds is called
	public void execute() {
		if(xblb.getAsBoolean()){
			driveWeight = 0.25;
			turnWeight = 0.25;
		} else {
			driveWeight = 0.9;
			turnWeight = 0.9;
		}
		setSpeeds(getStickHorizontal('l'), getTriggerValue('r'), getTriggerValue('l'));
		drivetrain.updatePose();
	}

  // Called when the command is initially scheduled.
	@Override
  public void initialize() {
	  //drivetrain.setStartpose(drivetrain.getPigeonCompassHeading());
	  drivetrain.initDrive();
	  //drivetrain.resetSensors();
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
