package org.usfirst.frc.team1414.robot;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot implements PIDOutput {
	CANTalon leftMaster, leftSlave, rightMaster, rightSlave, climberMaster, climberSlave;
	Joystick stick, steeringWheel;
	DoubleSolenoid shifter;
	DoubleSolenoid gearMech;
	Compressor compressor;
	RobotDrive robot;
	AHRS ahrs;
	boolean delay = false;
	boolean path1 = false;
	boolean rotation1 = true;
	boolean path2 = true;
	final String mobilityAuto = "Mobility Auto";
	final String frontGearAuto = "Front Gear Auto";
	final String sideGearRightAutoBlue = "Side Gear Right Auto Blue";
	final String sideGearLeftAutoBlue = "Side Gear Left Auto Blue";
	final String sideGearRightAutoRed = "Side Gear Right Auto Red";
	final String sideGearLeftAutoRed = "Side Gear Left Auto Red";
	double straightControllerValue;
	private PIDController turnController;	
	private PIDController straightController;
	String autoSelected;
	boolean isHighGear;
	boolean isNavX;
	boolean distanceRun = false;
	double magnitudeValue;
	double wheel;
	double throttle; 
	double mQuickStopAccumulator;
    
	SendableChooser<String> chooser = new SendableChooser<>();
	
	@Override
	public void robotInit() {
		leftMaster = new CANTalon(2);
		leftSlave = new CANTalon(3);
		rightMaster = new CANTalon(4);
		rightSlave = new CANTalon(5);
		climberMaster = new CANTalon(6);
		climberSlave = new CANTalon(7);
		robot = new RobotDrive (leftMaster, leftSlave, rightMaster, rightSlave);
		stick = new Joystick(0);
		compressor = new Compressor(1);
		steeringWheel = new Joystick(1);
		shifter = new DoubleSolenoid(2,0,1);
		gearMech = new DoubleSolenoid(1,0,1);
		compressor.setClosedLoopControl(true);
		leftMaster.enableBrakeMode(true);
		leftSlave.enableBrakeMode(true);
		rightMaster.enableBrakeMode(true);
		rightSlave.enableBrakeMode(true);
		climberMaster.enableBrakeMode(false);
		climberSlave.enableBrakeMode(false);
		chooser.addDefault("Mobility Auto", mobilityAuto);
		chooser.addObject("Front Gear Auto", frontGearAuto);
		chooser.addObject("Side Gear Right Auto Blue", sideGearRightAutoBlue);
		chooser.addObject("Side Gear Left Auto Blue", sideGearLeftAutoBlue);
		chooser.addObject("Side Gear Right Auto Red", sideGearRightAutoRed);
		chooser.addObject("Side Gear Left Auto Red", sideGearLeftAutoRed);
		SmartDashboard.putData("Auto choices", chooser);
		
		 try {
		    	ahrs = new AHRS(SPI.Port.kMXP);
		    	isNavX = true;
		    }
		    catch (RuntimeException ex) {
		    	DriverStation.reportError("Error Connecting to navX" + ex.getMessage(), true);
		    	isNavX = false; 
		    }
		    
		    turnController = new PIDController(0.04, 0.00003, 0.1, 0.00, ahrs, this);
		    turnController.setOutputRange(-0.5, 0.5);
		    turnController.setAbsoluteTolerance(0.0);
		    turnController.setContinuous(true);
		    
		    straightController = new PIDController(0.03, 0.0, 0.08, 0.00, ahrs, this); // P could also = 0.01
		    straightController.setOutputRange(-1.0, 1.0);
		    straightController.setAbsoluteTolerance(0.0);
		    straightController.setContinuous(true);
	}

	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		SmartDashboard.putString("Selected Auto", autoSelected);
		System.out.println("Auto selected: " + autoSelected);
		
		//AUTO RESET
		rightMaster.setEncPosition(0);
		leftMaster.setEncPosition(0);
		path1 = false;
		rotation1 = false;
		path2 = false;
		setHighGear(false);
		ahrs.zeroYaw();
	}

	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putNumber("Current Angle", ahrs.getAngle());
		SmartDashboard.putNumber("Straight Controller Output", straightController.get());
		SmartDashboard.putNumber("Turn Controller Output", turnController.get());
		SmartDashboard.putNumber("Straight Controller Set Point", straightController.getSetpoint());
		SmartDashboard.putNumber("Turn Controller Output", turnController.getSetpoint());
		SmartDashboard.putNumber("Left Drive Master Set Point", leftMaster.getSetpoint());
		SmartDashboard.putNumber("Left Drive Slave Set Point", leftSlave.getSetpoint());
		SmartDashboard.putNumber("Right Drive Master Set Point", rightMaster.getSetpoint());
		SmartDashboard.putNumber("Right Drive Slave Set Point", rightSlave.getSetpoint());
		
		switch (autoSelected) {
		case mobilityAuto:
			if (delay = false) {
				Timer.delay(0.5);
				delay = true;
			}
			else if (getRightDistance() < 140) {
				straightController.setSetpoint(0);
				straightController.enable();
				driveStraight(0.5, straightController.get());
			}
			else if (getRightDistance() < 192) {
				driveStraight(0.2, straightController.get());
			}
			
			break;
			
		case frontGearAuto:
			if (delay = false) {
				Timer.delay(0.5);
				delay = true;
			}
			else if (getRightDistance() < 140) {
				straightController.setSetpoint(0);
				straightController.enable();
				driveStraight(0.5, straightController.get());
			}
			else if (getRightDistance() < 192) {
				driveStraight(0.2, straightController.get());
			}
			
			break;
					
		case sideGearLeftAutoRed:

			if (path1 == false){
				if(getRightDistance() < 107)
				{
					straightController.setSetpoint(0);
					straightController.enable();
					driveStraight(0.7, straightController.get());
					
				}
				else if(getRightDistance() < 132) {
					driveStraight(0.2, straightController.get());
				} 
				else {
					path1 = true;
					rotation1 = true;
					rightMaster.setEncPosition(0);
					leftMaster.setEncPosition(0);
					straightController.disable();
				}
			}
			
			if (rotation1 == true){
				if (ahrs.getAngle() < 54 || ahrs.getAngle() > 56) {
					turnController.setSetpoint(55);
					turnController.enable();
					rightMaster.set(turnController.get());
					rightSlave.set(turnController.get());
					leftMaster.set(turnController.get());
					leftSlave.set(turnController.get());
				}
				else {
					rotation1 = false;
					path2 = true;
					rightMaster.setEncPosition(0);
					leftMaster.setEncPosition(0);
					turnController.disable();
				}
			}
			
			if (path2 == true) {
				if (getRightDistance() < 130) {
					straightController.setSetpoint(55);
					straightController.enable();
					driveStraight(0.7, straightController.get());
				} else if(getRightDistance() < 180) {
					driveStraight(0.2, straightController.get());
				}
			} 
			
			break;
			
		case sideGearRightAutoRed:
			if (path1 == false) {
				if (getRightDistance() < 105)
				{
					straightController.setSetpoint(0.0);
					straightController.enable();
					driveStraight(0.7, straightController.get());
				}
				else if (getRightDistance() < 132) {
					driveStraight(0.2, straightController.get());
				}
				else {
					path1 = true;
					rotation1 = true;
					rightMaster.setEncPosition(0);
					leftMaster.setEncPosition(0);
					straightController.disable();
				}
			}
			
			if (rotation1 == true) {
				if (ahrs.getAngle() > -55 || ahrs.getAngle() < -57) {
					turnController.setSetpoint(-56.0);
					turnController.enable();
					rightMaster.set(turnController.get());
					rightSlave.set(turnController.get());
					leftMaster.set(turnController.get());
					leftSlave.set(turnController.get());
				}
				else {
					rotation1 = false;
					path2 = true;
					rightMaster.setEncPosition(0);
					leftMaster.setEncPosition(0);
					turnController.disable();
				}
			}
			
			if (path2 == true) {
				if (getRightDistance() < 120) {
					straightController.setSetpoint(-56.0);
					straightController.enable();
					driveStraight(0.7, straightController.get());
				}
				else if (getRightDistance() < 170) {
					driveStraight(0.2, straightController.get());
				}
			} 
			
			break;
			
		case sideGearLeftAutoBlue:
			if (path1 == false) {
				if (getRightDistance() < 105)
				{
					straightController.setSetpoint(0.0);
					straightController.enable();
					driveStraight(0.7, straightController.get());
				}
				else if (getRightDistance() < 132) {
					driveStraight(0.2, straightController.get());
				}
				else {
					path1 = true;
					rotation1 = true;
					rightMaster.setEncPosition(0);
					leftMaster.setEncPosition(0);
					straightController.disable();
				}
			}
			
			if (rotation1 == true) {
				if (ahrs.getAngle() < 55 || ahrs.getAngle() > 57) {
					turnController.setSetpoint(56.0);
					turnController.enable();
					rightMaster.set(turnController.get());
					rightSlave.set(turnController.get());
					leftMaster.set(turnController.get());
					leftSlave.set(turnController.get());
				}
				else {
					rotation1 = false;
					path2 = true;
					rightMaster.setEncPosition(0);
					leftMaster.setEncPosition(0);
					turnController.disable();
				}
			}
			
			if (path2 == true) {
				if (getRightDistance() < 120) {
					straightController.setSetpoint(56.0);
					straightController.enable();
					driveStraight(0.7, straightController.get());
				}
				else if (getRightDistance() < 170) {
					driveStraight(0.2, straightController.get());
				}
			} 
			
			break;
			
		case sideGearRightAutoBlue:

			if (path1 == false){
				if(getRightDistance() < 107)
				{
					straightController.setSetpoint(0);
					straightController.enable();
					driveStraight(0.7, straightController.get());
					
				}
				else if(getRightDistance() < 132) {
					driveStraight(0.2, straightController.get());
				} 
				else {
					path1 = true;
					rotation1 = true;
					rightMaster.setEncPosition(0);
					leftMaster.setEncPosition(0);
					straightController.disable();
				}
			}
			
			if (rotation1 == true){
				if (ahrs.getAngle() > -54 || ahrs.getAngle() < -56) {
					turnController.setSetpoint(-55);
					turnController.enable();
					rightMaster.set(turnController.get());
					rightSlave.set(turnController.get());
					leftMaster.set(turnController.get());
					leftSlave.set(turnController.get());
				}
				else {
					rotation1 = false;
					path2 = true;
					rightMaster.setEncPosition(0);
					leftMaster.setEncPosition(0);
					turnController.disable();
				}
			}
			
			if (path2 == true) {
				if (getRightDistance() < 130) {
					straightController.setSetpoint(-55);
					straightController.enable();
					driveStraight(0.7, straightController.get());
				} else if(getRightDistance() < 180) {
					driveStraight(0.2, straightController.get());
				}
			} 
			
			break;
		
		default:

			if (delay = false) {
				Timer.delay(0.5);
				delay = true;
			}
			else if (getRightDistance() < 140) {
				straightController.setSetpoint(0);
				straightController.enable();
				driveStraight(0.5, straightController.get());
			}
			else if (getRightDistance() < 192) {
				driveStraight(0.2, straightController.get());
			}
			
			break;
			
		}
	}

	@Override
	public void teleopPeriodic() {
		
		SmartDashboard.putNumber("Left Drive Master Set Point", leftMaster.getSetpoint());
		SmartDashboard.putNumber("Left Drive Slave Set Point", leftSlave.getSetpoint());
		SmartDashboard.putNumber("Right Drive Master Set Point", rightMaster.getSetpoint());
		SmartDashboard.putNumber("Right Drive Slave Set Point", rightSlave.getSetpoint());
		
		
		magnitudeValue = steeringWheel.getY() - steeringWheel.getZ();
		
		if (stick.getTrigger()){
			climberMaster.set(1.0);
			climberSlave.set(1.0);
		}
		else{
			
			climberMaster.set(Math.abs(stick.getY()));
			climberSlave.set(Math.abs(stick.getY()));
		}
		
        if (steeringWheel.getRawButton(7)) {
        	if(distanceRun == false){
        		straightController.setSetpoint(ahrs.getAngle());
        		straightController.enable();
        		distanceRun = true;
        	}
        	else {
        		driveStraight(-magnitudeValue, straightController.get());
        	}
        }
        else {
        	straightController.disable();
        	distanceRun = false;
        	robot.drive(-magnitudeValue, steeringWheel.getX());
        }
        
        if (stick.getRawButton(3)){
        	gearMech.set(DoubleSolenoid.Value.kForward);
        }
        
        if(stick.getRawButton(2)){
        	gearMech.set(DoubleSolenoid.Value.kReverse);
        }
        
        if(steeringWheel.getRawButton(5)){
        	setHighGear(true);
        }
        
        if(steeringWheel.getRawButton(6)){
        	setHighGear(false);
        }
	}
	
	public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
	
	public static double limit(double v, double limit) {
        return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
    }
	
	public void setHighGear(boolean on) {
		if (on == true) {
			shifter.set(DoubleSolenoid.Value.kReverse);
			isHighGear = true;
		}
		else if(on == false) {
			shifter.set(DoubleSolenoid.Value.kForward);
			isHighGear = false;
		}
	}
	
	public synchronized double getRightDistance() {
		return rightMaster.getEncPosition() * (Math.PI * 4 /128);
	}

	public synchronized double getLeftDistance() {
		return leftMaster.getEncPosition() * (Math.PI * 4 /128);
	}
	
	public void driveStraight(double motorOutput, double correctionAngle)
	{
		robot.drive(motorOutput, correctionAngle);
	}
	
	@Override
	public void testPeriodic() {
	}

	@Override
	public void pidWrite(double output) {
	}
}

