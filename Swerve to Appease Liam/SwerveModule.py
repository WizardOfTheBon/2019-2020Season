import math
import rev
import wpilib
from networktables import NetworkTables
from wpilib import controller as controller

class SwerveModule:
	wheelDiameter = 4 #inches
	turnMotorEncoderConversion = 20 #NEO encoder gives 0-18 as 1 full rotation
	absoluteEncoderConversion = .08877
	
	def __init__(self,driveID,turnID,encoderID,encoderOffset,name):
		self.kPDrive = .0039 #these aren't actually tuned, and even if they are close, they're for driving a light robot on tile
		self.kIDrive = 0
		self.kDDrive = 0
		self.kS = 0
		self.kV = 1
		self.kA = 0
		
		self.kPTurn = .0039
		self.kITurn = 0
		self.kDTurn = 0
		
		self.driveMotor = rev.CANSparkMax(driveID,rev.MotorType.kBrushless)
		self.turnMotor = rev.CANSparkMax(turnID,rev.MotorType.kBrushless)
		self.turnEncoder = self.turnMotor.getEncoder()
		self.turnEncoder.setPositionConversionFactor(self.turnMotorEncoderConversion) #now is 0-360
		
		self.absoluteEncoder = wpilib.AnalogInput(encoderID)
		self.encoderOffset = encoderOffset
		#the above line centers the absolute encoder and then changes its direction, as the absolute encoders spin
		#the opposite direction of the NEO encoders
		
		self.turnController = wpilib.controller.PIDController(self.kPTurn, self.kITurn, self.kDTurn)
		self.turnController.enableContinuousInput(-180,180) #the angle range we decided to make standard
		
		self.driveEncoder = self.driveMotor.getEncoder()
		self.driveController = wpilib.controller.PIDController(self.kPDrive, self.kIDrive, self.kDDrive, self.kIZoneDrive, self.kFFDrive)
		self.driveFeedforward = wpilib.controller.PIDController(self.kS, self.kV, self.kA)
		
		self.turnDeadband = .035
		
		self.moduleName = name
		
		self.lastPosition = self.encoderBoundedPosition()
		
	def encoderBoundedPosition(self):
		position = self.turnEncoder.getPosition()%360 #this limits the encoder input
		if position < 0: #to be on a single circle
			position += 360
		if position < 90: #this translates those values to correspond with what the
			position += 90 #atan2 function returns (-180, 180)
		else:
			position -= 270
		return position
		
	def move(self,driveSpeed,angle):
		position = self.encoderBoundedPosition()
		
		#this sets optimizes the wheel turning by taking the shortest path to the goal angle
		if self.lastPosition > position:
			if (360 - self.lastPosition + position) < (position - self.lastPosition):
				position = position += 360
		elif self.lastposition < position:
			if (360 - self.lastPosition + position) > (position - self.lastPosition):
				position = position - 360
		
		self.turnController.setSetpoint(angle) #tells the PID controller what our goal is
		turnSpeed = self.turnController.calculate(position) #gets the ideal speed from the PID controller
		
		if abs(turnSpeed) < self.turnDeadband:
			turnSpeed = 0
		
		#this finds the desired speed using PID and then the desired voltage using feedforward for the drive motor
		self.driveController.setSetpoint(driveSpeed)
		currentSpeed = self.driveEncoder.getVelocity()
		driveAcceleration = self.driveController.calculate(currentSpeed)
		driveVoltage = self.driveFeedforward.calculate(currentSpeed, driveAcceleration)
		
		self.driveMotor.setVoltage(driveVoltage)
		self.turnMotor.set(turnSpeed)
		
		self.lastPosition = self.encoderBoundedPosition()
		
		wpilib.SmartDashboard.putNumber(self.moduleName,position)
		
	def stationary(self):
		self.driveMotor.set(0) #this will be smoother once we drive with velocity PID (by setting setpoint to 0)
		
		position = self.encoderBoundedPosition()
		turnSpeed = self.turnController.calculate(position)
		
		if abs(turnSpeed) < self.turnDeadband:
			turnSpeed = 0
		
		self.turnMotor.set(turnSpeed) #just let the turn motor go to its most recent goal
		
	def zeroEncoder(self):
		absolutePosition = 360 - self.absoluteEncoder.getValue()*self.absoluteEncoderConversion + self.encoderOffset
		self.turnEncoder.setPosition(absolutePosition)
		self.turnController.setSetpoint(0)
		
	def brake(self):
		self.driveMotor.setIdleMode(rev.IdleMode.kBrake)
		self.turnMotor.setIdleMode(rev.IdleMode.kBrake)
		
	def coast(self):
		self.driveMotor.setIdleMode(rev.IdleMode.kCoast)
		self.turnMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def checkEncoders(self):
		absolutePosition = self.absoluteEncoder.getValue()*self.absoluteEncoderConversion
		wpilib.SmartDashboard.putNumber(self.moduleName,absolutePosition)