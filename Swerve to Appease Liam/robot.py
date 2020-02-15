#!/usr/bin/env python3

import wpilib
import math
import rev
from wpilib import controller as controller
#these next imports allow us to type the "ClassName" instead of the "FileName.ClassName"
from DriveTrain import DriveTrain #without this it would be DriveTrain.DriveTrain
from Climb import Climb
from Feeder import Feeder
from Intake import Intake
from ManualTurret import ManualTurret
from navx import AHRS

class MyRobot(wpilib.TimedRobot):
	def robotInit(self):
		self.navx = AHRS.create_spi()
		self.navx.reset()
		
		#joysticks
		self.joystick = wpilib.Joystick(0)
		self.buttonsJoystick = wpilib.Joystick(1)
		self.joystickDeadband = .2
		
		#buttons
		self.climbExtendButton = wpilib.DriverStation.getStickButton(1, 1)
		self.climbContractButton = wpilib.DriverStation.getStickButton(1, 2)
		self.feederInButton = wpilib.DriverStation.getStickButton(1, 3)
		self.feederOutButton = wpilib.DriverStation.getStickButton(1, 4)
		self.intakeInButton = wpilib.DriverStation.getStickButton(0, 1)
		self.intakeOutButton = wpilib.DriverStation.getStickButton(0, 2)
		
		self.manualTurretHoodDial = wpilib.DriverStation.getStickButton(1, 5)
		self.manualTurretSpinDial = wpilib.DriverStation.getStickButton(1, 6)
		
		#climbing object instantiation
		self.climb = Climb(9,0.5) #motorID and speed
		
		#feeder object instantiation
		self.feeder = Feed(10,0.5) #motorID and speed
		
		#intake object instantiation
		self.intake = Intake(11,12,0.5,0.5) #intakeID, halfMoonID, and corresponding speeds
		
		#manual turret object instantiation
		self.manualTurret = ManualTurret(13,14,1,20) #rotate motor, flywheel motor, servoID, and rotateThreshold from middle to max
		
		#drivetrain object instantiation and init
		self.drive = DriveTrain() #keep in mind that these are all arbitrary names
		self.drive.zeroEncoders() #feel free to change the weird ones
		self.fieldOriented = False
		
		#debugging messages
		self.scaling = .5
		wpilib.SmartDashboard.putNumber("Joystick scale factor", self.scaling)
		wpilib.SmartDashboard.putNumber("Joystick deadband", self.joystickDeadband)
		wpilib.SmartDashboard.putBoolean("Field Oriented", self.fieldOriented)
		
	def checkDeadband(self, axis):
		deadband = wpilib.SmartDashboard.getNumber("Joystick deadband", self.joystickDeadband)
		if abs(axis) < deadband:
			axis = 0
		return axis
		
	def autonomousInit(self):
		self.drive.coast()
		print('autonomous started')
		
	def autonomousPeriodic(self):
		self.drive.checkEncoders()
		
	def teleopInit(self):
		self.drive.zeroEncoders()
		self.drive.brake()
		self.navx.reset() #please delete this before competition
		print('teleop started')
		
	def CheckSwitches(self):
		if self.climbExtendButton:
			self.climb.extend()
		elif self.climbContractButton:
			self.climb.contract()
		else:
			self.climb.brake()
		
		if autoManualSwitch: #manual
			self.manualTurret.returnToOrigin()
			
			if self.feederInButton:
				self.feeder.feed()
			elif self.feederOutButton:
				self.feeder.puke()
			else:
				self.feeder.coast()
			
			if self.intakeInButton:
				self.intake.collect()
			elif self.intakeOutButton:
				self.intake.expel()
			else:
				self.intake.coast()
			
			self.manualTurret.setHoodAngle(self.manualTurretHoodDial)
			self.manualTurret.spin(self.manualTurretSpinDial)
			
		
	def teleopPeriodic(self):
		CheckSwitches()
		
		scale = wpilib.SmartDashboard.getNumber("Joystick scale factor", self.scaling)
		fieldOriented = wpilib.SmartDashboard.getBoolean("Field Oriented", self.fieldOriented)
		
		x = scale*self.checkDeadband(self.joystick.getX())
		y = -scale*self.checkDeadband(self.joystick.getY())
		z = scale*self.checkDeadband(self.joystick.getZ())
		
		angle = -1*self.navx.getRoll() + 90
		wpilib.SmartDashboard.putNumber("angle",angle)
		angle *= math.pi/180
		
		if fieldOriented:
			cos = math.cos(angle)
			sin = math.sin(angle)
			temp = x*sin - y*cos
			y = x*cos + y*sin
			x = temp
		
		if max(abs(x),abs(y),abs(z)) != 0:
			self.drive.move(x,y,z)
		else:
			self.drive.stationary()
		
	def disabledInit(self):
		self.drive.coast()

if __name__ == "__main__":
	wpilib.run(MyRobot)