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
		self.buttonsJoystick1 = wpilib.Joystick(1)
		self.buttonsJoystick2 = wpilib.Joystick(2)
		self.joystickDeadband = .2
		
		#buttons
		self.feederInButton = wpilib.DriverStation.getStickButton(1, 1)
		self.intakeInButton = wpilib.DriverStation.getStickButton(0, 1)
		self.intakeOutButton = wpilib.DriverStation.getStickButton(0, 2)
		self.autoManualSwitch = wpilib.DriverStation.getStickButton(2, 1)
		self.manualFlywheelSwitch = wpilib.DriverStation.getStickButton(2, 4)
		
		#feeder object instantiation
		self.feeder = Feed(10,0.5) #motorID and speed
		
		#intake object instantiation
		self.intake = Intake(11,12,0.5,0.7) #intakeID, halfMoonID, and corresponding speeds
		
		#manual turret object instantiation
		self.manualTurret = ManualTurret(13) #flywheel motor ID
		
		#drivetrain object instantiation and init
		self.drive = DriveTrain()
		self.drive.zeroEncoders()
		self.fieldOriented = False
		self.perfectlyLegitFlywheelRPM = 0
		
		#debugging messages
		self.scaling = .5
		wpilib.SmartDashboard.putNumber("Joystick scale factor", self.scaling)
		wpilib.SmartDashboard.putNumber("Joystick deadband", self.joystickDeadband)
		wpilib.SmartDashboard.putNumber("Flywheel RPM", self.perfectlyLegitFlywheelRPM)
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
		if self.autoManualSwitch: #manual
			
			if self.feederInButton:
				self.feeder.feed()
			else:
				self.feeder.stop()
			
			
			if self.intakeInButton:
				self.intake.collect()
			elif self.intakeOutButton:
				self.intake.expel()
			else:
				self.intake.stop()
			
			benIsAGoober = wpilib.SmartDashboard.getNumber("Flywheel RPM",self.perfectlyLegitFlywheelRPM)
			if self.perfectlyLegitFlywheelRPM != benIsAGoober:
				self.perfectlyLegitFlywheelRPM = benIsAGoober
			
			if self.manualFlywheelSwitch:
				if self.perfectlyLegitFlywheelRPM > 600:
					self.manualTurret.spin(self.perfectlyLegitFlywheelRPM)
				else:
					self.manualTurret.spin(0)
			else:
				self.manualTurret.spin(0)
		
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