  
import wpilib
import math
import rev
from wpilib_controller import PIDController

class MyRobot(wpilib.TimedRobot):
	
	def robotInit(self):
	
		self.green = 1
		self.red = 1
		self.blue = 1
		self.yellow = 1
		# these buttons are for the different stages, where the wheel will spin until it has the right color or the right number of rotations
		self.rotationsSpinButton = wpilib.DriverStation.getStickButton(0,0) #joystick channel and button number
		self.colorSpinButton = wpilib.DriverStation.getStickButton(0,1)
		self.ethansMotor = rev.CANSparkMax(4,rev.MotorType.kBrushless)
		self.ethansServo = wpilib.PWM(1) #this is the servo that flips up the wheel, it is connected to pwm channel 1
		# .setPosition(0.0-1.0) means -90 degrees to 90 degrees

		self.spinning = False
		self.targetColor= green
		self.colorTime = False
		self.colorSensor = ??
	def autonomousInit(self):
		pass
	def autonomousPeriodic(self):
		pass
	


		
	def teleopInit(self):
		pass
	def teleopPeriodic(self):
		#use this to control the motor: rev.CANSparkMax(4,rev.MotorType.kBrushless).set(0.5)
		if (self.spinButton == True) or self.spinning:
		#rev.MotorType.kBrushless
		#rev.CANSparkMax(4,rev.MotorType.kBrushless).getEncoder()
			if colorChanges < 21:
				self.ethansMotor.set(0.3)
				self.spinning = True
				
			if self.colorChanges > 21:
				self.spinning = False
		
		#colors go green,blue yellow red green
		#positive is clockwise and negative is counterclockwise
		if self.colorButton == True:
			if self.targetColor == green and self.colorSensor == blue:
				self.revTarget = 2
			
			if self.targetColor == blue and self.colorSensor == green:
				self.revTarget = 2
			
			if self.targetColor == yellow and self.colorSensor == green:
				self.revTarget = 0
			if
		
		
if __name__ == '__main__':
	wpilib.run(MyRobot)