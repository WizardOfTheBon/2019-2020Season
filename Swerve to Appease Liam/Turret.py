import wpilib
import rev

class ManualTurret:
	def __init__(self, flyWheelID):
		self.flyWheelMotor = rev.CANSparkMax(flyWheelID, MotorType.kBrushless)
		self.flyWheelEncoder = self.flyWheelMotor.getEncoder()
		
	def spin(self, goalSpeed):
		percent = goalSpeed/5700
		self.flyWheelMotor.set(percent)
		
	def spinPID(self, goalSpeed):
		
		
	def spinStop(self):
		self.flyWheelMotor.set(0)
		
	def spinBrake(self):
		self.flyWheelMotor.setIdleMode(rev.IdleMode.kBrake)
		
	def spinCoast(self):
		self.flyWheelMotor.setIdleMode(rev.IdleMode.kCoast)
