import wpilib
import rev

class Feeder:
	def __init__(self, motorID, speed):
		self.feederMotor = rev.CANSparkMax(ID, MotorType.kBrushless)
		self.speed = speed
	def feed(self):
		self.feederMotor.set(self.speed)
	def puke(self): #the idea of this makes me want to puke -Liam
		self.feederMotor.set(-self.speed)
	def coast(self):
		self.feederMotor.setIdleMode(rev.IdleMode.kCoast)
	def brake(self):
		self.feederMotor.setIdleMode(rev.IdleMode.kBrake)
	def stop(self):
		self.feederMotor.set(0)