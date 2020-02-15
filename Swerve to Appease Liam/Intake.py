import wpilib
import rev

class Intake:
	def __init__(self, intakeID, halfMoonID, speed):
		self.intakeMotor = rev.CANSparkMax(intakeID, MotorType.kBrushless)
		self.halfMoonMotor = rev.CANSparkMax(halfMoonID, MotorType.kBrushless)
		self.speed = speed
	def collect(self):
		self.intakeMotor.set(self.speed)
		self.halfMoonMotor.set(self.speed)
	def expel(self):
		self.intakeMotor.set(-self.speed)
		self.halfMoonMotor.set(-self.speed)
	def coast(self):
		self.intakeMotor.setIdleMode(rev.IdleMode.kCoast)
		self.halfMoonMotor.setIdleMode(rev.IdleMode.kCoast)
	def brake(self):
		self.intakeMotor.setIdleMode(rev.IdleMode.kBrake)
		self.halfMoonMotor.setIdleMode(rev.IdleMode.kBrake)