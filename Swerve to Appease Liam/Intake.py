import wpilib
import rev

class Intake:
	def __init__(self, intakeID, halfMoonID, intakeSpeed, halfMoonSpeed):
		self.intakeMotor = rev.CANSparkMax(intakeID, MotorType.kBrushless)
		self.halfMoonMotor = rev.CANSparkMax(halfMoonID, MotorType.kBrushless)
		self.intakeSpeed = intakeSpeed
		self.halfMoonSpeed = halfMoonSpeed
	def collect(self):
		self.intakeMotor.set(self.intakeSpeed)
		self.halfMoonMotor.set(self.halfMoonSpeed)
	def expel(self):
		self.intakeMotor.set(-self.intakeSpeed)
		self.halfMoonMotor.set(-self.halfMoonSpeed)
	def coast(self):
		self.intakeMotor.setIdleMode(rev.IdleMode.kCoast)
		self.halfMoonMotor.setIdleMode(rev.IdleMode.kCoast)
	def brake(self):
		self.intakeMotor.setIdleMode(rev.IdleMode.kBrake)
		self.halfMoonMotor.setIdleMode(rev.IdleMode.kBrake)
	def stop(self):
		self.intakeMotor.set(0)
		self.halfMoonMotor.set(0)