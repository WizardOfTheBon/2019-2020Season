import wpilib
import rev

class ManualTurret:
	def __init__(self, rotateID, flyWheelID, speed, threshold):
		self.rotateMotor = rev.CANSparkMax(rotateID, MotorType.kBrushless)
		self.rotateEncoder = self.rotateMotor.getEncoder()
		self.encoderConversion = 20
		self.rotateEncoder = setPositionConversionFactor(self.encoderConversion)
		self.flyWheelMotor = rev.CANSparkMax(flyWheelID, MotorType.kBrushless)
		self.speed = speed
		self.threshold = threshold
	def rotate(self):
		if self.rotateEncoder < self.threshold:
			pass