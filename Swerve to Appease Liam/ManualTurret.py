import wpilib
import rev

class ManualTurret:
	def __init__(self, rotateID, flyWheelID):
		self.rotateMotor = rev.CANSparkMax(rotateID, MotorType.kBrushless)
		self.rotateEncoder = self.rotateMotor.getEncoder()
		self.flyWheelMotor = rev.CANSparkMax(flyWheelID, MotorType.kBrushless)
		self.flyWheelEncoder = self.flyWheelMotor.getEncoder()
		self.kP = 0
		self.kI = 0
		self.kD = 0
		self.kIz = 0
		self.kFF = 0
		self.flyWheelController = wpilib.controller.PIDController(self.kP,self.kI,self.kD,self.kIz,self.kFF)
	def rotate(self):
		if self.rotateEncoder < threshold:
			pass
	def spin(self, goalSpeed):
		currentSpeed = self.flyWheelEncoder.getVelocity()
		
		self.flyWheelController.setSetpoint(goalSpeed)
		spinAcceleration = self.flyWheelController.setSpeed(currentSpeed)
		
		#figure out how to change normal PID into velocity PID, and figure out what kIz and kFF are
		
		pass
