import wpilib
import rev

class ManualTurret:
	def __init__(self, flyWheelID):
		self.flyWheelMotor = rev.CANSparkMax(flyWheelID, MotorType.kBrushless)
		self.flyWheelEncoder = self.flyWheelMotor.getEncoder()
		self.flyWheelController = self.flyWheelMotor.getPIDController()
		self.kP = 0.005
		self.kI = 0
		self.kD = 0
		self.kIZone = 0
		self.kFF = 1
		self.flyWheelController.setP(self.kP)
		self.flyWheelController.setI(self.kI)
		self.flyWheelController.setD(self.kD)
		self.flyWheelController.setIZone(self.kIZone)
		self.flyWheelController.setFF(self.kFF)
		
	def spin(self, goalSpeed):
		percent = goalSpeed/5700
		self.flyWheelMotor.set(percent)
		
	def spinPID(self, goalSpeed):
		self.flyWheelController.setReference(goalSpeed, ControlType.kVelocity, arbFeedforward = self.kFF)
		
	def spinStop(self):
		self.flyWheelMotor.set(0)
		
	def spinBrake(self):
		self.flyWheelMotor.setIdleMode(rev.IdleMode.kBrake)
		
	def spinCoast(self):
		self.flyWheelMotor.setIdleMode(rev.IdleMode.kCoast)
