import wpilib
import rev

class ManualTurret:
	encoderConversion = 20
	
	#PID arguments
	kP = 0
	kI = 0
	kD = 0
	
	#FeedForward arguments
	kS = 0
	kV = 0
	kA = 0
	
	def __init__(self, rotateID, flyWheelID, servoID, rotateThreshold):
		self.rotateMotor = rev.CANSparkMax(rotateID, MotorType.kBrushless)
		self.rotateEncoder = self.rotateMotor.getEncoder()
		self.rotateThreshold = rotateThreshold
		
		self.flyWheelMotor = rev.CANSparkMax(flyWheelID, MotorType.kBrushless)
		self.flyWheelEncoder = self.flyWheelMotor.getEncoder()
		
		self.flyWheelController = wpilib.controller.PIDController(kP,kI,kD)
		self.flyWheelFeedForward = wpilib.controller.SimpleMotorFeedforward(kS, kV, kA)
		
		self.hoodServo = wpilib.Servo(servoID)
		self.hoodServoGearRatio = 1/25 #25 degrees of movement is from 0 to 1
		self.hoodStartAngle = 30
		
	def rotate(self):
		if self.rotateEncoder > 0:
			self.rotateMotor.set(-0.1)
		elif self.rotateEncoder < 0:
			self.rotateMotor.set(0.1)
		
	def spin(self, goalSpeed):
		currentSpeed = self.flyWheelEncoder.getVelocity()*encoderConversion
		
		self.flyWheelController.setSetpoint(goalSpeed)
		acceleration = self.flyWheelController.setSpeed(currentSpeed)
		
		self.flyWheelMotor(self.flyWheelFeedForward.calculate(currentSpeed, acceleration))
		
	def spinBrake(self):
		self.flyWheelMotor.setIdleMode(rev.IdleMode.kBrake)
		
	def spinCoast(self):
		self.flyWheelMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def setHoodAngle(self, angle):
		angle -= self.hoodStartAngle
		self.hoodServo.set(angle * self.hoodServoGearRatio)
