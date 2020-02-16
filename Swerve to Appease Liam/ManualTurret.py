import wpilib
import rev

class ManualTurret:
	encoderConversion = 20
	
	#PID arguments (need to be adjusted)
	kP = 0.005
	kI = 0
	kD = 0
	
	#FeedForward arguments (need to be adjusted)
	kIZone = 0 #the distance to the goal where kI kicks in (so it doesn't overpower everything else)
	kFF = 1 #voltage per unit of speed
	
	def __init__(self, rotateID, flyWheelID, servoID, rotateThreshold):
		self.rotateMotor = rev.CANSparkMax(rotateID, MotorType.kBrushless)
		self.rotateEncoder = self.rotateMotor.getEncoder()
		self.rotateThreshold = rotateThreshold
		
		self.flyWheelMotor = rev.CANSparkMax(flyWheelID, MotorType.kBrushless)
		self.flyWheelEncoder = self.flyWheelMotor.getEncoder()
		
		self.flyWheelController = rev.CANPIDController(self.flyWheelMotor)
		self.flyWheelController.setP(kP)
		self.flyWheelController.setI(kI)
		self.flyWheelController.setD(kD)
		self.flyWheelController.setIZone(kIZone)
		self.flyWheelController.setFF(kFF)
		
		#self.hoodServo = wpilib.Servo(servoID)
		#self.hoodServoGearRatio = 1/25 #25 degrees of movement is from 0 to 1
		#self.hoodStartAngle = 30
		
	def returnToOrigin(self):
		if self.rotateEncoder > 20:
			self.rotateMotor.set(-0.1)
		elif self.rotateEncoder < -20:
			self.rotateMotor.set(0.1)
		
	def spin(self, goalSpeed):
		#currentSpeed = self.flyWheelEncoder.getVelocity()*encoderConversion
		percent = goalSpeed/5700
		self.flyWheelMotor.set(percent)
		
	def spinBrake(self):
		self.flyWheelMotor.setIdleMode(rev.IdleMode.kBrake)
		
	def spinCoast(self):
		self.flyWheelMotor.setIdleMode(rev.IdleMode.kCoast)
		
	def setHoodAngle(self, angle):
		pass
		#angle -= self.hoodStartAngle
		#self.hoodServo.set(angle * self.hoodServoGearRatio)
