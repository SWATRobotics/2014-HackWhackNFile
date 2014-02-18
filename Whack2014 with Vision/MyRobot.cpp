#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"

#define Y_IMAGE_RES 480		//X Image resolution in pixels, should be 120, 240 or 480
#define VIEW_ANGLE 49		//Axis M1013
//#define VIEW_ANGLE 41.7		//Axis 206 camera
//#define VIEW_ANGLE 37.4  //Axis M1011 camera
#define PI 3.141592653
#define RECTANGULARITY_LIMIT 40
#define ASPECT_RATIO_LIMIT 55
#define TAPE_WIDTH_LIMIT 50
#define VERTICAL_SCORE_LIMIT 50
#define LR_SCORE_LIMIT 50
#define AREA_MINIMUM 150
#define MAX_PARTICLES 8

class Whack : public SimpleRobot
{
	struct Scores {
			double rectangularity;
			double aspectRatioVertical;
			double aspectRatioHorizontal;
		};
		
		struct TargetReport {
			int verticalIndex;
			int horizontalIndex;
			bool Hot;
			double totalScore;
			double leftScore;
			double rightScore;
			double tapeWidthScore;
			double verticalScore;
		};
		
	RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick
	Victor shooter;
	Victor forklift;
	Victor  winch;
	DigitalInput stopBottom;//Limit Switch
	DigitalInput stopTop;
	DoubleSolenoid shooterSole;//Festo
	Compressor compressor;
	Encoder shooterEnconder;
	
public:
	Whack():
		myRobot(1, 2, 3, 4),	//front left, rear left, front right, rear right
		stick(1),		// usb number
		shooter(6),
		forklift(5),
		winch(7),
		stopBottom(1),//initialize limit switch
		stopTop(2),
		shooterSole(9,10),//initialize festo valves
					//forward channel, reverse channel
		compressor(7, 8),
			//(pressure switch channel, compressor relay channel) or
				//(pressure switch module number, pressure switch channel, compressor relay module number, compressor relay channel)
		shooterEnconder(5,6,false)//uint32_t aChannel, uint32_t bChannel, bool reverseDirection=false, EncodingType encodingType=k4X
	{
		myRobot.SetExpiration(0.1);
	}

	
	void Autonomous()
	{
		compressor.Start();//start compressor
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
		Wait(2.0); 				//    for 2 seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
		
		Scores *scores;
				TargetReport target;
				int verticalTargets[MAX_PARTICLES];
				int horizontalTargets[MAX_PARTICLES];
				int verticalTargetCount, horizontalTargetCount;
				Threshold threshold(105, 137, 230, 255, 133, 183);	//HSV threshold criteria, ranges are in that order ie. Hue is 60-100
				ParticleFilterCriteria2 criteria[] = {
						{IMAQ_MT_AREA, AREA_MINIMUM, 65535, false, false}
				};												//Particle filter criteria, used to filter out small particles
				 AxisCamera &camera = AxisCamera::GetInstance();	//To use the Axis camera uncomment this line
				
				while (IsAutonomous() && IsEnabled()) {
		            /**
		             * Do the image capture with the camera and apply the algorithm described above. This
		             * sample will either get images from the camera or from an image file stored in the top
		             * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
		             */
					ColorImage *image;
					//image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash

					image = camera.GetImage();				//To get the images from the camera comment the line above and uncomment this one
					BinaryImage *thresholdImage = image->ThresholdHSV(threshold);	// get just the green target pixels
					//thresholdImage->Write("/threshold.bmp");
					BinaryImage *filteredImage = thresholdImage->ParticleFilter(criteria, 1);	//Remove small particles
					//filteredImage->Write("Filtered.bmp");

					vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();  //get a particle analysis report for each particle

					verticalTargetCount = horizontalTargetCount = 0;
					//Iterate through each particle, scoring it and determining whether it is a target or not
					if(reports->size() > 0)
					{
						scores = new Scores[reports->size()];
						for (unsigned int i = 0; i < MAX_PARTICLES && i < reports->size(); i++) {
							ParticleAnalysisReport *report = &(reports->at(i));
							
							//Score each particle on rectangularity and aspect ratio
							scores[i].rectangularity = scoreRectangularity(report);
							scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, true);
							scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, false);			
							
							//Check if the particle is a horizontal target, if not, check if it's a vertical target
							if(scoreCompare(scores[i], false))
							{
								printf("particle: %d  is a Horizontal Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
								horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
							} else if (scoreCompare(scores[i], true)) {
								printf("particle: %d  is a Vertical Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
								verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
							} else {
								printf("particle: %d  is not a Target centerX: %d  centerY: %d \n", i, report->center_mass_x, report->center_mass_y);
							}
							printf("Scores rect: %f  ARvert: %f \n", scores[i].rectangularity, scores[i].aspectRatioVertical);
							printf("ARhoriz: %f  \n", scores[i].aspectRatioHorizontal);	
						}

						//Zero out scores and set verticalIndex to first target in case there are no horizontal targets
						target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
						target.verticalIndex = verticalTargets[0];
						for (int i = 0; i < verticalTargetCount; i++)
						{
							ParticleAnalysisReport *verticalReport = &(reports->at(verticalTargets[i]));
							for (int j = 0; j < horizontalTargetCount; j++)
							{
								ParticleAnalysisReport *horizontalReport = &(reports->at(horizontalTargets[j]));
								double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;
			
								//Measure equivalent rectangle sides for use in score calculation
								imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &horizWidth);
								imaqMeasureParticle(filteredImage->GetImaqImage(), verticalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &vertWidth);
								imaqMeasureParticle(filteredImage->GetImaqImage(), horizontalReport->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &horizHeight);
								
								//Determine if the horizontal target is in the expected location to the left of the vertical target
								leftScore = ratioToScore(1.2*(verticalReport->boundingRect.left - horizontalReport->center_mass_x)/horizWidth);
								//Determine if the horizontal target is in the expected location to the right of the  vertical target
								rightScore = ratioToScore(1.2*(horizontalReport->center_mass_x - verticalReport->boundingRect.left - verticalReport->boundingRect.width)/horizWidth);
								//Determine if the width of the tape on the two targets appears to be the same
								tapeWidthScore = ratioToScore(vertWidth/horizHeight);
								//Determine if the vertical location of the horizontal target appears to be correct
								verticalScore = ratioToScore(1-(verticalReport->boundingRect.top - horizontalReport->center_mass_y)/(4*horizHeight));
								total = leftScore > rightScore ? leftScore:rightScore;
								total += tapeWidthScore + verticalScore;
								
								//If the target is the best detected so far store the information about it
								if(total > target.totalScore)
								{
									target.horizontalIndex = horizontalTargets[j];
									target.verticalIndex = verticalTargets[i];
									target.totalScore = total;
									target.leftScore = leftScore;
									target.rightScore = rightScore;
									target.tapeWidthScore = tapeWidthScore;
									target.verticalScore = verticalScore;
								}
							}
							//Determine if the best target is a Hot target
							target.Hot = hotOrNot(target);
						}
						
						if(verticalTargetCount > 0)
						{
							//Information about the target is contained in the "target" structure
							//To get measurement information such as sizes or locations use the
							//horizontal or vertical index to get the particle report as shown below
							ParticleAnalysisReport *distanceReport = &(reports->at(target.verticalIndex));
							double distance = computeDistance(filteredImage, distanceReport);
							if(target.Hot)
							{
								printf("Hot target located \n");
								printf("Distance: %f \n", distance);
								//Rotary Econder stuff for distance, should be set distance so the number should stay the same
//NEED TO TEST TO FIND OUT DISTANCE FOR ROTARY ENCODER
								while (shooterEnconder.Get()<=150){
								shooter.SetSpeed(1);
								}
								shooterSole.Set(shooterSole.kForward);//Sets Solenoid forward, shoot ball
								shooterSole.Set(shooterSole.kReverse);//Sets Solenoid backward
															
								
							} else {
								printf("No hot target present \n");
								printf("Distance: %f \n", distance);
								Wait(5.2);
								//Rotary Econder stuff for distance, should be set distance so the number should stay the same
//NEED TO TEST TO FIND OUT DISTANCE FOR ROTARY ENCODER
								while (shooterEnconder.Get()<=150){
								shooter.SetSpeed(1);
								}
								shooterSole.Set(shooterSole.kForward);//Sets Solenoid forward, shoot ball
								shooterSole.Set(shooterSole.kReverse);//Sets Solenoid backward
							}
						}
					}

					// be sure to delete images after using them
					delete filteredImage;
					delete thresholdImage;
					delete image;
					
					//delete allocated reports and Scores objects also
					delete scores;
					delete reports;
					
					myRobot.Drive(1.0, -1.0);//turn around for 0.5 seconds, we have to check to see if it takes that long
					Wait(0.5);//wait for 0.5 seconds
					myRobot.Drive(0.0, 0.0);//stop robot
				
	}

	
	void OperatorControl()
	{
		myRobot.SetSafetyEnabled(true);
		myRobot.SetInvertedMotor(myRobot.kRearLeftMotor, true);//Invert rear left Motor
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor, true);//Invert rear right motor
		int forkCount = 0;
		while (IsOperatorControl())
		{
			myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			
			if (!stopBottom.Get() && stick.GetRawButton(4) || stick.GetRawButton(4) && !stopTop.Get()){
				shooter.SetSpeed(1);
			}
			if (stopBottom.Get() || !stick.GetRawButton(4)|| stopTop.Get()){
				shooter.SetSpeed(0);
			}
			
			if (stick.GetRawButton(5)){
				shooterEnconder.Start();
				while (shooterEnconder.Get()<=150){
					shooter.SetSpeed(1);
				}
			}
			
			if (stick.GetRawButton(1)){
				shooterSole.Set(shooterSole.kForward);//Sets Solenoid forward
				Wait(0.005);
				shooterSole.Set(shooterSole.kReverse);//Sets Solenoid backward
			}
			if (stick.GetRawButton(2)){
				if (forkCount == 0){
				forklift.SetSpeed(0.5);
				forkCount = 1;
				}
				if (forkCount == 1){
				forklift.SetSpeed(0);
				forkCount = 0;
				}
			}
			if (stick.GetRawButton(3)){
				if (forkCount == 0){
				forklift.SetSpeed(-0.5);
				forkCount = 1;
				}
				if (forkCount == 1){
				forklift.SetSpeed(0);
				forkCount = 0;
				}
			} 
			if (stick.GetRawButton(11)){
				winch.SetSpeed(1);
				Wait(0.5);
				winch.SetSpeed(0);
			}
			if (stick.GetRawButton(12)){
				winch.SetSpeed(-1);
				Wait(0.5);
				winch.SetSpeed(0);
			}
			Wait(0.005);// wait for a motor update time
		}
	}

	void Test() {

	}
	
	double computeDistance (BinaryImage *image, ParticleAnalysisReport *report) {
		double rectLong, height;
		int targetHeight;
		
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
		//using the smaller of the estimated rectangle long side and the bounding rectangle height results in better performance
		//on skewed rectangles
		height = min(report->boundingRect.height, rectLong);
		targetHeight = 32;
		
		return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
	}
	
	/**
	 * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
	 * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
	 * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
	 * and particle perimeter= 2x+2y
	 * 
	 * @param image The image containing the particle to score, needed to perform additional measurements
	 * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
	 * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
	 * @return The aspect ratio score (0-100)
	 */
	double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool vertical){
		double rectLong, rectShort, idealAspectRatio, aspectRatio;
		idealAspectRatio = vertical ? (4.0/32) : (23.5/4);	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall
		
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE, &rectLong);
		imaqMeasureParticle(image->GetImaqImage(), report->particleIndex, 0, IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE, &rectShort);
		
		//Divide width by height to measure aspect ratio
		if(report->boundingRect.width > report->boundingRect.height){
			//particle is wider than it is tall, divide long by short
			aspectRatio = ratioToScore(((rectLong/rectShort)/idealAspectRatio));
		} else {
			//particle is taller than it is wide, divide short by long
			aspectRatio = ratioToScore(((rectShort/rectLong)/idealAspectRatio));
		}
		return aspectRatio;		//force to be in range 0-100
	}
	
	/**
	 * Compares scores to defined limits and returns true if the particle appears to be a target
	 * 
	 * @param scores The structure containing the scores to compare
	 * @param vertical True if the particle should be treated as a vertical target, false to treat it as a horizontal target
	 * 
	 * @return True if the particle meets all limits, false otherwise
	 */
	bool scoreCompare(Scores scores, bool vertical){
		bool isTarget = true;

		isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
		if(vertical){
			isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
		} else {
			isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
		}

		return isTarget;
	}
	
	/**
	 * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
	 * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
	 * 
	 * @param report The Particle Analysis Report for the particle to score
	 * @return The rectangularity score (0-100)
	 */
	double scoreRectangularity(ParticleAnalysisReport *report){
		if(report->boundingRect.width*report->boundingRect.height !=0){
			return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
		} else {
			return 0;
		}	
	}	
	
	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
	 * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
	 */
	double ratioToScore(double ratio)
	{
		return (max(0, min(100*(1-fabs(1-ratio)), 100)));
	}
	
	/**
	 * Takes in a report on a target and compares the scores to the defined score limits to evaluate
	 * if the target is a hot target or not.
	 * 
	 * Returns True if the target is hot. False if it is not.
	 */
	bool hotOrNot(TargetReport target)
	{
		bool isHot = true;
		
		isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
		isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
		isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);
		
		return isHot;
	}
};

START_ROBOT_CLASS(Whack);

