#include <Servo.h>

Servo servos[6];
int servoPins[6] = {0, 1, 2, 3, 4, 5};

// Tracks the angles of the servos
float currentIKShoulder = 90.0;
float currentIKElbow = 90.0;

// Tracks the coordinates of the main servo arm end point
float currentX = 100.0;
float currentY = 100.0;

// Calibration offsets for the spline misfitting
const int shoulderOffset = -6; 
const int elbowOffset = -11;     

// Length of the 3D-printed arms
float armLength = 100.0;    
float foreArmLength = 100.0; 

void setup() {
  Serial.begin(115200);

  // Attach with 180-degree pulse widths
  servos[1].attach(servoPins[1], 500, 2500); 
  servos[2].attach(servoPins[2], 500, 2500); 

  // Safely home the arm to a known value
  float startShoulder, startElbow;
  calculateIK(currentX, currentY, startShoulder, startElbow);
  servos[1].write(startShoulder + shoulderOffset);
  servos[2].write(startElbow + elbowOffset);
  currentIKShoulder = startShoulder;
  currentIKElbow = startElbow;

  delay(2000);
  Serial.println("--- COMMAND PARSER READY ---");
  Serial.println("Normal Move -> Type: 200 0");
  Serial.println("Linear Move -> Type: line 200 0");
}

void loop() {
  if (Serial.available() > 0) {
    // 1. Read the full text string
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    
    if (input.length() == 0) return; // Ignore empty blank lines

    // 2. Make it all lowercase to catch all e.g. "Line" or "LINE"
    input.toLowerCase();
    
    // 3. Check for the "line" command
    bool isLineMovement = false;
    if (input.startsWith("line ")) {
      isLineMovement = true;
      input = input.substring(5); // Chop off the first 5 characters ("line ")
      input.trim();
    }

    // 4. Find the space between the numbers and remove them
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex > 0) {
      float targetX = input.substring(0, spaceIndex).toFloat();
      float targetY = input.substring(spaceIndex + 1).toFloat();

      Serial.print("--- Target Received | X: ");
      Serial.print(targetX);
      Serial.print(" Y: ");
      Serial.println(targetY);

      float nextShoulder = 0;
      float nextElbow = 0;

      // 5. Run IK and execute the movement
      if (calculateIK(targetX, targetY, nextShoulder, nextElbow)) {
        
        if (isLineMovement) {
          Serial.println("Mode: STRAIGHT LINE INTERPOLATION");
          moveLine(targetX, targetY);
        } else {
          Serial.println("Mode: NORMAL ARC SNAP");
          
          // Update memory so the robot knows where it is for the next command
          currentIKShoulder = nextShoulder;
          currentIKElbow = nextElbow;
          currentX = targetX;
          currentY = targetY;

          // Execute instant move (withou line)
          servos[1].write(currentIKShoulder + shoulderOffset);
          servos[2].write(currentIKElbow + elbowOffset);
        }
      }
    } else {
      Serial.println("Error: Format unrecognized. Try '150 100' or 'line 150 100'");
    }
  }
}

// Straight line movement function
void moveLine(float targetX, float targetY) {
  float distance = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));
  
  int steps = (int)distance; 
  if (steps == 0) return; 

  for (int i = 1; i <= steps; i++) {
    float t = (float)i / steps;
    
    float stepX = currentX + (targetX - currentX) * t;
    float stepY = currentY + (targetY - currentY) * t;

    float stepShoulder = 0;
    float stepElbow = 0;

    if (calculateIK(stepX, stepY, stepShoulder, stepElbow)) {
      servos[1].write(stepShoulder + shoulderOffset);
      servos[2].write(stepElbow + elbowOffset);
      delay(30); 
    }
  }
  
  currentX = targetX;
  currentY = targetY;
  
  float finalShoulder, finalElbow;
  calculateIK(currentX, currentY, finalShoulder, finalElbow);
  currentIKShoulder = finalShoulder;
  currentIKElbow = finalElbow;
}

// Inverse kinematics maths function (to use coordinates instead of individual servo angles)
bool calculateIK(float targetX, float targetY, float &targetShoulderAngle, float &targetElbowAngle) {
  float hypo = sqrt((targetX * targetX) + (targetY * targetY)); 

  if (targetY < 0) {
    Serial.println("Error: Target out of reach (underground).");
    return false;
  }

  if (hypo <= (armLength + foreArmLength)) {
      float intElbowRad = acos(((armLength * armLength) + (foreArmLength * foreArmLength) - (hypo * hypo)) / (2 * foreArmLength * armLength));
      float intElbowDeg = intElbowRad * (180/PI); 

      float extHypoRad = atan2(targetY, targetX);
      float intShoulderRad = acos(((armLength * armLength) + (hypo * hypo) - (foreArmLength * foreArmLength)) / (2 * armLength * hypo));
      float intShoulderDeg = intShoulderRad * (180/PI);
      float extHypoDeg = extHypoRad * (180/PI);
      
      if (targetX >= 0) {
        targetShoulderAngle = extHypoDeg + intShoulderDeg;
        targetElbowAngle = intElbowDeg - 90; 
      } else {
        targetShoulderAngle = extHypoDeg - intShoulderDeg;
        targetElbowAngle = 270 - intElbowDeg; 
      }
      
      if (targetElbowAngle < 0 || targetElbowAngle > 180 || targetShoulderAngle < 0 || targetShoulderAngle > 180) {
        Serial.println("Error: Angles outside limits.");
        return false;
      }
      return true; 
  } else {
    Serial.println("Error: Target out of reach.");
    return false;
  }
}