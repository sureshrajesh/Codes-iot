1--> Soil moisture:- 

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
int pin = 8;
int analogPin = A0;
int sensorValue = 0;
void setup()
{
  pinMode(analogPin, INPUT);
  pinMode(pin, OUTPUT);
  lcd.begin(16, 2);
  lcd.print("what is the air");
  lcd.print("Quality Today");
  Serial.begin(9600);
  lcd.display();
}
void loop()
{
  delay(100);
  sensorValue = analogRead(analogPin);
  Serial.print("air Quality in RPM=");
  Serial.println(sensorValue);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Air Quality:");
  lcd.print(sensorValue);
  if (sensorValue<=100)
  {
    Serial.print("Fresh air");
    Serial.print("\r\n");
    lcd.setCursor(0,1);
    lcd.print("Fresh Air");
  }
  else if(sensorValue>=101 && sensorValue<=200)
  {
    Serial.print("Poor Air");
    Serial.print("\r\n");	
    lcd.setCursor(0,1);
    lcd.print("Poor Air");
  }
  else if (sensorValue>=400)
  {
    Serial.print("Very Poor Air");
    Serial.print("\r\n");	
    lcd.setCursor(0,1);
    lcd.print("Very Poor Air");
  }
if(sensorValue>400)
    {
    digitalWrite(pin, HIGH);
    }
else
    {
      digitalWrite(pin, LOW);
    }
}


2--> Air Quality:-

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
int pin = 8;
int analogPin = A0;
int sensorValue = 0;
void setup()
{
  pinMode(analogPin, INPUT);
  pinMode(pin8, OUTPUT);
  lcd.begin(16, 2);
  lcd.print("what is the air");
  lcd.print("Quality Today");
  Serial.begin(9600);
  lcd.display();
}
void loop()
{
  delay(100);
  sensorValue = analogRead(analogPin);
  Serial.print("air Quality in RPM=");
  Serial.println(sensorValue);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Air Quality:");
  lcd.print(sensorValue);
  if (sensorValue<=500)
  {
    Serial.print("Fresh air");
    Serial.print("\r\n");
    lcd.setCursor(0,1);
    lcd.print("Fresh Air");
  }
  else if(sensorValue>=500 && sensorValue<=600)
  {
    Serial.print("Poor Air");
    Serial.print("\r\n");	
    lcd.setCursor(0,1);
    lcd.print("Poor Air");
  }
  else if (sensorValue>=650)
  {
    Serial.print("Very Poor Air");
    Serial.print("\r\n");	
    lcd.setCursor(0,1);
    lcd.print("Very Poor Air");
  }
}
  if(sensorValue>650)
    {
    digitalWrite(pin8, HIGH);
    }
    else
    {
      digitalWrite(pin8, LOW);
    }
  }

3--> PIR sensor

int pirsensor = 0;
void setup()
{
 pinMode(13, OUTPUT);
 pinMode(2, INPUT);
}
void loop()
{
 pirsensor = digitalRead(2);
 if(pirsensor==HIGH)
 {
 digitalWrite(13, HIGH);
 }
 else
 {
 digitalWrite(13, LOW);
 }
 delay(10);
 
}

4-->Temp and humid:-

 float val, voltage, temp;
String ssid = "Simulator Wifi"; 
String password = ""; 
String host = "api.thingspeak.com";
const int httpPort = 80;
String url = "/update?api_key=4J2ET3G2MFQRYELM&field1=";
void setupESP8266(void)
{ 
 Serial.begin(115200); 
 Serial.println("AT"); 
 delay(10); 
 if (Serial.find("OK"))
 Serial.println("ESP8266 OK!!!");
 Serial.println("AT+CWJAP=\"" + ssid + "\",\"" + password + "\"");
 delay(10); 
 if (Serial.find("OK"))
 Serial.println("Connected to WiFi!!!");
 Serial.println("AT+CIPSTART=\"TCP\",\"" + host + "\"," + httpPort);
 delay(50); 
 if (Serial.find("OK")) 
 Serial.println("ESP8266 Connected to server!!!"); 
}
void anydata(void)
{ 
 val=analogRead(A0);
 voltage=val*0.0048828125; 
 temp = (voltage - 0.5) * 100.0; 
 String httpPacket = "GET " + url + String(temp) + " HTTP/1.1\r\nHost: " + host + "\r\n\r\n";
 int length = httpPacket.length(); 
 Serial.print("AT+CIPSEND=");
 Serial.println(length);
 delay(10); 
 Serial.print(httpPacket);
delay(10); 
 if (Serial.find("SEND OK\r\n"))
 Serial.println("ESP8266 sends data to the server"); 
}
void setup()
{
 pinMode(A0, INPUT);
 setupESP8266(); 
}
void loop()
{ 
anydata();
 delay(4000); 
}

10-->Image processing:-


import cv2
import numpy as np
from google.colab.patches import cv2_imshow
img = cv2.imread('jawlines.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5, 5), 0)
edges = cv2.Canny(blur, 50, 150)
kernel = np.ones((5,5),np.uint8)
dilated = cv2.dilate(edges, kernel, iterations = 1)
cv2_imshow(img)
cv2_imshow(gray)
cv2_imshow(blur)
cv2_imshow(edges)
cv2_imshow(dilated)
cv2.waitKey(0)
cv2.destroyAllWindows()

7-->Forward Kinematics:-

import numpy as np

# Define the joint angles in radians
theta1 = np.pi/6
theta2 = np.pi/4
theta3 = np.pi/3
theta4 = np.pi/2

# Define the lengths of the links
L1 = 2
L2 = 1.5
L3 = 1
L4 = 0.5

# Calculate the end effector position
x = L1*np.cos(theta1) + L2*np.cos(theta1+theta2) + L3*np.cos(theta1+theta2+theta3) + L4*np.cos(theta1+theta2+theta3+theta4)
y = L1*np.sin(theta1) + L2*np.sin(theta1+theta2) + L3*np.sin(theta1+theta2+theta3) + L4*np.sin(theta1+theta2+theta3+theta4)

print(f"The end effector position is ({x}, {y})")


8-->Inverse Kinematics:-


import numpy as np

# Define the end effector position
x = 1.5
y = 0.5

# Define the link lengths
L1 = 2
L2 = 1.5

# Calculate the joint angles using inverse kinematics
theta2 = np.arccos((x*2 + y2 - L12 - L2*2) / (2 * L1 * L2))
theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))

print(f"The joint angles are ({theta1}, {theta2}) in radians")

