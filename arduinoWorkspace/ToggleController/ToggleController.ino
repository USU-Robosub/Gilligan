#define buttonPin   0
#define outputPin   2

#define OFF 0
#define ON 1

#define RELEASED 0
#define PUSHED 1

int button = 0;
byte state = 0;
int oldButton = 0;


void checkButton()
{
  int v1 = digitalRead(buttonPin);
  delay(25);
  int v2 = digitalRead(buttonPin);
  
  if (v1 == v1)
  {
    button = v1;
  }
}

void toggleButton()
{
  state = !state;
  
  digitalWrite(outputPin, state);
}

void setup()
{
  pinMode(outputPin, OUTPUT);
  pinMode(buttonPin, INPUT);
}

void loop()
{
  checkButton();
  if (oldButton == OFF && button == ON)
  {
    toggleButton();
  }
  oldButton = button;
}
