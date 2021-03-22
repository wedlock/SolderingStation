#include "Arduino.h" // @suppress("Lack of copyright information")
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>

#include <EEPROM.h>
#define ADC_RESOLUTION 10  // set to ADC bit resolution, 10 is default

#define NOTE_F4  349
#define NOTE_A4  440
#define NOTE_C5  523
int tempo = 120;

const int numReadings = 5;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int melody[] = { NOTE_A4, 4, NOTE_A4, 4, NOTE_A4, 4, NOTE_F4, -8, NOTE_C5, 16,
NOTE_A4, 4, NOTE_F4, -8, NOTE_C5, 16, NOTE_A4, 2, };
int notes = sizeof(melody) / sizeof(melody[0]) / 2;
int wholenote = (60000 * 4) / tempo;
int divider = 0, noteDuration = 0;

// Pins define section
#define CLK_PIN  3
#define DATA_PIN 2
#define BUTTON_PIN 5
#define BUZZER_PIN 12
#define PWM_PIN 10
#define ADC_PIN A1          		// set to ADC pin used
#define VREF_PIN A3
#define SW_PIN 11

#define MAX_MENU_ITEMS 7
#define MAX_TIPS 5

LiquidCrystal_I2C lcd(0x27, 20, 4);

double aggKp = 4, aggKi = 0.2, aggKd = 1;
double consKp = 1, consKi = 0.05, consKd = 0.25;
double Setpoint, Input, Output;
float InputTemp;
double spBackup;
int SleepTimer;
int ShutDownTimer;

static uint8_t prevNextCode = 0;
static uint16_t store = 0;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, P_ON_E, DIRECT);

byte bar1[] = { 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00 };
byte bar2[] = { 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00 };
byte bar3[] = { 0x00, 0x00, 0x1C, 0x1C, 0x1C, 0x1C, 0x00, 0x00 };
byte bar4[] = { 0x00, 0x00, 0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00 };
byte bar5[] = { 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x00 };
typedef struct {
	String Name;
	uint16_t Value;
	byte Step;
	uint16_t ValueDefault;
	uint16_t ValueRange[2];
	byte ValueType;  //0 - string, 1 - int ,
	String Prefix[2];
} menuItem;
typedef struct {
	String Name;
	int C_Point[3];
	bool isCalibrated;
	bool isActivated;
} tips;
uint16_t tipsC_Point[3] { 100, 250, 400 };
uint16_t TempRange[2] = { 100, 450 };
String tip_name[MAX_TIPS] = { "K", "BC3", "B2", "D16", "BL" };
tips Tips[MAX_TIPS];
byte SelectedTip = 99;
String MMItems[7] = { { "Sleep" }, { "Shutdown" }, { "Tip Calibrate" }, { "Select Tip" }, { "Buzzer" }, {
		"Boost" }, { "Reset" } };

byte MenuRange[7] = { 2, 1, 1, 1, 1, 1, 1 };

static menuItem SMI1[2] = { { "Sleep time", 5, 1, 5, 0, 5, 1, "Min" }, { "Sleep temp.", 150, 5, 150, 50, 300,
		1, "C" } };

menuItem SMI2[1] = { { "Shutdown time", 20, 1, 20, 0, 20, 1, "Min" } };
menuItem SMI3[1] = { { "", 0, 1, 1, 0, MAX_TIPS - 1, 1, "" } };
menuItem SMI6[1] = { { "", 0, 1, 1, 0, MAX_TIPS - 1, 1, "" } };
menuItem SMI4[1] = { { "Buzzer", 1, 1, 1, 0, 1, 0, "off", "on" } };
menuItem SMI5[1] = { { "Boost temp", 400, 5, 400, 300, 450, 1, "C" } };
menuItem SMI7[1] = { { "RESET press", 0, 1, 0, 1, 0, 0, "Button", "" } };
menuItem *MyMenu[7] = { SMI1, SMI2, SMI3, SMI6, SMI4, SMI5, SMI7 };

int16_t oldPosition = 90;
int16_t enc_position = 0;
int16_t enc_old_position = 99;
int16_t counter = 0;

long unsigned buttonTimer = 0;
int ButtonLastState = 0;
long unsigned longPressTime = 500;
long unsigned buttonDelayTime = 50;
long unsigned lastMillis;
long unsigned lastMillis2;

bool isMenuMode;
bool refreshMenu = true;
bool refreshMenu2 = true;
bool isTipInserted = false;
bool lastTipState;
byte menuIndex = 0;
bool isMainMenu = true;
bool isPropertyMode = false;
bool propertyRefresh = true;
bool isTipMenu = false;
bool isHeatingMode = false;

bool isBoost = false;
bool isSleep = false;
bool isShutDown = false;

boolean buttonActive = false;
boolean longPressActive = false;
boolean shortPress = false;

void setup() {
	myPID.SetMode(AUTOMATIC);
	pinMode(CLK_PIN, INPUT);
	pinMode(DATA_PIN, INPUT);
	//pinMode(PWM_PIN, OUTPUT);
	pinMode(BUTTON_PIN, INPUT_PULLUP);
	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(SW_PIN, INPUT_PULLUP);
	pinMode(VREF_PIN, INPUT);
	Setpoint = 100;
	//pinMode(0, INPUT);
	setPwmFrequency(PWM_PIN, 1);
	lcd.init();                      // initialize the lcd
	lcd.createChar(0, bar1);
	lcd.createChar(1, bar2);
	lcd.createChar(2, bar3);
	lcd.createChar(3, bar4);
	lcd.createChar(4, bar5);
	lcd.backlight();
	Serial.begin(9600);
	Serial.println(F("Start :"));
	isMenuMode = false;
	refreshMenu = true;
	SetTipsName();
	SelectedTip = 99;
	clearReadings();
//	Reset();

//	SaveEEPROM();
	LoadEEPROM();

	lastMillis2 = millis();
	isSleep = false;
	isShutDown = false;
	TimerRst();
	lcd.clear();
	//lcd.print(F("Arduino Controlled"));
	lcd.setCursor(0, 0);
	lcd.print(F("Soldering Iron"));
	lcd.setCursor(0, 2);
	lcd.print(F("Wedlocki@gmail.com"));
	lcd.setCursor(0, 3);
	//lcd.print(F("V1.0"));
//	PlayMusic();
}
void (*resetFunc)(void) = 0; //declare reset function @ address 0
void clearReadings() {
	for (int i = 0; i < numReadings; i++) {
		readings[i] = 0;
	}
	readIndex = 0;
	total = 0;
}
void SetTipsName() {
	for (uint8_t i = 0; i < MAX_TIPS; i++) {
		Tips[i].Name = tip_name[i];
	}
}
float get_voltage(int raw_adc) {
	float refvoltage = (5.0 / 1023.0) * analogRead(VREF_PIN);
	refvoltage = refvoltage - 0.3;
	return raw_adc * (refvoltage / (pow(2, ADC_RESOLUTION) - 1));
}

int get_temperature(float voltage) {
	return (voltage - 1.25) / 0.005;
}
int ReadTemp() {
	SetPower(0);
	delay(20);
	float reading, voltage, temperature;
	reading = analogRead(ADC_PIN);
	voltage = get_voltage(reading);
	temperature = get_temperature(voltage);
	if (temperature < 600) {
		total = total - readings[readIndex];
		readings[readIndex] = temperature;
		total = total + readings[readIndex];
		readIndex = readIndex + 1;
		if (readIndex >= numReadings) {
			readIndex = 0;
		}
		average = total / numReadings;
	} else {
		clearReadings();
		average = -100;
	}
	delay(20);
	SetPower();
	return average;
}
void Reset() {
	clearReadings();
	SetTipsName();
	for (uint8_t i = 0; i < MAX_TIPS; i++) {
		Tips[i].C_Point[0] = -1;
		Tips[i].C_Point[1] = -1;
		Tips[i].C_Point[2] = -1;

		Tips[i].isCalibrated = false;
	}

	for (byte i = 0; i < MAX_MENU_ITEMS; i++) {
		if (MenuRange[i] > 1) {
			for (byte z = 0; z < MenuRange[i]; z++) {
				MyMenu[i][z].Value = MyMenu[i][z].ValueDefault;
			}
		} else {
			MyMenu[i][0].Value = MyMenu[i][0].ValueDefault;
		}
	}
	Setpoint = 100;
	SelectedTip = 99;
	SaveEEPROM();
	MenuPrint(1, 1, 1);

}
/*
 void testEEPROM() {
 for (uint8_t i = 0; i < MAX_TIPS; i++) {
 Tips[i].C_Point[0] = -1;
 Tips[i].C_Point[1] = -1;
 Tips[i].C_Point[2] = -1;
 Tips[i].Name = tip_name[i];
 Tips[i].isCalibrated = false;
 }
 Serial.println("Tips values");
 for (byte i = 0; i < MAX_TIPS; ++i) {
 Serial.print(Tips[i].C_Point[0]);
 Serial.print(" : ");
 Serial.print(Tips[i].C_Point[1]);
 Serial.print(" : ");
 Serial.print(Tips[i].C_Point[2]);
 Serial.print(" : ");
 Serial.print(Tips[i].isActivated);
 Serial.print(" : ");
 Serial.print(Tips[i].isCalibrated);
 Serial.print(" : ");
 Serial.println(Tips[i].Name);
 }
 Serial.println("Menu values");
 for (byte i = 0; i < MAX_MENU_ITEMS; i++) {
 if (MenuRange[i] > 1) {
 for (byte z = 0; z < MenuRange[i]; z++) {
 Serial.print(MyMenu[i][z].Value);
 Serial.print(" : ");
 }
 } else {
 Serial.print(MyMenu[i][0].Value);
 Serial.print(" : ");
 }
 }
 Serial.println("");
 Serial.print("Set Point : ");
 Serial.println(spBackup);
 }
 */

void MenuPrint(int position, int min, int max) {
	if (refreshMenu) {
		refreshMenu = false;
		unsigned int line = 0 + min;
		lcd.clear();
		if (max > 4)
			max = 4;
		for (int i = 0; i < max; i++) {
			lcd.setCursor(0, line + min);
			lcd.print(F(" ["));
			if (isMainMenu) {
				if (position < 3) {
					lcd.print(MMItems[i]);
				} else {
					lcd.print(MMItems[i + (position - 3)]);
				}
			} else {
				if (position < 3) {
					lcd.print(MyMenu[menuIndex][i].Name);
				} else {
					lcd.print(MyMenu[menuIndex][i + (position - 3)].Name);
				}
			}
			lcd.setCursor(18, line + min);
			lcd.print(F("] "));
			line++;
		}
		PrintSelectionLine(position);
	}
}

void PrintSelectionLine(int line) {
	if (line > 3)
		line = 3;
	if (line < 0)
		line = 0;
	lcd.setCursor(0, line);
	lcd.print(char(255));
	lcd.setCursor(19, line);
	lcd.print(char(255));
}

int8_t read_rotary() {
	static int8_t rot_enc_table[] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
	prevNextCode <<= 2;
	if (digitalRead(DATA_PIN))
		prevNextCode |= 0x02;
	if (digitalRead(CLK_PIN))
		prevNextCode |= 0x01;
	prevNextCode &= 0x0f;
	// If valid then store as 16 bit data.
	if (rot_enc_table[prevNextCode]) {
		store <<= 4;
		store |= prevNextCode;
		//if (store==0xd42b) return 1;
		//if (store==0xe817) return -1;
		if ((store & 0xff) == 0x2b) {
			//	Beep(1);
			return 1;
		}
		if ((store & 0xff) == 0x17) {
			//	Beep(1);
			return -1;
		}
	}
	return 0;
}
/// <summary>
/// Reads the encoder.
/// </summary>
/// <param name="rangemin">The rangemin.</param>
/// <param name="rangemax">The rangemax.</param>
/// <param name="step">The step.</param>
/// <returns></returns>
int ReadEncoder(int rangemin, int rangemax, int step) {
	static uint16_t state = 0;
	state = (state << 1) | digitalRead(CLK_PIN) | 0xe000;
	if (state == 0xf000) {
		state = 0x0000;
		if (digitalRead(DATA_PIN)) {
			counter = counter - step;
			//	Beep(1);
		} else {
			counter = counter + step;
			//	Beep(1);
		}
		if (counter - step < rangemin) {
			counter = rangemin;
		}
		if (counter > rangemax) {
			counter = rangemax;
		}
	}
	return counter;
}
//int EncoderState() {
//	int rotation = 0;
//	static uint16_t state = 0;
//	state = (state << 1) | digitalRead(CLK_PIN) | 0xe000;
//	if (state == 0xf000) {
//		state = 0x0000;
//		if (digitalRead(DATA_PIN)) rotation = 1;
//		else rotation = -1;
//	}
//	Serial.println( read_rotary());
//	return rotation;
//}

/// <summary>
/// Reads the button.
/// </summary>
/// <returns></returns>
int ReadButton() {
	int state = 0;  //.no button pressed
	if (digitalRead(BUTTON_PIN) == LOW) {
		if (buttonActive == false) {
			buttonActive = true;
			buttonTimer = millis();
		}
		if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
			longPressActive = true;
			shortPress = false;
			state = -1;
			Beep(2);
			//			Serial.println(F("Long Press"));
		}
	} else {
		if (buttonActive == true) {
			if (longPressActive == true) {
				longPressActive = false;
			} else {
				//			Serial.println("Short Press");
				shortPress = true;
				state = 1;
				Beep(1);
			}
			buttonActive = false;
		}
	}

	return state;
}

/// <summary>
/// asdasd
/// </summary>
/// <param name="position"></param>
void PropertyPrint(int position) {
	if (propertyRefresh) {
		//	Serial.println(position);
		propertyRefresh = false;
		int line = 1;
		lcd.clear();
		lcd.setCursor(0, line);
		lcd.print(F(" ["));
		if (menuIndex == 3 || menuIndex == 2) {

			lcd.print(MMItems[menuIndex]);
		} else {

			lcd.print(MyMenu[menuIndex][position].Name);
		}

		lcd.setCursor(18, line);
		lcd.print(F("] "));
		line++;
		lcd.setCursor(0, line);
		lcd.print(F(">>>"));
		if (menuIndex == 2 || menuIndex == 3) {
			//special property select tip
			if (menuIndex == 3) {
				lcd.setCursor(7, line);
				lcd.print(Tips[position].Name);
				//Serial.println(Tips[position].Name);
				//Serial.println(position);
				lcd.setCursor(12, line);
			}
			//special property tip calibrate
			if (menuIndex == 2) {
				lcd.setCursor(7, line);
				lcd.print(Tips[MyMenu[menuIndex][position].Value].Name);
//				Serial.println(Tips[MyMenu[menuIndex][position].Value].Name);
				lcd.setCursor(12, line);
				if (Tips[MyMenu[menuIndex][position].Value].isCalibrated) {
					lcd.print("+");
				} else {
					lcd.print("-");
				}
			}
		} else {
			if (MyMenu[menuIndex][position].ValueType == 1) {
				if (MyMenu[menuIndex][position].Value > 0) {
					lcd.setCursor(7, line);
					lcd.print(MyMenu[menuIndex][position].Value);
					lcd.setCursor(11, line);
					lcd.print(MyMenu[menuIndex][position].Prefix[0]);
				} else {
					lcd.setCursor(7, line);
					lcd.print(F("off"));
				}
			}
			if (MyMenu[menuIndex][position].ValueType == 0) {
				lcd.setCursor(7, line);
				int value = MyMenu[menuIndex][position].Value;
				lcd.print(MyMenu[menuIndex][position].Prefix[value]);
			}
		}
		lcd.setCursor(17, line);
		lcd.print(F("<<<"));
	}
}

void CalibtareTip(int position) {
	bool iscalibration = true;
	int btn = 0;
	int backup1 = enc_position;
	int backup2 = enc_old_position;
	int backup3 = counter;
	int backup4 = Setpoint;
	Setpoint = 50;
	ResetPID();
	counter = 50;
	lcd.clear();
	lcd.print(F("Calibrate tip : "));
	lcd.print(Tips[position].Name);
	byte cpoint = 0;
	uint16_t temp = 0;
	unsigned long time = 0;
	lcd.setCursor(0, 3);
	lcd.print(F("Target:"));
	lcd.setCursor(8, 3);
	lcd.print(tipsC_Point[cpoint]);
	lcd.print(F(" C "));
	while (iscalibration) {
		Input = ReadTemp();
		CheckTIP();
		if (millis() - time > 300) {
			time = millis();
			PrintTemp(1, Input);
			PrintBar(Input, 1, 0, 255);
			PrintPower(2);
			PrintBar(Output, 2, 0, 255);
			//			Serial.print("Setpoint:");
			//			Serial.println(Setpoint);
		}
		if (isTipInserted) {
			myPID.SetTunings(consKp, consKi, consKd);
			myPID.Compute();
			SetPower();
		}

		btn = ReadButton();
		if (btn == 1) {
			Setpoint = 50;
			counter = 50;
			Tips[position].C_Point[cpoint] = temp;
			cpoint++;
			//Serial.print(cpoint);
			if (cpoint == 3) {
				iscalibration = false;
				lcd.clear();
				Tips[position].isCalibrated = true;
			}
			lcd.setCursor(8, 3);
			lcd.print(tipsC_Point[cpoint]);
			lcd.print(F(" C "));
		}
		Setpoint = ReadEncoder(TempRange[0] - 50, TempRange[1] + 50, 5);
	}
	enc_position = backup1;
	enc_old_position = backup2;
	counter = backup3;
	Setpoint = backup4;
}
void SelectTIP() {

	isMainMenu = true;
	isMenuMode = true;
	SelectedTip = 99;
	int tipsCount = 0;
	int validTips[MAX_TIPS];
	for (int a = 0; a < MAX_TIPS; a++) {
		if (Tips[a].isCalibrated == true) {
//			Serial.print("Znaleziono TIP: ");
//			Serial.println(tipsCount);
			validTips[tipsCount] = a;
			tipsCount++;
		}
	}
	if (tipsCount > 0) {
		bool isSelectionMode = true;
		propertyRefresh = true;
		int btn = 0;
		enc_position = 0;
		counter = 0;
		lcd.clear();

		while (isSelectionMode) {
			PropertyPrint(validTips[enc_position]);
			btn = ReadButton();
			if (btn == 1) {
				isSelectionMode = false;
				SelectedTip = validTips[enc_position];
			}
			enc_position = ReadEncoder(0, tipsCount - 1, 1);
			if (enc_old_position != enc_position) {
				enc_old_position = enc_position;
				propertyRefresh = true;
			}
		}
		isMainMenu = false;			//we selected tip so back to main srcreen
		isMenuMode = false;
	}

	isPropertyMode = false;
	propertyRefresh = false;

	lcd.clear();
	refreshMenu = true;
}
void PlayMusic() {
	for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
		divider = melody[thisNote + 1];
		if (divider > 0) {
			noteDuration = (wholenote) / divider;
		} else if (divider < 0) {
			noteDuration = (wholenote) / abs(divider);
			noteDuration *= 1.5; // increases the duration in half for dotted notes
		}
		tone(BUZZER_PIN, melody[thisNote], noteDuration * 0.9);
		delay(noteDuration);
		noTone(BUZZER_PIN);
	}
}
void Beep(byte a) {
	if (SMI4[0].Value == 1) {
		if (a > 1) {
			for (int b = 0; b < a; ++b) {
				tone(BUZZER_PIN, 5000, 100);
				delay(200);
			}
		} else
			tone(BUZZER_PIN, 5000, 100);
	}
}
void PropertyMenu(int position) {
	int min = MyMenu[menuIndex][position].ValueRange[0];
	int max = MyMenu[menuIndex][position].ValueRange[1];
	int actual = MyMenu[menuIndex][position].Value;
	int btn = 0;
	int backup1 = enc_position;
	int backup2 = enc_old_position;
	int backup3 = counter;
	counter = actual;
	propertyRefresh = true;
	while (isPropertyMode) {
		btn = ReadButton();
		PropertyPrint(position);
		if (btn == 1 && menuIndex == 2) {	//special property. calibrate TIP
			delay(100);
			CalibtareTip(enc_position);
			enc_old_position = 99;
			btn = 0;
		}
		if (menuIndex == 3) {	//special property. select TIP
			delay(100);
			SelectTIP();
			btn = 0;
		}
		if (btn == 1 && menuIndex == MAX_MENU_ITEMS - 1) {	//RESET
			delay(100);
			Reset();
			/*			Serial.println(F("RESET"));
			 Serial.print(F("Selected TIP : "));
			 Serial.println(SelectedTip);
			 Serial.println(F("")); */
			SaveEEPROM();
			delay(200);
			resetFunc();  //call reset
			Beep(3);
		}
		if (btn == -1) {
			delay(100);
			btn = 0;
			if (MenuRange[menuIndex] == 1) {
				isMainMenu = true;
				backup3 = menuIndex;
			}
			propertyRefresh = false;
			isPropertyMode = false;
			lcd.clear();
			refreshMenu = true;
			btn = 0;
		}
		enc_position = ReadEncoder(min, max, MyMenu[menuIndex][position].Step);
		if (enc_old_position != enc_position) {
			enc_old_position = enc_position;
			propertyRefresh = true;
			MyMenu[menuIndex][position].Value = enc_position;
		}
	}
	enc_position = backup1;
	enc_old_position = backup2;
	counter = backup3;
}

void SettingsMenu() {
	isMainMenu = true;
	int max = 0;
	int old_counter = 0;
	while (isMenuMode) {
		Output = 0;
		SetPower();
		int btn = ReadButton();
		if (btn == 1) {
			delay(100);
			btn = 0;							 //short press
			if (!isMainMenu) {							//wejscie w property
				isPropertyMode = true;
			} else {									//wejscie w sub menu
				menuIndex = enc_position;
				old_counter = counter;
				counter = 0;
				if (MenuRange[menuIndex] == 1) {		 //jedno sub menu
//					Serial.println(MyMenu[menuIndex][0].Name);
//					Serial.print("menuIndex : ");
//					Serial.println(menuIndex);
					menuIndex = enc_position;
					old_counter = counter;
					counter = 0;
					isPropertyMode = true;
					propertyRefresh = true;
				}
			}
			refreshMenu = true;
			isMainMenu = false;
		}
		if (btn == -1) {
			btn = 0;
			delay(100);							//long press
			if (isMainMenu) {							//exit from settings
				lcd.clear();
				refreshMenu = false;
				isMenuMode = false;
				isMainMenu = false;
				SaveEEPROM();
				TimerRst();
			} else {				//return from submenu
				counter = old_counter;
				//	menuIndex = 0;
				refreshMenu = true;
				isMainMenu = true;
			}
		}
		if (isMainMenu && !isPropertyMode)
			max = MAX_MENU_ITEMS;				//mainmenu
		else
			max = MenuRange[menuIndex];				//submenu
		enc_position = ReadEncoder(0, max - 1, 1);
		if (enc_old_position != enc_position) {
			enc_old_position = enc_position;
			refreshMenu = true;
		}
		MenuPrint(enc_position, 0, max);
		PropertyMenu(enc_position);
	}
}

void PrintTemp(byte line, int data) {
	lcd.setCursor(0, line);
	int i = (int) data;
	lcd.print(i);
	lcd.print(F("  "));
	lcd.setCursor(4, line);
	lcd.print(F("C"));
}

void PrintPower(byte line) {
	lcd.setCursor(0, line);
	lcd.print(F("Power"));
}

void msValue(int x, int y, String name, int value) {
	lcd.setCursor(x, y);
	lcd.print(name);
	if (value == 0)
		lcd.print(" ");
	if (value == 1)
		lcd.print("*");
}

void PrintBar(int toPrint, int line, int min, int max) {
	if (toPrint > max)
		toPrint = max;
	if (toPrint < min)
		toPrint = min;
	byte kolumny = 13;
	int value = toPrint - min;
	max = max - min;
	byte fullChars = (long) value * kolumny / max;
	byte mod = ((long) value * kolumny * 5 / max) % 5;
	lcd.setCursor(6, line);
	for (int i = 0; i < fullChars; i++) {
		lcd.write(4);
	}
	lcd.write(mod);
	for (int i = fullChars; i < 13; i++) {
		lcd.print(" ");
	}
	//delay(50);
}

void PrintMainScrHeat() {
	if (refreshMenu2 == true) {
		refreshMenu2 = false;
		PrintTemp(0, Input);
		PrintPower(1);
		PrintBar(Input, 0, 0, TempRange[1]);
		PrintBar(Output, 1, 0, 255);
		lcd.setCursor(0, 2);
		lcd.print(F("Tip:"));
		if (isTipInserted)
			lcd.print(tip_name[SelectedTip]);
		else
			lcd.print(F("-   "));
		lcd.setCursor(0, 3);
		if (isBoost) {

			lcd.print(F("BOOST"));

		} else {
			lcd.print("     ");

		}
		lcd.setCursor(12, 2);
		lcd.print(F("->"));
		lcd.print((int) Setpoint);
		lcd.print(F("  "));
	}
}

void TimerRst() {
	SleepTimer = SMI1[0].Value * 60;
	ShutDownTimer = SMI2[0].Value * 60;
}

void Timer() {

	if (millis() - lastMillis2 > 1000) {
		lastMillis2 = millis();

		//every 1 sec make a counter
		if (SMI1[0].Value > 0 && !isSleep) {
			SleepTimer--;
			if (SleepTimer < 31) {
				lcd.setCursor(16, 3);
				lcd.print(F("  "));
				lcd.setCursor(16, 3);
				lcd.print(SleepTimer);
			} else {
				lcd.setCursor(16, 3);
				lcd.print(F("  "));
			}
			if (SleepTimer < 6) {
				Beep(1);
			}
			if (SleepTimer == 0) {
				isSleep = true;
				SleepTimer = SMI1[0].Value * 60;
			}
		}
		if (SMI2[0].Value > 0 && !isShutDown) {
			ShutDownTimer--;
			if (ShutDownTimer == 0) {
				isShutDown = true;
				isSleep = true;
				ShutDownTimer = SMI2[0].Value * 60;
				Beep(3);
			}
		}
	}

	if (digitalRead(SW_PIN)) {
		//Serial.println(F("SW PIN"));
	}

}

void CheckTIP() {

	isTipInserted = true;
	if (SelectedTip > 90) {
		isTipInserted = false;
	}
	if (ReadTemp() < -90) {
		isTipInserted = false;

	}
	if (!isTipInserted) {
		Output = 0;
		Input = 0;
		ResetPID();
		Output = 0;
		SetPower();
	}
}

void SetPower() {
//	analogWrite(PWM_PIN, Output);
	analogWrite(PWM_PIN, Output / 5);
}
void SetPower(byte power) {
	analogWrite(PWM_PIN, power);
}
void ResetPID() {
	double backupSetpoint = Setpoint;
	myPID.SetMode(!myPID.GetMode());
	Setpoint = 1;
	Input = 999;
	Output = 0;
	myPID.SetTunings(aggKp, aggKi, aggKd);
	myPID.Compute();
	SetPower();
	myPID.SetMode(!myPID.GetMode());
	myPID.Compute();
	Setpoint = backupSetpoint;
	Input = 0;
}

void MainScreenHeating() {

	if (millis() - lastMillis > 200) {
		lastMillis = millis();
		refreshMenu2 = true;
		Input = ReadTemp();
		CheckTIP();
	}
	if (lastTipState != isTipInserted) {
		lastTipState = isTipInserted;
		if (isTipInserted) {
			ResetPID();
		}
	}
	Timer();
	int state2 = read_rotary();
	if (state2 == 1) {
		TimerRst();
		if (!isBoost) {
			spBackup = Setpoint;
			isBoost = true;
			refreshMenu2 = true;
			Setpoint = MyMenu[5][0].Value;
			Beep(2);
		}
	}
	if (state2 == -1) {
		TimerRst();
		if (isBoost) {
			isBoost = false;
			refreshMenu2 = true;
			Setpoint = spBackup;
			Beep(2);
		}
	}
	if (isTipInserted) {
		double gap = abs(Setpoint - Input);  //distance away from setpoint
		if (gap < 100) { //we're close to setpoint, use conservative tuning parameters
			myPID.SetTunings(consKp, consKi, consKd);
		} else {
			myPID.SetTunings(aggKp, aggKi, aggKd); //we're far from setpoint, use aggressive tuning parameters
		}
		myPID.Compute();
		SetPower();
	}
	PrintMainScrHeat();
}

void MainScreenNoHeating() {
	bool loop = true;
	lcd.clear();
	lcd.print(F("Temperature:"));
	Input = Setpoint;
	counter = Input;
	unsigned long time = 0;
	Output = 0;
	SetPower();
	while (loop) {
		Input = ReadEncoder(TempRange[0], TempRange[1], 5);
		if (millis() - time > 200) {
			time = millis();
			PrintTemp(2, Input);
			PrintBar(Input, 2, TempRange[0], TempRange[1]);
		}
		int state = ReadButton();
		if (state == 1) {	//short press
			loop = false;
			isHeatingMode = true;
		}
		if (state == -1) {	//long press
			isMenuMode = true;
			menuIndex = 0;
			refreshMenu = true;
			counter = 0;
			loop = false;
		}
	}
	lcd.clear();
	int Setpointbackup = Input;
	Output = 255;
	Input = 0;
	Setpoint = 0;
	for (int i = 0; i < 10; ++i) {	//reset PID
		myPID.Compute();
		SetPower();
	}
	Setpoint = Setpointbackup;
	SaveEEPROM();
}

void LoadEEPROM() {
	byte addr = 0;
	for (int i = 0; i < MAX_TIPS; ++i) {
		Tips[i].C_Point[0] = loadE2(addr);
		Tips[i].C_Point[1] = loadE2(addr);
		Tips[i].C_Point[2] = loadE2(addr);
		Tips[i].isActivated = loadE1(addr);
		Tips[i].isCalibrated = loadE1(addr);
	}
	for (byte i = 0; i < MAX_MENU_ITEMS; i++) {
		if (MenuRange[i] > 1) {
			for (byte z = 0; z < MenuRange[i]; z++) {
				MyMenu[i][z].Value = loadE2(addr);
			}
		} else {
			MyMenu[i][0].Value = loadE2(addr);
		}
	}
	Setpoint = loadE2(addr);
	SleepTimer = SMI1[0].Value;
//	Serial.println(addr);
	SelectedTip = loadE2(addr);
//	Serial.print(F("Load : "));
//	Serial.println(SelectedTip);
}
int loadE2(byte &addr) {
	byte byte1 = EEPROM.read(addr);
	byte byte2 = EEPROM.read(addr + 1);
	EEPROM.read(addr);
	EEPROM.read(addr + 1);
	addr = addr + 2;
	return (byte1 << 8) + byte2;
}
int loadE1(byte &addr) {
	byte byte1 = EEPROM.read(addr);
	addr = addr + 1;
	return byte1;
}

void SaveEEPROM() {
	byte addr = 0;
	for (int i = 0; i < MAX_TIPS; ++i) {
		saveE2(addr, Tips[i].C_Point[0]);
		saveE2(addr, Tips[i].C_Point[1]);
		saveE2(addr, Tips[i].C_Point[2]);
		saveE1(addr, Tips[i].isActivated);
		saveE1(addr, Tips[i].isCalibrated);
	}
	for (byte i = 0; i < MAX_MENU_ITEMS; i++) {
		if (MenuRange[i] > 1) {
			for (byte z = 0; z < MenuRange[i]; z++) {
				int a = MyMenu[i][z].Value;
				saveE2(addr, a);
			}
		} else {
			int a = MyMenu[i][0].Value;
			saveE2(addr, a);
		}
	}
	saveE2(addr, Setpoint);
//	Serial.print(F("addr : "));
//	Serial.println(addr);
	saveE2(addr, SelectedTip);
	delay(100);
//	Serial.print(F("Save : "));
//	Serial.println(SelectedTip);
}
void saveE1(byte &addr, bool data) {
	EEPROM.write(addr, data);
	addr = addr + 1;
}
void saveE2(byte &addr, int data) {
	byte byte1 = data >> 8;
	byte byte2 = data & 0xFF;
	EEPROM.write(addr, byte1);
	EEPROM.write(addr + 1, byte2);
	addr = addr + 2;
}
void loop() {
	int state = ReadButton();
	if (state == -1) {	//long press
		state = 0;
		delay(100);
		isMenuMode = true;
		menuIndex = 0;
		refreshMenu = true;
		counter = 0;
		TimerRst();
	}
	if (state == 1) {   //short press
		state = 0;
		//delay(100);
		isHeatingMode = !isHeatingMode;
		TimerRst();
	}
	if (isHeatingMode) {
		MainScreenHeating();
	} else {
		MainScreenNoHeating();
	}
	SettingsMenu();

	Sleep();
}
/// <summary>
/// Station go to Sleep mode
/// </summary>
void Sleep() {
	double spb;
	if (isBoost)
		spb = spBackup;
	if (!isBoost)
		spb = Setpoint;

	bool makeOnce = true;
	while (isSleep) {
		if (makeOnce) {
			//lcd.clear();
			makeOnce = false;
			isBoost = false;
			lcd.setCursor(14, 3);
			lcd.print(F("SLEEP"));
			//spb = Setpoint;
			Setpoint = SMI1[1].Value;
			Beep(3);
			//lcd.noBacklight();
		}
		Timer();
		int state = ReadButton();
		if (state == 1) {   //short press
			isSleep = false;
			isShutDown = false;
			lcd.setCursor(14, 3);
			lcd.print(F("     "));
			Setpoint = spb;
			TimerRst();
			lcd.backlight();
			Beep(2);
		}
		Input = ReadTemp();
		myPID.SetTunings(2, 5, 1);
		myPID.Compute();
		if (isShutDown) {
			lcd.noBacklight();
			Output = 0;
			Setpoint = 0;
		}
		SetPower();
		if (millis() - lastMillis > 200) {
			lastMillis = millis();
			refreshMenu2 = true;
		}
		PrintMainScrHeat();
	}

}

void setPwmFrequency(int pin, int divisor) {
	byte mode;
	if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
		switch (divisor) {
		case 1:
			mode = 0x01;
			break;
		case 8:
			mode = 0x02;
			break;
		case 64:
			mode = 0x03;
			break;
		case 256:
			mode = 0x04;
			break;
		case 1024:
			mode = 0x05;
			break;
		default:
			return;
		}
		if (pin == 5 || pin == 6) {
			TCCR0B = ((TCCR0B & 0b11111000) | mode);
		} else {
			TCCR1B = ((TCCR1B & 0b11111000) | mode);
		}
	} else if (pin == 3 || pin == 11) {
		switch (divisor) {
		case 1:
			mode = 0x01;
			break;
		case 8:
			mode = 0x02;
			break;
		case 32:
			mode = 0x03;
			break;
		case 64:
			mode = 0x04;
			break;
		case 128:
			mode = 0x05;
			break;
		case 256:
			mode = 0x06;
			break;
		case 1024:
			mode = 0x07;
			break;
		default:
			return;
		}
		TCCR2B = ((TCCR2B & 0b11111000) | mode);
	}
}
