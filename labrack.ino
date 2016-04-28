#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>
#include <Wire.h>
#include <eRCaGuy_analogReadXXbit.h>
#include <EEPROM.h>
#include "simple.pb.c"

//defines
#define SLAVE_ADDRESS 0x04
#define FLOATS_TO_SEND 2

float data[FLOATS_TO_SEND] = { 0.00f, 0.00f };

//dmm vars
#define ADC_PRECISION  14
#define ADC_SAMPLING_SIZE 10

//pin assignments
#define CURRENT_PWM_PIN  9
#define VOLTAGE_ADC_PIN  A2
#define CURRENT_ADC_PIN  A1
#define RELAY_CHANNEL_2  7
#define ENTER_PIN  6


//this is the PWM voltage as close as possible to 1000ma
float FULL_SCALE_PWM = 2.84;
//this is the voltage that the senese module should have at 1000ma 
float FULL_SCALE_CUR = 1.962;


float TARGET_CURRENT = 0;
const int UP_PIN = 8;
const int DOWN_PIN = 5;

eRCaGuy_analogReadXXbit adc;


//dmm constants
const float MAX_14_bit = 16368.0;
const float MAX_16_bit = 65472.0;

int inputMagicNumber=0;


/* Configure digital pins 9 and 10 as 16-bit PWM outputs. */
void setupPWM16() {
	DDRB |= _BV(PB1) | _BV(PB2);        /* set pins as outputs */
	TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
		| _BV(WGM11);                   /* mode 14: fast PWM, TOP=ICR1 */
	TCCR1B = _BV(WGM13) | _BV(WGM12)
		| _BV(CS10);                    /* no prescaling */
	ICR1 = 0xffff;                      /* TOP counter value */
}

/* 16-bit version of analogWrite(). Works only on pins 9 and 10. */
void analogWrite16(uint8_t pin, uint16_t val)
{
	switch (pin) {
	case  9: OCR1A = val; break;
	case 10: OCR1B = val; break;
	}
}

void setup() {
	int i = 0;

	//setup pins
	analogReference(EXTERNAL);
	setupPWM16();
	pinMode(UP_PIN, INPUT_PULLUP);
	pinMode(DOWN_PIN, INPUT_PULLUP);
	pinMode(ENTER_PIN, INPUT_PULLUP);
	pinMode(RELAY_CHANNEL_2, OUTPUT);
	
	//setup serial line
	Serial.begin(115200);

	// setup i2c
	Wire.begin(SLAVE_ADDRESS);
	Wire.onReceive(receiveData);
	Wire.onRequest(sendData);

}
 
void loop() {
	//  int loop_cnt = 0;
	float current_reading;
	float target_current = 0.1;	
	float target_voltage = FULL_SCALE_PWM * target_current;
	float delta;

	while (1){		
		//current reading is basically the percentage of the full scale voltage
		analogWrite(CURRENT_PWM_PIN, (MAX_16_bit * target_voltage));
		current_reading = adc.analogReadXXbit(CURRENT_ADC_PIN, ADC_PRECISION, ADC_SAMPLING_SIZE) / MAX_14_bit * 3 / FULL_SCALE_CUR;
		if (current_reading > 0){
			delta = (abs(current_reading / target_current) - 1) * 0.01;		
			if (abs(delta) > 0.000002){
				target_voltage -= delta;
			}
		}		
	}
}


// callback for received data
void receiveData(int byteCount){
	const size_t message_length = 128;
	uint8_t buffer[message_length] = { NULL };
	
	CommandMessage message = CommandMessage_init_zero;
	bool status;
	int i = 0;

	while (Wire.available()) {
		buffer[i] = Wire.read();
		i++;
	}
	/* Create a stream that reads from the buffer. */
	pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);
	////Serial.println("Received: ");

	/* Now we are ready to decode the message. */
	status = pb_decode(&stream, CommandMessage_fields, &message);
	if (!status)
	{
		Serial.print("Decoding failed : ");
		Serial.println(PB_GET_ERROR(&stream));
	}
	else{
		Serial.println(message.verb);
	//	byte data_buffer[sizeof(sensorCommand_t)];
	//	int i = 0;
	//	sensorCommand_t* tmp;

	//	while (Wire.available()) {
	//	data_buffer[i] = Wire.read();
	//	i++;
	//	}
	//	tmp = (sensorCommand_t*)data_buffer;*/
	//	Serial.println("Received: ");
	//	Serial.println(message.lucky_number);
	//	inputMagicNumber = message.lucky_number;
	}
	//todo: clean this up a lot
	//analogWrite(CURRENT_PWM_PIN, (255 * (tmp->value / 3.3)));
}

// callback for sending data
void sendData(){
//	bool status;
//	size_t message_length;
	uint8_t buffer[128];
//	
//	SimpleMessage message = SimpleMessage_init_zero;
//	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
//	message.lucky_number = inputMagicNumber+1;
//	Serial.println("Sending back: ");
//	Serial.println(message.lucky_number);
//	status = pb_encode(&stream, SimpleMessage_fields, &message);
//	message_length = stream.bytes_written;
//	
//	if (!status)
//	{
//		printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));		
//	}
//	
//	{
//		
//	}
//	Wire.write(buffer, message_length);
}
