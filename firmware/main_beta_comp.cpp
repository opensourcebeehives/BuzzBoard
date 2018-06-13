// BzBoard Firmware Application - June 2018
// OSBeehives, Inc. Development
// Created by: Peter Naylor, Javier Abors 
//
//
// LOTS of help and reference from Particle.io community:
// ricckas7, ScruffR, peekay123, kennethlimcp, etc.
// 
//
// Libraries can be found here:
// https://github.com/rickkas7/photonAudio
// https://github.com/sparkfun/Battery_Babysitter
//
// TODO:
// 		- Deep sleep
//		- Memory card backup for audio samples
//		- Sleep mode for peripherals


#include "Particle.h"
#include "softap_bb.h"
#include "google-maps-device-locator.h"
#include "ClosedCube_SHT31D.h"
#include "SparkFunBQ27441.h" 
#include "MMA8452-xlero-OSBH.h"
#include "adc_hal.h"
#include "gpio_hal.h"
#include "pinmap_hal.h"
#include "pinmap_impl.h"
#include <vector>
#include "params.h"
#include "downsampler.h"

using namespace std;

const int FW_VERSION = 1;

// System threading is required for this project
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

// Class instances
MMA8452Q xlero;
SHT31D_CC::ClosedCube_SHT31D sht31d;
TCPClient client;
Downsampler d(10);
GoogleMapsDeviceLocator locator;

const unsigned int BATTERY_CAPACITY = 850; // e.g. 850mAh battery
 
// This is the pin connected to the INT1 pin on the MMA8452 chip
const int ACCEL_INT_PIN = D2;

// Various timing constants
const unsigned long MAX_RECORDING_LENGTH_MS = 10000;			// audio sample time
const unsigned long MAX_TIME_TO_PUBLISH_MS = 30000;				// Only stay awake for 20 seconds trying to connect to the cloud and publish
const unsigned long TIME_AFTER_PUBLISH_MS = 1000; 				// After publish, wait 4 seconds for data to go out
const unsigned long TIME_AFTER_BOOT_MS = 1000; 					// At boot, wait 1 seconds before going to publish
const unsigned long TIME_PUBLISH_BATTERY_SEC = 20 * 60;			// every 10 minutes, send an update
const unsigned long TIME_PUBLISH_BATTERY_SEC_LP = 60 * 60; 		// <LP_SLOWDOWN, send update every 15 minutes
const int LP_SLOWDOWN = 30; 									// battery percentage which kicks in lower frequency updates
const int LP_SHUTDOWN_PERCENTAGE = 15;

const unsigned long RSSI_DELAY = 1000;	
const unsigned long RETRY_TIME = 50;							// time to re-try sending TCP packets

// Stuff for the finite state machine
enum State {BOOT_WAIT_STATE, LP_CHECK, PUBLISH_STATE, PUBLISH_WAIT_STATE, AWS_CONNECT, HEADER_WAIT_STATE, AWS_STREAM, AWS_FINISH, SLEEP_STATE, GO_TO_SLEEP, SETUP_LOOP, RSSI_TEST_LOOP_PRE, RSSI_TEST_LOOP};
State state = LP_CHECK;	// {{PETER}} LP_CHECK

// 2048 is a good size for this buffer. This is the number of samples; the number of bytes is twice
// this, but only half the buffer is sent a time, and then each pair of samples is averaged, so
// this results in 1024 byte writes from TCP, which is optimal.
// This uses 4096 bytes of RAM, which is also reasonable.
const size_t SAMPLE_BUF_SIZE = 2048;

// This is the pin the microphone is connected to.
const int SAMPLE_PIN = A0;

// The audio sample rate. The minimum is probably 8000 for minimally acceptable audio quality.
// Not sure what the maximum rate is, but it's pretty high.
const long SAMPLE_RATE = 63000;


// Audio server is running on (AWS)
const char serverAddr[] = "osbeehives.com";
int serverPort = 8080;

//POST headers
const char postHeader1test[] = "POST /write?db=AudioData HTTP/1.1\r\n"
					 "User-Agent: ";

const char postHeader2test[] = "\r\n"
                  	"Host: osbeehives.com:8080\r\n"                       		
                  	"Accept: ";                       		

const char postHeader3test[] = " */*";                       		

const char postHeader4test[] = "fw_";

const char postHeader5test[] = "\r\nConnection: keep-alive\r\n";

const char postHeaderAudiotest[] = "Content-Type: application/x-www-form-urlencoded\r\n"
    	           		"Content-Length: \r\n\r\n";

const char postHeaderStatetest[] = "Content-Type: text/plain\r\n"
						"Content-Length: \r\n\r\n";

char hive_state_msg_test[2] = "0";
char myIDtest[25];

uint16_t samples[SAMPLE_BUF_SIZE];	// recorded samples

//WiFi audio send buffer
const int MAX_BUF = 24;		// number of 1024b TCP packets to buffer
uint8_t **samplez;			// array of audio sample pointers

float x;
uint16_t re_sample;
int8_t re_sample_i8;
int16_t re_sample_i16;
uint16_t sample_num = 0;
char aud_select = 0;
char send_select = 0;

unsigned long recordingStart;
unsigned long publish_time;
unsigned long timer_data;
unsigned long timer_serial = 0;

// T/RH values
float tempC_in;
unsigned int RH_in;

// LiPo values
unsigned int soc;
unsigned int volts;
unsigned int chg;
int current;
int power;
unsigned int soh;
const int LP_FULL_CHARGE = 100;

// main loop vars
uint32_t freemem;
unsigned long stateTime = 0;
unsigned long debug_timer = 0;
unsigned long retry_timer = 0;
bool accel_awake = false;
int qrn = 0;

float longitude;
float latitude;
float accuracy;

bool ota_active = false;
bool bo_once = false;
bool p_once = false;
bool filter_creation = true;
unsigned long pass_count = 0;
unsigned long good_count = 0;
unsigned long error_count = 0;
unsigned long overflow_count = 0;
unsigned long total_count = 0;
int times = 0;
int rssi = 0;
int wifi_timeout = 0;
int wifi_goo = 0;
float wifi_rat = 0;
float aud_rat = 0;
bool setup_once = false;

// ADCDMA - Class to use Photon ADC in DMA Mode
class ADCDMA {
public:
	ADCDMA(int pin, uint16_t *buf, size_t bufSize);
	virtual ~ADCDMA();

	void start(size_t freqHZ);
	void stop();

private:
	int pin;
	uint16_t *buf;
	size_t bufSize;
};

// Helpful post:
// https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=https%3a%2f%2fmy%2est%2ecom%2fpublic%2fSTe2ecommunities%2fmcu%2fLists%2fcortex%5fmx%5fstm32%2fstm32f207%20ADC%2bTIMER%2bDMA%20%20Poor%20Peripheral%20Library%20Examples&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=6249

ADCDMA::ADCDMA(int pin, uint16_t *buf, size_t bufSize) : pin(pin), buf(buf), bufSize(bufSize) {
}

ADCDMA::~ADCDMA() {

}

void ADCDMA::start(size_t freqHZ) {

    // Using Dual ADC Regular Simultaneous DMA Mode 1

	// Using Timer3. To change timers, make sure you edit all of:
	// RCC_APB1Periph_TIM3, TIM3, ADC_ExternalTrigConv_T3_TRGO

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Set the pin as analog input
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    HAL_Pin_Mode(pin, AN_INPUT);

	// Enable the DMA Stream IRQ Channel
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// 60000000UL = 60 MHz Timer Clock = HCLK / 2
	// Even low audio rates like 8000 Hz will fit in a 16-bit counter with no prescaler (period = 7500)
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (60000000UL / freqHZ) - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T3_TRGO
	TIM_Cmd(TIM3, ENABLE);

	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	// DMA2 Stream0 channel0 configuration
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buf;
	DMA_InitStructure.DMA_PeripheralBaseAddr =  0x40012308; // CDR_ADDRESS; Packed ADC1, ADC2;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = bufSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	// Don't enable DMA Stream Half / Transfer Complete interrupt
	// Since we want to write out of loop anyway, there's no real advantage to using the interrupt, and as
	// far as I can tell, you can't set the interrupt handler for DMA2_Stream0 without modifying
	// system firmware because there's no built-in handler for it.
	// DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);

	DMA_Cmd(DMA2_Stream0, ENABLE);

	// ADC Common Init
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// ADC1 configuration
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	// ADC2 configuration - same
	ADC_Init(ADC2, &ADC_InitStructure);

	//
	ADC_RegularChannelConfig(ADC1, PIN_MAP[pin].adc_channel, 1, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC2, PIN_MAP[pin].adc_channel, 1, ADC_SampleTime_15Cycles);

	// Enable DMA request after last transfer (Multi-ADC mode)
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	// Enable ADCs
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);

	ADC_SoftwareStartConv(ADC1);
}

void ADCDMA::stop() {
	// Stop the ADC
	ADC_Cmd(ADC1, DISABLE);
	ADC_Cmd(ADC2, DISABLE);

	DMA_Cmd(DMA2_Stream0, DISABLE);

	// Stop the timer
	TIM_Cmd(TIM3, DISABLE);
}

ADCDMA adcDMA(SAMPLE_PIN, samples, SAMPLE_BUF_SIZE);

// End ADCDMA


void locationCallback(float lat, float lon, float acc){
	latitude = lat;
	longitude = lon;
	accuracy = acc;
	Serial.printlnf("lat=%f lon=%f acc=%f", lat, lon, acc);
}

/** Performs majority voting on stored beehive states
*/
int majorityVoting(vector<int> states)
{
	//Count all states
	std::array<int,11> count={0,0,0,0,0,0,0,0,0,0,0};
	for( int i = 0; i < states.size(); i++ )
	{	
		count[states[i]]++;
	}

	//Determine most repeated state
	int indexWinner = 1;
	for( int i = 1; i < count.size(); i++ )
	{
		if( count[i] > count[indexWinner] )
		{
			indexWinner = i;
		}
	}
	return indexWinner;
}

void printResult(String text, SHT31D_CC::SHT31D result) {
	if (result.error == SHT31D_CC::NO_ERROR) {
		Serial.print(text);
		Serial.print(": T=");
		Serial.print(result.t);
		Serial.print("C, RH=");
		Serial.print(result.rh);
		Serial.println("%");
		tempC_in = result.t;
		RH_in = result.rh;
	}
	else {
		Serial.print(text);
		Serial.print(": [ERROR] Code #");
		Serial.println(result.error);
	}
}


void update_params(void){
		Serial.println("update params...");
		printResult("SHT31", sht31d.readTempAndHumidity(SHT31D_CC::REPEATABILITY_LOW, SHT31D_CC::MODE_CLOCK_STRETCH, 50));

		// BQ27441 Battery Manager calls
		soc = lipo.soc(); // Read state-of-charge (in %)
		volts = lipo.voltage(); // Read voltage (in mV)
		current = lipo.current(AVG); // Read average current (in mA)
		power = lipo.power(); // Read power consumption (in mW)
		soh = lipo.soh(); // Read state-of-health (in %)

}

void free_mem(void){
	freemem = System.freeMemory();
	Serial.print("  -----  free memory: ");
	Serial.println(freemem);
}

void buff_in_mem(){
//uint16_t samplez[MAX_BUF][SAMPLE_BUF_SIZE/4];		// MAX_BUF number of samples
 	samplez = (uint8_t **) malloc (sizeof(uint8_t *)*MAX_BUF);
	Serial.printlnf("size: %i", sizeof(uint8_t *)*MAX_BUF);
 	for (int r = 0; r < MAX_BUF; r++) {
 		samplez[r] = (uint8_t *) malloc(sizeof(uint8_t)*SAMPLE_BUF_SIZE/2);
 		Serial.printlnf("size %i: %i", r, sizeof(uint8_t)*SAMPLE_BUF_SIZE/2);
	}
}

void buff_out_mem(){
	for (int r = 0; r < MAX_BUF; r++) {
		free(samplez[r]);
		Serial.printlnf("row: %i", r);
		free_mem();
	}
	free(samplez);
}

void otaHandler(){
	Serial.println("ota update");
	stateTime = millis();
	state = SETUP_LOOP;
	ota_active = true;
	Particle.process();
}

void button_click_handler(system_event_t event, int param)
{
	times = times + system_button_clicks(param);
    if(times%2){
    	Serial.println("===test mode===");
	    state = RSSI_TEST_LOOP_PRE;
    }
    else{
       	Serial.println("===run mode===");
	    state = BOOT_WAIT_STATE;
    }
}

void rssi_report(){
	rssi = WiFi.RSSI();
	wifi_rat = (float)(wifi_goo + 1) / (wifi_timeout + wifi_goo + 1);
	aud_rat = (float)(good_count + 1) / (total_count + 1);	
	Serial.printlnf("good: %i total: %i", good_count, total_count);
	Serial.printlnf("RSSI: %i, NG/OK: %i/%i, wifi_rat: %.2f, A_NG/OK: %i/%i, aud_rat: %.2f", rssi, wifi_timeout, wifi_goo, wifi_rat, (total_count - good_count), total_count, aud_rat );

}

void batt_report(){
	Serial.print("State of Charge/Health: ");
	Serial.print(soc);
	Serial.print(", ");
	Serial.println(soh);
	int16_t l = -600, h = -400;
	for(int j = 0; j <= 5 ; j++){
		if((power >= l && power < h) || j ==5){
			chg = j;
			break;
		}
		l = l + 200;
		h = l + 200;
	}
	Serial.printlnf("chg: %i, mA: %i ,mW: %i", chg, current, power);
}

bool WiFi_Listen_Check(){						// checks for WiFi listening mode on Photon
		if(WiFi.listening() && !setup_once){	
			state = SETUP_LOOP;
			stateTime = millis();
			Serial.println("SETUP LOOP");
			setup_once = true;
//			buff_out_mem();
			return true;
		}
		else return false;
}

int8_t ALaw_Encode(int16_t number)
{
   const uint16_t ALAW_MAX = 0xFFF;
   uint16_t mask = 0x800;
   uint8_t sign = 0;
   uint8_t position = 11;
   uint8_t lsb = 0;
   if (number < 0)
   {
      number = -number;
      sign = 0x80;
   }
   if (number > ALAW_MAX)
   {
      number = ALAW_MAX;
   }
   for (; ((number & mask) != mask && position >= 5); mask >>= 1, position--);
   lsb = (number >> ((position == 4) ? (1) : (position - 4))) & 0x0f;
   return (sign | ((position - 4) << 4) | lsb) ^ 0x55;
}

ApplicationWatchdog wd(30000, System.reset, 256);		// declare watchdog


void setup() {

	Serial.begin(57600);
	WiFi.useDynamicIP();
	delay(1000);

	Serial.printlnf("System firmware version: %s", System.version().c_str());
	Serial.printlnf("BuzzBoard FW version: %i", FW_VERSION);
	Serial.println("Setup");
	free_mem();

	String myID;
	myID = System.deviceID();
	myID.toCharArray(myIDtest, sizeof(myIDtest));

	Serial.printlnf("DeviceID: %s", myIDtest);

	WiFi.selectAntenna(ANT_EXTERNAL);

	sht31d.begin(0x44);
	Serial.print("SHT31D Serial #");
	Serial.println(sht31d.readSerialNumber());

	// Initialization for the battery monitor
	if (!lipo.begin())
	{
    	Serial.println("Error: Unable to communicate with BQ27441.");
    	Serial.println("Check wiring and try again.");
    	Serial.println("(Battery must be plugged into Battery Babysitter!)");
	}

	Serial.println("Connected to BQ27441!");
	lipo.setCapacity(BATTERY_CAPACITY); // Configure BQ27441 to assume a 1000 mAh battery

	// Initialization for accelerometer
	Serial.println("MMA xlerometer init");
	xlero.init(SCALE_2G, ODR_12);			// 2g scale, sampled at 12 Hz

	pinMode(D7, OUTPUT);
	free_mem();
	System.on(firmware_update_pending, otaHandler);
	System.on(button_click, button_click_handler);
	
	Serial.println("Setup COMPLETE");
}


void loop() {

	uint16_t *sendBuf = NULL;

	if(millis() >= debug_timer){
		if(state != 5){
			Serial.printlnf("State: %d", state);
		}
		wd.checkin();						// reset watchdog timer
		Particle.process();
		WiFi_Listen_Check();
		free_mem();
		locator.loop();
		debug_timer = millis() + 1000;		/// try this
		if(xlero.readTap() != 0){
			Serial.println("xlrometer disturbance~!");
		}
	}


	switch(state) {

	case BOOT_WAIT_STATE:
		if (millis() - stateTime >= TIME_AFTER_BOOT_MS) {
			// To publish the battery stats after boot, set state to PUBLISH_STATE
			// To go to sleep immediately, set state to SLEEP_STATE
			Particle.connect();
			p_once = false;
			state = PUBLISH_STATE;
		}
		break;

	case LP_CHECK:		// Li-Ion battery check
		delay(250);

		// check to see if reason for waking was xlrometer disturbance, or timer
		if(xlero.readTap() != 0){
			Serial.println("xlrometer disturbance~!");
			accel_awake = true;
		}
		else{
			accel_awake = false;
		}

		update_params();
		if(soc < LP_SHUTDOWN_PERCENTAGE || soc > LP_FULL_CHARGE){	
			Serial.println("Low Battery, system will shutdown until charged");
			batt_report();
//			WiFi.off();
			state = SLEEP_STATE;		// go to sleep
		}
		if(soc >= LP_SHUTDOWN_PERCENTAGE && soc <= LP_FULL_CHARGE){
			state = BOOT_WAIT_STATE;				// continue normal operation
			WiFi.useDynamicIP();
			WiFi.on();
		}
		if(soc < LP_SLOWDOWN){								// set low-power update time
			publish_time = TIME_PUBLISH_BATTERY_SEC_LP;
		}
		if(soc >= LP_SLOWDOWN && soc <= LP_FULL_CHARGE){		// set normal update time
			publish_time = TIME_PUBLISH_BATTERY_SEC;
		}

		break;

	case PUBLISH_STATE:

		if (!p_once) {
		// BME280 reads
			update_params();
//			free_mem();
			rssi_report();
			batt_report();

		// Debug printouts
			Serial.println("SHT30: ");
			Serial.print("temp (C): ");
			Serial.print(tempC_in);
			Serial.print("RH (%): ");
			Serial.println(RH_in);

			p_once = true;
		}
		
		if(WiFi_Listen_Check())	break;

		if (Particle.connected()) {

			locator.withEventName("geoLocate").withSubscribe(locationCallback);
			locator.publishLocation();
//			delay(500);

			rssi_report();

			if(soc >= LP_SHUTDOWN_PERCENTAGE && soc < LP_SLOWDOWN){
				Particle.publish("Low Battery Warning");
				Serial.println("Low Battery Warning");
			}

			Serial.println("publish");
			if(accel_awake){
				Particle.publish("disturbance");	
			}

			update_params();
			batt_report();

			char data[512];
			snprintf(data, sizeof(data), "{\"T_in\": %.01f, \"RH_i\": %d, \"soc\": %d, \"soh\": %d, \"chg\": %d, \"I\": %d, \"P\": %d, \"RSSI\": %i, \"w_r\": %.2f, \"a_r\": %.2f, \"acc\": %f, \"lon\": %f, \"lat\": %f}", tempC_in, RH_in, soc, soh, chg, current, power, rssi, wifi_rat, aud_rat, accuracy, longitude, latitude);
			Particle.publish("measurements", data, PRIVATE);

			// Wait for the publish to go out
			stateTime = millis();
			wifi_goo++;
			state = PUBLISH_WAIT_STATE;
		}
		else {
			// Haven't come online yet
			if (millis() - stateTime >= MAX_TIME_TO_PUBLISH_MS) {
				// Took too long to publish, just go to sleep
				state = SLEEP_STATE;  // {{PETER}} SLEEP_STATE
				wifi_timeout++;
			}
		}
		break;

	case PUBLISH_WAIT_STATE:
		if (millis() - stateTime >= TIME_AFTER_PUBLISH_MS) {
			state = AWS_CONNECT;		//AWS_CONNECT
		}
		break;

	case AWS_CONNECT:
		// Ready to connect to the server via TCP
		if (client.connect(serverAddr, serverPort)) {
			// Connected

			Serial.println("stream starting");
//			Serial.println("packet #,ok?,good,re-try,failed");
			free_mem();

			char influxDbMessage[256];
			snprintf(influxDbMessage, sizeof(influxDbMessage), "%s%s%s%s%d%s%s", postHeader1test, myIDtest, postHeader2test, postHeader4test, FW_VERSION, postHeader5test, postHeaderAudiotest);

			int count = client.write(influxDbMessage);	

			if(count == strlen(influxDbMessage)){
				Serial.println(influxDbMessage);
			}
			free_mem();

			buff_in_mem();					// aalocate audio buffer in memory
			free_mem();

			Serial.printlnf("sizeof samplez: %i, %i", sizeof(samplez), sizeof(samplez[0]));

			adcDMA.start(SAMPLE_RATE);
			recordingStart = millis();
			digitalWrite(D7, HIGH);
			stateTime = millis();
			state = AWS_STREAM;
		}
		else {
			Serial.println("failed to connect to server");
			delay(500);
			state = SLEEP_STATE;	//{{PETER}} SLEEP_STATE, AWS_CONNECT
		}
		break;

	case HEADER_WAIT_STATE:
		if (millis() - stateTime >= TIME_AFTER_PUBLISH_MS) {
			adcDMA.start(SAMPLE_RATE);
			recordingStart = millis();
			digitalWrite(D7, HIGH);
			state = AWS_STREAM;		//
		}
		break;


	case AWS_STREAM:
		bool success;

		if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_HTIF0)) {			// first half
		    DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_HTIF0);
		    sendBuf = samples;
		}
		if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_TCIF0)) {			// second half
		    DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);
		    sendBuf = &samples[SAMPLE_BUF_SIZE / 2];
		}

		if (sendBuf != NULL) {			// process 32kHz samples pass to downsampler
			// There is a sample buffer to send
			// Average the pairs of samples and format them
			for(size_t ii = 0, jj = 0; ii < SAMPLE_BUF_SIZE / 2; ii += 2, jj++) {
				uint32_t sum = (uint32_t)sendBuf[ii] + (uint32_t)sendBuf[ii + 1];
				sendBuf[jj] = (uint16_t)(sum / 2);

				x = (((float)sendBuf[jj] - 32768)/65536);
				d.update(x);

				if(d.isReady()){				// downsample ready
					x = d.getSample();
					re_sample_i16 = (int16_t)(x * 65536);		// convert to uint16 for audio stream
					re_sample_i8 = ALaw_Encode(re_sample_i16);
					samplez[aud_select][sample_num] = re_sample_i8;		//fill send-stream buffer
					sample_num++;

					if(sample_num >= (SAMPLE_BUF_SIZE / 2)){		// send-stream sample ready /// /2
						total_count++;					// total number of TCP buffers filled
						sample_num = 0;
						aud_select++;
						retry_timer = 0;
						bo_once = false;
						if(aud_select > MAX_BUF - 1)	aud_select = 0;
					}

				}
			}
		}


		if(aud_select != send_select && millis() >= retry_timer){				// send sample packet

					Serial.println("-----");
					Serial.printlnf("aud_/send_select: %i / %i, millis(%lu)", aud_select, send_select, millis());
					//Send buffer to server
					int count = client.write((uint8_t *)samplez[send_select], SAMPLE_BUF_SIZE / 2);

					if (count == SAMPLE_BUF_SIZE / 2) {
					// Success
						good_count++;
						success = true;

						send_select++;						
						if(send_select > MAX_BUF - 1)	send_select = 0;

						Serial.printlnf("success! %d, %d", good_count, count);
						}

					else
					if (count == -16) {
					// TCP Buffer full
						error_count++;
						success = false;
						retry_timer = millis() + RETRY_TIME;
						Serial.printlnf("TCP buffer full");
					}
					else {
					// Error
						Serial.printlnf("error writing %d", count);
						pass_count = 0;

						adcDMA.stop();
						digitalWrite(D7, LOW);
						client.stop();			

						accel_awake = false;
						stateTime = millis();

						state = AWS_FINISH;
					}
				}
					if((aud_select == send_select) && !success && !bo_once){						// error, aud_buffer has looped around and reached send_buffer
						Serial.printlnf("BUFFER OVERWRITE: aud: %i, send: %i", aud_select, send_select);
						bo_once = true;
						overflow_count++;
					}

		if (millis() - recordingStart >= MAX_RECORDING_LENGTH_MS) {		// recording time reached
			state = AWS_FINISH;
		}
		break;

	case AWS_FINISH:
			adcDMA.stop();
			buff_out_mem();
			digitalWrite(D7, LOW);
			Serial.println("aws stopping");
			Serial.println("testnum entries ; good/fails/overflows (x16) // total packets:  ");
			Serial.printlnf("%lu ; %d/%d/%d // %d", pass_count, good_count, error_count, overflow_count, total_count); 

			client.stop();

			//If feature extractor is ready
		if (client.connect(serverAddr, serverPort)) {
			// Connected

			Serial.println("OBC send");

			char influxDbMessage[256];
			snprintf(influxDbMessage, sizeof(influxDbMessage), "%s%s%s%s%s%s%s\r\n\r\n", postHeader1test, myIDtest, postHeader2test, postHeader3test, postHeader5test, postHeaderStatetest, hive_state_msg_test);

			int count = client.write(influxDbMessage);	
			if(count == strlen(influxDbMessage)){
				Serial.println(influxDbMessage);
			}
			client.stop();			
		}
		else {
			Serial.println("failed to connect to server");
			state = SLEEP_STATE;
		}

			//print On-Board-Classification State
			state = SLEEP_STATE;
		break;

	case SLEEP_STATE:				// Sleep

		Serial.print("Go to sleep...");

		pass_count = 0;
		aud_select = 0;
		send_select = 0;

		if (WiFi_Listen_Check())	break;

		state = GO_TO_SLEEP;
		break;


	case GO_TO_SLEEP:

		Serial.println(" zzz");
		xlero.readTap();
		Particle.disconnect();
		WiFi.off();
		System.sleep(ACCEL_INT_PIN, RISING, publish_time);		// slow to sleep after waking, only when no wifi connection 
		state = LP_CHECK;
		stateTime = millis();
		break;

	case SETUP_LOOP:
		if (millis() - stateTime >= 300000) {
			state = GO_TO_SLEEP;		//AWS_CONNECT
		}
		if(!WiFi.listening() && !ota_active){
			stateTime = millis();
			state = BOOT_WAIT_STATE;
			setup_once = false;
		}

		break;


	case RSSI_TEST_LOOP_PRE:

		pass_count = 0;
		adcDMA.stop();
		digitalWrite(D7, LOW);
		client.stop();			

		accel_awake = false;

		state = RSSI_TEST_LOOP;
		break;


	case RSSI_TEST_LOOP:
		if(millis() - stateTime >= RSSI_DELAY){

			rssi_report();
			update_params();
			batt_report();

			stateTime = millis();
		}
		break;

	}

}




// all samples working!!
