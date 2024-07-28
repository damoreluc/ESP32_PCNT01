#include <Arduino.h>
#include <esp32_pcnt.h>
/*
 * Conteggio encoder incrementale x1 mediante modulo PCNT #0 - solo canale ch0
 *  impiego dei pin:
 *    pin_sig_ch0   sul GPIO_NUM_14
 *    pin_ctrl_ch0  sul GPIO_NUM_12
 *
 * visualizzazione del conteggio up/down su seriale.
 *
 * Documentazione HW:
 * link: https://docs.espressif.com/projects/esp-idf/en/v5.2.2/esp32/api-reference/peripherals/pcnt.html
 *
 * A detailed description of the pulse counter hardware is available in Chapter 17
 * of the Technical Reference Manual :
 *  https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
 *
 * Documentazione wrapper class esp32_pcnt
 * link: https://github.com/mikegofton/ESP32PulseCounter
 *
 * Nota: nella definizione del metodo initialise, viene affermato che:
 *      //only use channel 0 of each counter unit.
 */
// linee di input dell'encoder:
#define pin_sig_ch0 GPIO_NUM_14
#define pin_ctrl_ch0 GPIO_NUM_12
#define pin_led GPIO_NUM_13

// counter threshold
#define threshold_0 10
#define threshold_1 20
#define high_limit 30

// create counter object
PulseCounter pc0;

// gestione interrupt del counter0
volatile bool pc0_int_flag = false; // counter 0 interrupt flag

// mutex for hardware pulse counters
portMUX_TYPE pcntMux0 = portMUX_INITIALIZER_UNLOCKED;

// exit value of Install PCNT ISR service
esp_err_t err_exit_pcnt_isr;

// pulse counter ISR
IRAM_ATTR void pc0_isr(void *);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Conteggio up/down encoder su pin GPIO_14 e pin GPIO_12");
  pinMode(pin_led, OUTPUT);
  pinMode(pin_sig_ch0, INPUT_PULLUP);
  pinMode(pin_ctrl_ch0, INPUT_PULLUP);

  digitalWrite(pin_led, HIGH);
  delay(2000);
  digitalWrite(pin_led, LOW);  

  // Install PCNT ISR service.
  //err_exit_pcnt_isr = pcnt_isr_service_install(ESP_INTR_FLAG_LOWMED);
//  err_exit_pcnt_isr = pcnt_isr_service_install(0);
/*  // parse exit result
  switch(err_exit_pcnt_isr) {
    case ESP_OK:
      Serial.println("PCNT ISR service installed");
      break;
    case ESP_ERR_INVALID_STATE:
      Serial.println("pcnt driver has not been initialized");
      break;  
    case ESP_ERR_NO_MEM:
      Serial.println("No memory to install this service");
      break;   
    case ESP_FAIL:
      Serial.println("Generic esp_err_t code indicating failure");
      break;       
  }
  
  if(err_exit_pcnt_isr != 0) {
    while(1){;}
  }
*/

  // setup hardware pulse counter
  // initialise counter unit 0, channel 0 with signal input GPIO pin and control signal input pin
  // Note: (0 = no control signal input)
  pc0.initialise(pin_sig_ch0, pin_ctrl_ch0);

  // count up on negative edges when ctrl is H,
  // count down on negative edges when ctrl is L,
  // count down on positive edges when ctrl is H,
  // count up on positive edges when ctrl is L
  pc0.set_mode(PCNT_COUNT_DEC, PCNT_COUNT_INC, PCNT_MODE_KEEP, PCNT_MODE_REVERSE);

  // set glich filter to ignore pulses less than 1000 x 2.5ns
  pc0.set_filter_value(1000);

  // set and enable threshold 0 and 1 watch points - these will trigger an interrupt.
  // the event can be disabled and enabled using pc0.event_disable(PCNT_EVT_THRES_0) or pc0.event_enable(PCNT_EVT_THRES_0)
  // thereshold 0 & 1 events do not stop the counter
  pc0.set_event_value(PCNT_EVT_THRES_0, threshold_0);
  pc0.set_event_value(PCNT_EVT_THRES_1, threshold_1);
  pc0.set_event_value(PCNT_EVT_H_LIM, high_limit);

  // register interrupt service routine for this counter unit
  pc0.attach_interrupt(pc0_isr);

  // show isr pointer
  Serial.printf("Pointer to pc0_isr: %p\n", pc0_isr);
  //while(1);

  pc0.interrupt_enable();

  // clear and restart the counter
  pc0.clear();
  pc0.resume();

  Serial.println("Start");
}

void loop()
{
  Serial.printf("Count: %d\t event status: %d\n", pc0.get_value(), pc0.event_status());
  if (pc0_int_flag)
  {
    pc0_int_flag = false;
    Serial.print("pc0 event: ");
    switch (pc0.event_status())
    {
    case evt_thres0:
      Serial.println("Threshold 0");
      break;

    case evt_thres1:
      Serial.println("Threshold 1");
      break;

    case evt_high_lim:
      Serial.println("High Limit");
      break;
    }
  }
  delay(50);
}

// pulse counter ISR
IRAM_ATTR void pc0_isr(void *)
{
  digitalWrite(pin_led, HIGH);
  // Prevent context switching during the interrupt service routine with an ISR spinlock
  portENTER_CRITICAL_ISR(&pcntMux0);
  // set interrupt flag
  pc0_int_flag = true;
  // get events
  portEXIT_CRITICAL_ISR(&pcntMux0);
}