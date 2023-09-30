//=====[Libraries]=============================================================

#include "mbed.h"
#include "arm_book_lib.h"
#include <string>
//=====[Defines]===============================================================

#define NUMBER_OF_KEYS                           4
#define BLINKING_TIME_GAS_ALARM               1000
#define BLINKING_TIME_OVER_TEMP_ALARM          500
#define BLINKING_TIME_GAS_AND_OVER_TEMP_ALARM  100
#define NUMBER_OF_AVG_SAMPLES                   100
#define OVER_TEMP_LEVEL                         50
#define TIME_INCREMENT_MS                       10
#define MODIFICACIONES_BUSIN_BUSOUT
#define ALARM_TEST_BUTTON D2
#define A_BUTTON D4
#define B_BUTTON D5
#define C_BUTTON D6
#define D_BUTTON D7
#define MQ2 PE_12
#define SIREN PE_10

#define ALARM_TEST_BUTTON_POSITION 0
#define A_BUTTON_POSITION 1
#define B_BUTTON_POSITION 2
#define C_BUTTON_POSITION 3
#define D_BUTTON_POSITION 4
#define MQ2_POSITION 5
#define SIREN_POSITION 0

#define ALARM_LED LED1
#define SYSTEM_BLOCKED_LED LED2
#define INCORRECT_CODE_LED LED3
#define ALARM_LED_POSITION 0
#define SYSTEM_BLOCKED_LED_POSITION 1
#define INCORRECT_CODE_LED_POSITION 2

#ifdef ORIGINAL_DIGITALIN_DIGITALOUT
//=====[Declaration and initialization of public global objects]===============

DigitalIn enterButton(BUTTON1);
DigitalIn alarmTestButton(D2);
DigitalIn aButton(D4);
DigitalIn bButton(D5);
DigitalIn cButton(D6);
DigitalIn dButton(D7);
DigitalIn mq2(PE_12);

DigitalOut alarmLed(LED1);
DigitalOut incorrectCodeLed(LED3);
DigitalOut systemBlockedLed(LED2);

DigitalInOut sirenPin(PE_10);
#endif

#ifdef MODIFICACIONES_BUSIN_BUSOUT
//=====[Declaration and initialization of public global objects]===============

BusIn inputs(ALARM_TEST_BUTTON, A_BUTTON, B_BUTTON, C_BUTTON, D_BUTTON, MQ2);
BusIn enterButton(BUTTON1);
BusOut leds(ALARM_LED, SYSTEM_BLOCKED_LED, INCORRECT_CODE_LED);

BusInOut sirenPin(SIREN);
#endif

/*
====================Clase UnbufferedSerial.h (resolución 6.c.i)====================
Objeto: uartUsb (instancia de UnbufferedSerial)
Métodos utilizados: -Constructor: Recibe como parámetro el pin TX (PD_8 - obtenido en PinNames.h), el pin RX (PD_9 - obtenido en PinNames.h) y el baudrate
                                  (por defecto vale MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE = 9600)
                    -readable(): Determina si hay un caracter valido para leer. Si lo hay devuelve 1/true;
                    -read(void *buffer, size_t size): lee la cantidad de bytes size que hay en buffer
                    -write(const void *buffer, size_t size): escribe desde buffer la cantidad de bytes size.
Clase: UnbufferedSerial
Herencia: UnbufferedSerial hereda de SerialBase, FileHandle y NonCopyable<UnbufferedSerial>
Interfaces: FileHandle posee un método virtual puro, por lo que esa clase sería una interfaz de UnbufferedSerial
Contructor: UnbufferedSerial(PinName tx, PinName rx, int baud = MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE);
Sobrecarga: Constructor tiene sobrecarga (Hay 3 constructores):
            -UnbufferedSerial(PinName tx, PinName rx, int baud = MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE);
            -UnbufferedSerial(const serial_pinmap_t &static_pinmap, int baud = MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE);
*/
UnbufferedSerial uartUsb(USBTX, USBRX, 115200);

/*
====================Clase AnalogIn (resolución 6.b.i)====================
Objeto: potentiometer, lm35 (son instancias de AnalogIn)
Métodos utilizados: -Constructor: Recibe como parámetro el pin que desea asociar a la instancia.
                    -read(): Lee una tensión a la entrada del pin. Devuelve un float que varía entre 0 y 1.
                             Siendo 0.0 0V y 1.0 3.3V (sería un porcentaje de la tensión de referencia del ADC).
Clase: AnalogIn
Herencia: No hay. Ni otras clases heredan de ella ni ella hereda de otras clases.
Interfaces: No aplica.
Contructor: AnalogIn(PinName pin, float vref = MBED_CONF_TARGET_DEFAULT_ADC_VREF); Recibe el pin que se desea definir como entrada analógica,
            y además una tensión de referencia. En este caso se usa la definida por defecto (MBED_CONF_TARGET_DEFAULT_ADC_VREF==NaN).
Sobrecarga: Constructor tiene sobrecarga (Hay 3 constructores):
            -AnalogIn(const PinMap &pinmap, float vref = MBED_CONF_TARGET_DEFAULT_ADC_VREF);
            -AnalogIn(const PinMap &&, float vref = MBED_CONF_TARGET_DEFAULT_ADC_VREF) = delete;
            -AnalogIn(PinName pin, float vref = MBED_CONF_TARGET_DEFAULT_ADC_VREF);
            operator float(): Se puede usar como un shortcut para read(). Ej: (potentiometer.read() > 0.5) = (potentiometer > 0.5)
*/
AnalogIn potentiometer(A0);    
AnalogIn lm35(A1);

//=====[Declaration and initialization of public global variables]=============

bool alarmState    = OFF;
bool incorrectCode = false;
bool overTempDetector = OFF;

int numberOfIncorrectCodes = 0;
int buttonBeingCompared    = 0;
int codeSequence[NUMBER_OF_KEYS]   = { 1, 1, 0, 0 };
int buttonsPressed[NUMBER_OF_KEYS] = { 0, 0, 0, 0 };
int accumulatedTimeAlarm = 0;

bool gasDetectorState          = OFF;
bool overTempDetectorState     = OFF;

float potentiometerReading = 0.0;
float lm35ReadingsAverage  = 0.0;
float lm35ReadingsSum      = 0.0;
float lm35ReadingsArray[NUMBER_OF_AVG_SAMPLES];
float lm35TempC            = 0.0;

//=====[Declarations (prototypes) of public functions]=========================

void inputsInit();
void outputsInit();

void alarmActivationUpdate();
void alarmDeactivationUpdate();

void uartTask();
void availableCommands();
bool areEqual();
float celsiusToFahrenheit( float tempInCelsiusDegrees );
float analogReadingScaledWithTheLM35Formula( float analogReading );

//=====[Main function, the program entry point after power on or reset]========

int main()
{
    inputsInit();       //inicializa las entradas (algunas como pulldown, opendrain)
    outputsInit();      //inicializa en estado bajo las salidas
    while (true) {
        alarmActivationUpdate();    //realiza una lectura de temperatura y detector de gas y dispara la alarma si supera los limites definidos.
                                    //apretando el test button también se dispara la alarma.
        alarmDeactivationUpdate();  //ingresando el codigo correcto desactiva la alarma, si ingresa 5 veces mal el codigo se bloquea
        uartTask();                 //Ejecuta el menú de la aplicación, mediante una interfaz por puerto serie realizando distintas funcionalidades
        delay(TIME_INCREMENT_MS);
    }
}

//=====[Implementations of public functions]===================================
void inputsInit()
{
#ifdef ORIGINAL_DIGITALIN_DIGITALOUT
    alarmTestButton.mode(PullDown);
    aButton.mode(PullDown);
    bButton.mode(PullDown);
    cButton.mode(PullDown);
    dButton.mode(PullDown);
    sirenPin.mode(OpenDrain);
    mq2.mode(PullDown); //Lo agregamos ya que no tenemos el divisor resistivo y estamos usando un switch.
    sirenPin.input();
#endif

#ifdef MODIFICACIONES_BUSIN_BUSOUT
    inputs.mode(PullDown);
    sirenPin.mode(OpenDrain);
    sirenPin.input();
#endif
}

void outputsInit()
{
#ifdef ORIGINAL_DIGITALIN_DIGITALOUT
    alarmLed = OFF;
    incorrectCodeLed = OFF;
    systemBlockedLed = OFF;
#endif

#ifdef MODIFICACIONES_BUSIN_BUSOUT
    leds.write(OFF);
#endif
}


void alarmActivationUpdate()
{
    static int lm35SampleIndex = 0;
    int i = 0;

    lm35ReadingsArray[lm35SampleIndex] = lm35.read();
    lm35SampleIndex++;
    if ( lm35SampleIndex >= NUMBER_OF_AVG_SAMPLES) {
        lm35SampleIndex = 0;
    }
    
       lm35ReadingsSum = 0.0;
    for (i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsSum = lm35ReadingsSum + lm35ReadingsArray[i];
    }
    lm35ReadingsAverage = lm35ReadingsSum / NUMBER_OF_AVG_SAMPLES;
       lm35TempC = analogReadingScaledWithTheLM35Formula ( lm35ReadingsAverage );    
    
    if ( lm35TempC > OVER_TEMP_LEVEL ) {
        overTempDetector = ON;
    } else {
        overTempDetector = OFF;
    }
#ifdef ORIGINAL_DIGITALIN_DIGITALOUT
    if( !mq2) {
        gasDetectorState = ON;
        alarmState = ON;
    }
    if( overTempDetector ) {
        overTempDetectorState = ON;
        alarmState = ON;
    }
    if( alarmTestButton ) {             
        overTempDetectorState = ON;
        gasDetectorState = ON;
        alarmState = ON;
    }    
    if( alarmState ) { 
        accumulatedTimeAlarm = accumulatedTimeAlarm + TIME_INCREMENT_MS;
        sirenPin.output();                                     
        sirenPin = LOW;                                        
    
        if( gasDetectorState && overTempDetectorState ) {
            if( accumulatedTimeAlarm >= BLINKING_TIME_GAS_AND_OVER_TEMP_ALARM ) {
                accumulatedTimeAlarm = 0;
                alarmLed = !alarmLed;
            }
        } else if( gasDetectorState ) {
            if( accumulatedTimeAlarm >= BLINKING_TIME_GAS_ALARM ) {
                accumulatedTimeAlarm = 0;
                alarmLed = !alarmLed;
            }
        } else if ( overTempDetectorState ) {
            if( accumulatedTimeAlarm >= BLINKING_TIME_OVER_TEMP_ALARM  ) {
                accumulatedTimeAlarm = 0;
                alarmLed = !alarmLed;
            }
        }
    } else{
        alarmLed = OFF;
        gasDetectorState = OFF;
        overTempDetectorState = OFF;
        sirenPin.input();                                  
    }
#endif

#ifdef MODIFICACIONES_BUSIN_BUSOUT
    if(!inputs[MQ2_POSITION]) {
            gasDetectorState = ON;
            alarmState = ON;
        }
        if( overTempDetector ) {
            overTempDetectorState = ON;
            alarmState = ON;
        }
        if( inputs[ALARM_TEST_BUTTON_POSITION] ) {             
            overTempDetectorState = ON;
            gasDetectorState = ON;
            alarmState = ON;
        }    
        if( alarmState ) { 
            accumulatedTimeAlarm = accumulatedTimeAlarm + TIME_INCREMENT_MS;
            sirenPin.output();                                     
            sirenPin[SIREN_POSITION] = LOW;                                        

            if( gasDetectorState && overTempDetectorState ) {
                if( accumulatedTimeAlarm >= BLINKING_TIME_GAS_AND_OVER_TEMP_ALARM ) {
                    accumulatedTimeAlarm = 0;
                    leds[ALARM_LED_POSITION] = !leds[ALARM_LED_POSITION];
                }
            } else if( gasDetectorState ) {
                if( accumulatedTimeAlarm >= BLINKING_TIME_GAS_ALARM ) {
                    accumulatedTimeAlarm = 0;
                    leds[ALARM_LED_POSITION] = !leds[ALARM_LED_POSITION];
                }
            } else if ( overTempDetectorState ) {
                if( accumulatedTimeAlarm >= BLINKING_TIME_OVER_TEMP_ALARM  ) {
                    accumulatedTimeAlarm = 0;
                    leds[ALARM_LED_POSITION] = !leds[ALARM_LED_POSITION];
                }
            }
        } else{
            leds[ALARM_LED_POSITION] = OFF;
            gasDetectorState = OFF;
            overTempDetectorState = OFF;
            sirenPin.input();                                  
        }   
#endif
}

void alarmDeactivationUpdate()
{
#ifdef ORIGINAL_DIGITALIN_DIGITALOUT
    if ( numberOfIncorrectCodes < 5 ) {
        if ( aButton && bButton && cButton && dButton && !enterButton ) {
            incorrectCodeLed = OFF;
        }
        if ( enterButton && !incorrectCodeLed && alarmState ) {
            buttonsPressed[0] = aButton;
            buttonsPressed[1] = bButton;
            buttonsPressed[2] = cButton;
            buttonsPressed[3] = dButton;
            if ( areEqual() ) {
                alarmState = OFF;
                numberOfIncorrectCodes = 0;
            } else {
                incorrectCodeLed = ON;
                numberOfIncorrectCodes++;
            }
        }
    } else {
        systemBlockedLed = ON;
    }
#endif

#ifdef MODIFICACIONES_BUSIN_BUSOUT
    if ( numberOfIncorrectCodes < 5 ) {
            if ( inputs[A_BUTTON_POSITION] && inputs[B_BUTTON_POSITION] && inputs[C_BUTTON_POSITION] && inputs[D_BUTTON_POSITION] && !enterButton[0] ) {
                leds[INCORRECT_CODE_LED_POSITION] = OFF;
            }
            if ( enterButton[0] && !leds[INCORRECT_CODE_LED_POSITION] && alarmState ) {
                buttonsPressed[0] = inputs[A_BUTTON_POSITION];
                buttonsPressed[1] = inputs[B_BUTTON_POSITION];
                buttonsPressed[2] = inputs[C_BUTTON_POSITION];
                buttonsPressed[3] = inputs[D_BUTTON_POSITION];
                if ( areEqual() ) {
                    alarmState = OFF;
                    numberOfIncorrectCodes = 0;
                } else {
                    leds[INCORRECT_CODE_LED_POSITION] = OFF;
                    numberOfIncorrectCodes++;
                }
            }
        } else {
            leds[SYSTEM_BLOCKED_LED_POSITION] = ON;
        }
#endif
}


void uartTask()
{
    char receivedChar = '\0';
    char str[100];
    int stringLength;
    if( uartUsb.readable() ) {
        uartUsb.read( &receivedChar, 1 );
        switch (receivedChar) {
        case '1':
            if ( alarmState ) {
                uartUsb.write( "The alarm is activated\r\n", 24);
            } else {
                uartUsb.write( "The alarm is not activated\r\n", 28);
            }
            break;
        case '2':
#ifdef ORIGINAL_DIGITALIN_DIGITALOUT
            if ( !mq2 ) {
                uartUsb.write( "Gas is being detected\r\n", 22);
            } else {
                uartUsb.write( "Gas is not being detected\r\n", 27);
            }
#endif
#ifdef MODIFICADO_BUSIN_BUSOUT
            if(!inputs[MQ2_POSITION]) {
                uartUsb.write( "Gas is being detected\r\n", 22);
            } else {
                uartUsb.write( "Gas is not being detected\r\n", 27);
            }
#endif
            break;

        case '3':
            if ( overTempDetector ) {
                uartUsb.write( "Temperature is above the maximum level\r\n", 40);
            } else {
                uartUsb.write( "Temperature is below the maximum level\r\n", 40);
            }
            break;
            
        case '4':
            uartUsb.write( "Please enter the code sequence.\r\n", 33 );
            uartUsb.write( "First enter 'A', then 'B', then 'C', and ", 41 ); 
            uartUsb.write( "finally 'D' button\r\n", 20 );
            uartUsb.write( "In each case type 1 for pressed or 0 for ", 41 );
            uartUsb.write( "not pressed\r\n", 13 );
            uartUsb.write( "For example, for 'A' = pressed, ", 32 );
            uartUsb.write( "'B' = pressed, 'C' = not pressed, ", 34);
            uartUsb.write( "'D' = not pressed, enter '1', then '1', ", 40 );
            uartUsb.write( "then '0', and finally '0'\r\n\r\n", 29 );

            incorrectCode = false;

            for ( buttonBeingCompared = 0;
                  buttonBeingCompared < NUMBER_OF_KEYS;
                  buttonBeingCompared++) {

                uartUsb.read( &receivedChar, 1 );
                uartUsb.write( "*", 1 );

                if ( receivedChar == '1' ) {
                    if ( codeSequence[buttonBeingCompared] != 1 ) {
                        incorrectCode = true;
                    }
                } else if ( receivedChar == '0' ) {
                    if ( codeSequence[buttonBeingCompared] != 0 ) {
                        incorrectCode = true;
                    }
                } else {
                    incorrectCode = true;
                }
            }

            if ( incorrectCode == false ) {
                uartUsb.write( "\r\nThe code is correct\r\n\r\n", 25 );
                alarmState = OFF;
#ifdef ORIGINAL_DIGITALIN_DIGITALOUT
                incorrectCodeLed = OFF;
#endif
#ifdef MODIFICADO_BUSIN_BUSOUT
                leds[INCORRECT_CODE_LED_POSITION] = OFF;
#endif
                numberOfIncorrectCodes = 0;

            } else {
                uartUsb.write( "\r\nThe code is incorrect\r\n\r\n", 27 );
#ifdef ORIGINAL_DIGITALIN_DIGITALOUT
                incorrectCodeLed = ON;
#endif
#ifdef MODIFICADO_BUSIN_BUSOUT
                leds[INCORRECT_CODE_LED_POSITION] = ON;
#endif
                numberOfIncorrectCodes++;
            }                
            break;

        case '5':
            uartUsb.write( "Please enter new code sequence\r\n", 32 );
            uartUsb.write( "First enter 'A', then 'B', then 'C', and ", 41 );
            uartUsb.write( "finally 'D' button\r\n", 20 );
            uartUsb.write( "In each case type 1 for pressed or 0 for not ", 45 );
            uartUsb.write( "pressed\r\n", 9 );
            uartUsb.write( "For example, for 'A' = pressed, 'B' = pressed,", 46 );
            uartUsb.write( " 'C' = not pressed,", 19 );
            uartUsb.write( "'D' = not pressed, enter '1', then '1', ", 40 );
            uartUsb.write( "then '0', and finally '0'\r\n\r\n", 29 );

            for ( buttonBeingCompared = 0; 
                  buttonBeingCompared < NUMBER_OF_KEYS; 
                  buttonBeingCompared++) {

                uartUsb.read( &receivedChar, 1 );
                uartUsb.write( "*", 1 );

                if ( receivedChar == '1' ) {
                    codeSequence[buttonBeingCompared] = 1;
                } else if ( receivedChar == '0' ) {
                    codeSequence[buttonBeingCompared] = 0;
                }
            }

            uartUsb.write( "\r\nNew code generated\r\n\r\n", 24 );
            break;
 
        case 'p':
        case 'P':
            potentiometerReading = potentiometer.read();    
            sprintf ( str, "Potentiometer: %.2f\r\n", potentiometerReading );
            stringLength = strlen(str);
            uartUsb.write( str, stringLength );
            break;

        case 'c':
        case 'C':
            sprintf ( str, "Temperature: %.2f \xB0 C\r\n", lm35TempC );
            stringLength = strlen(str);
            uartUsb.write( str, stringLength );
            break;

        case 'f':
        case 'F':
            sprintf ( str, "Temperature: %.2f \xB0 F\r\n", 
                celsiusToFahrenheit( lm35TempC ) );
            stringLength = strlen(str);
            uartUsb.write( str, stringLength );
            break;

        default:
            availableCommands();
            break;

        }
    }
}

void availableCommands()
{
    uartUsb.write( "Available commands:\r\n", 21 );
    uartUsb.write( "Press '1' to get the alarm state\r\n", 34 );
    uartUsb.write( "Press '2' to get the gas detector state\r\n", 41 );
    uartUsb.write( "Press '3' to get the over temperature detector state\r\n", 54 );
    uartUsb.write( "Press '4' to enter the code sequence\r\n", 38 );
    uartUsb.write( "Press '5' to enter a new code\r\n", 31 );
    uartUsb.write( "Press 'P' or 'p' to get potentiometer reading\r\n", 47 );
    uartUsb.write( "Press 'f' or 'F' to get lm35 reading in Fahrenheit\r\n", 52 );
    uartUsb.write( "Press 'c' or 'C' to get lm35 reading in Celsius\r\n\r\n", 51 );
}

bool areEqual()
{
    int i;

    for (i = 0; i < NUMBER_OF_KEYS; i++) {
        if (codeSequence[i] != buttonsPressed[i]) {
            return false;
        }
    }

    return true;
}

float analogReadingScaledWithTheLM35Formula( float analogReading )
{
    return ( analogReading * 3.3 / 0.01 );
}

float celsiusToFahrenheit( float tempInCelsiusDegrees )
{
    return ( tempInCelsiusDegrees * 9.0 / 5.0 + 32.0 );
}
