//Ultima version de la computadora de vuelo 2025 13 de enero a 2025 6:57
//Reclutas de avionica 2025

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h> //Biblioteca que proporciona una interfaz estándar para manejar sensores en las bibliotecas de Adafruit
#include <SD.h>              // Biblioteca para uso de la SD

// Configuración de los pines para I2C1
#define SDA_PIN 2         // GP2 como SDA
#define SCL_PIN 3         // GP3 como SCL
#define PIN_LED    26     // Pin del led 
#define MPUADRESS  0x68   // Dirección en hexadecimal del MPU
#define BMPADRESS  0x77   // Dirección en hexadecimal del BMP
#define T_PERIODO   100L  // Periodo de tiempo de muestreo en milisegundos
#define VENT 8            // Ventana del filtro movil
#define PINBUZZER  0      // Pin del modulo buzzer
#define N_MUESTRAS  10    // Número de muestreos para ejecutra el flush() en la SD
#define ch1         19    // Pin del canal pirotecnico 1
#define ch2         18    // Pin del segundo canal pirotecnico
#define SD_MISO 12        // MISO para el lector SD - GP12 
#define SD_MOSI 11        // MOSI para el lector SD - GP11
#define SD_SCK  10        // SCK para el lector SD - GP10
#define SD_CS   9         // Chip Select  para el lector SD - GP9 
#define PRESION_MAR 1013.25 //Presion a nivel del mar usada para calculos de altura
#define COCIENTE_TL 44330 //Constante T/L usada en el calculo de alturas
#define ALT_MAIN 500    // Altitud de apertura del segundo paracaidas a 500 m
#define ALT_APG 10        // Altura desde el apogeo en la que se abre el drogue 
#define ALT_SEG 50        //Seguro que bloquea los canales pirotecnicos si no hay una altitud minima
#define TIEMPO_NEU 5000   //Tiempo que esta abierto la valvula solenoide
#define TIEMPO_MAIN 250  //Tiempo que esta activado el canal pirotecnico del main
#define RESISTENCIA1 1000 // Resistencia que permite medir la carga de la bateria
#define RESISTENCIA2 3300  // Resistencia que permite medir la carga de la bateria
#define Vbatt 27           // Pin que permite conocer la carga de la bateria


String archnom = "lcv0.csv";    // Lecturas Computadora de Vuelo. El tipo de dato es string para posteriormente facilitar el trabajo con el nombre
uint16_t cont_archivos = 1;     // Contador que servirá para no repetir archivos
uint32_t t;                     // Tiempo en milisegundos
uint32_t t_anterior;            // Tiempo en milisegundos
uint32_t n_muestreos = 1;       // Contador de muestreos realizados en el loop()
bool primer_valor = false;      // Indicador que ayuda a la nedua movil
float altitud2;                 // Variable de altitud en bruto
float suma1 = 0;                // Suma para la media movil
uint8_t i;                      // Contador que ayuda a la media movil
float bmp_altitud_filtrado = 0; // Variable filtrada de altitud
float buffer1[VENT]={0};        // Buffer de filtro de altitud
uint8_t indice = 0;                 // Indice del buffer
float altura_inicial = 0.0;     // Variable que guarda la altitud inicial
float altura_mas_alta = 0.0;    // Variable que guarda la altitud mayor
uint32_t contparac_inicial=0;    // Contador que ayuda a la toma de mediciones 
uint32_t tiempo_ult_act = 0;    // Tiempo de la ultima activacion
uint32_t tiempo_ult_act_main=0; // Tiempo de la activacion del droge
bool recuperacion_act = false;  // Indicador si el primer paracaidas se desplego
bool recuperacion_act_main=false; 
bool conf_despegue = false;     // Indica si el cohete ya despego
bool segun_parac = false;        // Indica para que el main se desplegue despues del droge

File archivo;                         // Objeto archivo para la SD

Adafruit_MPU6050 mpu; // Se crea un objeto del tipo Adafruit_MPU6050 (que servirá para utilizar la biblioteca)
Adafruit_BMP280 bmp(&Wire1); // Asignar Wire1 como bus I2C para el sensor
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup() {
  // put your setup code here, to run once:

  pinMode(ch1,OUTPUT);       
  pinMode(ch2,OUTPUT);
  pinMode(PINBUZZER,OUTPUT);
  pinMode(PIN_LED,OUTPUT);
  
  digitalWrite(ch1,LOW);   
  digitalWrite(ch2,LOW);
  
  t_anterior = millis();          // Toma el tiempo inicial para el manejo de tiempos

  // Configurar SPI1 para la escritura de datos en el SD, con los nuevos pines
  SPI1.setRX(SD_MISO);
  SPI1.setTX(SD_MOSI);
  SPI1.setSCK(SD_SCK);

  // Configurar manualmente los pines I2C1, con los nuevos pines
  Wire1.setSDA(SDA_PIN); 
  Wire1.setSCL(SCL_PIN);
  Wire1.begin(); // Iniciar el bus I2C1

  // <Conexión a la SD>
   while(!SD.begin(SD_CS, SPI1))
    {
     digitalWrite(PIN_LED, HIGH); // Indica un error al parpadear el led. Solo escritura de datos
     delay(250);
     digitalWrite(PIN_LED, LOW);
     delay(250);
    } //Cierre while

   while(SD.exists(archnom))     // Función para crear un archivo con nombre único
    {
     archnom = "lcv" + String(cont_archivos) + ".csv";
     cont_archivos++;
    } //Cierre while

    archivo = SD.open(archnom, FILE_WRITE); // A la variable archivo se le asigna el valor acorde si la conexión pudo o no establecerse
    
   while(!archivo)
    {
     digitalWrite(PIN_LED, HIGH); // Indica un error al parpadear el led. Solo escritura de datos
     delay(250);
     digitalWrite(PIN_LED, LOW);
     delay(250);
    }
    
    archivo.println("t [s], Presion [kPa], Temperatura[C],Aceleracion en X,Aceleración en Y,Aceleración en Z,Giro en X,Giro en Y,Giro en Z,Altitud,Altitud filtrada,Altitud inicial,Altitud mas alta,Confirmacion del main,Confirmacion del droge,Confirmacion de despegue,Segunda confirmacion del droge,Voltaje");
    archivo.flush();

  //BMP//
  unsigned status;
  // La dirección I2C del BMP280 es 0x77 (verificar en tu sensor)
  status = bmp.begin(BMPADRESS);
  while (!status) {
    archivo = SD.open(archnom, FILE_WRITE);
    archivo.println("Error con BMP280");
    archivo.flush();
    tone(PINBUZZER,252,1000);
    digitalWrite(PIN_LED,HIGH);
    delay(250);
    digitalWrite(PIN_LED,LOW);
    delay(250);
  }

  //MPU//
  while (!mpu.begin(MPUADRESS, &Wire1)) {   //Se establecen primero la dirección en hexadecimal, y después la conexión por I2C
    archivo = SD.open(archnom, FILE_WRITE);
    archivo.println("Error con MPU6050");
    archivo.flush();
    tone(PINBUZZER,252,1000);
    digitalWrite(PIN_LED,HIGH);
    delay(250);
    digitalWrite(PIN_LED,LOW);
    delay(250);
     }

  /* Configuración predeterminada */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
  
}

void loop() {
  // Manejo de tiempos
  delay(20000);
  digitalWrite(ch1,HIGH);
  
  t = millis();
  if((t - t_anterior)>=T_PERIODO)
  {
    t_anterior=t;                //Actualizar la variable de tiempo
    sensors_event_t a, g, temp;  //Declara identificadors para los valores a leer
    mpu.getEvent(&a, &g, &temp);
    sensors_event_t temp_event, pressure_event;
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);
    contparac_inicial++;

    int adcValue = analogRead(Vbatt); //Lectura del pin analógico, int porque esta funcion regresa un valor entre 0 y 1023
    float voltaje = (adcValue /1024.0)*3.3*(RESISTENCIA1 + RESISTENCIA2)/RESISTENCIA2; //Calculo del voltaje de la bateria

    if(!bmp_temp || !bmp_pressure){ //Indica por medio del sonido un error en la toma de datos
    tone(PINBUZZER,262,250);
    }

    //Formula para calcular la altitud
    float P = (float)pressure_event.pressure;
    altitud2= -8431.7771f*log(0.0098692327f*(P/10.0f));

    //Filtro media movil a la variable de altitud
    //Hace las copias necesarias para obtener el primer valor de la media movil
    if(!primer_valor){
      for(i=0;i < VENT;i++){
        buffer1[i] = altitud2;
        suma1 += altitud2;
      }
      primer_valor =  true;
    }
    else{
      //Aqui se aplica la asignaciones de valores del buffer, remplazando el mas viejo con el mas nuevo
      suma1 = suma1 - buffer1[indice] + altitud2;
      buffer1[indice] = altitud2;
      indice = (indice + 1)%VENT;
    }
    bmp_altitud_filtrado = suma1 / VENT;

    //Nos aseguramos de tener una toma de valores en la primer iteracion
    if(contparac_inicial == 1){
      altura_inicial = bmp_altitud_filtrado;
      altura_mas_alta = bmp_altitud_filtrado;
    }

    //Hacemos una comparativa para tener la mayor altitud
    if(bmp_altitud_filtrado >= altura_mas_alta){
      altura_mas_alta = bmp_altitud_filtrado;
    }

    //Nos aseguramos que los sistemas de liberacion no se activen en el suelo o durante el despegue
    if((altura_mas_alta - altura_inicial) >= ALT_SEG){  
      if((altura_mas_alta - bmp_altitud_filtrado) >= ALT_APG && !recuperacion_act){ //Se busca que haya una diferencia de 10 metros respecto al apogeo ya que es poco mas de un segundo
        digitalWrite(ch1,HIGH);
        tone(PINBUZZER,222,5000);
        tiempo_ult_act = millis();
        recuperacion_act = true;
        segun_parac = true;
      }

      if((millis() - tiempo_ult_act) >= TIEMPO_NEU && segun_parac == true){
        digitalWrite(ch1,LOW);
        recuperacion_act_main = true;
        if((bmp_altitud_filtrado - altura_inicial)<= ALT_MAIN && recuperacion_act_main){
          digitalWrite(ch2,HIGH);
          tone(PINBUZZER,222,5000);
          tiempo_ult_act_main = millis();
          if((millis() - tiempo_ult_act_main) >= TIEMPO_MAIN){
            digitalWrite(ch2,LOW);
            recuperacion_act_main= false;
          }
        }
      }
    }


    if(archivo){
      archivo.print(t);
      archivo.print(",");
      archivo.print( P / 10);
      archivo.print(",");
      archivo.print(temp_event.temperature);
      archivo.print(",");
      archivo.print(a.acceleration.x);
      archivo.print(",");
      archivo.print(a.acceleration.y);
      archivo.print(",");
      archivo.print(a.acceleration.z);
      archivo.print(",");
      archivo.print(g.gyro.x);
      archivo.print(",");
      archivo.print(g.gyro.y);
      archivo.print(",");
      archivo.print(g.gyro.z);
      archivo.print(",");
      archivo.print(altitud2);
      archivo.print(",");
      archivo.print(bmp_altitud_filtrado);
      archivo.print(",");
      archivo.print(altura_inicial);
      archivo.print(",");
      archivo.print(altura_mas_alta);
      archivo.print(",");
      archivo.print(recuperacion_act_main);
      archivo.print(",");
      archivo.print(recuperacion_act);
      archivo.print(",");
      archivo.print(conf_despegue);
      archivo.print(",");
      archivo.print(segun_parac);
      archivo.print(",");
      archivo.println(voltaje);

      if( n_muestreos%N_MUESTRAS == 0 )
        archivo.flush();
    }
    else{
     tone(PINBUZZER,222,5000);
     digitalWrite(PIN_LED, HIGH); // Indica un error al parpadear el led. Solo escritura de datos
     delay(250);
     digitalWrite(PIN_LED, LOW);
     delay(250);
    }
    n_muestreos++;                                    // Aumentar el contador al finalizar un muestreo 
  }
}
