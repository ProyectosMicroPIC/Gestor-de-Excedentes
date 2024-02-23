#include "dimmer.hpp"

#define serial_activado

// Pinout
 uint8_t pinSalida; // Usamos el pin de TEST porque el pin 1 no va 1; // Pin GPIO1 que excitará al Triac
 uint8_t pinEntrada; // Pin GPIO0 para detectar los cruces por 0
 uint8_t pinAnalogico; // Pin GPIO4 entrada analógica para leer consumo
 uint8_t pinSCL;
 uint8_t pinSDA;


// Variables dimmer  
static hw_timer_t *timer ;                       // Declaración de la variable timer
static uint16_t contadorPulsos;           // cuenta los microintervalos de 100us (en cada semionda hay 100)
static const uint16_t NumMuestras = E_LeyendoUltsemi * NumPeriodos; // Número de muestras totales
static uint16_t LecturasADC[NumMuestras];              // Array para guardar todas las lecturas
static uint16_t ContadorLecturas;                    // Puntero al array
static uint16_t num_calculos;                    // Puntero al array
static uint8_t estado;   
static uint8_t CicloDimmer;                      // ángulo de disparo del triac: de 0 a 100
static uint16_t objetivo;                          // objetivo de consumo recibido del servidor
static uint32_t potencia_acumulada;                // suma de la potencia obtenida en varias muestras
static uint8_t incremento;                         // Para acercarnos al objetivo usamos pasos de tamaño variable
static uint8_t direccion;                 // Por defecto la dirección de los pasos es ascendente
static uint16_t OBJETIVO_MAXIMO;
float param0, param1, param2;

void dimmer::begin(uint16_t objmax, uint8_t p_entrada, uint8_t p_salida, uint8_t p_analogico) {
    OBJETIVO_MAXIMO = objmax;
    pinEntrada = p_entrada;
    pinSalida = p_salida;
    pinAnalogico = p_analogico;


    pinMode(pinEntrada, INPUT);                   // Definimos el pin de entrada para la detección de cruces por 0
    pinMode(pinSalida, OUTPUT);                   // Definimos el pin de salida que excitará al Triac

    // Asignamos la función "deteccion0" a la interrupción por un flanco de subida en la entrada
    attachInterrupt(digitalPinToInterrupt(pinEntrada), dimmer::deteccion0, RISING);
    
    // Configura un temporizador para llamar a una función cada 100us (0.0001 segundos)
    timer = timerBegin(0, 80, true);              // Usamos el grupo de temporizadores 0 y la escala de prescaler 80
    timerAttachInterrupt(timer, dimmer::reloj100us, true); // Adjunta la función reloj100us como interrupción
    timerAlarmWrite(timer, 100, true);            // Establece el período del temporizador (100 microsegundos)
        
    estado = E_Reposo;                            // Estado inicial del dimmer
    contadorPulsos = 0;
    CicloDimmer = 100;
    objetivo = 0; 
    potencia_acumulada = 0; 
    num_calculos = 0; 
    ContadorLecturas=0;
    incremento = 0; 
    direccion = ascendente;
    #ifdef serial_activado
    Serial.begin(115200);                         // Activamos puerto serie
    #endif
}

void dimmer::configuracion( float p0, float p1, float p2) {
    param0 = p0;
    param1 = p1;
    param2 = p2;
}

// Interrupción externa. Se lanza cada vez que se detecta una señal de subida en el pin de entrada
void IRAM_ATTR dimmer::deteccion0() {
  contadorPulsos=0;    // Al detectar un cruce por 0 reiniciamos el contador de pulsos
  
  detachInterrupt(digitalPinToInterrupt(pinEntrada)); // y desactivamos la interrupción externa para evitar ruidos
  timerAlarmEnable(timer); // Habilita la alarma del temporizador
  if (estado < E_Calculando) // en cada semionda subimos un punto el estado hasta llegar al momento del cálculo
    estado++; 
}

// La interrupción del Timer se ejecutará cada 100 microsegundos
// Como un segundo tiene 100 seminodas, cada una de ellas dura 10.000 microsegundos
// En cada semionda ejecutaremos esta función 100 veces
void dimmer::reloj100us() {
  contadorPulsos++; // cada vez que entramos incrementamos el contador de pulsos

  if (estado<=E_Calculando || estado == E_Encendido) {
    if (contadorPulsos==CicloDimmer && CicloDimmer<98) // llegados al punto de la onda en que queremos encender ...
      digitalWrite(pinSalida, true); // ... activa la salida del Triac 

    if (contadorPulsos==CicloDimmer+10) { // y un ciclo del reloj más tarde ...
      if (CicloDimmer>1)
        digitalWrite(pinSalida, false); // ... desactiva la salida del Triac 
      }
    if (contadorPulsos==vDimmerActivaDeteccion)
      // para detectar el próximo cruce por 0 es necesario activar la interrupción externa
      attachInterrupt(digitalPinToInterrupt(pinEntrada), deteccion0, RISING);
  }
  // Si estamos en proceso de lectura de la señal, cogemos una muestra y la guardamos en el array
  if ((estado >= E_Leyendo1semi && estado <= E_LeyendoUltsemi ) || (estado == E_Encendido ))
    if (ContadorLecturas<NumMuestras)
      if (CicloDimmer>=vDimmerMAX)  // a partir de un punto cercano al cruce por cero no leemos para evitar ruido
        LecturasADC[ContadorLecturas++] = ResolucionADC/2;
      else
        LecturasADC[ContadorLecturas++] = analogRead(pinAnalogico);
}
 
///////////////////////////////////////////////////////////
// MEDIDA DE POTENCIA
///////////////////////////////////////////////////////////

float dimmer::calibracion (float x) {
  return ( param0*x*x + param1*x + param2);
}

uint16_t dimmer::getPotencia()
{
  int16_t voltaje;
  
  uint16_t p_lecturas;
  uint8_t p_curvavoltaje;
  int8_t sentido;
  uint32_t PuntoMedio = 0;
  uint32_t potencia_consumida;
  
  float corriente_instantanea, pot_instantanea;

  // Calculamos el punto medio de las muestras. Es más exacto hacerlo así
  // que utilizar directamente 4096/2=2048 ya que la señal puede tener
  // cierto offset
  for (p_lecturas=0;p_lecturas<ContadorLecturas;p_lecturas++) {
      PuntoMedio+= LecturasADC[p_lecturas] ;
  }
  PuntoMedio /= ContadorLecturas;

  potencia_consumida = 0;
  p_curvavoltaje=0;
  sentido=-1;

  // Recorremos todas las muestras
  for (p_lecturas=0;p_lecturas<ContadorLecturas;p_lecturas++) {
    // la diferencia entre una lectura y el punto medio es la amplitud de la onda = voltaje
    voltaje = LecturasADC[p_lecturas] - PuntoMedio;

    // Si la onda está recortada, las muestras antes del disparo del triac serán 0
    // En caso contrario calculamos la corriente instantánea utilizando el valor
    // absoluto de la lectura de voltaje, la tensión de referencia del ADC, su
    // resolución y su sensibildad
    // La potencia instantánea será la corriente instantánea por la tensión
    // que obtenemos del array que define la curva
    if ((p_lecturas % NumPeriodos) >= CicloDimmer) {
      corriente_instantanea = abs(voltaje) * ADCV / ResolucionADC / Sensibilidad;
      pot_instantanea = corriente_instantanea * v_teorica [p_curvavoltaje];
    } else {
      pot_instantanea = 0; 
    }

    // Vamos acumulando las potencias instantáneas en una variable
    potencia_consumida += pot_instantanea;  
    
    // Si hemos pasado de los 90º cambiamos el sentido del puntero de la curva de tensión
    if ((p_lecturas%50) == 0)
      sentido = -sentido;
    p_curvavoltaje += sentido;
  }

  // la potencia total dividida entre el nº de muestras nos da la potencia promedio
  return (calibracion(potencia_consumida / ContadorLecturas));
}


// Cuando queremos entregar la onda completa o dejar la salida apagada
// dejamos el triac estático en estado ON u OFF
void dimmer::desactiva_triac() {
  detachInterrupt(digitalPinToInterrupt(pinEntrada));
  if (estado == E_Encendido) {
    digitalWrite(pinSalida, true);
  } else {
    digitalWrite(pinSalida, false);
  }
}


void dimmer::ajusta_dimmer (int16_t v_pot, int16_t v_obj) {
  if (v_pot>v_obj) {                              // Si está entregando más potencia que el objetivo
                                                  // tenemos que reducir la potencia subiendo el ángulo de disparo
      if (direccion == descendente) {             // en caso en que la búsqueda del objetivo fuera hacia abajo
        direccion = ascendente;                   // cambiamos la dirección hacia arriba
        if (incremento>0) {                       // Si el paso incremental no es 0
          if (incremento==10)                     // si era el paso mayor, como nos hemos pasado
            incremento = 5;                       // lo reducimos a la mitad
          else                      
            incremento--;                         // en caso contrario reducimos el paso en una unidad
          }
      };                                          // Una vez que tenemos el incremento adecuado, se lo agregamos a 
      if (CicloDimmer<(vDimmerMAX-incremento)) {  // Ciclodimmer (si no nos pasamos del máximo)
        CicloDimmer+=incremento;
      } else {
        CicloDimmer = vDimmerMAX;                 // Si hemos llegado al máximo lo dejamos ahí
        incremento=0;                             // y anulamos los incrementos
      }
    } else {                                      // potencia>=objetivo            
      if (direccion == ascendente) {              // Análogo al bloque anterior pero en sentido contrario
        direccion = descendente;
        if (incremento>0) {
          if (incremento==10)
            incremento = 5;
          else
            incremento--;
          }
      };
      if (CicloDimmer>(incremento+vDimmerMIN)) {
        CicloDimmer-=incremento;
      } else {
        CicloDimmer = vDimmerMIN;
        incremento = 0;
      }
    }
}

void dimmer::nuevo_objetivo(uint16_t newobjetivo){
    objetivo = newobjetivo; 
    if (newobjetivo == OBJETIVO_MINIMO) {
        estado = E_Apagado;
        desactiva_triac();
        #ifdef serial_activado
        Serial.println ("Triac DESACTIVADO");
        #endif
    } else {
        if (newobjetivo == OBJETIVO_MAXIMO) {
        estado = E_Encendido;
        desactiva_triac();
        #ifdef serial_activado
        Serial.println ("Triac ACTIVADO");
        #endif
        } 
        // Si no es OBJETIVO MÍNIMO, entonces es objetivo válido y lo procesamos
        #ifdef serial_activado
        Serial.print("Objetivo establecido en: ");  // mostramos el valor leído al usuario
        Serial.println(objetivo);
        #endif
        estado = E_Reposo;                          // El primer estado del dimmer es reposo
        incremento = 10;                            // Establecemos pasos de 10
        CicloDimmer = 50;                           // Y situamos el dimmer en el punto medio
        attachInterrupt(digitalPinToInterrupt(pinEntrada), deteccion0, RISING);   // Activación del cruce por 0
    }
}

void dimmer::loop(void) {
    uint16_t potencia;

  if (estado == E_Calculando || estado == E_Encendido) {                 // Cuando se han capturado todas las muestras toca calcular
    potencia = getPotencia();                   // leemos la potencia consumida
    #ifdef serial_activado
    Serial.print("/*");
    Serial.print(potencia);         
    Serial.print(", ");
    Serial.print(objetivo);
    Serial.print(", ");
    Serial.print(CicloDimmer);
    Serial.println("*/");
    #endif
    ajusta_dimmer(potencia,objetivo);           // y ajustamos el ángulo de disparo para que la potencia se acerque al objetivo
    ContadorLecturas=0;                         // reinicio del contador de lecturas para una nueva toma de muestras
    estado = E_Reposo;                          // estado inicial
    potencia_acumulada += potencia;             // Vamos acumulando la potencia en un acumulador para sacar promedios
    num_calculos++;                          // Y llevamos un contador de todas las lecturas
  }
  if (estado == E_Apagado) {
      potencia_acumulada += OBJETIVO_MINIMO;
    }
    
}

uint16_t dimmer::pot_consumida(void){
  uint16_t respuesta;
  if (objetivo==OBJETIVO_MINIMO)
    respuesta = OBJETIVO_MINIMO;
  else
    respuesta = (uint32_t)(potencia_acumulada/ num_calculos);
  potencia_acumulada=0;                       // y reiniciamos el acumulador de potencias
  num_calculos=0;
  return (respuesta);
}