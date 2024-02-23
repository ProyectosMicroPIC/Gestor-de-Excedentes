#include <Arduino.h>


// Configuración código
#define WiFi_activado
#define serial_activado


// Para la lectura de CONSUMO
const uint16_t ResolucionADC = 4096;
const float ADCV = 5;                           // La Vcc del ACS724 es de 5V
const float Sensibilidad = 0.200;               // 200mV por cada A en el ACS724
const float TensionRed = 230.0;                 // 230Vac

// Curva de Tensión para 230V
// cálculos de un cuarto de onda senoidal, desde 0º a 90º. El cuadrantes de 90º a 180º se calcula utilizando
// estos mismos valores pero en orden inverso
const float v_teorica[51] = { 
0,10.311578386872,20.6127738883835,30.8932140738594,41.1425474114685,51.3504536913387,61.5066544171333,71.6009231556257,
81.6230958338507,91.5630809734646,101.410869852008,111.156546580842,120.790298089606,130.302424007149,139.683346428981,
148.923619561408,158.013939232652,166.945152261357,175.708265673069,184.294455755401,192.695076942758,200.901670521697,
208.905973148125,216.699925167795,224.275678731686,231.62560569813,238.742305313698,245.618611665134,252.247600894821,
258.62259817252,264.737184416359,270.585202756301,276.160764733592,281.458256229943,286.472343120464,291.197976644684,
295.630398490218,299.765145583989,303.59805458616,307.125266082267,310.343228469324,313.248701531981,315.838759705151,
318.110795019803,320.062519728968,321.691968611303,322.997500949908,323.977802184381,324.631885234478,324.959091494009,
325
};


class dimmer {
private:
    // Estados dimmer
    #define E_Reposo        0
    #define E_Leyendo1semi  1
    #define E_LeyendoUltsemi 10
    #define E_Calculando    11
    #define E_Apagado       12
    #define E_Encendido     13
    
    #define NumPeriodos   100                       // número de periodos en los que dividimos la semionda
    #define vDimmerMIN 10                           // Mínimo ángulo de disparo del triac
    #define vDimmerMAX 98                           // Máximo ángulo de disparo del triac
    #define vDimmerActivaDeteccion 80               // Ángulo de activación de la interrupción de cruce por cero
    #define OBJETIVO_MINIMO 0                       // Potencia mínima entregable
     

    static void IRAM_ATTR deteccion0(void);
    static void reloj100us(void);
    float calibracion (float x);
    uint16_t getPotencia(void);
    void desactiva_triac(void);
    void reconectar_mqtt(void);
    void ajusta_dimmer (int16_t v_pot, int16_t v_obj);
 
public:




    // Configuración dimmer


    #define ascendente  1
    #define descendente 0

    void nuevo_objetivo(uint16_t newobjetivo);
    void loop(void);
    void begin(uint16_t objmax, uint8_t p_entrada, uint8_t p_salida, uint8_t p_analogico);
    void configuracion( float p0, float p1, float p2);
    uint16_t pot_consumida(void);



};

