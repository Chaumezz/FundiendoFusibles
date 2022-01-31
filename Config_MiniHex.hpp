//  Definicion servos MiniHex2, Paco y Chaume
const int COXA1_SERVO  = 22;  //pata 1, frontal derecha
const int FEMUR1_SERVO = 23;  //pata 1, frontal derecha
const int TIBIA1_SERVO = 24;  //pata 1, frontal derecha
const int COXA2_SERVO  = 26;  //pata 2, media derecha
const int FEMUR2_SERVO = 27;  //pata 2, media derecha
const int TIBIA2_SERVO = 28;  //pata 2, media derecha
const int COXA3_SERVO  = 30;  //pata 3, trasera derecha
const int FEMUR3_SERVO = 31;  //pata 3, trasera derecha
const int TIBIA3_SERVO = 32;  //pata 3, trasera derecha
const int COXA4_SERVO  = 37;  //pata 4, trasera izquierda
const int FEMUR4_SERVO = 36;  //pata 4, trasera izquierda
const int TIBIA4_SERVO = 35;  //pata 4, trasera izquierda
const int COXA5_SERVO  = 41;  //pata 5, media izquierda
const int FEMUR5_SERVO = 40;  //pata 5, media izquierda
const int TIBIA5_SERVO = 39;  //pata 5, media izquierda
const int COXA6_SERVO  = 45;  //pata 6, frontal izquierda
const int FEMUR6_SERVO = 44;  //pata 6, frontal izquierda
const int TIBIA6_SERVO = 43;  //pata 6, frontal izquierda

const int HEAD_GRIP_SERVO = 19;     //PINZA
const int HEAD_TILT_SERVO = 20;    //Tilt de la cabeza
const int HEAD_PAN_SERVO = 21;     //Paneo de la cabeza

const int NEOPIXELS_BAR = 52;

const int MS_SERVO_MIN = 500;     //milisegundos del PWM para posición 0 grados del servo
const int MS_SERVO_MAX = 2500;    //milisegundos del PWM para posición 180 grados del servo

const int COXA_LENGTH = 30;   //longitud de la coxa en mm  (Desde el eje del servo de la coxa al eje del servo del fémur)
const int FEMUR_LENGTH = 62;  //longitud del fémur en mm (Desde el eje del servo del fémur al eje del servo de la tibia)
const int TIBIA_LENGTH = 95; //longitud de la tibia en mm (Desde el eje del servo de la tibia a la punta de la pata)

// Posicion por defecto de las patitas, sin movimiento en cada punta.
// Hay que ajustarla en cada robot o servo para encontrar la posicion de descanso
// Posiciones home Coxa-to-toe.

//               PATITA    #1    #2    #3    #4    #5    #6
const float HOME_X[6] = { 40.0,  0.0,-40.0,-40.0,  0.0, 40.0};  //adelante positivo, atrás negativo
const float HOME_Y[6] = { 40.0, 62.0, 40.0,-40.0,-62.0,-40.0};  //derecha positivo, izquierda negativo
const float HOME_Z[6] = {-95.0,-95.0,-95.0,-95.0,-95.0,-95.0};  //arriba positivo, abajo negativo

// Body center to coxa servos distances
const float BODY_X[6] = { 82.0, 0.0, -82.0, -82.0, 0.0, 82.0};  //adelante positivo, atrás negativo
const float BODY_Y[6] = {  38.0, 51.0, 38.0, -38.0, -51.0, -38.0};  //derecha positivo, izquierda negativo
const float BODY_Z[6] = {  -0.0, -0.0, -0.0, -0.0, -0.0, -0.0};  //arriba positivo, abajo negativo, antes a -138

// Offsets you can use to correct for servo centering errors.
const int COXA_CAL[6]  = {-36, -2, 40, -44,-5, 50};                     //offsets de coxas desde la 1 a la 6
const int FEMUR_CAL[6] = { -15,0,-8,-10,-2,-10};                    //offsets de fémur desde la 1 a la 6
const int TIBIA_CAL[6] = {40, 48,48,37,40,54};

const int HEAD_PAN = 8;
const int HEAD_TILT = -12;
const int HEAD_GRIP = 0;

const int TRAVEL = 25;        //the maximum distance in millimeters that the robot’s body moves in the translate mode

const int STRIDE_X_MULTIPLIER = 40;
const int STRIDE_Y_MULTIPLIER = 40;
const int STRIDE_R_MULTIPLIER = 20;
