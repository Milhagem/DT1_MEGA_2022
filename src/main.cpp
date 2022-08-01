/*
 *  O seguinte código é utilizado num arduino MEGA, cujo papel é a de um controlador de um motor BLDC
 *  O Arduino recebe sinal dos sensores de efeito hall e comuta os mosfets de acordo com os valores lidos
 *  O motor é ligado apenas sob certas condições de interruptores e fins de curso
 *  A entrada para o motor é uma entrada rampa, com incremento gradual
 */

#include <Arduino.h>

// Relação de portas Shield Controlador -> Powerdriver
#define MOSFET_A_HIGH 5
#define MOSFET_B_HIGH 6
#define MOSFET_C_HIGH 7
#define MOSFET_A_LOW 9
#define MOSFET_B_LOW 10
#define MOSFET_C_LOW 11

// Portas definidas MEGA
#define hallA 4           // Entrada hall A do motor
#define hallB 3           // Entrada hall B do motor
#define hallC 2           // Entrada hall C do motor
#define botao1_pedal 15   // Fim de Curso Dianteiro. Entrada do botão 1 de aceleração (Começando a pressionar)
#define botao2_pedal 16   // Fim de Curso Traseiro. Entrada do botão 2 de aceleração (Completamente pressionado)
#define chave_controle 19 // Configura o valor inicial da rampa. Sendo o valor mais baixo para arrancada e o maior para ao longo da corrida
#define acelerador 8      // Para ligar o carro sem acionar o pedal e o pivotamento
#define sinal_corrente 22 // Desativa a alimentação se a corrente estiver muito alta

int hallA_estado;                                  // Estado do hall A
int hallB_estado;                                  // Estado do hall B
int hallC_estado;                                  // Estado do hall C
int hall_val = 1;                                  // Estado dos tres halls (Fase do motor)
int botao1_pedal_anterior = 3;                     // Estado anterior do fim de curso dianteiro
int botao2_pedal_anterior = 3;                     // Estado anterior do fim de curso traseiro
int estado_controle, estado_controle_ant = LOW;    // Serve para zerar o dutyCycle para o início da rampa
unsigned long tempo_Atual = 0, tempo_Anterior = 0; // Servem para incrementar o PWM depois de x microsegundos

// Variáveis Relevantes
int Incremento_rampa = 0; // Duty cycle em % , de 0 a 100%
int inicio_rampa = 0;     // Valor inicial da rampa
int pwwm = 0;             // Armazena o PWM que irá para os MOSFET's

void setup()
{
  // Entradas do Arduino MEGA
  pinMode(hallA, INPUT);
  pinMode(hallB, INPUT);
  pinMode(hallC, INPUT);
  pinMode(sinal_corrente, INPUT);

  // Saídas do Arduino MEGA
  // Saídas para ativação MOSFET's do inversor trifásico
  // Saídas digitais HIGH SIDE
  pinMode(MOSFET_A_HIGH, OUTPUT);
  pinMode(MOSFET_B_HIGH, OUTPUT);
  pinMode(MOSFET_C_HIGH, OUTPUT);

  // Saídas PWM LOW SIDE
  pinMode(MOSFET_A_LOW, OUTPUT);
  pinMode(MOSFET_B_LOW, OUTPUT);
  pinMode(MOSFET_C_LOW, OUTPUT);

  // Coloca as saídas dos MOSFET's em low
  PORTE &= ~(1 << PORTE3); // pin 5
  PORTH &= ~(1 << PORTH3); // pin 6
  PORTH &= ~(1 << PORTH4); // pin 7
  analogWrite(9, 0);
  analogWrite(10, 0);
  analogWrite(11, 0);

  // Define a frequência do PWM para os MOSFET's
  //  Seta Timer 1 (1B) para 490 Hz para frequência do PWM (pino 11)
  //  Seta Timer 2 (2B) para 980 Hz para frequência do PWM (pino 9 e 10)

  int myEraser = 7;      // this is 111 in binary and is used as an eraser
  TCCR1B &= ~myEraser;   // this operation (AND plus NOT), set the three bits in TCCR1B to 0
  TCCR2B &= ~myEraser;   // this operation (AND plus NOT), set the three bits in TCCR2B to 0
  int myPrescaler = 3;   // this could be a number in [1 , 6]. In this case, 1 corresponds in binary to 001 (31kHz)
  TCCR1B |= myPrescaler; // this operation (OR), replaces the last three bits in TCCR1B with our new value 001
  TCCR2B |= myPrescaler; // this operation (OR), replaces the last three bits in TCCR2B with our new value 001
}
void loop()
{
  tempo_Atual = millis();
  estado_controle = digitalRead(chave_controle);

  // Se houver mudança da chave controle, então a rampa é "zerada". Voltando a incrementar do zero
  if (estado_controle != estado_controle_ant)
  {
    Duty_Cycle = 0;
    estado_controle_ant = estado_controle;
  }

  // Configura o valor do início da rampa de acordo com o selecionado na chave_controle
  // O valor mais baixo é para a arrancada, e o mais alto é para a pista
  // Os valores podem mudar conforme variar o peso do carro e do piloto
  if (digitalRead(chave_controle) == HIGH)
  {
    inicio_rampa = 40;
  }
  else
  {
    inicio_rampa = 50;
  }

  // Liga o carro se o pedal estiver completamente pressionado. Ou se o acelerador estiver ligado
  if (((digitalRead(botao1_pedal) == LOW) && (digitalRead(botao2_pedal) == HIGH)) || (digitalRead(acelerador) == LOW))
  {
    if (Incremento_rampa < 90 - inicio_rampa)
    { // Se o PWM sobre os MOSFET's ainda é menor do que 90%
      if ((tempo_Atual - tempo_Anterior) >= 70)
      { // Incrementa a cada 70 milisegundos
        Incremento_rampa += 1;
        tempo_Anterior = tempo_Atual;
      }
    }
    pwwm = map(Incremento_rampa + inicio_rampa, 0, 100, 0, 255);
  }
  else
  {
    Incremento_rampa = 0;
    pwwm = map(Incremento_rampa, 0, 100, 0, 255); // Não entendi porque colocaram essa variável sendo que podia só por o                                                           // valor zero, por mim acho que pode por o zero
  }

  hallA_estado = digitalRead(4); // variavel recebe o valor do sensor Hall A
  hallB_estado = digitalRead(3); // variavel recebe o valor do sensor Hall B
  hallC_estado = digitalRead(2); // variavel recebe o valor do sensor Hall C

  hall_val = (hallA_estado) + (2 * hallB_estado) + (4 * hallC_estado); // converte o valor dos 3 sensores Hall para números decimais

  switch (hall_val)
  {
    /*
     * Funcionando em sentido horário, esses valores estão descrito no TCC que está na biblioteca do Notion sobre o motor BLDC
     *
     * --------------------------------------------------------------------------------------------------------------------------
     *
     * Sequência das fases para os mosfets ligados
     * BL AH
     * BL CH
     * AL CH
     * AL BH
     * CL BH
     * CL AH
     *
     * --------------------------------------------------------------------------------------------------------------------------
     *
     * PORTD : muda o estado dos pinos usando registradores, mais rápido que o analog/digital Write
     * PORTD = B001xxx00;             Saída desejada para pinos 0-7; xxx refere-se às entradas Hall e pinos 0 e 1 do serial, que
     *não devem ser alteradas
     * Operação AND para atribuir 000 nas saídas HIGH SIDE (zerar valor anterior) e OR para alterar o valor da desejado
     *
     * --------------------------------------------------------------------------------------------------------------------------
     *
     * É importante garantir que todos os transistores estejam abertos (ou ao menos 5 deles) para fazer a comutação dos MOSFET's
     * Caso isso não seja feito, há o risco de causar um curto-circuito na bateria
     *
     * --------------------------------------------------------------------------------------------------------------------------
     */

    // FASE 1
  case 5:
    PORTE &= ~(1 << PORTE3); // Pino 5 (AH) desligado
    PORTH &= ~(1 << PORTH3); // Pino 6 (BH) desligado
    PORTH &= ~(1 << PORTH4); // Pino 7 (CH) desligado
    analogWrite(9, 0);       // Pino 9 (AL) desligado
    analogWrite(11, 0);      // Pino 10 (BL) desligado

    PORTE |= 1 << PORTE3;  // Pino 5 (AH) ligado
    analogWrite(10, pwwm); // Transistor BL com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 2
  case 1:
    PORTE &= ~(1 << PORTE3); // Pino 5 (AH) desligado
    PORTH &= ~(1 << PORTH3); // Pino 6 (BH) desligado
    PORTH &= ~(1 << PORTH4); // Pino 7 (CH) desligado
    analogWrite(9, 0);       // Pino 9 (AL) desligado
    analogWrite(11, 0);      // Pino 11 (CL) desligado

    PORTH |= 1 << PORTH4;  // Pino 7 (CH) ligado
    analogWrite(10, pwwm); // Transistor BL com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 3
  case 3:
    PORTE &= ~(1 << PORTE3); // Pino 5 (AH) desligado
    PORTH &= ~(1 << PORTH3); // Pino 6 (BH) desligado
    PORTH &= ~(1 << PORTH4); // Pino 7 (CH) desligado
    analogWrite(10, 0);      // Pino 10 (BL) desligado
    analogWrite(11, 0);      // Pino 11 (CL) desligado

    PORTH |= 1 << PORTH4; // Pino 7 (CH) ligado
    analogWrite(9, pwwm); // Transistor AL com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 4
  case 2:
    PORTE &= ~(1 << PORTE3); // Pino 5 (AH) desligado
    PORTH &= ~(1 << PORTH3); // Pino 6 (BH) desligado
    PORTH &= ~(1 << PORTH4); // Pino 7 (CH) desligado
    analogWrite(10, 0);      // Pino 10 (BL) desligado
    analogWrite(11, 0);      // Pino 11 (CL) desligado

    PORTH |= 1 << PORTH3; // Pino 6 (BH) ligado
    analogWrite(9, pwwm); // Transistor AL com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 5
  case 6:
    PORTE &= ~(1 << PORTE3); // Pino 5 (AH) desligado
    PORTH &= ~(1 << PORTH3); // Pino 6 (BH) desligado
    PORTH &= ~(1 << PORTH4); // Pino 7 (CH) desligado
    analogWrite(9, 0);       // Pino 9 (AL) desligado
    analogWrite(10, 0);      // Pino 10 (BL) desligado

    PORTH |= 1 << PORTH3;  // Pino 6 (BH) ligado
    analogWrite(11, pwwm); // Transistor CL com PWM ativo
    break;
    /* ----------------------------------------------------------------------------------------------------------------------- */
    // FASE 6
  case 4:
    PORTE &= ~(1 << PORTE3); // Pino 5 (AH) desligado
    PORTH &= ~(1 << PORTH3); // Pino 6 (BH) desligado
    PORTH &= ~(1 << PORTH4); // Pino 7 (CH) desligado
    analogWrite(9, 0);       // Pino 9 (AL) desligado
    analogWrite(10, 0);      // Pino 10 (BL) desligado

    PORTE |= 1 << PORTE3;  // Pino 5 (AH) em High
    analogWrite(11, pwwm); // Transistor CL com PWM ativo
    break;
  }
}
