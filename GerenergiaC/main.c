#include <stdlib.h>
#include <stdio.h>
#include <tm4c123gh6pm.h>
#include <inttypes.h>

/***********************************************************************************************************************************************************
 * DEFINES
 **********************************************************************************************************************************************************/

#define CONST_LDR 40.95
#define SYSCTL_RCGCGPIO_PORTA (*((volatile uint32_t*)0x43FCC100))
#define F_PWM  16000000
//Tempo de 20ms, feito o cálculo e deu 320000, que dividito por 4 deu 80000
#define SYSTICK_TIME 80000
//ADC0_SSFIFO0_R REGISTRADOR DO ADC
#define SOMA 5

/***********************************************************************************************************************************************************
 * FUNÇÕES
 **********************************************************************************************************************************************************/

void setup();
void configSerial();
void sendData(unsigned char dados);
void setupADC();
void setupInterrupcaoSysTick(long int time);
void wfi();
void configInterrupcaoSerial();
void trataInterrup();
void trataST();
void configPWM(uint16_t percent);
void setupButtonTiva();
void loop(uint32_t cycles);
void LCD_write(unsigned char bytes);
void pulseEnable();
void setupLCD();
void LCD_print(unsigned char c);
void LCD_print_int(uint8_t n);
void LCD_cursor(uint8_t lin, uint8_t col);
void LCD_clear();
void LCD_write_name();
void PWM_begin(uint16_t freq, uint8_t porcentage);
//Se for alterar freq altera a de baixo também
void alteraPowerPWM(uint8_t percent);
uint8_t calculaPotMedia();
uint8_t readData();
char* itoa(int i, char b[]);
void LCD_write_labels();
void setupButtonTiva();
void trataBotao();
void setupInterrupcaoPort_x_Type_x();
void LCD_write_values();

/***********************************************************************************************************************************************************
 * VARIAVEIS
 **********************************************************************************************************************************************************/

typedef struct {
    uint8_t numPontos;
    uint8_t valorMin[100];
    uint8_t valorMax[100];
    uint8_t power[100];
    uint8_t id;
    uint8_t pontoAtivo;
    char nome[16];
}Perfil;

Perfil perfil;

uint8_t LDR;
uint8_t flagManual = 1;
uint8_t flagPerfil = 0;

int i = 0, j = 0, w = 0;

char powerLCD[33];

uint8_t pot[10];
uint8_t contpot = 0;
uint8_t potMedia = 0;
uint32_t contadorEnvioLCD = 0 ;

/***********************************************************************************************************************************************************
 * FUNÇÕES DE INTERRUPÇÕES
 **********************************************************************************************************************************************************/

//A cada 20ms (50HZ) ele chama essa função da qual envia um set de bytes representando os dados a serem persistidos no banco e exibidos na tela em tempo real
void trataST()
{
    if(contpot == 10){
        contpot = 0;

        potMedia = calculaPotMedia();
    }

    //Calculo potência instantânea que é armazenada no vetor
    //media a cada 200ms!!

    //tensão instantânea / Resistência = corrente
    // 23.7 do divisor de tensao
    // 6.7 resistencia interna do motor
    //Corrente * tensão = potência
//    pot[contpot] = (((((ADC1_SSFIFO0_R * 3.3)/4095)*100)/23.7)/6.7) * ((((ADC1_SSFIFO0_R * 3.3)/4095)*100)/23.7);

    //0.22 = 220mAh corrente de drenagem com 12v
    pot[contpot] = ((((ADC1_SSFIFO0_R * 3.3)/4095)*100)/23.7) * 0.22 ;
    contpot++;

    //Enviar aviso de estatistica
    sendData('^');

    //Enviar LDR
    sendData(LDR);

    //Enviar PWM
    sendData(perfil.power[perfil.pontoAtivo]);

    //Enviar Pot media
    if (potMedia >= 16)
        sendData(0);
    else
        sendData(potMedia);

    //LCD_print_int(potMedia);

    //Enviar ponto de operacao ativo do perfil
    sendData(perfil.pontoAtivo);
}

//Toda vez que ele receber um byte, ele verifica se é um perfil e lê os valores, preenchendo o struct do perfil
void trataInterrup(){
    //Antigo: UART0_FR_R
    char receivedData = ((char)(UART0_DR_R & 0xFF));


    //SOMANDO CONTADOR ENVIO LDC
    contadorEnvioLCD ++ ;
    if (contadorEnvioLCD == 99999){
//        LCD_write_name();
        LCD_write_values();
        contadorEnvioLCD = 0;
    }

    //Se o char recebido for um '=' significa que está recebendo um perfil
    if (receivedData == '='){
        flagPerfil = 1;
        flagManual = 0;
        perfil.numPontos = readData();

        for(i = 0; i < perfil.numPontos; i++){
            perfil.valorMin[i] = readData();
            perfil.valorMax[i] = readData();
            perfil.power[i] = readData();
        }

        perfil.id = readData();

        for(i = 0; i < 16; i++){
            perfil.nome[i] = readData();
        }

        flagManual = 0;
        LCD_write_name();
    }
}

void trataBotao()
{
    if (flagManual == 0){
        flagManual = 1;
        perfil.pontoAtivo = 0;
        LCD_write_labels();
        sendData('~');
    }

    GPIO_PORTF_ICR_R |= 1 << 4;
    GPIO_PORTF_ICR_R |= 1 << 0;
    //bit 6 do portB: clique do botão digital do joystick analógico
    //UNICO CHAR QUE ENVIAREMOS PRIORITARIAMENTE, POR UTILIZARMOS COMO FUNÇÃO 'START/PAUSE'
    if (((GPIO_PORTF_DATA_R >> 4) & 1) == 0 )
    {

        if (perfil.power[perfil.pontoAtivo] != 100){
            perfil.power[perfil.pontoAtivo] += SOMA ;
        }
    } else if (((GPIO_PORTF_DATA_R >> 4) & 1) == 1) {

    }
    if (((GPIO_PORTF_DATA_R >> 0) & 1) == 0 )
    {
        if ( perfil.power[perfil.pontoAtivo] != 0){
            perfil.power[perfil.pontoAtivo] -= SOMA ;
        }

    } else if (((GPIO_PORTF_DATA_R >> 0) & 1) == 1) {

    }
    LCD_write_labels();
    alteraPowerPWM(perfil.power[perfil.pontoAtivo]);

}

/***********************************************************************************************************************************************************
 * MAIN
 **********************************************************************************************************************************************************/

int main()
{
    //Setup de todos os módulos
    setup();

    //Pooling que lê o valor de LDR, compara com os valores de pontos de operação e seta o PWM de acordo
    while (1)
    {
        //lê o ldr e converte para uint8_t
        int aux = ADC0_SSFIFO0_R/40.95;
        LDR = (uint8_t) aux;

        if (perfil.numPontos > 0 && flagManual == 0){
            for(j = 0; j < perfil.numPontos; j++){
                if (LDR > perfil.valorMin[j] && LDR < perfil.valorMax[j]){
                    alteraPowerPWM((uint8_t)perfil.power[j]);
                    perfil.pontoAtivo = j;
                }
            }
        }
    }
}

/***********************************************************************************************************************************************************
 * CONFIGURAÇÃO DOS MÓDULOS
 **********************************************************************************************************************************************************/

void setup()
{
    setupADC();
    setupButtonTiva();
    setupInterrupcaoPort_x_Type_x();
    setupLCD();
    LCD_clear();
    LCD_write_labels();
    configSerial();
    uint8_t porc = 50 ;
    perfil.power[perfil.pontoAtivo] = porc;
    PWM_begin(32000, perfil.power[perfil.pontoAtivo]);

    setupInterrupcaoSysTick(SYSTICK_TIME); // usar func trataST(void)
}

void configSerial()
{
    //Instruções na página 902 do manual, ative a UARTO nos pinos A0 e A1 (USB)
    SYSCTL_RCGCGPIO_PORTA = 1; //Ativando port A

    SYSCTL_RCGCUART_R = 1; //Clock para o Port A
    GPIO_PORTA_AFSEL_R = 3; //Alternate Function SELect
    GPIO_PORTA_DEN_R |= 3; //Digital ENable nos pinos 0 e 1, visto que Serial ainda é um sinal digital
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0xFFFFFF00) + 0x00000011; //Definir função alternativa do PortA como UART
    GPIO_PORTA_AMSEL_R &= ~0x03;
    UART0_CTL_R &= ~UART_CTL_UARTEN; //Desabilita o UART
    UART0_IBRD_R = 8; //Baud Rate IBRD = 16000000 / (16*115200) = int(86805555...) ~= int(8,6806) = 8
    UART0_FBRD_R = 44; //Baud Rate FBRD = 0.6806 * 64 + 0.5 = int(44.0584) ~= 44
    UART0_LCRH_R = (UART_LCRH_WLEN_8 | UART_LCRH_FEN); //Tamanho: 8 bits, um bit de stop, sem paridade, com FIFO's
    UART0_CC_R = 0; // Só para ter certeza que o clock usado é o da tiva mesmo
    configInterrupcaoSerial();
    UART0_CTL_R |= UART_CTL_UARTEN; //Habilita o UART
}

void configInterrupcaoSerial(){
    //UART0_FR_R - REGISTRADOR DE LEITURA SERIAL
    //Habilitar NVIC
    //Registradores de interrupção do UART
    //dentro do capitulo da uart tem um da interrup, im e mais um para configurar
    //ICR e o RIS é o que usam para ler

    NVIC_EN0_R |= 1<<5;
    UART0_IM_R = UART_IM_RXIM | UART_IM_TXIM | UART_IM_RTIM;
    UART0_IFLS_R = UART_IFLS_RX7_8;
}

void setupADC()
{
    SYSCTL_RCGCGPIO_R |= 1 << 3; // Energizando o PORTD
    SYSCTL_RCGCADC_R |= 1 << 0 | 1 << 1; // HABILITANDO/ATIVANDO ADC 0 E 1
    GPIO_PORTD_AFSEL_R |= 1 << 3 | 1 << 1; // FUNÇÕES ESPECIAIS PARA OS PINOS 2 E 1 DO PORT D
    GPIO_PORTD_AMSEL_R |= 1 << 3 | 1 << 1; // DEFININDO QUE A FUNÇÃO ESPECIAL É O ADC

    ADC0_ACTSS_R = 0; //ADC Módulo 0 - Ativador da Sequencialização (Desabilitando as Sequências por segurança)
    ADC0_EMUX_R = 15; //Data sheet inidica que 15 - sempre ficara lendo (existem outras funcoes) ;
    ADC0_SSMUX0_R = 6; // Canal de leitura - Lendo PD1 - Canal 6
    ADC0_SSCTL0_R = 2 << 0; // (  TS0  -  IE0  -  END0  -  D0  )
                            // ( pos3  - pos2  -  pos1  - pos0 )
                            // (   0   -   0   -    1   -   0  ) = 2
                            // D0   -> Interrupções - Default 0
                            // END0 -> Final Sequence (Bit 1 significa final da leitura de bits dos Canais)
                            // IE0  -> Interruption Enable (Ligar junto com o D0)
                            // TS0  -> Sensor de temperatura interno da TIVA (Gera interrupcoes ao esquentar mto)
    ADC0_ACTSS_R = 1 << 0;  // ADC Módulo 0 - Ativador da Sequencialização
    ADC0_PSSI_R = 1 << 0;   // repetir o valor acima

    ADC1_ACTSS_R = 0;
    ADC1_EMUX_R = 15;
    ADC1_SSMUX0_R = 4;  // Canal de leitura - Lendo PD0 - Canal 7
    ADC1_SSCTL0_R = 2 << 0;
    ADC1_ACTSS_R = 1 << 0;
    ADC1_PSSI_R = 1 << 0;

}

void setupInterrupcaoSysTick(long int time)
{
    //TIME IN MILISECONDS
    NVIC_ST_RELOAD_R = time;
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN;
}

void setupInterrupcaoPort_x_Type_x()
{
    NVIC_EN0_R |= 1 << 30;
    GPIO_PORTF_IBE_R |= 1 << 0 | 1 << 4;
    GPIO_PORTF_IM_R |= 1 << 0 | 1 << 4 ;
}

void setupButtonTiva()
{
    SYSCTL_RCGCGPIO_R == 0 ? (SYSCTL_RCGCGPIO_R = 1 << 5) : (SYSCTL_RCGCGPIO_R |= 1 << 5) ;
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R |= 1 << 0 ;
//    if( ( (GPIO_PORTF_LOCK_R >> 0) & 1) == 1){
//        GPIO_PORTF_LOCK_R = 0 << 0 ;
//    }

//    GPIO_PORTF_DIR_R =  0 << 1;
    GPIO_PORTF_PUR_R == 0 ? (GPIO_PORTF_PUR_R = 1 << 4 | 1 << 0 ) : (GPIO_PORTF_PUR_R |= 1 << 4 | 1 << 0);
    GPIO_PORTF_DEN_R == 0 ? (GPIO_PORTF_DEN_R = 1 << 4 | 1 << 0 ) : (GPIO_PORTF_DEN_R |= 1 << 4 | 1 << 0);
}

void setupLCD()
{
    //PORT F É USADO NOS PINOS EN, RW E RS
    //ATIVANDO PORTF => 5
    SYSCTL_RCGCGPIO_R |= 1 << 5;
    GPIO_PORTF_DIR_R |= 1 << 2 | 1 << 3;// | 1 << 0;
    GPIO_PORTF_DATA_R = 0;
    GPIO_PORTF_DEN_R |= 1 << 2 | 1 << 3;// | 1 << 0;

    //Recebe os 4 ultimos bits de um byte
    //ATIVANDO PORTC => 2
    SYSCTL_RCGCGPIO_R |= 1 << 2;
    GPIO_PORTC_DIR_R |= 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7;
    GPIO_PORTC_DATA_R = 0;
    GPIO_PORTC_DEN_R |= 1 << 4 | 1 << 5 | 1 << 6 | 1 << 7;

    //recebe os 4 primeiros bits de um byte
    //ATIVANDO PORTE => 4
    SYSCTL_RCGCGPIO_R |= 1 << 4;
    GPIO_PORTE_DIR_R |= 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3;
    GPIO_PORTE_DATA_R = 0;
    GPIO_PORTE_DEN_R |= 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3;
    GPIO_PORTE_DATA_R = 1 << 0 | 1 << 1 | 1 << 2 | 1 << 3;

    loop(1500);

    //script de inicialização do LCD (a cadeia de comandos dos slides lá)
    LCD_write(0x30);
    loop(500);
    LCD_write(0x30);
    loop(16);
    LCD_write(0x30);
    loop(16);
    LCD_write(0x38);
    LCD_write(0x10);
    LCD_write(0x0C);
    LCD_write(0x06);
}

/***********************************************************************************************************************************************************
 * FUNÇÕES AUXILIARES
 **********************************************************************************************************************************************************/

uint8_t readData(){
    //Enquanto a fila estiver vazia
    while((UART0_FR_R & UART_FR_RXFE) != 0);
    return ((char)(UART0_DR_R & 0xFF));
}

uint8_t calculaPotMedia(){
    int aux = 0, cont = 0;
    for (cont = 0; cont < 10; cont++){
        aux += pot[cont];
    }

    uint8_t retorno = (aux/10) ;
//    if (retorno >= 16) return 0;
    return retorno;
}

void sendData(unsigned char dados)
{
    while ((UART0_FR_R & UART_FR_TXFF) != 0); //Aguarda enquanto a flag de FIFO cheia estiver acionada
    UART0_DR_R = dados; //coloca os dados requisitados para envio através do registrador UART0 Data Register, adicionando-o na FIFO
}

void PWM_begin(uint16_t freq, uint8_t percent)
{

    //Ligar o PortB
    SYSCTL_RCGCGPIO_R |= 1 << 1;
    loop(2000);
    //Habilitar função alternativa do PB6
    GPIO_PORTB_AFSEL_R = 1 << 6;
    loop(2000);
    //Definir que a função alternativa do PB6 é PWM
    GPIO_PORTB_PCTL_R = 1 << 26;
    loop(2000);
    //Habilita PB6 digitalmente
    GPIO_PORTB_DEN_R = 1 << 6;
    loop(2000);

    loop(2000);
    //Ativa o modulo de PWM 0
    SYSCTL_RCGCPWM_R = 1;
    loop(2000);
    //Nivel baixo em LOAD / Nivel alto em CMPA
    PWM0_0_GENA_R = 0xC8;
    loop(2000);
    //Valor que frequencia do PWM
    PWM0_0_LOAD_R = freq;
    loop(2000);
    //Valor de porcentagem do PWM
    PWM0_0_CMPA_R = ((freq * percent) / 100) - 1;
    loop(2000);
    //Ativa o controle do elemento 0 do PWM0
    PWM0_0_CTL_R = 1;
    loop(2000);
    //Inicializa o PWM0
    PWM0_ENABLE_R = 1;
    loop(2000);

}

void wfi()
{
    __asm("WFI");
}

char* itoa(int i, char b[]){
    char const digit[] = "0123456789";
    char* p = b;
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    int shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return b;
}

/***********************************************************************************************************************************************************
 * LCD
 **********************************************************************************************************************************************************/

//Faz o processador aguardar por alguns ciclos de clock, usado apenas para configurar o LCD mesmo, afinal, systick para que né
void loop(uint32_t cycles)
{
    uint32_t k = 0;
    for (k = 0; k < cycles * 40; k++);
}

//Escreve um comando no LCD e dá um pulso de clock no pin En para executar o comando
void LCD_write(unsigned char bytes)
{
    GPIO_PORTC_DATA_R = bytes;
    GPIO_PORTE_DATA_R = bytes;
    GPIO_PORTF_DATA_R = 0; // En -> 0
                           // RW -> 0
                           // RS -> 0

    loop(6);
    GPIO_PORTF_DATA_R = 1 << 2; // En -> 1
                                // RW -> 0
                                // RS -> 0
    loop(6);
    GPIO_PORTF_DATA_R = 0; // En -> 0
                           // RW -> 0
                           // RS -> 0
    loop(40);
}

void alteraPowerPWM(uint8_t percent)
{
    PWM0_0_CMPA_R = ((32000 * percent) / 100) - 1;
}

void LCD_write_name(){
    LCD_cursor(0,0);
    for (w = 0; w < 16; w++){
        //LCD_cursor(0,w);
        LCD_print(perfil.nome[w]);
    }
}
void LCD_write_values(){
    LCD_cursor(1,0);
    LCD_print('P');
    LCD_print('O');
    LCD_print('W');
    LCD_print('=');
    LCD_cursor(1, 4);
    LCD_print_int((int)potMedia);


    LCD_cursor(1,8);
    LCD_print('P');
    LCD_print('W');
    LCD_print('M');
    LCD_print('=');
    LCD_cursor(1, 12);
    LCD_print_int((int)perfil.power[perfil.pontoAtivo]);
}

void LCD_write_labels(){
    LCD_cursor(0,0);
    perfil.nome[0] = 'P';
    perfil.nome[1] = 'F';
    perfil.nome[2] = ' ';
    perfil.nome[3] = 'L';
    perfil.nome[4] = 'I';
    perfil.nome[5] = 'V';
    perfil.nome[6] = 'R';
    perfil.nome[7] = 'E';
    perfil.nome[8] = ' ';
    perfil.nome[9] = ' ';
    perfil.nome[10] = ' ';
    perfil.nome[11] = ' ';
    perfil.nome[12] = ' ';
    perfil.nome[13] = ' ';
    perfil.nome[14] = ' ';
    perfil.nome[15] = ' ';

    LCD_write_name();

    LCD_cursor(1,0);
    LCD_print('P');
    LCD_print('O');
    LCD_print('W');
    LCD_print('=');
    LCD_cursor(1, 4);
    LCD_print_int((int)potMedia);


    LCD_cursor(1,8);
    LCD_print('P');
    LCD_print('W');
    LCD_print('M');
    LCD_print('=');
    LCD_print_int((int)perfil.power[perfil.pontoAtivo]);
}

//Envia um char para o LCD e avisa que é uma letra a ser enviada para exibir na posição configurada do cursor
void LCD_print(unsigned char c)
{
    GPIO_PORTC_DATA_R = c;
    GPIO_PORTE_DATA_R = c;
    GPIO_PORTF_DATA_R = 1 << 3; // En -> 0
                                // RW -> 0
                                // RS -> 1
    loop(6);
    GPIO_PORTF_DATA_R = 1 << 2 | 1 << 3; // En -> 1
                              // RW -> 0
                              // RS -> 1
    loop(6);
    GPIO_PORTF_DATA_R = 1 << 3; // En -> 0
                                // RW -> 0
                                // RS -> 1
    loop(40);
}

//Converte números inteiros para o LCD, convertendo para um array de char primeiro
void LCD_print_int(uint8_t n){
    itoa(n, powerLCD);
    if (n > 99){
        for (w = 0; w < 3; w++){
            LCD_print(powerLCD[w]);
        }
    } else if (n > 9){
        for (w = 0; w < 2; w++){
            LCD_print(powerLCD[w]);
        }
        LCD_print(' ');
    } else {
        LCD_print(powerLCD[0]);
        LCD_print(' ');
        LCD_print(' ');
    }
}

//Posiciona o cursor do LCD na linha e coluna desejados
void LCD_cursor(uint8_t lin, uint8_t col){
    if (lin == 0)
    {
        LCD_write(0x02);
        uint32_t cs = 0;
        for (cs = 0; cs < col; cs++)
        {
            LCD_write(0x14);
        }
    }
    else if (lin == 1)
    {
        LCD_write(0xC0 + col);
    }
    loop(1600);
}

//Limpa a tela do LCD
void LCD_clear()
{
    LCD_write(0x01);
    loop(1600);
}
