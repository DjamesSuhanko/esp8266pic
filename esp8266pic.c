#define ON 0
#define OFF 1
#define MSGLEN 19
#define OK 1
#define DONE 0
#define MYID "1"
#define TCPID "4"
#define SERVER "192.168.1.232"
#define PORT "9000"
#define PROTOCOL "TCP"
#define LOW 0
#define HIGH 1
#define TIMEOUT 10
#define YES 1
#define NO 0
#define UDELAY 20
#define DELAY 100
#define CIPSIZE 3
#include <string.h>

//=-=-=-=-=-=-=-=-=-=-=-= int =-=-=-=-=-=-=-=-=-=-=-=-=-=//
//contador de uso geral                                  //
int count           = 0;                                 //
//acumulador de overflows do timer0, ate 1seg (1000ms).  //
int oneSecond       = 0;                                 //
//media de 3 amostragens do sensor                       //
int sensorSample = 0;                                    //
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//

//-=-=-=-=-=-=-=-=-=-=-=-= char =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//
// 19 bytes. triste, mas inevitavel por causa da mensagem de conexao TCP //
char text[MSGLEN]   = {0};                                               //
//tamanho da mensagem de envio. Ex.: esp8266:15,1                        //
char cipsendSize[CIPSIZE] = {0};                                         //
//acumulador de segundos. trabalha em conjunto com o oneSecond           //
char secs      = 0;                                                      //
//guarda a palavra a encontrar na resposta do reader()                   //
char word = 0;                                                           //
//guarda o valor do sensor                                               //
char sensorVal = 0;                                                      //
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//

//mapeamento dos pinos utilizados
//=-=-=-=-=-=-= PIC16F883 =-=-=-=-=-=-=-=
 /*
          -|1  U  28|-
          -|2     27|- RB6 READER (flag)
          -|3     26|- RB5 (Sensor - Analogico 13)
          -|4     25|-
          -|5     24|-
          -|6     23|-
          -|7     22|-
      VSS -|8     21|-
          -|9     20|- VDD
          -|10    19|- VSS
RESET RC0 -|11    18|- RC7 (RX)
      RC1 -|12    17|- RC6 (TX)
  DBG RC2 -|13    16|-
  LED RC3 -|14    15|- RC4
  
--->Descritivo:
RC0 fica em HIGH. Eh utilizado para fazer reset no ESP8266
RC2 utilizado apenas para debug durante o desenvolvimento
RC3 LED que devera permanecer e avaliar interrupcao na UART. Estado devera ser OFF
RC6 Transmissao UART
RC7 Recepcao UART, sem conversor de nivel porque o PIC esta configurado a 3.3v
RB5 Configurado como entrada analogica para ler sensor (nesse caso, analogico)
RB6 Flag para chamada da funcao reader() no loop principal
 */
 //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//=-=-=-=-=-=-=-= ALIASES =-=-=-=-=-=-=-=
sbit RX_port     at RC7_bit; //RX
sbit TX_port     at RC6_bit; //TX

//TESTE de delay, conexao etc.
sbit TRIS_LED    at TRISC3_bit;
sbit LED_port    at RC3_bit;

//debugger temporario
sbit TRIS_DBG    at TRISC2_bit;
sbit DBG_port    at RC2_bit;

// hard reset
sbit TRIS_RESET  at TRISC0_bit;
sbit RESET_port  at RC0_bit;

//flag para leitura da uart
sbit TRIS_READER at TRISB6_bit;
sbit READER_port at RB6_bit;

//SENSOR  ANALOGICO
sbit TRIS_SENSOR at TRISB5_bit;
sbit SENSOR_port at RB5_bit;

//UART
sbit TRIS_RX     at TRISC7_bit;
sbit TRIS_TX     at TRISC6_bit;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//limpar array. Desse modo economiza-se processamento, pois o tamanho já está definido
void clear(char *var,short int size){
    for (count=0;count<size;count++){
        var[count] = '\0';
    }
    count = 0; //i eh utilizada em outros lugares, entregar limpo o que pegou emprestado
}
//essa funcao escreve a mensagem contida em 'text'.
void write(char *msg){
    if (UART1_Tx_Idle()){
        UART1_Write_Text(msg);
        clear(text,MSGLEN);
    }
}

//o valor do sensor eh enviado atraves dessa funcao, que eh a ultima a ser executada
void sensorStatus(){
    //esp8266:id,stat - ex.: esp8266:1,0 (0 - nao esta em uso)
    write("esp8266:");
    write(MYID);
    write(",");
    write(&sensorVal);
    word = 'O';
    write("\r\n");
    Delay_ms(DELAY);
}

/*-=-=-=-=-=-=-=-=-=-= reader() -=-=-=-=-=-=-=-=-==-=-=-=-=-
Essa funcao considera apenas OK em inicio de linha. Isto pode ser muito
falho se a mensagem for dinamica, mas tratando apenas o retorno do
ESP8266, nao devera haver problemas.
*/
void reader(){
        READER_port = LOW;
        //while (UART1_Data_Ready()){
           text[count] = UART1_Read(); //le o byte na posicao text[posicao]
           //mesmo que vier \r\n nao tem erro pq text tem espaco. O prompt abre na mesma linha, nao tem terminador
               if (text[count] == '>'){
                   sensorStatus();
                   DBG_port = !DBG_port; //TODO: remover o led de debug (?)
                   count = 5;
               }
               else if (text[count] == '\r' || text[count] == '\n'){
                   text[count+1] = '\0';
                   //se casar, limpa as variaveis
                   if (word == 'O'){
                       if (strcmp(text,"OK") == 0){ //encontrou!
                           //DEBUG no LED - TODO: remover
                           DBG_port = !DBG_port;
                           //se for \r ou \n, a linha acabou de qualquer modo
                           count = 5; //limpeza por conta do proximo if
                       }
                   }
                   //esse if eh uma repeticao estupida, mas por causa da memoria.
                   //do modo 'clean' estava ocupando 194 bytes, agora esta com
                   //173 bytes de RAM em uso.
                   else if (word == 'L'){
                       if (strcmp(text,"Linked") == 0){ //encontrou!
                           //DEBUG no LED
                           DBG_port = !DBG_port;
                           //se for \r ou \n, a linha acabou de qualquer modo
                           count = 5; //limpeza por conta do proximo if
                       }
                   }

               }
               count++;
               if (count>5){
                   clear(text,MSGLEN); //apaga o buffer
               }
        //while}
}

/* =-=-==-=-=-=-=-=-= interrupt() =-=-=-=-=-=-=-=-=-
Tratamento de interrupcoes. Somente tratando interrupcao
de entrada (RX) UART e TMR0, utilizado para temporizar o envio de status para
o servidor.
*/
void interrupt(){
    //desabilita a chave geral de interrupcoes pra nao chegar mais nada
    GIE_bit = 0;
    //se for UART...
    if (RCIF_bit == 1){
        Delay_ms(300);
        //LIGA O LED sinalizando recepcao UART
        LED_port = ON;
        //apontando essa flag, o reader() eh executado no loop principal
        READER_port = HIGH;
        //breve delay para manter aceso...
        Delay_ms(200);
        //... e desliga o LED novamente
        LED_port = OFF;
        //RCIF_bit nao pode ser limpo por software, tem que ser assim nessa MCU;
        //lendo o RCREG para uma variavel
        count = RCREG;
        count = 0;
        //zera o timer e comeca novamente
        /*Se houve leitura UART, reset do timer ate que nao tenha mais mensagem
         a ler, entao temporiza os 10 segundos*/
        TMR0 = oneSecond = secs = 0;
    }
    //se for TMR0
    if (TMR0IF_bit){
        //zera o timer
        TMR0 = 0;
        TMR0IF_bit = 0; //essa eh a flag apos o overflow; limpando...
        oneSecond++; //temporizador de 1 segundo (acumula os estouros de TMR0)
        if(oneSecond > 999){   //prescaler 1:8 - 1000ms
            secs++; //somador de segundos (limpo no loop principal)
            //Apos ter acumulado um segundo de overflow, volta a 0
            oneSecond=0;
        }
    }
    //flags limpas e eventos tratados, basta levantar a chave geral novamente
    GIE_bit = 1;
}

/* =-=-=-=-=-=-=-=-= atStatus() =-=-=-=-=-=-=-=
Antes de iniciar a rotina de sensoramento, verifica se o ESP8266 esta respondendo.
AT retorna OK, portanto o tratamento dessa mensagem eh uma das mais tranquilas.
*/
void atStatus(){
    word = 'O';
    //A funcao reader() se encarrega de fazer o tratamento da resposta
    write("AT\r\n");
    Delay_ms(DELAY);
}

/* =-=-=-=-=-=-=-=-=-=-= cipmux() =-=-=-=-=-=-=-=-=-=-=
Para iniciar multiplas conexoes, primeiramente envia-se CIPMUX=1
Para saber o status atual, envia-se CIPMUX?
AT+CIPMUX=1 para habilitar ou AT+CIPMUX? para consulta.
*/
void cipmux(){
    word = 'O';
    //libera a mensagem para a funcao write()
    write("AT+CIPMUX=1\r\n");
    Delay_ms(DELAY);
}

/* =-=-=-=-=-=-=-=-=-=-= tcpStart() =-=-=-=-=-=-=-=-=-=-=
Tendo habilitado multiplas conexoes, inicia-se uma conexao TCP com o server.
O comando tem o seguinte formato:
AT+CIPSTART=id,"protocolo","ip",porta
Ex.:
AT+CIPSTART=4,"TCP","192.168.1.232",9000
*/
void tcpStart(){
    word = 'L';
    //garantir que o array de msg esta limpo, apesar de nao usado.
    /* A mensagem eh grande demais. Para compor uma string com o P16F883
    seria necessario manipular o banco de memoria atraves do bit IRP, mas
    nao seria uma tarefa trivial. Como o ESP8266 espera por \r\n (CF,LF ou
    para os puristas, 0x0D 0x0A), da pra ir escrevendo chuncks.
    */
    clear(text,MSGLEN);
    //reservar memoria pra fazer apenas 1 write nao daria porque o PIC16F883
    //reclama da alocacao de memoria. Assim resolve-se o problema e ainda
    //foram economizados muitos bytes.
    write("AT+CIPSTART=");
    Delay_ms(UDELAY);
    //strcat(text,TCPID);
    write(TCPID);
    Delay_ms(UDELAY);
    //strcat(text,',');
    write(",");
    Delay_ms(UDELAY);
    //strcat(text,"\"TCP\",");
    write("\"TCP\",");
    Delay_ms(UDELAY);
    //strcat(text,"\"");
    write("\"");
    Delay_ms(UDELAY);
    //strcat(text,SERVER);
    write(SERVER);
    Delay_ms(UDELAY);
    //strcat(text,"\",");
    write("\",");
    Delay_ms(UDELAY);
    //strcat(text,PORT);
    write(PORT);
    Delay_ms(UDELAY);
    //strcat(text,"\r\n");
    write("\r\n");
    Delay_ms(DELAY);
}

/* =-=-=-=-=-=-=-=-=- getSensorValue() =-=-=-=-=-=-=-=-=-=-
Faz uma leitura do sensor. Essa funcao eh chamada dentro do sender()
*/

char getSensorValue(){
    //deve usar 3 amostragens
    for (count=0;count<3;count++){
        //sensorSample += ADC_Get_Sample(0);
    }
    //sensorSample = sensorSample/3;
    //TODO: descomentar as duas linhas acima
    sensorSample = 20;
    
    if (sensorSample > 60){
    //em uso, levanta a flag.
        return '1';
    }
    else if (sensorSample < 60){
        //estava em uso. parou agora, entao envia
        return '0';
    }
}

/* =-=-=-=-=-=-=-=-=-=-=-=-= sender() =-=-=-=-=-=-=-=-=-=-=-=-=
Coleta o valor do sensor e compoe a mensagem de envio no formato:
esp8266:ID,STATUS
Ex.:
esp8266:20,1
Significa que o dispositivo se identifica como '20' e status de uso
0: livre, 1: em uso
*/
void sender(){
    sensorVal = getSensorValue();

     //clear usa count no loop. como a var eh reaproveitada, faz-se o clear
     //primeiro e depois calcula-se o tamanho de alocacao da msg
     clear(text,MSGLEN);
     
    //1 - aloca o tamanho da mensagem a enviar
    //10 = esp8266:X, ; -2 = \r\n ; final = esp8266:X,y\r\n onde X pode ter 2 bytes
    count = 10 + strlen(MYID); //\r\n nao entra na conta

    /* IntToStr precisa de no minimo 7 bytes, por isso eh melhor utilizar o array 
    text, depois copiar o resultado para cipsendSize, que soh ocupa 2 bytes.
    O lado negativo eh que um loop foi necessario.
    */
    IntToStr(count,text); //- - - - - - - - - - - - _ '1' '\0'
    clear(cipsendSize,CIPSIZE);

    for (count=4;count<7;count++){ //0 1 2 3 [4] [5] [6]
        //se a posicao nao for em branco e nem terminador nulo...
        if (text[count] != ' ' && text[count] != 0){
            if (cipsendSize[0] == 0){
                cipsendSize[0] = text[count];
            }
            else{
                cipsendSize[1] = text[count];
                break;
            }
        }
    }
    //limpa as variaveis utilizadas   (count eh limpo pela funcao clear())
    clear(text,MSGLEN);

    /*A mensagem a enviar eh dividida em 2 etapas:
    CIPSEND: Indica a conexao (pelo ID) e o tamanho da mensagem que sera enviada.
    Apos enviar essa mensagem, um prompt deve ser recebido na leitura. A interrupcao
    devera tratar a resposta, aguardando por um '>', entao a segunda fase eh o envio
    da mensagem, do tamanho informado nesse comando. O envio de '>' pelo ESP8266
    eh tratado na interrupcao, que informa ao reader() que tem dados. A funcao
    reader() le e se encontra '>', executa sensorStatus().
    */
    strcpy(text,"AT+CIPSEND="); //formata a primeira mensagem
    strcat(text,TCPID);  //escolha da conexao a utilizar (tcp 4)
    strcat(text,",");    //...
    strcat(text,cipsendSize); //... e concatena o tamanho da msg TCP
    strcat(text,"\r\n");
    
    clear(cipsendSize,CIPSIZE);
    word = '>';
    count = 0; //pena ter que fazer isso aqui, mas o contador precisa estar limpo
               //na interrupcao do reader
    write(text);
    Delay_ms(DELAY);
}

/* =-=-=-=-=-=-=-=-=-=-= hardReset() =-=-=-=-=-=-=-=-=-=
Da menos trabalho fazer reset no pino do que montar comando AT, tratar
resposta, mudar flag de status, entao esquecamos AT+RST...
*/
void hardReset(){
    RESET_port = LOW;
    Delay_ms(50);
    RESET_port = HIGH;
}

//envia uma amostra para o server seguindo o criterio:
void sendSample(){
    //1 - esp8266 responde?
    atStatus();
    //Forca cipmux, nao importa se ja esta
    cipmux();
    //estabelece a conexao TCP remota
    tcpStart();
    //envia a mensagem
    sender();
}

void main(){
    //Desabilita comparadores
    C1ON_bit = 0;
    C2ON_bit = 0;

    //Tratamento dos pinos Analogicos
    ANSEL  = 0x00;              // Leitura digital
    ANSELH = 0b00100000;         // pino 26 (AN13) analogico (sensor)

    //Inicializacao do modulo AD
    ADC_Init();

    //TRIS
    TRISA        = 0xFF; //TUDO INPUT nas portas A...
    TRIS_LED     = 0; //output, mas aterra. input eh so pra leitura analogica
    TRIS_SENSOR  = 1; //input para leitura analogica
    TRIS_DBG     = 0; //aterra tb
    TRIS_RESET   = 0; //hard reset
    TRIS_READER  = 0; //flag para ler da UART
    
    //O pino de reset se mantem em HIGH exceto deseje-se um reset
    RESET_port   = HIGH;
    //LED de depuracao - deve ser removido e desconfigurado tambem
    DBG_port     = HIGH;
    //baixando o pino, aterra e acende (porque o LED esta sendo alimentado fora)
    LED_port     = OFF;
    //marca leitura como low no inicio
    READER_port  = LOW;
    
    //interrupcoes
    //               RCIE___./ .___PEIE___./ .___GIE
    RCIE_bit   = 1;         // habilitar interrupcao  em RX
    TXIE_bit   = 0;         // desabilita interrupcao em TX (default)
    PEIE_bit   = 1;         // habilitar/desabilitar interrupcoes perifericas (INTCON)
    GIE_bit    = 1;         // liga chave geral das interrupcoes
    TMR0IE_bit = 1;         //interrupcao do timer0
    //PIE1     = 0b01100000; // - AD RX TX SSP CCP T2 T1
    
    //oscilador interno a 8MHz
    OSCCON = 0b1110101;     

     /* =-=-=-=-= Calculando o valor do timer =-=-=-=-=-=
    ciclo de maquina = Fosc/4 para o oscilador interno
    Tempo (t) do overflow:
    t = ciclo * prescaler * TMR0
    O clock esta configurado para 8Mhz (8000000)
       
    ciclo = 8/4 = 2us
    prescaler = 1:2 (2)
    t = 2 * 2 * 256 (TMR0 guarda de 0 a 255 antes do overflow porque ele tem 8 bits)
    t = 1ms
    Desse modo, a piscada eh a cada 1 segundo. Com prescaler a 1:4, dobra o tempo, logo,
    blink de 1 segundo.

    //RBPU   1 - pullup disabled
    //INTEDG 0 - detecta interrupcao em LOW
    //T0CS   0 - Fosc/4 (clock interno)
    //T0SE   0 - incrementa da baixa pra alta
    //PSA    0 - prescaler enabled
    //PS2,PS1,PS0 010 - 1:4 no prescaler do TMR0
    */
    OPTION_REG = 0b10000010;

    UART1_Init(38400);
    Delay_ms(500);

    TMR0 = 0;
    
    while(1){ //loop infinito
        //Se a flag estiver up, ler UART. Essa flag sobe na interrupcao
        if (READER_port){
            READER_port = OFF;
            reader();
        }
        
        /*
        Os acumuladores de segundo e segundos sao incrementados na interrupcao.
        Aqui simplesmente escolhe-se o numero de segundos para disparar uma funcao.
        TIMEOUT eh uma macro que esta configurada para ~10 segundos (6 msg/minuto).
        A condicional abaixo verifica tambem se nao existe nada pra ser lido, para
        evitar atropelamento de status.
        */
         if (secs>TIMEOUT-1 && READER_port == LOW){
               //LED_port = !LED_port;
               secs=0;
               //amostra temporizada
               sendSample();
         }
    }
}