#define F_CPU 16000000

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>    // do zmiennej typu BOOL
  

int prev_err = 0, przestrzelony = 0; 
int blad, pop_blad = 0, Kp = 1, Kd = 0; 
int V_zad = 40; // wymagane zmienne globalne 
int tab_czujnikow[7] = {1,0,7,2,5,4,3}; // ustawienie czujnikow na płytce
int czujniki[7];    // wartości czujnikow
char send[4];


void UART_init(unsigned int baud,bool RX,bool TX){
    // ustawienie prędkości
    unsigned int ubrr;
    ubrr=(((F_CPU / (baud * 16UL))) - 1);
    UBRRH = (unsigned char)(ubrr>>8);
    UBRRL = (unsigned char)ubrr;
    // UBRRH = 0;   // 9600 bps @ 16MHz
    // UBRRL = 0x67;    // 9600 bps @ 16MHz
    // ustawienie bitów parzystości
    UCSRC |= 1<<UPM1;// 1 bit parzystości
    // ustawienie ilości bitów danych
    UCSRC |= (1 << URSEL) | (1<<UCSZ1) | (1<<UCSZ0);    //
    // ustawienie bitów stopu
    // domyślnie jest 1
 
    if(RX){
        UCSRB |= 1<<RXEN;
    }
    if(TX){
        UCSRB |= 1<<TXEN;
    }
}

void USART_send( unsigned char data){
    while(!(UCSRA & (1<<UDRE)));
    UDR = data;
}

void toStringInt(int wartosc){
    for(char i=3;i>=0;i--){
        send[i] = (wartosc%10) + 0x30;
        wartosc /= 10;
    }
}



void init(){
    DDRB |= _BV(0) | _BV(1) | _BV(2);
    DDRD |= _BV(2) | _BV(5) | _BV(6) | _BV(7);

    PORTD &= ~_BV(2);   // STANDBY
    PORTD &= ~_BV(6);   // AIN1
    PORTD |= _BV(5);    // AIN2

    PORTD &= ~_BV(7);   // BIN1
    PORTB |= _BV(0);    // BIN2

    TCCR1A |= _BV(COM1A1) | _BV(COM1B1);	//wyj�cia PB1(OC1A) i PB2(OC1B) sterowane przez Timer1
	TCCR1A |= _BV(WGM10);	//FastPWM 8-bit przy WGM11=0 (tutaj tylko WGM10 ustawiany na 1); Table 21-5 str.137
	TCCR1B |= _BV(WGM12);	//FastPWM 8-bit; Table 25-2 str.137
    TCCR1B |= _BV(CS11);	//wyb�r �r�d�a zegarowego dla Timer1; Preskaler na clock/8; Table 21-6 str.139; dla 16MHz daje ~5,4kHz
	// OCR1A = 80;		//wype�nienie ~4% (10/256)
	// OCR1B = 250;	//wype�nienie 50% (127/256)
    
    // DDRD  |= _BV(3);	//pin jako wyj�cie. Mo�na bardziej wprost DDRD  |= _BV(PB7) lub  DDRB |= _BV(DDB5);
    PORTD |= _BV(3);
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADEN); 
    ADMUX |= _BV(REFS0);
}



void rightMotor(int v){
    if(v > 0){
        if(v > 250)
            v = 250;
        PORTD |= _BV(7);   // BIN1
        PORTB &= ~_BV(0);    // BIN2
        OCR1B = v;
    }else{
        if(v < -250)
            v = -250;
        PORTD &= ~_BV(7);   // BIN1
        PORTB |= _BV(0);    // BIN2
        OCR1B = -v;
    }
}

void leftMotor(int v){
    if(v > 0){
        if(v > 250)
            v = 250;
        PORTD |= _BV(6);   // AIN1
        PORTD &= ~_BV(5);    // AIN2
        OCR1A = v;
    }else{
        if(v < -250)
            v = -250;
        PORTD &= ~_BV(6);   // AIN1
        PORTD |= _BV(5);    // AIN2
        OCR1A = -v;
    }
}

void stop(){
    leftMotor(0);
    rightMotor(0);
}

void print(){
    //   PORTD &= ~_BV(2);
    for(int i = 0; i < 7;i++){
        toStringInt(czujniki[i]);
        USART_send('(');
        USART_send(i+0x30);
        USART_send(')');
        USART_send(' ');
        for(short j=0;j<4;j++){
            USART_send(send[j]);
        }
        USART_send('\n');
        USART_send('\r');
    }
    USART_send('\n');
    USART_send('\r');
    return ;
}

void petla_LF();
int PD();
int licz_blad();
void czytaj_adc();


int main(void){
    // init
    init();
    PORTD |= _BV(2);    // standby (?)
    UART_init(9600,false,true);
    bool flaga = false;
    // głowna petla
    while(1){
        if(!(PIND & (1<<PD3))){    // przycisk gdy wciśniety
            flaga = !flaga;
            if(!flaga){
                // leftMotor(250);
                // rightMotor(250);
                stop();
                // leftMotor(100);
            }
            //deleay ulatwiajacy obsluge przycisku
            _delay_ms(1000);
        }else{
            if(flaga)
                petla_LF();
        }

        _delay_ms(10);
    }
    return 0;
}

void petla_LF(){ 
    czytaj_adc(); 
    blad = licz_blad(); 
    int regulacja = PD(); 
    leftMotor(V_zad + blad);
    rightMotor(V_zad - blad); 
}

void czytaj_adc() 
{ 
    for(int i=0; i<7; i++){ 
        ADMUX &= 0b11000000; 
        ADMUX |= tab_czujnikow[i];
        // ADMUX |= 0x0e; 
        ADCSRA |= _BV(ADSC); 
        while(ADCSRA & _BV(ADSC)) {}; 
        
        // czujniki[i] = (ADCH<<8) + ADCL;
        czujniki[i] = ADCL + (ADCH<<8);

        // czujniki[i] = ADCL;
        // char h = ADCH;

        if(czujniki[i] > 200)                    // odczyt 8 starszych bitów i progowanie; próg = 150 
            czujniki[i] = 1; 
        else 
            czujniki[i] = 0; 
    } 
}

int licz_blad(){ 
    int err = 0; 
    int ilosc = 0; 
    
    for(int i=0; i<7; i++) 
    { 
        err += czujniki[i]*(i-3)*10; 
        ilosc += czujniki[i]; 
    } 
    
    if(ilosc != 0) 
    { 
        err /= ilosc; 
        prev_err = err; 
    } 
    else 
    { 
        if(prev_err < -20)                // linia ostatanio widziana po lewej stronie - ustalamy ujemny błąd większy od błędu skrajnego lewego czujnika 
            err = -40; 
        else if(prev_err > 20)            // linia ostatanio widziana po prawej stronie - analogicznie do powyższego 
            err = 40; 
        else                            // przerwa w linii - trzeba jechać prosto 
            err = 0; 
    } 
    
    return 2*err; 
}

int PD(){ 
    //zmienna blad zawiera aktualny wynik fukcji licz_blad() 
    int rozniczka = blad - pop_blad; 
    pop_blad = blad; 
    return Kp*blad + Kd*rozniczka; 
}

