/*
 * File:   main.c
 * Author: Bastien THOMAS et Raphael LAVERGNE
 *
 * Created on 8 novembre 2022, 12:07
 */

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <pic18F4550.h> 
#include "glcd.h" // file header of glcd

//************************************************************************
// Configuration
//************************************************************************

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSC_EC        // Oscillator Selection bits (HS oscillator (HS))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

//************************************************************************
// on definit la fréquence à 8 MHz
//************************************************************************

#define _XTAL_FREQ 8000000

//************************************************************************
// Variables globales
//************************************************************************

unsigned char tab_bin[]={0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F}; // tableau binaire pour le 7 segment
int d0=0,d1=0,d2=0,d3=0; // pour les DIS du 7 segment 

int adc=0; // entier pour récuperer l'ADC
int nb=0; // nombre pour choisir le mode
int compteur=0; // compteur pour la conversion 
int position_x=0; // recuperer le compteur pour la position X du Pixel
unsigned int tableau[128]={0}; // tableau d'entiers qui recupere la valeur de l'ADC pour chaque indice du tableau

//************************************************************************
// Initialisation
//************************************************************************

void prog_init()
{
   OSCCON=0x72; // internal oscillator of 8MHz Frequency   
  
   /* PORTA, PORTB, PORTC et PORTD sont des sorties */
   TRISA = 0x00;
   PORTA = 0x00; // PORTA pour afficher les valeurs sur chaque DIS du 7 SEGMENT
   
   TRISB = 0b10000000; // le bit RB7 est une entrée (pour le bouton) 
   PORTB = 0x00; // le PORTB est lié au GLCD pour la PIC18F (RB0 à RB5)
   TRISC = 0x00;
   PORTC = 0x00; // le PORTC sert à afficher les LEDS RC0 à RC2 pour vérifier le fonctionnement du changement de mode
   /* Rappel : Nous avons aussi utilisé le PORTC en sortie pour la PWM */
   
   TRISD = 0x00;
   PORTD = 0x00; // le PORTD est la valeur du 7 segment et aussi la donnée pour l ecran GLCD
   
   ADCON1=0b00001101; // Configuration pour les bits PCFG 1101, dont AN0 et AN1 sont des pins analogues en entrée
   /* Rappel : AN0 le potentiomètre qui régule la tension d'entrée entre 0 et 1023 grace à l'ADC */
   /* et AN1 le potentiomètre qui nous servira pour la PWM */
      
   /* Configuration pour le Timer 0 */
   T0CONbits.TMR0ON = 1;
   T0CONbits.T08BIT = 1;
   T0CONbits.T0CS = 0;
   
   /* Prescaler 32 du Timer 0 */
   T0CONbits.T0PS2 = 1; 
   T0CONbits.T0PS1 = 0; 
   T0CONbits.T0PS0 = 0;
   T0CONbits.PSA = 0;
   
   /* Interruption pour le Timer 0 */
   INTCONbits.GIE = 1; // Global interrupt
   INTCONbits.TMR0IE = 1; 
   TMR0 = 0;
   
   /* Interruption (externe) pour le bouton */
   PIE1bits.ADIE = 1;
   INTCONbits.PEIE = 1; // Peripheral interrupt 
   INTCONbits.RBIE = 1;

   /* Interruption pour l'ADC sur le registre ADCON0 */
   ADCON0bits.ADON = 1;
   ADCON0bits.GO_DONE = 1;
   // le channel CHS3:0 sera configure dans une fonction comme nous utilisons AN0 et AN1 
   
   /* Right Justified, 8 TAD (Temps d'acquisition) et Fosc/32 */
   ADCON2 = 0b10100010; 
}

//************************************************************************
// Sous programme de l'ADC avec un parametre le channel de l'entrée analogique (soit AN0 ou AN1)
//************************************************************************

unsigned int ADC(int channel)
{
    ADCON0 &= 0b00000011; // on clear le channel si on utilise cette fonction plusieurs fois 
    ADCON0bits.CHS=channel;// choisit le bon channel sur le registre  CHS
    while(ADCON0bits.GO_DONE==1); // boucle pour vérifier le lancement de la conversion 
    return ADRESL + (ADRESH*256); // on retourne la valeur de l'ADC avec les bits ADRESH et ADRESL
}

//************************************************************************
// Sous programme pour afficher le signal (running)
//************************************************************************

void running()
{
    position_x=compteur; // la position X est le compteur qui s'incremente à chaque conversion dans l'interruption
    glcd_PlotPixel(position_x,-(tableau[position_x]*62/1000)+64,0); // on efface le pixel à la coord (X,Y)
    tableau[position_x]=ADC(0b0000); // on recupère l'ADC dans le tableau d'entiers pour chaque indice du tableau
    glcd_PlotPixel(position_x,-(tableau[position_x]*62/1000)+64,1);  // on affiche le pixel à la coord (X,Y) et en ajustant Y dans l'ecran GLCD 
}

//************************************************************************
// Sous programme pour afficher le signal - 1 ere version (moins fonctionnel)
//************************************************************************

void affiche_Courbe(void)
{
    for(int i=0;i<127;i++) // boucle for pour la position X de 0 à 127 sur le GLCD
    {
        /* on convertit un entier en char en rajoutant '0'*/
        d0 = (adc%10)+'0'; // le 4ieme nombre de l'ADC (le reste de la division par 10)
        d1 = ((adc/10)%10)+'0'; // le 3ieme nombre
        d2 = ((adc/100)%10)+'0'; // le 2ieme nombre 
        d3 = ((adc/1000)%10)+'0'; // le 1er nombre 
        
        /* notre chaine de caractère */
        unsigned char string[5] = { d3, d2, d1, d0, '\0' };
        /* on se place à une position précise du GLCD */
        glcd_SetCursor(54,1);	
        /* on ecrit la chaine de caractère */
        glcd_WriteString(string,f8X8,1);
        
        int yp=((adc*62/1000)+'0'); // la position Y pour le pixel en prenant la valeur de l'ADC et en le convertissant en caractère
        unsigned char s1=-yp+64; // on règle cette position pour bien visualiser sur l'ecran
        if(adc <= 300) // si cette valeur de l'ADC est inférieure à 300 (soit une tension faible) 
        {
            glcd_PlotPixel(i,0,1); // on affiche le pixel pour Y=0 (pour pas qu'on a les valeurs negatives mal réglé dans l'echelle de l'écran)
        }
        else
        {
            glcd_PlotPixel(i,s1,1); // on allume le pixel à la bonne position
        }
        if(i==126) // dès que le signal atteint l'ecran à droite (le maximum de la position X)
        {
            glcd_FillScreen(0); // on efface l'ecran (il fallait le faire sans, d'ou la 2eme version : running)
        }
    }
}

//************************************************************************
// Sous programme pour afficher sur les 7 segments
//************************************************************************

void display_7seg()
{
    d0 = (adc%10); // le 4ieme nombre de l'ADC sur le DIS0 (le reste de la division par 10)
    d1 = ((adc/10)%10); // le 3ieme sur le DIS1 
    d2 = ((adc/100)%10); // le 2ieme sur le DIS2
    d3 = ((adc/1000)%10); // le 1er nombre sur DIS3 

    PORTD = tab_bin[d0]; // on trouve la valeur en Hexa par rapport à l'entier du DIS0 et on le met sur le PORTD
    PORTAbits.RA0 = 1; // on affiche sur le 7 segment et on repete sur chaque DIS pour chaque RA0:3
    __delay_ms(4); // un delay de 4 ms
    PORTAbits.RA0 = 0;
    PORTD = tab_bin[d1];
    PORTAbits.RA1 = 1;
    __delay_ms(4);  
    PORTAbits.RA1 = 0;
    PORTD = tab_bin[d2];
    PORTAbits.RA2 = 1;
    __delay_ms(4);  
    PORTAbits.RA2 = 0;
    PORTD = tab_bin[d3];
    PORTAbits.RA3 = 1;
    __delay_ms(4);  
    PORTAbits.RA3 = 0;
}

//************************************************************************
// Interruption
//************************************************************************

void __interrupt() ISR(void)
{
   if(PIR1bits.ADIF == 1) // si le flag de la conversion est levé
   {
       PIR1bits.ADIF = 0; // flag à 0
       compteur=(compteur+1) % 128; // compteur s'incrémente jusquà 128
   }
   if(INTCONbits.TMR0IF == 1) // si le flag du timer 0 est overflow
   {
       INTCONbits.TMR0IF = 0; // remet à 0 le flag
       adc=ADC(0b0000); // recupère l'ADC sur AN0
       TMR0H = 0x0B; // une seconde pour le timer 0 
       TMR0L = 0xDC;
   }
   if(INTCONbits.RBIF == 1) // si le flag du bouton d'interruption
   {
        if(PORTBbits.RB7 == 0) {  // si le bouton RB7 est appuyé  
            nb++; // on incremente le nombre pour changer le mode
            PORTC++; // on verifie sur le PORTC avec les LEDS RC0 à RC2
        }
        INTCONbits.RBIF = 0; // flag à 0 
   }
   return;
}

//************************************************************************
// Mode trigger
//************************************************************************

void trigger()
{
    unsigned char msg[7]={'E','R','R','E','U','R','\0'}; // message d'erreur (alors qu'il ne fallait pas afficher erreur sur l'ecran)
    bool valeur_faible=0;
    bool test=0; // on declare deux booleens pour vérifier si la tension d'entrée dépasse 3V
    if(test==0)
    {
        if(tableau[position_x]*5/1023 >= 3) // si ca dépasse 3V
        {
            valeur_faible=0;
        }
        else
        {
            valeur_faible=1;
        }
        test=1;
    }
    if(test==1) // double condition 
    {
        if(valeur_faible==0)
        {
            running(); // quand ca dépasse le seuil 3V, on affiche le signal
        }
        if(valeur_faible==1) // sinon on affiche Erreur
        {
            glcd_SetCursor(61,4); //place le curseur
            glcd_WriteString(msg,f8X8,1); // on ecrit le message
            __delay_us(10);
            glcd_FillScreen(0); // on efface l'ecran
        }
        /* Il fallait qu'on ne modifie pas l'ecran pour le mode trigger (c'est expliqué dans le rapport) */
    }
}

//************************************************************************
// Initialisation de la PWM
//************************************************************************

void PWM_Init()
{
    TRISCbits.RC2 = 0; // RC2 en sortie 
    CCP1CON = 0b00001100; // le mode PWM avec les bits CCP1M3:2 à 1 
    CCPR1L = 0x00; // 8 bits (High) du rapport cyclique à 0
    
    /* Configuration du Timer2  */
    T2CON = 0b00000011; // Prescalar 16  
    PR2 = 255; // PR2 avec une période de 255, cela permet d'avoir la période de la PWM à environ 2 ms
    T2CONbits.TMR2ON = 1; // Timer2 on
}

//************************************************************************
// Changer le rapport cyclique de la PWM
//************************************************************************

void PWM_set(unsigned int rapport_cyclique)
{
    CCPR1L = rapport_cyclique >> 2; // le rapport cyclique dans les 8 bits de poids fort 
    CCP1CONbits.DC1B = 0b00; // DC1B0 et DCB1 à 0, cela correspond aux 2 bits de poids faibles (on initialise)
    CCP1CON |= ((rapport_cyclique<<4)&0x30); // le rapport cyclique dans les 2 bits de poids faible 
    /* le rapport cyclique est codé dans 10 bits */
}

//************************************************************************
// Sous programme pour choisir le mode
//************************************************************************

void mode(void)
{
    if(nb%2==0) // si le nombre est pair
    {
        running(); // mode running
    }
    else
    {
        trigger(); // sinon le mode trigger
    }  
}

//************************************************************************
// Main
//************************************************************************

void main(void) {
  
   prog_init(); // lance l initialisation
	
   glcd_Init(1); // lance l initialisation du glcd en ON
   PWM_Init(); // lance l initialisation du PWM 

   while(1) // boucle infinie 
   {
       ADCON0bits.GO_DONE = 1; // lance la conversion 
       //display_7seg(); // afficher sur le 7segment (module 1)
       mode(); // fonction mode (module 2)
   }
   return;
}

