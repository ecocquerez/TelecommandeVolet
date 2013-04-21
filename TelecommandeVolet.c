/*
 * File:   FirstMiwi.c
 * Author: Eric
 *
 * Created on 15 octobre 2011, 18:00
 */
#include <p18cxxx.h>
#ifdef PIC18F25K50
#include <p18f25k50.h>
#endif
#ifdef PIC18F2550
#include <p18f2550.h>
#endif
#include <delays.h>
#include <spi.h>
#include <EEP.h>
#include <timers.h>
#include "SystemProfile.h"
#include "WirelessProtocols/MCHP_API.h"
#include "domoproto.h"
#include "RecepteurVolet/WirelessProtocols/MCHP_API.h"
#include "RecepteurVolet/HardwareProfile.h"

#ifdef PIC18F25K50
// CONFIG1L
#pragma config PLLSEL = PLL4X   // PLL Selection (4x clock multiplier)
#pragma config CFGPLLEN = OFF   // PLL Enable Configuration bit (PLL Disabled)
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Postscaler (CPU uses system clock (no divide))
#pragma config LS48MHZ = SYS24X4// Low Speed USB mode with 48 MHz system clock (System clock at 24 MHz, USB clock divider is set to 4)

// CONFIG1H
#pragma config FOSC = INTOSCIO  // Oscillator Selection (Internal oscillator)
#pragma config PCLKEN = ON      // Primary Oscillator Shutdown (Primary oscillator enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config nPWRTEN = ON     // Power-up Timer Enable (Power up timer enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable (BOR enabled in hardware (SBOREN is ignored))
#pragma config BORV = 190       // Brown-out Reset Voltage (BOR set to 1.9V nominal)
#pragma config nLPBOR = OFF     // Low-Power Brown-out Reset (Low-Power Brown-out Reset disabled)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (WDT disabled in hardware (SWDTEN ignored))
#pragma config WDTPS = 32768    // Watchdog Timer Postscaler (1:32768)

// CONFIG3H
#pragma config CCP2MX = RC1     // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config T3CMX = RC0      // Timer3 Clock Input MUX bit (T3CKI function is on RC0)
#pragma config SDOMX = RB3      // SDO Output MUX bit (SDO function is on RB3)
#pragma config MCLRE = ON       // Master Clear Reset Pin Enable (MCLR pin enabled; RE3 input disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Block 0 Code Protect (Block 0 is not code-protected)
#pragma config CP1 = OFF        // Block 1 Code Protect (Block 1 is not code-protected)
#pragma config CP2 = OFF        // Block 2 Code Protect (Block 2 is not code-protected)
#pragma config CP3 = OFF        // Block 3 Code Protect (Block 3 is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protect (Boot block is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protect (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Block 0 Write Protect (Block 0 (0800-1FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Block 1 Write Protect (Block 1 (2000-3FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Block 2 Write Protect (Block 2 (04000-5FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Block 3 Write Protect (Block 3 (06000-7FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Registers Write Protect (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protect (Boot block (0000-7FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protect (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Block 0 Table Read Protect (Block 0 is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Block 1 Table Read Protect (Block 1 is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Block 2 Table Read Protect (Block 2 is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Block 3 Table Read Protect (Block 3 is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protect (Boot block is not protected from table reads executed in other blocks)
#endif
#ifdef PIC18F2550
// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSC_HS // Oscillator Selection bits (Internal oscillator, HS oscillator used by USB (INTHS))
#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
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

#endif

typedef struct defInternal
{
    BYTE SendAlive;
    BYTE SendAliveTimeOut;
    BYTE MyGroup;
    BYTE IndexInGroup;
    BYTE IndexOnBoard;
    BYTE MaxTimeOut;
}Internal, * pInternal;

typedef struct defShutterState
{
    BYTE stateShutter1;
    BYTE stateShutter2;
}ShutterState;

#if ADDITIONAL_NODE_ID_SIZE > 0
BYTE    AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {0x32};
#endif

#pragma romdata eeprom_data=0xf00000
rom unsigned char info_eeprom[] = {
0x06,
0x08,
0x26,
0x83,
0x82,
0x00,
0x00,
0x01,    //Le master a l'adresse 1
0x14,
0x01,
0x01,    //Send Alive
0x78,    //Periode
0x01,    //Group
0x01,    //Index In Group
0x01,    //Index On Board
0x50    //TimeOut
};
#pragma code


extern ACTIVE_SCAN_RESULT ActiveScanResults[ACTIVE_SCAN_RESULT_SIZE];
extern CONNECTION_ENTRY ConnectionTable[CONNECTION_SIZE];
extern BYTE myLongAddress[MY_ADDRESS_LENGTH];
extern BYTE TxBuffer[TX_BUFFER_SIZE];
extern WORD myPrivatePanId;
// Initialisation des paramètres hardware de la carte

volatile unsigned char HaveToSendHearthBeat;
volatile unsigned char longPressButton;
volatile BYTE ledValue = 0;
volatile BYTE smallWait = 0;
Entete Emission;
Internal travail;

void BoardInit(void);
void LitMyMiwiAddress(void);
void LitMyPrivatePanID(void);
void EcritMyPrivatePanID(void);
void EcritMyMiwiAddress(void);
unsigned char DoConnection(void);
void LitInternalParameters(pInternal pLecture);
void WriteEntete(void);
void SmallWait(BYTE waiting);

#pragma code
void main(void)
{
    Entete * pReception;
    ShutterState EtatCourrant;
    char value = 0;
    unsigned char lastValue = 0;
    unsigned char ShutterSelected = 0;
    BYTE Destinataire[MY_ADDRESS_LENGTH] = {0x06,0x08,0x26,0x83,0x82,0x00,0x00,0x02};
    //Settings oscillator
    // primary internal oscillator
#ifdef PIC18F2550
    OSCCON = 0b01110010;
    while(OSCCONbits.IOFS == 0)
    {
        Nop();
    }
#endif
#ifdef PIC18F25K50
    OSCCON = 0b01101000;
    while(OSCCONbits.HFIOFS == 0)
    {
        //Waiting for stable frequency
        Nop();
    }
#endif
    BoardInit();

    EtatCourrant.stateShutter1 = Iddle;
    EtatCourrant.stateShutter2 = Iddle;
    HaveToSendHearthBeat = 0;
    // SPI configuration
#ifdef PIC18F25K50
    DisableIntSPI1;
    CloseSPI1();
    OpenSPI1(SPI_FOSC_64,MODE_00,SMPMID);
#endif
#ifdef PIC18F2550
    DisableIntSPI;
    CloseSPI();
    OpenSPI(SPI_FOSC_64,MODE_00,SMPMID);
#endif
    //Read necessary Miwi Information in eeprom
    LitMyMiwiAddress();
    LitMyPrivatePanID();
    //Read internal parameters
    LitInternalParameters(&travail);
    value = 0;
//    while(value < 8)
//    {
//        Destinataire[value] = myLongAddress[value];
//        value++;
//    }
//    value = 0;
    Emission.DeviceType = Shutter;
    Emission.Group = travail.MyGroup;
    Emission.IndexInGroup = travail.IndexInGroup;
    Emission.IndexOnBoard = travail.IndexOnBoard;
    Emission.lenValue = 0;
    SELECT_SHUTTER_1 = 1;
    SELECT_SHUTTER_2 = 1;
    SELECT_SHUTTER_3 = 1;
    SELECT_SHUTTER_4 = 1;

    do
    {
        value = DoConnection();
    }while(value == 0xff);
    SELECT_SHUTTER_1 = 0;
    SELECT_SHUTTER_2 = 0;
    SELECT_SHUTTER_3 = 0;
    SELECT_SHUTTER_4 = 0;

    //On allume la led de controle
    LATCbits.LATC6  = 1;
    while(1)
    {
        if(CMD_SELECT_SHUTTER == 0)
        {
            while(CMD_SELECT_SHUTTER == 0)
            {
                SmallWait(2);
            }
            ShutterSelected++;
            lastValue = 0;
            if(ShutterSelected == 6)
            {
                ShutterSelected = 1;
            }
            switch (ShutterSelected)
            {
                case 1:
                    SELECT_SHUTTER_1 = 1;
                    SELECT_SHUTTER_2 = 0;
                    SELECT_SHUTTER_3 = 0;
                    SELECT_SHUTTER_4 = 0;
                    break;
                case 2:
                    SELECT_SHUTTER_1 = 0;
                    SELECT_SHUTTER_2 = 1;
                    SELECT_SHUTTER_3 = 0;
                    SELECT_SHUTTER_4 = 0;
                    break;
                case 3:
                    SELECT_SHUTTER_1 = 0;
                    SELECT_SHUTTER_2 = 0;
                    SELECT_SHUTTER_3 = 1;
                    SELECT_SHUTTER_4 = 0;
                    break;
                case 4:
                    SELECT_SHUTTER_1 = 0;
                    SELECT_SHUTTER_2 = 0;
                    SELECT_SHUTTER_3 = 0;
                    SELECT_SHUTTER_4 = 1;
                    break;
                case 5:
                    SELECT_SHUTTER_1 = 1;
                    SELECT_SHUTTER_2 = 1;
                    SELECT_SHUTTER_3 = 1;
                    SELECT_SHUTTER_4 = 1;
                    break;


            }
        }
        if(CMD_UP == 1 && lastValue == ShutterUp)
        {
            if(longPressButton == 0)
            {
                Emission.Command = ShutterStop;
                lastValue = Emission.Command;
                WriteEntete();
                if(ShutterSelected == 5)
                {
                    MiApp_BroadcastPacket(FALSE);
                }
                else
                {
                    Destinataire[7] = ShutterSelected + 1;
                    value = MiApp_UnicastAddress(Destinataire,TRUE,FALSE);
                }
            }
            lastValue = 0;
        }
        if(CMD_DOWN == 1 && lastValue == ShutterDown)
        {
            if(longPressButton == 0)
            {
                Emission.Command = ShutterStop;
                lastValue = Emission.Command;
                WriteEntete();
                if(ShutterSelected == 5)
                {
                    MiApp_BroadcastPacket(FALSE);
                }
                else
                {
                    Destinataire[7] = ShutterSelected + 1;
                    value = MiApp_UnicastAddress(Destinataire,TRUE,FALSE);
                }
            }
            lastValue = 0;
        }

        if(CMD_UP == 0 && lastValue != ShutterUp)
        {
            SmallWait(2);
            if(CMD_UP == 0)
            {
                longPressButton = 1;
                Emission.Command = ShutterUp;
                lastValue = Emission.Command;
                WriteEntete();
                if(ShutterSelected == 5)
                {
                    MiApp_BroadcastPacket(FALSE);
                }
                else
                {
                    Destinataire[7] = ShutterSelected + 1;
                    value = MiApp_UnicastAddress(Destinataire,TRUE,FALSE);
                }
            }
        }
        if(CMD_DOWN == 0 && lastValue != ShutterDown)
        {
            SmallWait(2);
            if(CMD_DOWN == 0)
            {
                longPressButton = 1;
                Emission.Command = ShutterDown;
                lastValue = Emission.Command;
                WriteEntete();
                if(ShutterSelected == 5)
                {
                    MiApp_BroadcastPacket(FALSE);
                }
                else
                {
                    Destinataire[7] = ShutterSelected + 1;
                    value = MiApp_UnicastAddress(Destinataire,TRUE,FALSE);
                }
            }
        }
        if(MiApp_MessageAvailable())
        {
            pReception = (Entete *)rxMessage.Payload;
            if(pReception->Command == HearthBeat)
            {
                if(pReception->IndexOnBoard == 0x01)
                {
                    SELECT_SHUTTER_1 = 0;
                    SmallWait(10);
                    SELECT_SHUTTER_1 = 1;
                    SmallWait(10);
                    SELECT_SHUTTER_1 = 0;
                    SmallWait(10);
                    SELECT_SHUTTER_1 = 1;
                    SmallWait(10);
                    SELECT_SHUTTER_1 = 0;
                    SmallWait(10);
                    SELECT_SHUTTER_1 = 1;
                    SmallWait(10);
                }
            }
            MiApp_DiscardMessage();
        }
        else
        {
            if(HaveToSendHearthBeat)
            {
                //If we have to send Hearth beat message, send it ant reset the flag
                HaveToSendHearthBeat = 0;
                Emission.Command = Pulse;
                WriteEntete();
                MiApp_UnicastConnection(0,FALSE );
            }
        }
    }
    return;
}

void WriteEntete(void)
{
    MiApp_FlushTx();
    MiApp_WriteData(Emission.DeviceType);
    MiApp_WriteData(Emission.Group);
    MiApp_WriteData(Emission.IndexInGroup);
    MiApp_WriteData(Emission.IndexOnBoard);
    MiApp_WriteData(Emission.Command);
    MiApp_WriteData(Emission.lenValue);


}
void UserInterruptHandler(void)
{
    static unsigned char compteurSeconde = 0x00;
    static unsigned char compteurLongPress = 0x00;
    static unsigned char ellapsed = 0x00;
    if(PIR1bits.TMR1IF == 1)
    {
        //Reset of timer
        TMR1H = 0x40;
        TMR1L = 0xA8; //We have to count 20 to have 1 second
        compteurSeconde++;
        if(smallWait != 0)
        {
            smallWait--;
        }
        if(longPressButton == 1)
        {
            compteurLongPress++;
            if(compteurLongPress == 10)
            {
                longPressButton = 0;
                compteurLongPress = 0;
            }
        }
        if(compteurSeconde == 20)
        {
            //Just visualize timer
            LATCbits.LATC6 = ledValue;
            ledValue = !ledValue;
            //One second elapsed
            //Check if we have to send HearthBeat
            if(travail.SendAlive == 1)
            {
                ellapsed++;
                if(ellapsed == travail.SendAliveTimeOut)
                {
                    ellapsed = 0;
                    HaveToSendHearthBeat = 1;
                }
            }
            compteurSeconde = 0;
        }
        PIR1bits.TMR1IF = 0;
    }
}
//Init de la connexion ou de la reconnexion
unsigned char DoConnection(void)
{
    unsigned char numberNetwork = 0x00;
    unsigned char i = 0;
    unsigned char value = 0xFF;
    unsigned char noise = 0x00;
    DWORD channelMap = 0x00000001;
    //Note from Microchip, We have to clear reception
    MiApp_ProtocolInit(FALSE);
    //On définit un port par défaut
    //MiApp_SetChannel(16);
    if(MiMAC_ReceivedPacket())
    {
        MiApp_DiscardMessage();
    }
    numberNetwork = MiApp_SearchConnection(10,0x07FFF800);
    MiApp_ConnectionMode(ENABLE_ALL_CONN);
    if(numberNetwork > 0)
    {
        for(i = 0; i < ACTIVE_SCAN_RESULT_SIZE ; i++)
        {
            if(ActiveScanResults[i].PANID.Val == myPANID.Val)
            {
                MiApp_SetChannel(ActiveScanResults[i].Channel);
                MiApp_ConnectionMode(ENABLE_ALL_CONN);
                value = MiApp_EstablishConnection(i,CONN_MODE_DIRECT);
                //We can connect to a lot of device
                //break;
            }
        }
    }
    else
    {
        //On recherche un canal pas trop brouillé
        value = MiApp_NoiseDetection(0x07FFF800,10,NOISE_DETECT_ENERGY,&noise);
        //On recalcule le channelMap
        channelMap = channelMap << value;
        //On lance le PAN sur le channel le moins bruité,
        value = MiApp_StartConnection(START_CONN_ENERGY_SCN,10,channelMap);
    }
    return value;
}

void BoardInit(void)
{
    WDTCONbits.SWDTEN = 0;
    INTCONbits.GIEH = 1;
    RFIF = 0;
    RFIE = 1;
#ifdef PIC18F25K50
    TRISBbits.TRISB3 = 0;
#endif
#ifdef PIC18F2550
    TRISCbits.TRISC7 = 0;
#endif
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 0;
    //Led de control
    TRISCbits.TRISC6 = 0;
    LATCbits.LATC6  = 0;

    CCP1CON = 0x00;
#ifdef PIC18F25K50
    LATA = 0x00;
    ANSELA = 0x00;
    ANSELB = 0x00;
#endif
#ifdef PIC18F2550
    ADCON1 = 0b00001111;
#endif
    TRISA = 0b01110000;
    TRISB = 0x04;
    TRISC = 0x00;
    
    INTCON2bits.RBPU = 0;
    INTCON2bits.INTEDG2 = 0;
    //Timer 1
    T1CON = 0b01110011; //1000000 inc per second
    TMR1H = 0x40;
    TMR1L = 0xA8; //We have to count 20 to have 1 second
    PIE1bits.TMR1IE = 1;
    T1CONbits.TMR1ON = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    INTCONbits.GIEH = 1;
    //On met la commande des volets en sortie et on les passe à 0
    CMD_UP = 0;
    CMD_DOWN = 0;
}

//Lecture de mon adresse dans la eeprom interne du pic
//Les 8 premières bytes sont l'address
void LitMyMiwiAddress(void)
{
    unsigned char compteur = 0;
    while(compteur <8)
    {
        myLongAddress[compteur] = Read_b_eep(compteur);
        compteur++;
    }
}
//Ecriture de mon adresse dans la EEProm
void EcritMyMiwiAddress(void)
{
    unsigned char compteur = 0;
    while(compteur <8)
    {
        Write_b_eep(compteur,myLongAddress[compteur]);
        Busy_eep();
        compteur++;
    }
}
//Lecture du PanId
void LitMyPrivatePanID(void)
{
    myPrivatePanId = 0;
    myPrivatePanId = Read_b_eep(8);
    myPrivatePanId = myPrivatePanId << 8;
    myPrivatePanId |= Read_b_eep(9);
}
//Ecriture du PanId
void EcritMyPrivatePanID(void)
{
    unsigned char travail  = 0;
    travail = myPrivatePanId >> 8;
    Write_b_eep(8,travail);
    Busy_eep();
    travail = myPrivatePanId & 0xFF;
    Write_b_eep(9,travail);
    Busy_eep();
}

void LitInternalParameters(pInternal pLecture)
{
    pLecture->SendAlive = Read_b_eep(10);
    pLecture->SendAliveTimeOut = Read_b_eep(11);
    pLecture->MyGroup = Read_b_eep(12);
    pLecture->IndexInGroup = Read_b_eep(13);
    pLecture->IndexOnBoard = Read_b_eep(14);
    pLecture->MaxTimeOut = Read_b_eep(15);
}

void SmallWait(BYTE waiting)
{
    smallWait = waiting;
    while(smallWait !=0);
}