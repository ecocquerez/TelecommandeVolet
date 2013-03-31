/*
 * File:   FirstMiwi.c
 * Author: Eric
 *
 * Created on 15 octobre 2011, 18:00
 */
#include <p18cxxx.h>
#include <p18f25k50.h>
#include <delays.h>
#include <spi.h>
#include <EEP.h>
#include <timers.h>
#include "SystemProfile.h"
#include "WirelessProtocols/MCHP_API.h"
#include "domoproto.h"
#include "RecepteurVolet/WirelessProtocols/MCHP_API.h"
#include "RecepteurVolet/HardwareProfile.h"

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

#pragma romdata myaddress
rom unsigned char addr1 = 0x06;
rom unsigned char addr2 = 0x08;
rom unsigned char addr3 = 0x26;
rom unsigned char addr4 = 0x83;
rom unsigned char addr5 = 0x82;
rom unsigned char addr6 = 0x00;
rom unsigned char addr7 = 0x00;
rom unsigned char addr8 = 0x01;     //Le master a l'adresse 1
rom unsigned char addr9 = 0x14;
rom unsigned char addr10 = 0x01;
rom unsigned char addr11 = 0x01;    //Send Alive
rom unsigned char addr12 = 0x78;    //Periode
rom unsigned char addr13 = 0x01;    //Group
rom unsigned char addr14 = 0x01;    //Index In Group
rom unsigned char addr15 = 0x01;    //Index On Board
rom unsigned char addr16 = 0x50;    //TimeOut
#pragma romdata


extern ACTIVE_SCAN_RESULT ActiveScanResults[ACTIVE_SCAN_RESULT_SIZE];
extern CONNECTION_ENTRY ConnectionTable[CONNECTION_SIZE];
extern BYTE myLongAddress[MY_ADDRESS_LENGTH];
extern BYTE TxBuffer[TX_BUFFER_SIZE];
extern WORD myPrivatePanId;
// Initialisation des param�tres hardware de la carte

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
    //OSCCON = 0x7B;
    OSCCON = 0b01101000;
    while(OSCCONbits.HFIOFS == 0)
    {
        //Waiting for stable frequency
        Nop();
    }

    EtatCourrant.stateShutter1 = Iddle;
    EtatCourrant.stateShutter2 = Iddle;
    HaveToSendHearthBeat = 0;
    CCP1CON = 0x00;
    ANSELA = 0x00;
    TRISA = 0X70;
    ANSELA = 0x00;
    TRISB = 0x00;
    //Shutter output
    INTCON2bits.RBPU = 0;
    INTCON2bits.INTEDG2 = 0;
    //Timer 1
    T1CON = 0b01110011; //1000000 inc per second
    TMR1H = 0x40;
    TMR1L = 0xB8; //We have to count 20 to have 1 second
    PIE1bits.TMR1IE = 1;
    T1CONbits.TMR1ON = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    INTCONbits.GIEH = 1;


    // SPI configuration
    //Reset du miwi
    DisableIntSPI1;

    BoardInit();
    CloseSPI1();
    OpenSPI1(SPI_FOSC_64,MODE_00,SMPMID);
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
    //On d�finit un port par d�faut
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
        //On recherche un canal pas trop brouill�
        value = MiApp_NoiseDetection(0x07FFF800,10,NOISE_DETECT_ENERGY,&noise);
        //On recalcule le channelMap
        channelMap = channelMap << value;
        //On lance le PAN sur le channel le moins bruit�,
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
    //Led de control
    TRISCbits.TRISC6 = 0;
    LATCbits.LATC6  = 0;
    //On met la commande des volets en sortie et on les passe � 0
    CMD_UP = 0;
    CMD_DOWN = 0;
}

//Lecture de mon adresse dans la eeprom interne du pic
//Les 8 premi�res bytes sont l'address
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