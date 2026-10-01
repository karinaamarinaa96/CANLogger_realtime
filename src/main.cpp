#include <Arduino.h>

// ============================================================
// UART
// ============================================================

HardwareSerial karina(PA3, PA2);

#define UART_BAUDRATE       460800


// ============================================================
// CAN
// STM32F103CB(T6)
// CAN1:
//   RX = PB8
//   TX = PB9
// ============================================================

#define CAN_PCLK_HZ         32000000UL

#define CAN_RX_PIN          PB8
#define CAN_TX_PIN          PB9

#define CAN_STB_PIN         PB6
#define LED_PIN             PB3


// ============================================================
// AUTO BAUD
// ============================================================

#define AUTO_BAUD_TIME_MS   1500
#define AUTO_BAUD_MIN_FRAMES 3

const uint32_t CAN_BAUDS[] =
{
    10000,
    20000,
    50000,
    100000,
    125000,
    200000,
    250000,
    500000,
    800000,
    1000000
};

#define NUM_CAN_BAUDS (sizeof(CAN_BAUDS) / sizeof(CAN_BAUDS[0]))


// ============================================================
// CAN FRAME
// ============================================================

struct CANFrame
{
    uint16_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};


// ============================================================
// CRC16-CCITT
// ============================================================

uint16_t CRC16_CCITT(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}


// ============================================================
// LED
// ============================================================

void led_on()
{
    digitalWrite(LED_PIN, HIGH);
}

void led_off()
{
    digitalWrite(LED_PIN, LOW);
}

void led_blink(uint32_t onTime, uint32_t offTime)
{
    led_on();
    delay(onTime);

    led_off();
    delay(offTime);
}


// ============================================================
// CAN BIT TIMING
//
// Search for a timing close to 87.5% sample point.
//
// BTR:
// [31]      SILM
// [30]      LBKM
// [24:25]   SJW
// [19:16]   TS2
// [15:12]   TS1
// [9:0]     BRP
// ============================================================

bool CAN_GetBTR(uint32_t baud, uint32_t &btr)
{
    uint32_t bestError = 0xFFFFFFFF;
    uint32_t bestBTR = 0;

    // Total time quanta per bit
    // Typical values: 8 ... 25
    for (uint32_t tq = 8; tq <= 25; tq++)
    {
        uint32_t denom = baud * tq;

        if (denom == 0)
            continue;

        uint32_t brp = (CAN_PCLK_HZ + denom / 2) / denom;

        if (brp < 1 || brp > 1024)
            continue;

        uint32_t realBaud =
            CAN_PCLK_HZ / (brp * tq);

        uint32_t baudError;

        if (realBaud > baud)
            baudError = realBaud - baud;
        else
            baudError = baud - realBaud;

        // Allow max ~1%
        if ((baudError * 100UL) > (baud * 1UL))
            continue;

        // Target sample point = 87.5%
        //
        // Sync segment = 1
        // TS1 + TS2 = tq - 1
        //
        // sample point = (1 + TS1) / tq
        //
        // TS1 = approximately 87.5% * tq - 1

        uint32_t ts1 =
            ((tq * 875UL) / 1000UL);

        if (ts1 < 1)
            ts1 = 1;

        if (ts1 > 16)
            ts1 = 16;

        uint32_t ts2 =
            tq - 1 - ts1;

        if (ts2 < 1 || ts2 > 8)
            continue;

        uint32_t samplePoint =
            ((1 + ts1) * 1000UL) / tq;

        uint32_t spError;

        if (samplePoint > 875)
            spError = samplePoint - 875;
        else
            spError = 875 - samplePoint;

        // Combine baud error + sample-point error
        uint32_t error =
            baudError * 1000UL + spError;

        if (error < bestError)
        {
            bestError = error;

            bestBTR =
                ((ts2 - 1) << 20) |
                ((ts1 - 1) << 16) |
                ((brp - 1) << 0);

            bestBTR |= (0 << 24); // SJW = 1
        }
    }

    if (bestError == 0xFFFFFFFF)
        return false;

    btr = bestBTR;

    return true;
}


// ============================================================
// CAN FILTER
//
// Accept everything.
//
// Filter 0:
// 32-bit identifier mask mode
// Mask = 0 -> everything accepted
// ============================================================

void CAN_Filter_All()
{
    CAN1->FMR |= CAN_FMR_FINIT;

    // Filter 0
    CAN1->FM1R &= ~(1UL << 0);   // mask mode

    CAN1->FS1R |= (1UL << 0);    // 32-bit scale

    CAN1->FFA1R &= ~(1UL << 0);  // FIFO0

    CAN1->sFilterRegister[0].FR1 = 0x00000000;
    CAN1->sFilterRegister[0].FR2 = 0x00000000;

    CAN1->FA1R |= (1UL << 0);    // activate filter

    CAN1->FMR &= ~CAN_FMR_FINIT;
}


// ============================================================
// CAN START
// ============================================================

bool CAN_Start(uint32_t baud)
{
    // Enable clocks
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // --------------------------------------------------------
    // CAN transceiver ON
    // --------------------------------------------------------

    pinMode(CAN_STB_PIN, OUTPUT);
    digitalWrite(CAN_STB_PIN, LOW);

    // --------------------------------------------------------
    // CAN1 full remap
    //
    // PB8 = CAN_RX
    // PB9 = CAN_TX
    // --------------------------------------------------------

    AFIO->MAPR &= ~(3UL << 13);
    AFIO->MAPR |=  (2UL << 13);

    // PB8 input floating
    GPIOB->CRH &= ~(0xFUL << 0);
    GPIOB->CRH |=  (0x4UL << 0);

    // PB9 AF Push-Pull 50 MHz
    GPIOB->CRH &= ~(0xFUL << 4);
    GPIOB->CRH |=  (0xBUL << 4);

    // --------------------------------------------------------
    // Reset CAN
    // --------------------------------------------------------

    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
    delayMicroseconds(10);
    RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;

    // --------------------------------------------------------
    // Enter initialization mode
    // --------------------------------------------------------

    CAN1->MCR = 0;

    CAN1->MCR |= CAN_MCR_INRQ;

    uint32_t timeout = millis();

    while (!(CAN1->MSR & CAN_MSR_INAK))
    {
        if ((millis() - timeout) > 100)
            return false;
    }

    // --------------------------------------------------------
    // Calculate BTR
    // --------------------------------------------------------

    uint32_t btr;

    if (!CAN_GetBTR(baud, btr))
        return false;

    CAN1->BTR = btr;

    // --------------------------------------------------------
    // Automatic bus-off recovery
    //
    // NART = 1
    // Disable automatic retransmission.
    //
    // ABOM = 1
    // Automatic bus-off recovery.
    // --------------------------------------------------------

    CAN1->MCR =
        CAN_MCR_INRQ |
        CAN_MCR_ABOM |
        CAN_MCR_NART;

    // --------------------------------------------------------
    // Filters
    // --------------------------------------------------------

    CAN_Filter_All();

    // --------------------------------------------------------
    // Leave initialization
    // --------------------------------------------------------

    CAN1->MCR &= ~CAN_MCR_INRQ;

    timeout = millis();

    while (CAN1->MSR & CAN_MSR_INAK)
    {
        if ((millis() - timeout) > 100)
            return false;
    }

    // Clear pending flags
    CAN1->RF0R = 0;
    CAN1->RF1R = 0;
    CAN1->MSR = 0;

    return true;
}


// ============================================================
// CAN STOP
// ============================================================

void CAN_Stop()
{
    CAN1->MCR |= CAN_MCR_INRQ;

    uint32_t timeout = millis();

    while (!(CAN1->MSR & CAN_MSR_INAK))
    {
        if ((millis() - timeout) > 100)
            break;
    }

    digitalWrite(CAN_STB_PIN, HIGH);
}


// ============================================================
// READ CAN FRAME
// ============================================================

bool CAN_ReadFrame(CANFrame &frame)
{
    if (!(CAN1->RF0R & CAN_RF0R_FMP0))
        return false;

    CAN_FIFOMailBox_TypeDef *mb =
        &CAN1->sFIFOMailBox[0];

    uint32_t rir = mb->RIR;
    uint32_t rdtr = mb->RDTR;
    uint32_t rdlr = mb->RDLR;
    uint32_t rdhr = mb->RDHR;

    // Release FIFO mailbox
    CAN1->RF0R |= CAN_RF0R_RFOM0;

    // Only standard data frames
    if (rir & CAN_RI0R_IDE)
        return false;

    if (rir & CAN_RI0R_RTR)
        return false;

    frame.id = (uint16_t)(rir >> 21);

    frame.dlc = (uint8_t)(rdtr & 0x0F);

    if (frame.dlc > 8)
        frame.dlc = 8;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (i < frame.dlc)
            frame.data[i] = (uint8_t)(rdlr >> (i * 8));

        if ((i + 4) < frame.dlc)
            frame.data[i + 4] =
                (uint8_t)(rdhr >> (i * 8));
    }

    return true;
}


// ============================================================
// BINARY PACKET
//
// AA 55
// LEN_H LEN_L
// ID_H ID_L
// DLC
// DATA...
// CRC_H CRC_L
//
// LEN = ID(2) + DLC(1) + DATA + CRC(2)
// ============================================================

void SendCANBinary(const CANFrame &frame)
{
    uint8_t packet[32];

    uint16_t len =
        2 + 1 + frame.dlc + 2;

    uint16_t pos = 0;

    // Header
    packet[pos++] = 0xAA;
    packet[pos++] = 0x55;

    // Length
    packet[pos++] = (uint8_t)(len >> 8);
    packet[pos++] = (uint8_t)(len & 0xFF);

    // ID
    packet[pos++] = (uint8_t)(frame.id >> 8);
    packet[pos++] = (uint8_t)(frame.id & 0xFF);

    // DLC
    packet[pos++] = frame.dlc;

    // Data
    for (uint8_t i = 0; i < frame.dlc; i++)
        packet[pos++] = frame.data[i];

    // CRC over:
    // LEN_H LEN_L ID_H ID_L DLC DATA
    uint16_t crc =
        CRC16_CCITT(&packet[2], pos - 2);

    packet[pos++] = (uint8_t)(crc >> 8);
    packet[pos++] = (uint8_t)(crc & 0xFF);

    // Arduino HardwareSerial
    karina.write(packet, pos);
}


// ============================================================
// AUTO BAUD DETECTION
// ============================================================

uint32_t CAN_AutoBaud()
{
    CANFrame frame;

    for (uint32_t i = 0; i < NUM_CAN_BAUDS; i++)
    {
        uint32_t baud = CAN_BAUDS[i];

        // Start CAN at this baud
        if (!CAN_Start(baud))
        {
            CAN_Stop();
            continue;
        }

        uint32_t startTime = millis();
        uint16_t validFrames = 0;

        while ((millis() - startTime) < AUTO_BAUD_TIME_MS)
        {
            if (CAN_ReadFrame(frame))
            {
                validFrames++;

                if (validFrames >= AUTO_BAUD_MIN_FRAMES)
                {
                    CAN_Stop();
                    return baud;
                }
            }
        }

        CAN_Stop();
    }

    return 0;
}


// ============================================================
// NO CAN FOUND
// ============================================================

void NoCANLoop()
{
    while (true)
    {
        led_blink(300, 700);
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // LED
    // --------------------------------------------------------

    pinMode(LED_PIN, OUTPUT);
    led_off();

    // --------------------------------------------------------
    // CAN standby
    // --------------------------------------------------------

    pinMode(CAN_STB_PIN, OUTPUT);
    digitalWrite(CAN_STB_PIN, HIGH);

    // --------------------------------------------------------
    // UART
    //
    // Arduino HardwareSerial
    // PA2 = TX
    // PA3 = RX
    // --------------------------------------------------------

    karina.begin(UART_BAUDRATE);

    delay(100);

    // --------------------------------------------------------
    // Auto baud
    // --------------------------------------------------------

    uint32_t detectedBaud = CAN_AutoBaud();

    // --------------------------------------------------------
    // No CAN traffic
    // --------------------------------------------------------

    if (detectedBaud == 0)
    {
        karina.write(0xEE);

        NoCANLoop();
    }

    // --------------------------------------------------------
    // Start CAN using detected baud
    // --------------------------------------------------------

    if (!CAN_Start(detectedBaud))
    {
        karina.write(0xEF);

        NoCANLoop();
    }
    CANFrame framebaud;
    framebaud.id=0x001;
    framebaud.dlc=2;
    uint16_t bitrate=detectedBaud/1000;
    // for (uint8_t i = 0; i < 2; i++)
    // {
    //         framebaud.data[i] = (uint8_t)(bitrate >> (i * 8));

    // }
    framebaud.data[0]= bitrate&0xff;
    framebaud.data[1]= (bitrate>>8)&0xff;
    SendCANBinary(framebaud);
    delay(500);
    // --------------------------------------------------------
    // Indicate successful CAN detection
    //
    // Binary status packet:
    //
    // AA 56
    // 04
    // baudrate 4 bytes
    // --------------------------------------------------------

    uint8_t status[7];

    status[0] = 0xAA;
    status[1] = 0x56;

    status[2] = (uint8_t)(detectedBaud >> 24);
    status[3] = (uint8_t)(detectedBaud >> 16);
    status[4] = (uint8_t)(detectedBaud >> 8);
    status[5] = (uint8_t)(detectedBaud);

    status[6] = 0x00;

    karina.write(status, sizeof(status));

    // --------------------------------------------------------
    // LED solid ON = CAN detected
    // --------------------------------------------------------

    led_on();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    CANFrame frame;

    // --------------------------------------------------------
    // Read every available CAN frame
    // --------------------------------------------------------

    while (CAN_ReadFrame(frame))
    {
        SendCANBinary(frame);
    }
}