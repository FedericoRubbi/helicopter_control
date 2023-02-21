#include <iostream>

#include "transmitter.h"
#include "pico/stdlib.h"
#include "pico/bootrom.h" // reset_usb_boot()
#include <tusb.h>         // tud_cdc_connected()
#ifdef TX_RF24
#include <RF24.h>
#endif

extern "C"
{
#include "control/control.h"
}

#ifdef TX_RF24
// Instantiate an object for the nRF24L01 transceiver.
RF24 radio(TX_CE_PIN, TX_CSN_PIN);                      // move to main.cpp
struct TxPayload_s payload_buf[TX_PAYLOAD_BUFFER_SIZE]; // payload buffer to store a packet
static void split_packet(struct Packet_s *packet);
static void cmd_feedback(Cmd_t *ack_payload);

void reset_bootloader()
{
    radio.powerDown();
    reset_usb_boot(0, 0);
}

bool setup_transmitter()
{
    std::cout << "Setting up radio." << std::endl;
    // Wait here until the CDC ACM (serial port emulation) is connected.
    while (!tud_cdc_connected())
        sleep_ms(10);

    // Initialize the transceiver on the SPI bus.
    while (!radio.begin())
        ;
    radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default
    radio.enableDynamicPayloads(); // ACK payloads are dynamically sized
    radio.enableAckPayload();
    radio.openWritingPipe(tx_pipe_addr);
    radio.openReadingPipe(1, rx_pipe_addr); // add macro for the pipe
    radio.stopListening();
    radio.printDetails();
    radio.printPrettyDetails();
    return true;
}

// Divide logic packet into payloads.
void split_packet(struct Packet_s *packet)
{
    for (uint8_t i = 0; i < TX_PAYLOAD_BUFFER_SIZE; ++i)
    {
        memcpy(payload_buf[i].data, packet->sensor_data + i * TX_PAYLOAD_DATA_SIZE, TX_PAYLOAD_DATA_SIZE);
        payload_buf[i].eop = TX_UNSET_EOP;
    }
    payload_buf[TX_PAYLOAD_BUFFER_SIZE - 1].eop = TX_SET_EOP; // set end-of-packet on last payload
}

// Send logic packet to receiver.
void send_packet(struct Packet_s *packet)
{
    split_packet(packet); // split packed into payloads
    for (uint8_t i = 0; i < TX_PAYLOAD_BUFFER_SIZE; ++i)
        if (radio.write(&payload_buf[i], TX_PAYLOAD_SIZE)) // best effort transmission of payloads
            std::cout << "[DEBUG] Message sent to receiver" << std::endl;
        else
            std::cout << "[DEBUG] Error sending message" << std::endl;
}

// Read command acknowledge packet from receiver.
bool read_cmd(Cmd_t *ack_payload)
{
    if (radio.available())
    {
        radio.read(ack_payload, sizeof(*ack_payload));
        std::cout << "[DEBUG] Command received: " << ack_payload->cmd_code << std::endl;
        return true;
    }
    return false;
}

// Mirror received payload to RX to ensure data correctness.
void cmd_feedback(Cmd_t *ack_payload)
{
    static TxPayload_s mirror_payload;
    uint8_t cmd = ack_payload->cmd_code;

    if (cmd >= TX_CMD_SET_PID0 && cmd <= TX_CMD_SET_DMAT2)
    {
        memcpy(mirror_payload.data, ack_payload->data, sizeof(mirror_payload.data));
        mirror_payload.eop = TX_SET_EOP;
        radio.write(ack_payload->data, sizeof(*ack_payload));
    }
}

// Execute received command.
void run_cmd(struct System_s *sys, Cmd_t *cmd)
{
    std::cout << "[DEBUG] Executing command: " << cmd->cmd_code << std::endl;
    switch (cmd->cmd_code)
    {
    case TX_CMD_STOP:
        stop(sys);
        break;
    case TX_CMD_SETPOINT:
        memcpy(sys->setpoint, cmd->data, sizeof(sys->setpoint));
        break;
    case TX_CMD_SET_PID0:
        memcpy(&sys->controllers[0].k_p, cmd->data, TX_PAYLOAD_DATA_SIZE);
        break;
    case TX_CMD_SET_PID1:
        memcpy(&sys->controllers[0].k_p + TX_PAYLOAD_DATA_SIZE, cmd->data,
               sizeof(sys->controllers) - TX_PAYLOAD_DATA_SIZE); // copy only remaining data
        break;
    case TX_CMD_SET_DMAT0:
        memcpy(&sys->ctrl_dependence, cmd->data, TX_PAYLOAD_DATA_SIZE);
        break;
    case TX_CMD_SET_DMAT1:
        memcpy(&sys->ctrl_dependence + TX_PAYLOAD_DATA_SIZE, cmd->data, TX_PAYLOAD_DATA_SIZE);
        break;
    case TX_CMD_SET_DMAT2:
        memcpy(&sys->ctrl_dependence + TX_PAYLOAD_DATA_SIZE * 2, cmd->data,
               sizeof(sys->ctrl_dependence) - TX_PAYLOAD_DATA_SIZE * 2.0f); // copy only remaining data
        break;
    default:
        break;
    }
}
#endif

#ifdef TX_SERIAL
void send_preamble()
{
    for (uint8_t i = 0; i < TX_PREAMBLE_SIZE; ++i)
        if (uart_is_writable(TX_UART))
            uart_write_blocking(TX_UART, &tx_preamble[i], sizeof(tx_preamble[i]));
}

bool connect_transmitter()
{
    bool connected = false;
    uint8_t resp;
    while (!connected)
    {
        sleep_ms(100);
        send_preamble();
        if (uart_is_readable_within_us(TX_UART, 1000)) // await 1ms for response
        {
            uart_read_blocking(TX_UART, &resp, sizeof(resp));
            connected = (resp == TX_CONNECTED_FLAG);
        }
    }
    return connected;
}

// Setup serial communication.
bool setup_transmitter()
{
    uart_init(TX_UART, TX_BAUDRATE);
    gpio_set_function(TX_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(TX_RX_PIN, GPIO_FUNC_UART);
    connect_transmitter(); // wait for receiver serial device to listen
    return true;
}

// Send logic packet to receiver.
void send_packet(struct Packet_s *packet)
{
    if (uart_is_writable(TX_UART))
        uart_write_blocking(TX_UART, packet->sensor_data, TX_PACKET_SIZE);
}

// Read command acknowledge packet from receiver.
// TODO: selective command data read can be implemented.
bool read_cmd(Cmd_t *cmd)
{
    if (uart_is_readable(TX_UART))
    {
        uart_read_blocking(TX_UART, &cmd->cmd_code, sizeof(cmd->cmd_code));
        std::cout << "[DEBUG] Command received: " << cmd->cmd_code << std::endl;
        uart_read_blocking(TX_UART, (uint8_t *)cmd->data, sizeof(cmd->data));
        return true;
    }
    return false;
}

// Execute received command.
void run_cmd(struct System_s *sys, Cmd_t *cmd)
{
    std::cout << "[DEBUG] Executing command: " << cmd->cmd_code << std::endl;
    switch (cmd->cmd_code)
    {
    case TX_CMD_STOP:
        stop(sys);
        break;
    case TX_CMD_SETPOINT:
        memcpy(sys->setpoint, cmd->data, sizeof(sys->setpoint));
        break;
    case TX_CMD_SET_PID0:
        memcpy(&sys->controllers[0].k_p, cmd->data, TX_CMD_DATA_SIZE);
        break;
    case TX_CMD_SET_PID1:
        memcpy(&sys->controllers[0].k_p + TX_CMD_DATA_SIZE, cmd->data,
               sizeof(sys->controllers) - TX_CMD_DATA_SIZE); // copy only remaining data
        break;
    case TX_CMD_SET_DMAT0:
        memcpy(&sys->ctrl_dependence, cmd->data, TX_CMD_DATA_SIZE);
        break;
    case TX_CMD_SET_DMAT1:
        memcpy(&sys->ctrl_dependence + TX_CMD_DATA_SIZE, cmd->data, TX_CMD_DATA_SIZE);
        break;
    case TX_CMD_SET_DMAT2:
        memcpy(&sys->ctrl_dependence + TX_CMD_DATA_SIZE * 2, cmd->data,
               sizeof(sys->ctrl_dependence) - TX_CMD_DATA_SIZE * 2.0f); // copy only remaining data
        break;
    default:
        break;
    }
}
#endif

// Check and execute new commands.
void check_cmd(struct System_s *sys)
{
    static Cmd_t cmd;
    if (read_cmd(&cmd))
        run_cmd(sys, &cmd);
}