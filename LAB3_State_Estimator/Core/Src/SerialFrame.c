/*
 * SerialFrame.c
 *
 *  Created on: Apr 23, 2026
 *      Author: Yhommy
 */

/* =============================================================================
 *  SerialFrame — Binary Serial Framing over UART (DMA-backed)
 * =============================================================================
 *  Frame layout:
 *    [ HEADER (1 byte) | field0 | field1 | ... | fieldN | TERMINATOR (1 byte) ]
 *
 *  Fields are packed in the order they were registered via Add_TX / Add_RX.
 *  Header and terminator bytes must not appear in the payload data.
 * =============================================================================
 */

#include "SerialFrame.h"
#include <string.h>

/* =============================================================================
 *  Private — TYPE_SIZE lookup table
 *  Maps SF_Type_t enum values to their byte sizes.
 * =============================================================================
 */
static const uint8_t TYPE_SIZE[8] = {
    1,  /* SF_UINT8  */
    1,  /* SF_INT8   */
    2,  /* SF_UINT16 */
    2,  /* SF_INT16  */
    4,  /* SF_UINT32 */
    4,  /* SF_INT32  */
    4,  /* SF_FLOAT  */
    8   /* SF_DOUBLE */
};

/* =============================================================================
 *  SerialFrame_Init
 * =============================================================================
 */
void SerialFrame_Init(SerialFrame_t *frame,
                      UART_HandleTypeDef *huart,
                      uint8_t header,
                      uint8_t terminator)
{
    frame->huart      = huart;
    frame->header     = header;
    frame->terminator = terminator;

    frame->tx_count      = 0;
    frame->rx_count      = 0;
    frame->tx_frame_size = 2;   /* header + terminator */
    frame->rx_frame_size = 2;

    memset(frame->tx_buf, 0, SF_MAX_FRAME_BYTES);
    memset(frame->rx_buf, 0, SF_MAX_FRAME_BYTES);
}

/* =============================================================================
 *  SerialFrame_Add_TX
 * =============================================================================
 */
int SerialFrame_Add_TX(SerialFrame_t *frame,
                       const char    *name,
                       void          *data_ptr,
                       SF_Type_t      type)
{
    if (frame->tx_count >= SF_MAX_FIELDS) return -1;

    uint8_t sz = TYPE_SIZE[type];
    if ((frame->tx_frame_size + sz) > SF_MAX_FRAME_BYTES) return -1;

    SF_Field_t *f  = &frame->tx_fields[frame->tx_count];
    f->name        = name;
    f->type        = type;
    f->size        = sz;
    f->data_ptr    = data_ptr;
    f->byte_offset = frame->tx_frame_size - 1;  /* insert before terminator */

    frame->tx_frame_size += sz;
    frame->tx_count++;
    return 0;
}

/* =============================================================================
 *  SerialFrame_Add_RX
 * =============================================================================
 */
int SerialFrame_Add_RX(SerialFrame_t *frame,
                       const char    *name,
                       void          *data_ptr,
                       SF_Type_t      type)
{
    if (frame->rx_count >= SF_MAX_FIELDS) return -1;

    uint8_t sz = TYPE_SIZE[type];
    if ((frame->rx_frame_size + sz) > SF_MAX_FRAME_BYTES) return -1;

    SF_Field_t *f  = &frame->rx_fields[frame->rx_count];
    f->name        = name;
    f->type        = type;
    f->size        = sz;
    f->data_ptr    = data_ptr;
    f->byte_offset = frame->rx_frame_size - 1;

    frame->rx_frame_size += sz;
    frame->rx_count++;
    return 0;
}

/* =============================================================================
 *  SerialFrame_Transmit
 * =============================================================================
 */
void SerialFrame_Transmit(SerialFrame_t *frame)
{
    /* Write header */
    frame->tx_buf[0] = frame->header;

    /* Pack each registered field into the TX buffer */
    for (uint8_t i = 0; i < frame->tx_count; i++) {
        SF_Field_t *f = &frame->tx_fields[i];
        memcpy(&frame->tx_buf[f->byte_offset], f->data_ptr, f->size);
    }

    /* Write terminator */
    frame->tx_buf[frame->tx_frame_size - 1] = frame->terminator;

    /* Non-blocking DMA transmit */
    HAL_UART_Transmit_DMA(frame->huart, frame->tx_buf, frame->tx_frame_size);
}

/* =============================================================================
 *  SerialFrame_Receive
 * =============================================================================
 */
void SerialFrame_Receive(SerialFrame_t *frame, UART_HandleTypeDef *huart)
{
    if (huart != frame->huart) return;

    /* Validate frame boundaries before unpacking */
    if (frame->rx_buf[0]                        == frame->header &&
        frame->rx_buf[frame->rx_frame_size - 1] == frame->terminator)
    {
        for (uint8_t i = 0; i < frame->rx_count; i++) {
            SF_Field_t *f = &frame->rx_fields[i];
            memcpy(f->data_ptr, &frame->rx_buf[f->byte_offset], f->size);
        }
    }

    /* Re-arm DMA receiver for the next frame */
    HAL_UART_Receive_DMA(frame->huart, frame->rx_buf, frame->rx_frame_size);
}
