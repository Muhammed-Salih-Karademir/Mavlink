#pragma once
// MESSAGE JETSON PACKING
//MAVlink JETSON DRONE
#define MAVLINK_MSG_ID_JETSON 60

MAVPACKED(
typedef struct __mavlink_jetson_t {
 uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
 float sicaklik; /*< [celcius] sicaklik derece*/
 float ruzgar;
 float nem;
 float gas;
 float basinc;
 float uzaklik;
}) mavlink_jetson_t;

#define MAVLINK_MSG_ID_JETSON_LEN 28
#define MAVLINK_MSG_ID_JETSON_MIN_LEN 28
#define MAVLINK_MSG_ID_30_LEN 28
#define MAVLINK_MSG_ID_30_MIN_LEN 28

#define MAVLINK_MSG_ID_JETSON_CRC 39
#define MAVLINK_MSG_ID_30_CRC 39



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_JETSON { \
    30, \
    "JETSON", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_jetson_t, time_boot_ms) }, \
         { "sicaklik", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_jetson_t, sicaklik) }, \
         { "ruzgar", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_jetson_t, ruzgar) }, \
         { "nem", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_jetson_t, nem) }, \
         { "gas", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_jetson_t, gas) }, \
         { "basinc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_jetson_t, basinc) }, \
         { "uzaklik", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_jetson_t, uzaklik) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_JETSON { \
    "JETSON", \
    7, \
    {  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_jetson_t, time_boot_ms) }, \
         { "sicaklik", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_jetson_t, sicaklik) }, \
         { "ruzgar", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_jetson_t, ruzgar) }, \
         { "nem", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_jetson_t, nem) }, \
         { "gas", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_jetson_t, gas) }, \
         { "basinc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_jetson_t, basinc) }, \
         { "uzaklik", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_jetson_t, uzaklik) }, \
         } \
}
#endif

/**
 * @brief Pack a jetson message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param sicaklik [rad] sicaklik angle (-pi..+pi)
 * @param ruzgar [rad] ruzgar angle (-pi..+pi)
 * @param nem [rad] nem angle (-pi..+pi)
 * @param gas [rad/s] sicaklik angular speed
 * @param basinc [rad/s] ruzgar angular speed
 * @param uzaklik [rad/s] nem angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_jetson_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint32_t time_boot_ms, float sicaklik, float ruzgar, float nem, float gas, float basinc, float uzaklik)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_JETSON_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, sicaklik);
    _mav_put_float(buf, 8, ruzgar);
    _mav_put_float(buf, 12, nem);
    _mav_put_float(buf, 16, gas);
    _mav_put_float(buf, 20, basinc);
    _mav_put_float(buf, 24, uzaklik);mavlink_msg_jetson_pack

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_JETSON_LEN);
#else
    mavlink_jetson_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.sicaklik = sicaklik;
    packet.ruzgar = ruzgar;
    packet.nem = nem;
    packet.gas = gas;
    packet.basinc = basinc;
    packet.uzaklik = uzaklik;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_JETSON_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_JETSON;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_JETSON_MIN_LEN, MAVLINK_MSG_ID_JETSON_LEN, MAVLINK_MSG_ID_JETSON_CRC);
}

/**
 * @brief Pack a jetson message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param sicaklik [rad] sicaklik angle (-pi..+pi)
 * @param ruzgar [rad] ruzgar angle (-pi..+pi)
 * @param nem [rad] nem angle (-pi..+pi)
 * @param gas [rad/s] sicaklik angular speed
 * @param basinc [rad/s] ruzgar angular speed
 * @param uzaklik [rad/s] nem angular speed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_jetson_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint32_t time_boot_ms,float sicaklik,float ruzgar,float nem,float gas,float basinc,float uzaklik)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_JETSON_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, sicaklik);
    _mav_put_float(buf, 8, ruzgar);
    _mav_put_float(buf, 12, nem);
    _mav_put_float(buf, 16, gas);
    _mav_put_float(buf, 20, basinc);
    _mav_put_float(buf, 24, uzaklik);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_JETSON_LEN);
#else
    mavlink_jetson_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.sicaklik = sicaklik;
    packet.ruzgar = ruzgar;
    packet.nem = nem;
    packet.gas = gas;
    packet.basinc = basinc;
    packet.uzaklik = uzaklik;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_JETSON_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_JETSON;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_JETSON_MIN_LEN, MAVLINK_MSG_ID_JETSON_LEN, MAVLINK_MSG_ID_JETSON_CRC);
}

/**
 * @brief Encode a jetson struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param jetson C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_jetson_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_jetson_t* jetson)
{
    return mavlink_msg_jetson_pack(system_id, component_id, msg, jetson->time_boot_ms, jetson->sicaklik, jetson->ruzgar, jetson->nem, jetson->gas, jetson->basinc, jetson->uzaklik);
}

/**
 * @brief Encode a jetson struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param jetson C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_jetson_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_jetson_t* jetson)
{
    return mavlink_msg_jetson_pack_chan(system_id, component_id, chan, msg, jetson->time_boot_ms, jetson->sicaklik, jetson->ruzgar, jetson->nem, jetson->gas, jetson->basinc, jetson->uzaklik);
}

/**
 * @brief Send a jetson message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms [ms] Timestamp (time since system boot).
 * @param sicaklik [rad] sicaklik angle (-pi..+pi)
 * @param ruzgar [rad] ruzgar angle (-pi..+pi)
 * @param nem [rad] nem angle (-pi..+pi)
 * @param gas [rad/s] sicaklik angular speed
 * @param basinc [rad/s] ruzgar angular speed
 * @param uzaklik [rad/s] nem angular speed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_jetson_send(mavlink_channel_t chan, uint32_t time_boot_ms, float sicaklik, float ruzgar, float nem, float gas, float basinc, float uzaklik)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_JETSON_LEN];
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, sicaklik);
    _mav_put_float(buf, 8, ruzgar);
    _mav_put_float(buf, 12, nem);
    _mav_put_float(buf, 16, gas);
    _mav_put_float(buf, 20, basinc);
    _mav_put_float(buf, 24, uzaklik);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JETSON, buf, MAVLINK_MSG_ID_JETSON_MIN_LEN, MAVLINK_MSG_ID_JETSON_LEN, MAVLINK_MSG_ID_JETSON_CRC);
#else
    mavlink_jetson_t packet;
    packet.time_boot_ms = time_boot_ms;
    packet.sicaklik = sicaklik;
    packet.ruzgar = ruzgar;
    packet.nem = nem;
    packet.gas = gas;
    packet.basinc = basinc;
    packet.uzaklik = uzaklik;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JETSON, (const char *)&packet, MAVLINK_MSG_ID_JETSON_MIN_LEN, MAVLINK_MSG_ID_JETSON_LEN, MAVLINK_MSG_ID_JETSON_CRC);
#endif
}

/**
 * @brief Send a jetson message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_jetson_send_struct(mavlink_channel_t chan, const mavlink_jetson_t* jetson)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_jetson_send(chan, jetson->time_boot_ms, jetson->sicaklik, jetson->ruzgar, jetson->nem, jetson->gas, jetson->basinc, jetson->uzaklik);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JETSON, (const char *)jetson, MAVLINK_MSG_ID_JETSON_MIN_LEN, MAVLINK_MSG_ID_JETSON_LEN, MAVLINK_MSG_ID_JETSON_CRC);
#endif
}

#if MAVLINK_MSG_ID_JETSON_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_jetson_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, float sicaklik, float ruzgar, float nem, float gas, float basinc, float uzaklik)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, time_boot_ms);
    _mav_put_float(buf, 4, sicaklik);
    _mav_put_float(buf, 8, ruzgar);
    _mav_put_float(buf, 12, nem);
    _mav_put_float(buf, 16, gas);
    _mav_put_float(buf, 20, basinc);
    _mav_put_float(buf, 24, uzaklik);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JETSON, buf, MAVLINK_MSG_ID_JETSON_MIN_LEN, MAVLINK_MSG_ID_JETSON_LEN, MAVLINK_MSG_ID_JETSON_CRC);
#else
    mavlink_jetson_t *packet = (mavlink_jetson_t *)msgbuf;
    packet->time_boot_ms = time_boot_ms;
    packet->sicaklik = sicaklik;
    packet->ruzgar = ruzgar;
    packet->nem = nem;
    packet->gas = gas;
    packet->basinc = basinc;
    packet->uzaklik = uzaklik;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_JETSON, (const char *)packet, MAVLINK_MSG_ID_JETSON_MIN_LEN, MAVLINK_MSG_ID_JETSON_LEN, MAVLINK_MSG_ID_JETSON_CRC);
#endif
}
#endif

#endif

// MESSAGE JETSON UNPACKING


/**
 * @brief Get field time_boot_ms from jetson message
 *
 * @return [ms] Timestamp (time since system boot).
 */
static inline uint32_t mavlink_msg_jetson_get_time_boot_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sicaklik from jetson message
 *
 * @return [rad] sicaklik angle (-pi..+pi)
 */
static inline float mavlink_msg_jetson_get_sicaklik(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field ruzgar from jetson message
 *
 * @return [rad] ruzgar angle (-pi..+pi)
 */
static inline float mavlink_msg_jetson_get_ruzgar(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field nem from jetson message
 *
 * @return [rad] nem angle (-pi..+pi)
 */
static inline float mavlink_msg_jetson_get_nem(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gas from jetson message
 *
 * @return [rad/s] sicaklik angular speed
 */
static inline float mavlink_msg_jetson_get_gas(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field basinc from jetson message
 *
 * @return [rad/s] ruzgar angular speed
 */
static inline float mavlink_msg_jetson_get_basinc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field uzaklik from jetson message
 *
 * @return [rad/s] nem angular speed
 */
static inline float mavlink_msg_jetson_get_uzaklik(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a jetson message into a struct
 *
 * @param msg The message to decode
 * @param jetson C-struct to decode the message contents into
 */
static inline void mavlink_msg_jetson_decode(const mavlink_message_t* msg, mavlink_jetson_t* jetson)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    jetson->time_boot_ms = mavlink_msg_jetson_get_time_boot_ms(msg);
    jetson->sicaklik = mavlink_msg_jetson_get_sicaklik(msg);
    jetson->ruzgar = mavlink_msg_jetson_get_ruzgar(msg);
    jetson->nem = mavlink_msg_jetson_get_nem(msg);
    jetson->gas = mavlink_msg_jetson_get_gas(msg);
    jetson->basinc = mavlink_msg_jetson_get_basinc(msg);
    jetson->uzaklik = mavlink_msg_jetson_get_uzaklik(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_JETSON_LEN? msg->len : MAVLINK_MSG_ID_JETSON_LEN;
        memset(jetson, 0, MAVLINK_MSG_ID_JETSON_LEN);
    memcpy(jetson, _MAV_PAYLOAD(msg), len);
#endif
}
