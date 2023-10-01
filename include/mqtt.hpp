#pragma once

#ifdef __linux__
    #include <unistd.h>
#endif

#include <iostream>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <math.h>
#include <errno.h>
#include <cstring>

#include <mosquitto/mosquitto.h>
#include <mosquitto/mosquittopp.h>
#include <cjson/cJSON.h>

#include "globals.hpp"

extern bool publisher_connected_flag;
extern bool subscriber_connected_flag;

/*######################### MQTT Settings #################################*/
#define MQTT_BROKER "192.168.171.1"
#define MQTT_PORT 1883
#define MQTT_KEEPALIVE 60



/* Callback called when the client receives a CONNACK message from the broker. */
void on_connect(struct mosquitto *mosq, void *obj, int reason_code);

void on_connect_subscribe(struct mosquitto *mosq, void *obj, int reason_code);



/* Callback called when the client receives a message. */
void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg);

/* Callback called when the broker sends a SUBACK in response to a SUBSCRIBE. */
void on_subscribe(struct mosquitto *mosq, void *obj, int mid, int qos_count, const int *granted_qos);

/* Callback called when the client knows to the best of its abilities that a
 * PUBLISH has been successfully sent. For QoS 0 this means the message has
 * been completely written to the operating system. For QoS 1 this means we
 * have received a PUBACK from the broker. For QoS 2 this means we have
 * received a PUBCOMP from the broker. */
void on_publish(struct mosquitto *mosq, void *obj, int mid);

int mqtt_config(mosquitto *publisher, mosquitto *subscriber);

void  publish_set_angle_relative(struct mosquitto *mosq, int value);

void  publish_set_slider_relative(struct mosquitto *mosq, int value);

void  publish_set_angle(struct mosquitto *mosq, int value);

void  publish_set_slider(struct mosquitto *mosq, int value);





