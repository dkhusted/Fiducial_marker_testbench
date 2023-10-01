#include "mqtt.hpp"
#include "globals.hpp"

bool publisher_connected_flag = true;
bool subscriber_connected_flag = true;

bool g_slider_position = false;
bool g_angler_position = false;
bool g_angler_status = false;
bool g_slider_status = false;
bool g_master_status = false;

const char TAG[] = "MQTT";

/*
	NOTE: Dont change the topics and payloads.
	If its needed change the nodes instead to match the topics and payloads.

*/

void on_connect_subscribe(struct mosquitto *mosq, void *obj, int reason_code)
{
	int rc;
	int i = 0;
	printf("on_connect: %s\n", mosquitto_connack_string(reason_code));
	if(reason_code != 0){
		/* If the connection fails for any reason, we don't want to keep on
		 * retrying in this example, so disconnect. Without this, the client
		 * will attempt to reconnect. */
		mosquitto_disconnect(mosq);
	}

	char *topics[] = {"/testBench/steppermotor/get/event", 
		"/testBench/master/set/status"};
	rc = mosquitto_subscribe_multiple(mosq, NULL, 2, topics, 0, 0, NULL);
	if(rc != MOSQ_ERR_SUCCESS){
		fprintf(stderr, "MQTT: Error subscribing: %s\n", mosquitto_strerror(rc));
		mosquitto_disconnect(mosq);
	}
	
}

void on_connect(struct mosquitto *mosq, void *obj, int reason_code)
{

	int i = 0;
	printf("MQTT: on_connect: %s\n", mosquitto_connack_string(reason_code));
	if(reason_code != 0){
		mosquitto_disconnect(mosq);
	}

}

void  publish_set_angle(struct mosquitto *mosq, int value)
{
	// char payload[20];
	int rc;

	cJSON *json = cJSON_CreateObject();
	cJSON_AddNumberToObject(json, "angler", value);

	char *payload = cJSON_Print(json);

	rc = mosquitto_publish(mosq, NULL, "/testBench/steppermotor/set/position", strlen(payload), payload, 0, false);
	if(rc != MOSQ_ERR_SUCCESS){
		fprintf(stderr, "Error publishing: %s\n", mosquitto_strerror(rc));
	}

	// printf("%s: Sent increase distance to angler\n", TAG);

	cJSON_free(payload);
	cJSON_Delete(json);

}

void  publish_set_slider(struct mosquitto *mosq, int value)
{
	// char payload[20];
	int rc;

	cJSON *json = cJSON_CreateObject();
	cJSON_AddNumberToObject(json, "slider", value);

	char *payload = cJSON_Print(json);

	rc = mosquitto_publish(mosq, NULL, "/testBench/steppermotor/set/position", strlen(payload), payload, 0, false);
	if(rc != MOSQ_ERR_SUCCESS){
		fprintf(stderr, "MQTT: Error publishing: %s\n", mosquitto_strerror(rc));
	}

	// printf("%s: Sent reset to slider\n", TAG);

	cJSON_free(payload);
	cJSON_Delete(json);
}

void  publish_set_angle_relative(struct mosquitto *mosq, int value)
{
	// char payload[20];
	int rc;

	cJSON *json = cJSON_CreateObject();
	cJSON_AddNumberToObject(json, "angler", value);

	char *payload = cJSON_Print(json);

	rc = mosquitto_publish(mosq, NULL, "/testBench/steppermotor/set/relative_position", strlen(payload), payload, 0, false);
	if(rc != MOSQ_ERR_SUCCESS){
		fprintf(stderr, "Error publishing: %s\n", mosquitto_strerror(rc));
	}

	// printf("%s: Sent increase distance to angler\n", TAG);

	cJSON_free(payload);
	cJSON_Delete(json);

}

void  publish_set_slider_relative(struct mosquitto *mosq, int value)
{
	// char payload[20];
	int rc;

	cJSON *json = cJSON_CreateObject();
	cJSON_AddNumberToObject(json, "slider", value);

	char *payload = cJSON_Print(json);

	rc = mosquitto_publish(mosq, NULL, "/testBench/steppermotor/set/relative_position", strlen(payload), payload, 0, false);
	if(rc != MOSQ_ERR_SUCCESS){
		fprintf(stderr, "MQTT: Error publishing: %s\n", mosquitto_strerror(rc));
	}

	// printf("%s: Sent reset to slider\n", TAG);

	cJSON_free(payload);
	cJSON_Delete(json);
}

void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
{
	// printf("MQTT message received\n");
	
	char payload_buf[100];

	cJSON *json = cJSON_Parse((char *)msg->payload);
	cJSON *motor_name = cJSON_GetObjectItem(json, "motor_name");
	

	{
		std::unique_lock<std::mutex> lck(g_write_mutex);
		printf("%s %d %s\n", msg->topic, msg->qos, (char *)msg->payload);
	}



	if(strcmp(msg->topic, "/testBench/steppermotor/get/event") == 0){
		cJSON *bool_value = cJSON_GetObjectItemCaseSensitive(json, "completed");

		if(strcmp(motor_name->valuestring, "slider") == 0){
			int command_id = cJSON_GetObjectItem(json, "id")->valueint;
			g_slider_position = cJSON_IsTrue(bool_value) ? true : false;

		}
		else if(strcmp(motor_name->valuestring, "angler") == 0){

			int command_id = cJSON_GetObjectItem(json, "id")->valueint;
			g_angler_position = cJSON_IsTrue(bool_value) ? true : false;

		}

	}
		
	else if(strcmp(msg->topic, "/testBench/master/set/status") == 0){
		g_master_status = cJSON_GetObjectItem(json, "value")->valueint;
		
	}

	if(g_master_status){
		std::lock_guard<std::mutex> lck(g_mutex); // Try to take the mutex and release it once we leave the scope
		
		g_master_ready = true;
		
		g_master.notify_one(); // tell main thread that the nodes are ready

		g_master_status = false;
	}

	if(g_slider_status && g_angler_status){
		std::lock_guard<std::mutex> lck(g_mutex); // Try to take the mutex and release it once we leave the scope
		g_ready = true;
		g_cv.notify_one(); // tell main thread that the nodes are ready

		g_angler_status = false;

		g_slider_status = false;
	}

	if(g_angler_position){
		std::lock_guard<std::mutex> lck(g_mutex);
		g_ready = true;
		g_angler_position = false;
		g_angler_cv.notify_one(); // tell main thread that the angler node is in position
	}

	else if(g_slider_position){
		std::lock_guard<std::mutex> lck(g_mutex);
		g_ready = true;
		g_slider_position = false;
		g_slider_cv.notify_one(); // tell main thread that the slider node is in position
	}

	cJSON_Delete(json);
}

void on_subscribe(struct mosquitto *mosq, void *obj, int mid, int qos_count, const int *granted_qos)
{
	int i;
	bool have_subscription = false;


	for(i=0; i<qos_count; i++){
		printf("MQTT: on_subscribe: %d:granted qos = %d\n", i, granted_qos[i]);
		if(granted_qos[i] <= 2){
			have_subscription = true;
		}
	}
	if(have_subscription == false){

		fprintf(stderr, "MQTT: Error: All subscriptions rejected.\n");
		mosquitto_disconnect(mosq);
	}
}

void on_publish(struct mosquitto *mosq, void *obj, int mid)
{

	{
		std::lock_guard<std::mutex> lock_printf(g_write_mutex);
		printf("MQTT: Message with mid %d has been published.\n", mid);
	}
}


int mqtt_config(mosquitto *publisher, mosquitto *subscriber){
    int err;
    mosquitto_connect_callback_set(publisher, on_connect);
	// mosquitto_publish_callback_set(publisher, on_publish);
    mosquitto_message_callback_set(publisher, on_message);

    mosquitto_connect_callback_set(subscriber, on_connect_subscribe);
    mosquitto_subscribe_callback_set(subscriber, on_subscribe);
    mosquitto_message_callback_set(subscriber, on_message);

    int i = 0;
    while(mosquitto_connect(publisher, MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE) != MOSQ_ERR_SUCCESS){
        if(i == 120){ // try to connect for 2 minutes
            printf("Error: MQTT connection failed\n");
            mosquitto_destroy(publisher);
            return 1;
        }
        sleep(1);
        i++;
    }

	i = 0;
    while(mosquitto_connect(subscriber, MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE) != MOSQ_ERR_SUCCESS){
        if(i == 120){
            printf("Error: MQTT connection failed\n");
            mosquitto_destroy(subscriber);
            return 1;
        }
        sleep(1);
        i++;
    }

    err = mosquitto_loop_start(subscriber);
	if(err != MOSQ_ERR_SUCCESS){
		mosquitto_destroy(subscriber);
		fprintf(stderr, "Error: %s\n", mosquitto_strerror(err));
		return 1;
	}

    return 0;
}

