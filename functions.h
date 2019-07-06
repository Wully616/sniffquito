// This-->tab == "functions.h"

// Expose Espressif SDK functionality
extern "C" {
#include "user_interface.h"
  typedef void (*freedom_outside_cb_t)(uint8 status);
  int  wifi_register_send_pkt_freedom_cb(freedom_outside_cb_t cb);
  void wifi_unregister_send_pkt_freedom_cb(void);
  int  wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}

#include <ESP8266WiFi.h>
#include "./structures.h"

#define MAX_APS_TRACKED 100
#define MAX_CLIENTS_TRACKED 200
#define MQTT_MAX_PACKET_SIZE 1024

beaconinfo aps_known[MAX_APS_TRACKED];                    // Array to save MACs of known APs
int aps_known_count = 0;                                  // Number of known APs
int nothing_new = 0;
clientinfo clients_known[MAX_CLIENTS_TRACKED];            // Array to save MACs of known CLIENTs
int clients_known_count = 0;                              // Number of known CLIENTs


const char* ssid = "/////";
const char* password = "/////";

const char* mqttServer = "192.168.1.X";
const int mqttPort = 1883;

WiFiClient espClient;
PubSubClient client(espClient);


int register_beacon(beaconinfo beacon)
{
  int known = 0;   // Clear known flag
  for (int u = 0; u < aps_known_count; u++)
  {
    if (! memcmp(aps_known[u].bssid, beacon.bssid, ETH_MAC_LEN)) {
      known = 1;
      break;
    }   // AP known => Set known flag
  }
  if (! known)  // AP is NEW, copy MAC to array and return it
  {
    memcpy(&aps_known[aps_known_count], &beacon, sizeof(beacon));
    aps_known_count++;

    if ((unsigned int) aps_known_count >=
        sizeof (aps_known) / sizeof (aps_known[0]) ) {
      Serial.printf("exceeded max aps_known\n");
      aps_known_count = 0;
    }
  }
  return known;
}

int register_client(clientinfo ci)
{
  int known = 0;   // Clear known flag
  for (int u = 0; u < clients_known_count; u++)
  {
    if (! memcmp(clients_known[u].station, ci.station, ETH_MAC_LEN)) {
      known = 1;
      break;
    }
  }
  if (! known)
  {
    memcpy(&clients_known[clients_known_count], &ci, sizeof(ci));
    clients_known_count++;

    if ((unsigned int) clients_known_count >=
        sizeof (clients_known) / sizeof (clients_known[0]) ) {
      Serial.printf("exceeded max clients_known\n");
      clients_known_count = 0;
    }
  }
  return known;
}


void promisc_cb(uint8_t *buf, uint16_t len)
{
  int i = 0;
 
  //control frame, no useful data
  if (len == 12) {
    struct RxControl *sniffer = (struct RxControl*) buf;
  } else if (len == 128) {
    
    struct sniffer_buf2 *sniffer = (struct sniffer_buf2*) buf;
    
    struct beaconinfo beacon = parse_beacon(sniffer->buf, 112, sniffer->rx_ctrl.rssi);
    
    if (register_beacon(beacon) == 0) {
      nothing_new = 0;
    }
    
  } else {
    struct sniffer_buf *sniffer = (struct sniffer_buf*) buf;
    //Is data or QOS?
    //if((buf[12]==0x88)||(buf[12]==0x40)||(buf[12]==0x94)||(buf[12]==0xa4)||(buf[12]==0xb4)||(buf[12]==0x08)){
      struct clientinfo ci = parse_data(sniffer->buf, 36, sniffer->rx_ctrl.rssi, sniffer->rx_ctrl.channel, buf[12]);
      if (memcmp(ci.bssid, ci.station, ETH_MAC_LEN)) {
        if (register_client(ci) == 0) {          
          nothing_new = 0;
        }
      }
    //}
  }
}

void listen_to_wifi()
{
  WiFi.softAP("espnode");
  Serial.println("I am an AP");
  delay(random(1000,2000));
  WiFi.softAPdisconnect();
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  delay(100);
  wifi_set_opmode(STATION_MODE);            // Promiscuous works only with station mode
  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(promisc_cb);   // Set up promiscuous callback
  wifi_promiscuous_enable(1);
}


void connect_to_wifi()
{
  wifi_set_opmode(STATION_MODE);            // Promiscuous works only with station mode
  wifi_promiscuous_enable(0);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  delay(100); 
  Serial.println("Connected to the WiFi network");

}

void connect_to_mqtt()
{
    client.setServer(mqttServer, mqttPort);
    
    while (client.connect("ESP8266Client") != true) {
      delay(500);
      Serial.println("Connecting to MQTT...State:");
      Serial.println(client.state());
    }
  Serial.println("Connected to to MQTT...State:");
  Serial.println(client.state());
  String payload = "{\"";
  payload.concat(WiFi.macAddress());
  payload.concat("-");
  payload.concat(WiFi.softAPmacAddress());
  payload.concat("\": \"");
  char mac[18] = "";
  for (int u = 0; u < clients_known_count; u++){
    if(clients_known[u].err == 0){
      if(u != 0){
        payload.concat(",");
      }   
      payload.concat("device|");
      sprintf(mac,"%02x:%02x:%02x:%02x:%02x:%02x",clients_known[u].station[0],clients_known[u].station[1],clients_known[u].station[2],clients_known[u].station[3],clients_known[u].station[4],clients_known[u].station[5]);
      payload.concat(String(mac)); //sender
      payload.concat("|");
      sprintf(mac,"%02x:%02x:%02x:%02x:%02x:%02x",clients_known[u].ap[0],clients_known[u].ap[1],clients_known[u].ap[2],clients_known[u].ap[3],clients_known[u].ap[4],clients_known[u].ap[5]);
      payload.concat(String(mac)); //dest      
      payload.concat("|");
      sprintf(mac,"%02x:%02x:%02x:%02x:%02x:%02x",clients_known[u].bssid[0],clients_known[u].bssid[1],clients_known[u].bssid[2],clients_known[u].bssid[3],clients_known[u].bssid[4],clients_known[u].bssid[5]);
      payload.concat(String(mac)); //ap
      payload.concat("|");
      payload.concat(clients_known[u].rssi);
    }
  }
  payload.concat(",");
  for (int u = 0; u < aps_known_count; u++){

    if(aps_known[u].err == 0){
      if(u != 0){
        payload.concat(",");
      }   
      payload.concat("beacon|");
      //print AP ssid
      payload.concat( String((char *)aps_known[u].ssid) );
      payload.concat("|");
      //print the AP MAC address
      sprintf(mac,"%02x:%02x:%02x:%02x:%02x:%02x",aps_known[u].bssid[0],aps_known[u].bssid[1],aps_known[u].bssid[2],aps_known[u].bssid[3],aps_known[u].bssid[4],aps_known[u].bssid[5]);
      payload.concat(String(mac)); //sender
      payload.concat("|");
      payload.concat(aps_known[u].rssi);
    }
  }
  payload.concat("\"}");

  //payload size
  int payload_len = payload.length() + 1;
  //prepare buffer
  char data[payload_len];
  
  payload.toCharArray(data, payload_len);
  Serial.print("MQTT Max payload size (bytes): "); Serial.println(MQTT_MAX_PACKET_SIZE);
  Serial.print("Payload size:"); Serial.println(payload_len);
  Serial.println(payload);
  
  if(client.publish("wully/wifi", data ) == false) {
    Serial.println("Failed to send payload:");
    Serial.println(client.state());
  } else {
    Serial.println("Data published!");
  }

  client.disconnect();
  memset(aps_known, 0, sizeof(aps_known));
  aps_known_count = 0;
  memset(clients_known, 0, sizeof(clients_known));
  clients_known_count = 0;
}
