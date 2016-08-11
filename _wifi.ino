#ifdef WIFI
void init_wifi() {
  //Serial.begin(115200);
  Serial.println();
  Serial.println();

  // We start by connecting to a WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
}

unsigned long request_pov_pixels(IPAddress& address, float angle)
{
    udp.beginPacket(address, SERVER_PORT);
    udp.write((char *) &angle, sizeof(angle));
    udp.endPacket();
}
#endif
