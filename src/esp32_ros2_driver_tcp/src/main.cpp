#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// Credenciales de tu red Wi-Fi
const char* ssid = "Lex";          // Reemplaza con el nombre de tu red Wi-Fi
const char* password = "Junior14";  // Reemplaza con la contraseña de tu red Wi-Fi

// Puerto TCP para la comunicación
const int tcpPort = 8080; // Puedes elegir cualquier puerto libre, 8080 es común

WiFiServer server(tcpPort); // Crea un objeto servidor TCP en el puerto especificado
WiFiClient client;          // Objeto cliente para la conexión activa

unsigned long lastUltrasonicSendTime = 0;
const long ultrasonicSendInterval = 500; // Enviar datos ultrasónicos cada 500 ms
float simulatedUltrasonicDistance = 50.0; // Distancia inicial simulada

void setup() {
  Serial.begin(115200); // Inicia la comunicación serial para depuración
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password); // Inicia la conexión Wi-Fi en modo STA

  // Espera a que se conecte a la red Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado.");
  Serial.print("Direccion IP del ESP32: ");
  Serial.println(WiFi.localIP()); // Muestra la IP asignada al ESP32

  server.begin(); // Inicia el servidor TCP
  Serial.print("Servidor TCP iniciado en el puerto: ");
  Serial.println(tcpPort);
  Serial.println("Esperando cliente...");
}

void loop() {
  // Comprobar si hay un cliente conectado
  if (!client || !client.connected()) {
    client = server.available(); // Intenta aceptar una nueva conexión
    if (client) {
      Serial.println("Nuevo cliente conectado!");
    }
  }

  // Si hay un cliente conectado
  if (client && client.connected()) {
    // Leer datos del cliente (si hay)
    while (client.available()) {
      char c = client.read();
      Serial.print("Recibido del cliente: ");
      Serial.write(c); // Escribe el carácter recibido en el monitor serial
      // Opcional: para leer una línea completa, si envías con println() desde el cliente
      // String receivedData = client.readStringUntil('\n');
      // Serial.print("Recibido del cliente: ");
      // Serial.println(receivedData);

      // (Opcional) Enviar de vuelta lo recibido (echo)
      // client.write(c);
    }

    // Enviar datos simulados del sensor ultrasónico cada cierto intervalo
    if (millis() - lastUltrasonicSendTime > ultrasonicSendInterval) {
      // Simular un cambio en la distancia
      simulatedUltrasonicDistance += 0.5;
      if (simulatedUltrasonicDistance > 100.0) {
        simulatedUltrasonicDistance = 20.0; // Reinicia para simular un obstáculo
      }
      
      // Formatear el mensaje de distancia (ej. "U:50.0\n")
      String ultrasonicMessage = "U:" + String(simulatedUltrasonicDistance, 1) + "\n"; // Formatear a un decimal

      client.print(ultrasonicMessage); // Envía el mensaje al cliente
      Serial.print("Enviado al cliente: ");
      Serial.print(ultrasonicMessage); // También imprime en el monitor serial
      lastUltrasonicSendTime = millis();
    }
  }
}