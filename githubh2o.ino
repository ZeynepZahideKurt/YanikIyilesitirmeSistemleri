#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include <Wire.h>
float IRPWMValue1;
float IRPWMValue2;
float IRPWMValue3;
float IRPWMValue4;
float LEDPWMValue;


int l_calismasure;
int l_calismabitis;
boolean l_calisma=false;
boolean l_calisma_ilk=false;
int l_periotsure; 
int l_periotbitis;
boolean l_periot;
boolean l_periot_ilk;
unsigned long l_yenizaman;
unsigned long l_simdi;
boolean l_tiklandi=false;

int o_calismasure;
int o_calismabitis;
boolean o_calisma=false;
boolean o_calisma_ilk=false;
int o_periotsure; 
int o_periotbitis;
boolean o_periot;
boolean o_periot_ilk;
unsigned long o_yenizaman;
unsigned long o_simdi;
boolean o_tiklandi=false;



int ac;
const int LEDPin = 4; //1.pin
const int IRPin1 = 2; //2.pin
const int IRPin2 = 13; //5.pin
const int IRPin3 = 12; //6.pin
const int IRPin4 = 3; //11.pin //rx-yeşil kablo
const int motor = 16; //15.pin
const int h2o = 1; //10.pin //tx-mavi kablo

int istenilen_nem;
float ort_humidity;
int istenilen_sicaklik;
float ort_cTemp;

// setting PWM properties
const int PWMFrequency = 4000;//40000000;
const int PWMResolation = 16; //0-65535   //0.0, 100.0
const int LEDChannel = 14;
const int IRChannel1 = 15;
const int IRChannel2 = 13;
const int IRChannel3 = 12;
const int IRChannel4 = 11;
// SHT31 I2C address is 0x44(68)
#define Addr 0x44
#define Addr2 0x45
#define I2C_SDA 14
#define I2C_SCL 15
const char* ssid = "iPhone";
const char* password = "11223344";
/*
const char* ssid = "Zehra";
const char* password = "11223344";
const char* ssid = "iPhone";
const char* password = "11223344";
const char* ssid = "Enelsis1_EXT";
const char* password = "1vn6egph";*/
// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
using namespace websockets;
WebsocketsServer socket_server;
camera_fb_t * fb = NULL;
void app_httpserver_init();
typedef struct
{
  uint8_t *image;
  box_array_t *net_boxes;
  dl_matrix3d_t *face_id;
} http_img_process_result;
static inline mtmn_config_t app_mtmn_config()
{
  mtmn_config_t mtmn_config = {0};
  mtmn_config.type = FAST;
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;
  return mtmn_config;
}
mtmn_config_t mtmn_config = app_mtmn_config();
httpd_handle_t camera_httpd = NULL;
typedef enum
{
  START_STREAM,
  DELETE_ALL,
} en_fsm_state;
en_fsm_state g_state;
void setup() {

  Wire.begin(I2C_SDA, I2C_SCL);
  
 // Serial.begin(115200);
  ledcSetup(IRChannel1, PWMFrequency, PWMResolation);
  ledcAttachPin(IRPin1, IRChannel1);
  ledcSetup(LEDChannel, PWMFrequency, PWMResolation);
  ledcAttachPin(LEDPin, LEDChannel);
  ledcSetup(IRChannel2, PWMFrequency, PWMResolation);
  ledcAttachPin(IRPin2, IRChannel2);
  ledcSetup(IRChannel3, PWMFrequency, PWMResolation);
  ledcAttachPin(IRPin3, IRChannel3);
  ledcSetup(IRChannel4, PWMFrequency, PWMResolation);
  ledcAttachPin(IRPin4, IRChannel4);

  
  
  //Serial.setDebugOutput(true);
  //Serial.println(); 
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
   // Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
   // Serial.print(".");
  }
  //al.println("");
//  Serial.println("WiFi connected");
  app_httpserver_init();
  socket_server.listen(82);
  //Serial.print("Camera Ready! Use 'http://");
// Serial.print(WiFi.localIP());
  //Serial.println("' to connect");
    pinMode(motor, OUTPUT);
  pinMode(h2o, OUTPUT);
}
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}
httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};
void app_httpserver_init ()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK)
   // Serial.println("httpd_start");
  {
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}
static esp_err_t LED_gelen(WebsocketsClient &client)
{
}

void o_calismabitisbelirlef(){
o_simdi = millis();
o_calismabitis=o_simdi+(1000*o_calismasure);
}

void o_calismasuref(){
  
  digitalWrite(h2o,HIGH);
  o_calisma=true;
  if(o_calisma_ilk==false){
    o_calismabitisbelirlef();
    o_calisma_ilk=true;
    }
    o_yenizaman= millis();
  if(o_yenizaman>=o_calismabitis){
    o_periotsuref();
    o_calisma=false;
    o_calisma_ilk=false;
  }  
}

void o_periotbitisbelirlef(){
o_simdi = millis();
o_periotbitis=o_simdi+(1000*o_periotsure);
}


void o_periotsuref(){
  digitalWrite(h2o,LOW);
  o_periot=true;
  if(o_periot_ilk==false){
    o_periotbitisbelirlef();
    o_periot_ilk=true;
    }
    o_yenizaman= millis();
  if(o_yenizaman>=o_periotbitis){
    o_calismasuref();
    o_periot=false;
    o_periot_ilk=false;
  }
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg)
{
if (msg.data().substring(0, 5) == "led1:") {
    IRPWMValue1=msg.data().substring(5).toFloat();
    IRPWMValue1 *= 650;
 //   Serial.print("IRLED1:");
   //  Serial.println(IRPWMValue1);
  }
   if (msg.data().substring(0, 5) == "led2:") {
    IRPWMValue2=msg.data().substring(5).toFloat();
    IRPWMValue2 *= 650;
     //Serial.print("IRLED2:");
     //Serial.println(IRPWMValue2);
   }
   if (msg.data().substring(0, 5) == "led3:") {
    IRPWMValue3=msg.data().substring(5).toFloat();
    IRPWMValue3 *= 650;
   //  Serial.print("IRLED3:");
    // Serial.println(IRPWMValue3);
   }
   if (msg.data().substring(0, 5) == "led4:") {
    IRPWMValue4=msg.data().substring(5).toFloat();
    IRPWMValue4 *= 650;
     //Serial.print("IRLED4:");
     //Serial.println(IRPWMValue4);
  }
  if (msg.data().substring(0, 5) == "led5:") {
    LEDPWMValue=msg.data().substring(5).toFloat();
    LEDPWMValue *= 650;
   //  Serial.print("kamera led:");
    // Serial.println(LEDPWMValue);
  }
  if (msg.data().substring(0, 6) == "motor:") {
    istenilen_nem =msg.data().substring(6).toInt(); //istenilen nem degeri
 // Serial.println(deger);
  }
    if (msg.data().substring(0, 4) == "Odur") { 
    
    o_periotsure=msg.data().substring(4).toInt(); //oksijenin periodu
    //Serial.println(durma);
     int bas=msg.data().indexOf('c');
     int son=bas+5;
     o_calismasure=msg.data().substring(son).toInt(); // oksijenin çalışması istenilen süre
  //  Serial.println(calis);
    o_tiklandi=true;
    o_calisma=true;
  }
  
  if (msg.data().substring(0, 4) == "Ldur") {
    int durma=msg.data().substring(4).toInt(); ////ledlerin durma süresi
// Serial.println(durma);
     int bas=msg.data().indexOf('c');
     int son=bas+5;
     int calis=msg.data().substring(son).toInt(); //ledlerin çalışma süresi
    //Serial.println(calis);
  }
   if (msg.data().substring(0, 9) == "sicaklik:") {
    float istenen=msg.data().substring(9).toFloat(); ////istenilen sıcaklık
 //Serial.println(istenen);
  }

  
  if (msg.data().substring(0, 4) == "min:") {
    float minimum=msg.data().substring(4).toFloat(); ///setup min sıcaklık değeri
 //Serial.println(minimum);
     int bas=msg.data().indexOf('a');
     int son=bas+2;
     float maximum=msg.data().substring(son).toFloat(); //setup sıcaklık max değeri
   // Serial.println(maximum);
  }
 
  
   
 
}
void loop() {
  auto client = socket_server.accept();
  client.onMessage(handle_message);
  client.send("STREAMING");
   while (client.available()) {
    client.poll();
    fb = esp_camera_fb_get();
    client.sendBinary((const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    fb = NULL;
    unsigned int data[6];
   // ledcWrite(ledChannel, 50);
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Send 16-bit command byte
  Wire.write(0x2C);
  Wire.write(0x06);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);
  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Stop I2C Transmission
  Wire.endTransmission();
  // Request 6 bytes of data
  Wire.requestFrom(Addr, 6);
  // Read 6 bytes of data
  // temp msb, temp lsb, temp crc, hum msb, hum lsb, hum crc
  if (Wire.available() == 6)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
  }
//second write and read
{
  unsigned int data2[6];
  // Start I2C Transmission
  Wire.beginTransmission(Addr2);
  // Send 16-bit command byte
  Wire.write(0x2C);
  Wire.write(0x06);
  // Stop I2C transmission
  Wire.endTransmission();
  delay(300);
  // Start I2C Transmission
  Wire.beginTransmission(Addr2);
  // Stop I2C Transmission
  Wire.endTransmission();
  // Request 6 bytes of data
  Wire.requestFrom(Addr2, 6);
  // Read 6 bytes of data
  // temp msb, temp lsb, temp crc, hum msb, hum lsb, hum crc
  if (Wire.available() == 6)
  {
    data2[0] = Wire.read();
    data2[1] = Wire.read();
    data2[2] = Wire.read();
    data2[3] = Wire.read();
    data2[4] = Wire.read();
    data2[5] = Wire.read();
   }
  // Convert the data
  int temp = (data[0] * 256) + data[1];
  float cTemp = -45.0 + (175.0 * temp / 65535.0);
  //float fTemp = (cTemp * 1.8) + 32.0;
  float humidity = (100.0 * ((data[3] * 256.0) + data[4])) / 65535.0;
   int temp2 = (data2[0] * 256) + data2[1];
  float cTemp2 = -45.0 + (175.0 * temp2 / 65535.0);
 // float fTemp2 = (cTemp2 * 1.8) + 32.0;
  float humidity2 = (100.0 * ((data2[3] * 256.0) + data2[4])) / 65535.0;
String sicaklik1;
String nem1;
String sicaklik2;
String nem2;
sicaklik1 = String(cTemp2);
nem1=String(humidity2);
sicaklik2 = String(cTemp);
nem2=String(humidity);
String hepsi=sicaklik1+"/"+nem1+"/"+sicaklik2+"/"+nem2;
  client.send(hepsi);
  client.send("sag_nem:"+nem2);
  client.send("sag_sicaklik:"+sicaklik2);
  client.send("sol_nem:"+nem1);
  client.send("sol_sicaklik:"+sicaklik1);
  if(0.00<humidity&&humidity<100.00){
    
    if(0.00<humidity2&&humidity2<100.00){
        ort_humidity=((humidity+humidity2)/2);
      }
      else{
        ort_humidity=humidity;
        }
    }
    
   else{
     if(0.00<humidity2&&humidity2<100.00){
        ort_humidity=humidity2;
      }
    else{
      //Serial.println("2 sensör de bozuk");
      }
    }
}

ledcWrite(IRChannel1,IRPWMValue1);
ledcWrite(IRChannel2,IRPWMValue2);
 ledcWrite(IRChannel3,IRPWMValue3);
 ledcWrite(IRChannel4,IRPWMValue4);
 ledcWrite(LEDChannel,LEDPWMValue);
 if(ort_humidity<istenilen_nem){
    digitalWrite(motor,HIGH);
  }else{
    digitalWrite(motor,LOW);
  }
 
 if(o_tiklandi==true){

    if(o_calisma==true){
      
     o_calismasuref();
   
    }else{
      o_periotsuref();
      
    }
  }

  }
}
