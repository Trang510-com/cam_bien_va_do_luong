#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; 
// CẤU HÌNH HỆ THỐNG
const int WARM_UP_SECONDS = 45; // Thời gian làm ấm chip (giây)
float P0 = 1013.25;             // Áp suất mốc mặt đất (sẽ được tự động cập nhật khi đo Bậc 0)
float base_altitude = 0.0f;     // Độ cao mốc tuyệt đối của sàn

// Cấu hình chu kỳ và mẫu cho mỗi bậc
unsigned long previousMillis = 0;
const long interval = 50;       // 50ms = tần số 20Hz
const int MAX_SAMPLES = 200;    // Lấy đúng 200 mẫu cho 10 giây
int sampleCount = 0;            
int currentStep = -1;           // Bắt đầu từ -1 để khi bấm Enter lần đầu sẽ là Bậc 0 (Mốc)

bool isMeasuring = false;       // Trạng thái kiểm soát đo

// CẤU HÌNH BỘ LỌC KALMAN
float Kalman_Altitude = 0.0f; 
float P_estimate = 1.0f;      
float Q = 0.02f;              // Độ nhạy phản hồi độ cao
float R = 0.25f;              // Bộ lọc nhiễu đo lường
float K_gain = 0.0f;          

float updateKalman(float measurement) {
  P_estimate = P_estimate + Q;
  K_gain = P_estimate / (P_estimate + R);
  Kalman_Altitude = Kalman_Altitude + K_gain * (measurement - Kalman_Altitude);
  P_estimate = (1.0f - K_gain) * P_estimate;
  return Kalman_Altitude;
}

// KHỞI TẠO (SETUP)
void setup() {
  Serial.begin(115200);
  while (!Serial); 

  Serial.println(F("    HỆ THỐNG ĐO TỪNG BẬC THANG NGẮT QUÃNG (20Hz)   "));

  if (!bmp.begin(0x76)) { 
    if (!bmp.begin(0x77)) {
      Serial.println(F("[FATAL] Không tìm thấy cảm biến BMP280!"));
      while (1);
    }
  }

  // Cấu hình phần cứng tối ưu
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X4,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 

  // BƯỚC 1: WARM-UP CHỦ ĐỘNG 45 GIÂY
  Serial.print(F("[WARM-UP] Đang làm ấm chip: "));
  for (int i = WARM_UP_SECONDS; i > 0; i--) {
    if (i % 5 == 0) { Serial.print(i); Serial.print(F("s... ")); }
    volatile float dummy_p = bmp.readPressure(); 
    volatile float dummy_t = bmp.readTemperature();
    delay(1000); 
  }
  Serial.println(F("Xong!"));
  
  inHuongDan();
}

// Hàm in hướng dẫn ra màn hình
void inHuongDan() {
  Serial.println(F("\n--------------------------------------------------"));
  if (currentStep == -1) {
    Serial.println(F(">>> Hãy đặt cảm biến cố định tại MẶT SÀN (MỐC 0)"));
  } else {
    Serial.print(F(">>> Hãy di chuyển cảm biến lên BẬC THỨ "));
    Serial.println(currentStep + 1);
  }
  Serial.println(F(">>> Gõ một ký tự bất kỳ vào Serial Monitor rồi bấm Enter để ĐO 10 GIÂY..."));
}

void loop() {
  // Kiểm tra lệnh từ người dùng qua Serial Monitor
  if (Serial.available() > 0) {
    // Đọc bỏ ký tự vừa nhập để xóa bộ đệm
    while(Serial.available() > 0) { Serial.read(); } 
    
    if (!isMeasuring) {
      currentStep++; // Tăng số thứ tự bậc lên (Lần đầu tiên sẽ là Bậc 0)
      sampleCount = 0; // Reset bộ đếm mẫu
      
      // Nếu là BẬC 0 (MỐC SÀN): Thực hiện lấy mẫu nhanh để khóa P0 làm chuẩn trước khi chạy 10s
      if (currentStep == 0) {
        float sum_press = 0;
        for (int i = 0; i < 20; i++) {
          sum_press += (bmp.readPressure() / 100.0F); 
          delay(10); 
        }
        P0 = sum_press / 20.0F; 
        base_altitude = bmp.readAltitude(P0); // Khóa độ cao gốc tuyệt đối
        Kalman_Altitude = 0.0f;               // Reset bộ lọc về 0
      } else {
        // Nếu là các bậc sau: Khởi tạo lại bộ lọc Kalman bằng giá trị tương đối tức thời
        float raw_init = bmp.readAltitude(P0) - base_altitude;
        Kalman_Altitude = raw_init;
      }
      
      Serial.println(F("\n=================================================================="));
      if (currentStep == 0) {
        Serial.println(F(">>> ĐANG ĐO: MỐC SÀN BAN ĐẦU (BẬC 0) <<<"));
      } else {
        Serial.print(F(">>> ĐANG ĐO: BẬC THANG SỐ ")); Serial.println(currentStep);
      }
      Serial.println(F("STT\tÁpSuất(hPa)\tThô(m)\t\tLọcKalman(m)\tNhiệtĐộ(C)"));
      
      previousMillis = millis();
      isMeasuring = true; // Kích hoạt trạng thái đo 10 giây
    }
  }

  // Nếu đang trong trạng thái đo (chu kỳ 50ms = 20Hz)
  if (isMeasuring) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      sampleCount++;

      // 1. Đọc áp suất thực tế hiện tại (đổi sang hPa)
      float current_pressure = bmp.readPressure() / 100.0F;

      // 2. Tính toán giá trị thô tương đối dựa trên P0 đã khóa
      float relative_raw_altitude = bmp.readAltitude(P0) - base_altitude;
      float raw_temperature = bmp.readTemperature();

      // 3. Lọc mượt độ cao bằng Kalman
      float filtered_altitude = updateKalman(relative_raw_altitude);

      // 4. In toàn bộ dữ liệu ra Serial Monitor
      Serial.print(sampleCount);
      Serial.print(F("\t"));
      
      Serial.print(current_pressure, 2);      // Cột Áp suất thực tế (hPa)
      Serial.print(F("\t\t"));
      
      Serial.print(relative_raw_altitude, 2);  // Cột Độ cao thô (m)
      Serial.print(F("\t\t"));
      
      Serial.print(filtered_altitude, 2);     // Cột Độ cao lọc Kalman (m)
      Serial.print(F("\t\t"));
      
      Serial.println(raw_temperature, 1);     // Cột Nhiệt độ (C)

      // Kiểm tra nếu đủ 200 mẫu (10 giây)
      if (sampleCount >= MAX_SAMPLES) {
        isMeasuring = false; // Ngừng đo
        Serial.println(F("------------------------------------------------------------------"));
        if (currentStep == 0) {
          Serial.println(F("[HOÀN THÀNH] Đã thu thập xong dữ liệu MỐC SÀN (BẬC 0)."));
          Serial.print(F("-> P0 cố định đã khóa: ")); Serial.print(P0, 2); Serial.println(F(" hPa."));
        } else {
          Serial.print(F("[HOÀN THÀ--NH] Đã đo xong BẬC SỐ ")); Serial.print(currentStep);
          Serial.println(F(" (Đủ 200 mẫu trong 10 giây)."));
        }
        
        // Nhắc nhở người dùng di chuyển sang bậc tiếp theo
        inHuongDan();
      }
    }
  }
}