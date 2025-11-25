ROS & MAVROS ile Drone Otonom Kontrol ve Görüntü İşleme

Bu proje, ROS (Robot Operating System) ve MAVROS kullanarak bir drone'un otonom olarak kalkış yapmasını, belirli bir irtifaya yükselmesini ve OpenCV kullanarak kamera görüntüsünde kırmızı bir obje aramasını simüle eder.

Proje, temel uçuş dinamiklerini öğrenmek ve görüntü işleme algoritmalarını otonom sistemlere entegre etmek amacıyla geliştirilmiştir.

📂 Proje İçeriği

Bu repo iki ana Python scripti içerir:

src/simple_takeoff.py

Amaç: Temel bağlantı ve uçuş testi.

İşlev: Drone'u ARM eder, GUIDED moda alır ve 10 metreye yükselir. Görüntü işleme içermez.

Kullanım: Bağlantı sorunlarını gidermek ve ilk uçuşu test etmek için kullanılır.

src/mission_red_search.py

Amaç: Tam görev senaryosu.

İşlev: Kalkış yapar, yükselir, kamera görüntüsünü analiz eder. Kırmızı bir obje tespit ettiğinde iniş (LAND) yapar.

Kullanım: Otonom görev simülasyonu için kullanılır.

🛠 Gereksinimler

Bu projeyi çalıştırmak için aşağıdaki sistem ve paketlere ihtiyacınız vardır:

ROS: Noetic (Ubuntu 20.04) veya Melodic (Ubuntu 18.04)

MAVROS: Drone ile iletişim için gerekli ROS paketi.

ArduPilot SITL: Simülasyon ortamı (veya gerçek bir uçuş kontrolcüsü).

OpenCV: Görüntü işleme kütüphanesi (cv2 ve cv_bridge).

🚀 Kurulum

Bu repoyu catkin çalışma alanınızın src klasörüne klonlayın veya dosyaları kopyalayın.

cd ~/catkin_ws/src
# Dosyaları buraya kopyalayın


Çalışma alanını derleyin:

cd ~/catkin_ws
catkin_make
source devel/setup.bash


Python dosyalarına çalıştırma izni verin:

chmod +x src/simple_takeoff.py
chmod +x src/mission_red_search.py


🎮 Çalıştırma

1. Simülasyonu Başlatın

Önce ArduPilot SITL ve MAVROS'u başlatın (UDP bağlantısı örneği):

roslaunch mavros apm.launch fcu_url:=udp://:14540@127.0.0.1:14550


2. Temel Kalkış Testi

Sadece bağlantıyı ve yükselmeyi test etmek için:

rosrun <paket_isminiz> simple_takeoff.py


3. Kırmızı Obje Arama Görevi

Kamera verisi ile görev yapmak için:

rosrun <paket_isminiz> mission_red_search.py


📝 Notlar ve İpuçları

Kamera Topic'i: Kod içerisinde /camera/image_raw topic'i dinlenmektedir. Kullandığınız Gazebo plugin'ine veya fiziksel kameraya göre bu topic ismi değişebilir.

MAVROS Bağlantısı: Script çalıştırıldığında "MAVROS bağlanıyor..." yazısında takılı kalıyorsa, FCU (Flight Control Unit) bağlantınızı ve fcu_url parametrelerini kontrol edin.

Güvenlik: Gerçek bir drone üzerinde test yapmadan önce mutlaka simülasyon ortamında (SITL/Gazebo) kodu doğrulayın.

Bu proje, otonom drone geliştirme çalışmaları kapsamında oluşturulmuş bir arşivdir.