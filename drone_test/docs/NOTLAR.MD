MAVROS ve Uçuş Kontrol Notları

Bu doküman, proje geliştirme sürecinde alınan notları, kullanılan MAVROS topic'lerini ve pratik ipuçlarını içerir.

MAVROS Topic ve Servisleri

Projede kullanılan temel topic ve servisler şunlardır:

Tip

İsim

Açıklama

Topic

/mavros/state

Drone'un o anki uçuş durumu (ARM, MODE, Connection).

Topic

/mavros/setpoint_position/local

Drone'a gitmesi gereken XYZ konumunu bildirmek için kullanılır.

Topic

/mavros/setpoint_velocity/cmd_vel

Drone'a hız komutları göndermek için kullanılır.

Topic

/camera/image_raw

Kamera görüntü akışı (Gazebo plugin ayarına göre değişebilir).

Service

/mavros/cmd/arming

Motorları ARM (çalıştır) veya DISARM (durdur) yapmak için.

Service

/mavros/set_mode

Uçuş modunu değiştirmek için (GUIDED, LAND, RTL vb.).

Uçuş Mantığı ve Pratik Bilgiler

MAVROS üzerinden bir drone'u kontrol etmek için genellikle şu sıra izlenir:

Bağlantı Kontrolü: FCU (Flight Control Unit) ile bağlantı var mı? (/mavros/state -> connected)

Setpoint Akışı: GUIDED moda geçmeden önce güvenlik amacıyla sisteme birkaç setpoint (hedef konum) gönderilmeye başlanmalıdır.

Mod Değişimi: Drone GUIDED moda alınır.

ARM: Motorlar çalıştırılır (arming servisi).

Kalkış: İstenilen irtifaya pose mesajları gönderilerek çıkılır.

Pratik Test Adımları (Manuel)

Kod yazmadan önce terminalden test etmek için:

Drone'u ARM et.

GUIDED moda geçir.

10 metreye çıkmasını sağla.

LAND komutu ile indirmeyi dene.

Bağlantı Parametreleri

ArduPilot SITL ile yerel bağlantı örneği (UDP):

roslaunch mavros apm.launch fcu_url:=udp://:14540@127.0.0.1:14550


Bağlantı başarılı olduğunda rostopic echo /mavros/state komutu şu çıktıyı vermelidir:

connected: True

armed: False (Başlangıçta)

mode: STABILIZE (Veya simülasyon başlangıç modu)