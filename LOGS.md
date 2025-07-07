# 18.06.2025

Bugün fark ettiğim kadarıyla doğru çalışma yapabilmek için öncelikle topic'ler arası eşzamanlılığın sağlandığına emin
olmamız gerekiyor. Buna emin olabilmek için [meas_topic_stats.py](meas_topic_stats.py) kodunu yazdım ve topic'ler arası
eşzamanlılık ölçümleri yapmaya başladım

# 19.06.2025

Citrus Farm verisetinin eşzamanlı topic'ler içerdiğine emin oldum. Daha sonra önişleme yaptığım `/cumulative_pointcloud`
gibi topic'lerde gecikme olduğunu gördüm. Bunu şimdilik `rosbag play -r 0.1` yaparak yani rate değerini düşürerek
elimine ettim. (
bkz. [topic_stats_20250619_230104_ten2one_rate.csv](./logs/topic_stats_20250619_230104_ten2one_rate.csv))

# 21.06.2025

Yeni profiler'lar cıtrusFarm `main.py` dosyasına eklendi. Bu sayede darboğazın `msg2pts` fonksiyonunda olduğu anlaşıldı.
Bu fonksiyon GPU'da çalışmaya daha uygun hale getirildi. Toplam işlem gecikmesi ~18 ms civalarına kadar düşürüldü (
nihayi fusion süresi için değil).

# 23.06.2025

Depth imajlari PC haline getiren fonksiyonlar tekrar aktive edildi ve profiler'lar bunlar için de eklendi. Anlaşıldığı
üzere `pc_msg = pc2.create_cloud(header, fields, cart_pts_np)` kod bloğunun işlenmesi bir darboğaz oluşturuyor.
Burasının otimize edilmesi gerektiğine karar verildi.

# 24.06.2025

Daha önceden darboğaza sebep olduğu tespit edilen `pc2.create_cloud` fonksiyonu yenisiyle (`create_cloud_from_np`)
değiştirildi.

# 25.06.2025

`lidar_upsample.py` koduna yönlendim ve buradaki takılma ve gecikme sorunlarına el atmaya başladım. ~500 ms olan gecikme
değerini düşürmek için önce profiling işlemlerini ekliyorum.

# 26.06.2025

Lidar upsample için optimizasyonlar yapıldı ve işlem süresi yaklaşık 20 kat düşürüldü.

# 27.06.2025

Lidar upsample'daki rosbag yeniden başlayınca donma problemi giderildi.

# 29.06.2025

VLP point cloud mesajindan depth imaj olusturma problemi giderildi.

# 01.07.2025

Algoritmadaki orijinden başlayan atlamaların sebebinin `NaN` noktaların `0` olarak değiştirilmesi olduğu görülmüştür.
Bunun çözümü olarak `40` değeri konularak denenmiş ve bu şekilde de yine sorun olduğu görülmüştür.

![pg_nan_filled_40.png](pg_nan_filled_40.png)

40

![pg_nan_filled_1.png](pg_nan_filled_1.png)

0

# 04.07.2025

Satranç tahtası şeklindeki sentetik veri kullanılarak algoritmanın çalıştığı doğrulandı. Aşağıdaki gibi sonuçlar elde
edildi. İsmail hoca sonuçların gayet iyi olduğunu söyledi. Benzer görüntünün gerçek veri ile elde edilememesinin
sebebinin ZED kamera ile VLP Lidarın FOV kesişimlerinin PG FOV olarak kullanılmamamsı olabileveğini söyledi.

ZED:

![dama_zed.png](assets/dama_zed.png)

VLP:

![dama_vlp.png](assets/dama_vlp.png)

PG:

![dama_pg.png](assets/dama_pg.png)

# 07.07.2025

- Önce yüzey düzlemlerinin bulunduğu ve bu düzlemlerin eksenlerine uygun olarak PG'nin işlendiği bir yapı kurulabilir.
  Bu sayede yakınsama daha iyi olacaktır. Örnek vermek gerekirse zemin yüzeyi her zaman kameraya dik olduğu için bu
  alanlarda yakınsama yapmak çok da mümkün olmayabilir. Halbuki önerilen yöntem ile doğru düzlemde PG uygulanabilir.
  Bunun uygulanması tamamen paralel olabilecektir.

- Bir diğer öneri ise derinlik imajı alınmadan önce bir önişleme olarak kamera yatay ekseninde tüm nokta kümelerinin
  rotasyona uyratılması. Bu şekilde zemin yüzeyi noktalarının derinlik imajında daha geniş bir alana projekt etmesi
  sağlanacaktır.

  > :warning: Bu yapılırken overlap olacak noktalara dikkat edilmelidir.

## PG parametre ayarları

Farklı frekans değerleri için elde edilen görüntüler aşağıdaki tabloda verilmiştir.

| Cutoff (Normalized) | Iterations | Image                                          |
|---------------------|------------|------------------------------------------------|
| 0.01                | 100        | ![dama_0.01_100.png](assets/dama_0.01_100.png) |
| 0.02                | 100        | ![dama_0.02_100.png](assets/dama_0.02_100.png) |
| 0.04                | 100        | ![dama_0.04_100.png](assets/dama_0.04_100.png) |
| 0.08                | 100        | ![dama_0.08_100.png](assets/dama_0.08_100.png) |
| 0.16                | 100        | ![dama_0.16_100.png](assets/dama_0.16_100.png) |

## VLP upsamling ile ilgili problem

Sensörlerin anlık olarak göremediği ancak historical olarak tutulan VLP verisinde olan ve aslında gölgelenmiş olması
gereken bir takım noktalar ortaya çıkıyor. Bu da algoritmanın ZED noktalarını bu uzak noktalara yakınsamaya calısmasına
sebep oluyor. Yakınsanmaya çalışılan ZED ve VLP noktaları birbirinden çok uzaklaştığı için low-pass etkiye maruz
kaldıklarında orijine kadar uzanan spike gibi bir geometri ortaya çıkıyor.

Çözüm önerileri:

- Band-reject filtre denenebilir
