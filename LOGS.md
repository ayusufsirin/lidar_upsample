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

![img.png](assets/dama_zed.png)

VLP:

![img_2.png](assets/dama_vlp.png)

PG:

![img_1.png](assets/dama_pg.png)

