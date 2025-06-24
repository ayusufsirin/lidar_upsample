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