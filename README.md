#DC-Motor-Kontrol-Duzenegi

Düzeneği çalıştırmak için spiITHal dosyasını STM32-F746ZG Nucleo kartına yükledikten sonra, simulink dosyası açılıp çalıştırılmalıdır. usbsetup.m dosyası ile simulink dosyası aynı dizinde değilse hata vericektir.
Digital Control dersi kapsamında matlab dosyaları çokça değişime uğramış olsalar da hepsi comment olarak durmakta ve çalışmayı etkilememektedir.

Düzeneği çalıştırırken iki adet usb kablosu gerekmektedir. Bir tanesi STM32 geliştirme kartını beslerken diğeri simulink haberleşmesini sağlamaktadır.
Besleme kablosundan kurtulmak için basitçe bir 12V-5V DC-DC dönüştürücü kullanılabilir. Bu sayede motoru besleyen 12V adaptörden güç alınabilir.

Düzeneğin detaylı açıklaması için aşağıdaki rapora göz atabilirsiniz.

[2247-c_sonuc_raporu_formu_HalilCemreGündoğdu.pdf](https://github.com/user-attachments/files/18229585/2247-c_sonuc_raporu_formu_HalilCemreGundogdu.pdf)

