## 📚 Документация по проекту

>Ознакомтесь, пожалуйста, с [дорожной картой проекта](./ROADMAP.md)
>Это более удобный вариант для быстрого ознакомления с проектной работой, чем через основной [.docx файл](./docs/reports/final%20reports/Проектная%20работа%20Умный%20робот-мышь.docx)

# Умный робот-мышь
Небольшой школьный учебный проект за 9 класс. Суть в том, чтобы здесь разобраться с нынешними технологиями и выделить самые эффективные, основопологающие алгоритмы, а также воплотить в жизнь идеального робота-мышь, который будет использовать различные комбинации алгоритмов и со временем все совершенствоваться и совершенствоваться - использовать что-то новое и более продуманое и избегать возможных ошибок и багов. Проект будет получать и дальнейшее развитие.

---

# Установка и запуск

## Запуск на Windows
1. Установите webots:
    - Установите с [оффициального сайта](https://cyberbotics.com/)

    ---
   
    - Или с помощью winget:

   ```powershell
    winget install Cyberbotics.Webots
   ```
   
1. Запустите webots, откройте мир - File > Open world... Выберите файл .wbt.

## Запуск на Linux
1. Установите webots одним из способов:
    - Установите с помощью apt (производные от debian дистрибутивы)

    ```shell
    sudo mkdir -p /etc/apt/keyrings
    cd /etc/apt/keyrings
    sudo wget -q https://cyberbotics.com/Cyberbotics.asc
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
    sudo apt install webots
    ```

    >Возможен баг с отсутствием зависимости libsndio - фиксится sudo apt install libsndio7.0

---

    - Cкачайте и установите релиз [snap пакета](https://github.com/cyberbotics/webots/releases/)
    Установите и запустите:

    ```shell
    sudo snap install --dangerous ./имя-файла.snap
    ```

    ---
   
    - Установите [релиз конкретно под вашу машину](https://github.com/cyberbotics/webots/releases/)
    Это может быть как deb пакет, так и простой appimage

    ---

    - Установите и запустите с помощью flatpak:
    >Внимание! Установка через flatpak настоятельно не рекомендуется. Используйте только в крайних случаях. Возможны проблемы, так как flatpak запускает программу в изолированной среде. Читайте [справку](https://github.com/cyberbotics/webots/discussions) и [issues](https://github.com/cyberbotics/webots/issues)

    ```shell
    flatpak install flathub com.cyberbotics.webots
    flatpak run com.cyberbotics.webots
    ```

    ---

    - Если хотите, [Сбилдите самостоятельно](https://github.com/cyberbotics/webots/wiki)

2. (Опционально) Добавьте переменную WEBOTS_HOME, если webots установлен через github-релиз или сбилжен самостоятельно, или просто хотите перестраховаться
   
    ```shell
    export WEBOTS_HOME=/usr/local/webots
    ```
    
3. Запустите webots, откройте мир - File > Open world... Выберите файл .wbt.

> Вам, возможно, потребуется скомпилировать в первый раз файл контроллера самостоятельно (my_controller.cpp). Подтяните все зависимости, требущиеся для компиляции (обычно хватает простого make и gcc, (base-devel)). Затем скомпилируйте:
> ```shell
> make
> ```

## Запуск на MacOs
1. Скачайте и установите [.dmg образ](https://github.com/cyberbotics/webots/releases/)
2. Запустите webots, откройте мир - File > Open world... Выберите файл .wbt.

---

# Contributing
Проект открыт для дальнейшего развития и контрибьюторы всегда приветствуются. Возможно в будущем появятся различные issues и/или pull requests. Если же вам хочется связаться со мной / внести что то свое, смело открывайте issue или пишите на почту.
