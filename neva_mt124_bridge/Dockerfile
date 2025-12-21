ARG BUILD_FROM
FROM $BUILD_FROM

# Установка Python и pip (Alpine-пакеты)
RUN apk add --no-cache python3 py3-pip

# Установка зависимостей
RUN pip3 install --no-cache-dir pyserial paho-mqtt

# Копируем скрипт
COPY run.py /

CMD ["python3", "/run.py"]
