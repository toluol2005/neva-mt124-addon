ARG BUILD_FROM
FROM $BUILD_FROM

# Установка Python и библиотек через apk
RUN apk add --no-cache python3 py3-pyserial py3-paho-mqtt

# Копируем скрипт
COPY run.py /

CMD ["python3", "/run.py"]
